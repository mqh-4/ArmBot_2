"""Bounded ALSA recording and lazy, reusable faster-whisper transcription."""
from contextlib import contextmanager
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import wave


def trim_encoder_padding(features):
    """Keep context for short utterances without encoding 30 seconds of zeros.

    faster-whisper pads mel features to 3000 frames before calling encode.
    CTranslate2 accepts shorter inputs. Retain at least six seconds, one
    second of trailing context, and the complete input for longer speech.
    """
    import numpy as np
    if features.ndim != 2 or features.shape[-1] != 3000:
        return features
    occupied = np.flatnonzero(np.any(features != 0, axis=0))
    frames = int(occupied[-1]) + 1 if occupied.size else 0
    if frames > 800:
        return features
    length = max(600, frames + 100)
    length += length % 2
    return np.ascontiguousarray(features[:, :length])


class SpeechInput:
    def __init__(self, device='default', seconds=5, model='small', language='zh',
                 download_root='', local_files_only=False, hf_endpoint='https://hf-mirror.com',
                 max_seconds=60, beam_size=1, cpu_threads=4, compact_encoder=True):
        if not 1 <= seconds <= 60:
            raise ValueError('Recording duration must be between 1 and 60 seconds')
        if not 1 <= max_seconds <= 600:
            raise ValueError('Maximum recording duration must be between 1 and 600 seconds')
        self.device = device
        self.seconds = seconds
        self.model_name = model
        self.language = language
        self.download_root = download_root or None
        self.local_files_only = local_files_only
        self.hf_endpoint = hf_endpoint
        self.max_seconds = max_seconds
        self.beam_size = beam_size
        self.cpu_threads = cpu_threads
        self.compact_encoder = compact_encoder
        self.model = None
        self.process = None
        self.lock = threading.Lock()
        self.model_lock = threading.Lock()
        self.stopped = threading.Event()

    @contextmanager
    def recording(self):
        with tempfile.TemporaryDirectory(prefix='armbot-voice-') as directory:
            path = str(Path(directory) / 'input.wav')
            with self.lock:
                if self.stopped.is_set():
                    raise RuntimeError('Recording was cancelled')
                self.process = subprocess.Popen([
                    'arecord', '-q', '-D', self.device, '-f', 'S16_LE', '-r', '16000',
                    '-c', '1', '-d', str(self.seconds), path,
                ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
                process = self.process
            try:
                try:
                    _, stderr = process.communicate(timeout=self.seconds + 10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate()
                    raise RuntimeError('Audio recording timed out')
                if self.stopped.is_set():
                    raise RuntimeError('Recording was cancelled')
                if process.returncode:
                    raise RuntimeError('arecord failed: ' + stderr.decode(errors='replace')[-300:])
                with wave.open(path, 'rb') as audio:
                    if (audio.getnframes() == 0 or audio.getframerate() != 16000
                            or audio.getnchannels() != 1 or audio.getsampwidth() != 2):
                        raise ValueError('Expected non-empty 16 kHz mono PCM16 WAV')
                yield path
            finally:
                with self.lock:
                    self.process = None

    @contextmanager
    def recording_until_stopped(self, stop_event):
        """Record until ``stop_event`` is set, with a hard duration limit."""
        with tempfile.TemporaryDirectory(prefix='armbot-voice-') as directory:
            path = str(Path(directory) / 'input.wav')
            with self.lock:
                if self.stopped.is_set():
                    raise RuntimeError('Recording was cancelled')
                self.process = subprocess.Popen([
                    'arecord', '-q', '-D', self.device, '-f', 'S16_LE', '-r', '16000',
                    '-c', '1', path,
                ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
                process = self.process
            try:
                elapsed = 0.0
                while process.poll() is None and elapsed < self.max_seconds:
                    if stop_event.wait(0.1):
                        break
                    elapsed += 0.1
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    _, stderr = process.communicate(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate()
                    raise RuntimeError('Audio recording timed out')
                if self.stopped.is_set():
                    raise RuntimeError('Recording was cancelled')
                if process.returncode and not stop_event.is_set():
                    raise RuntimeError('arecord failed: ' + stderr.decode(errors='replace')[-300:])
                with wave.open(path, 'rb') as audio:
                    if (audio.getnframes() == 0 or audio.getframerate() != 16000
                            or audio.getnchannels() != 1 or audio.getsampwidth() != 2):
                        raise ValueError('Expected non-empty 16 kHz mono PCM16 WAV')
                yield path
            finally:
                with self.lock:
                    self.process = None

    def load_model(self):
        if self.stopped.is_set():
            raise RuntimeError('Speech processing was cancelled')
        with self.model_lock:
            self._load_model()

    def _load_model(self):
        if self.model is None:
            if self.hf_endpoint:
                os.environ.setdefault('HF_ENDPOINT', self.hf_endpoint)
            from faster_whisper import WhisperModel
            try:
                self.model = WhisperModel(
                    self.model_name, device='cpu', compute_type='int8',
                    download_root=self.download_root, local_files_only=self.local_files_only,
                    cpu_threads=self.cpu_threads,
                )
            except Exception as error:
                detail = str(error).strip().replace('\n', ' ')[-300:]
                raise RuntimeError(
                    'Whisper model unavailable: %s. Set whisper.model to a local '
                    'faster-whisper directory or make the model mirror reachable.' % detail,
                ) from error
    def transcribe(self, path):
        self.load_model()
        chunk_length = None
        if self.compact_encoder:
            try:
                with wave.open(path, 'rb') as audio:
                    duration = audio.getnframes() / float(audio.getframerate())
                if duration <= 8.0:
                    chunk_length = max(6, min(30, int(duration) + 2))
            except (OSError, wave.Error):
                # Keep compatibility with callers passing file-like fixtures.
                chunk_length = None
        segments, _ = self.model.transcribe(
            path, language=self.language or None, vad_filter=True,
            beam_size=self.beam_size, best_of=1, temperature=0.0,
            condition_on_previous_text=False, without_timestamps=True,
            chunk_length=chunk_length,
        )
        return ''.join(segment.text.strip() for segment in segments).strip()

    def stop(self):
        self.stopped.set()
        self.request_stop()

    def request_stop(self):
        """Stop only the active recording; keep the speech input reusable."""
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                try:
                    self.process.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
