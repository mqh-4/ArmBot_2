#!/usr/bin/env python3
import math
import os
import re
import subprocess
import sys
import tempfile
import wave
from array import array
from collections import deque


def pcm_rms_s16le(pcm):
    """Return the RMS amplitude of signed 16-bit little-endian PCM."""
    usable = len(pcm) - (len(pcm) % 2)
    if usable == 0:
        return 0.0
    samples = array('h')
    samples.frombytes(pcm[:usable])
    if sys.byteorder != 'little':
        samples.byteswap()
    energy = sum(sample * sample for sample in samples)
    return math.sqrt(float(energy) / len(samples))


class SpeechSegmenter:
    """Energy-based streaming speech segmenter for fixed-duration PCM frames."""

    def __init__(self, threshold, frame_ms=30, start_ms=90, silence_ms=800,
                 pre_roll_ms=300, min_speech_ms=300, max_speech_ms=8000):
        if threshold <= 0 or frame_ms <= 0:
            raise ValueError('threshold and frame_ms must be positive')
        self.threshold = float(threshold)
        self.start_frames = max(1, int(math.ceil(float(start_ms) / frame_ms)))
        self.silence_frames = max(1, int(math.ceil(float(silence_ms) / frame_ms)))
        self.pre_roll_frames = max(1, int(math.ceil(float(pre_roll_ms) / frame_ms)))
        self.min_voice_frames = max(1, int(math.ceil(float(min_speech_ms) / frame_ms)))
        self.max_frames = max(1, int(math.ceil(float(max_speech_ms) / frame_ms)))
        self._pre_roll = deque(maxlen=self.pre_roll_frames)
        self.reset()

    def reset(self):
        self._active = False
        self._start_count = 0
        self._silent_count = 0
        self._voice_count = 0
        self._frames = []
        self._pre_roll.clear()

    def push(self, frame):
        voiced = pcm_rms_s16le(frame) >= self.threshold
        if not self._active:
            self._pre_roll.append(frame)
            self._start_count = self._start_count + 1 if voiced else 0
            if self._start_count < self.start_frames:
                return None
            self._active = True
            self._frames = list(self._pre_roll)
            self._voice_count = self._start_count
            self._silent_count = 0
            return None

        self._frames.append(frame)
        if voiced:
            self._voice_count += 1
            self._silent_count = 0
        else:
            self._silent_count += 1

        timed_out = len(self._frames) >= self.max_frames
        ended = self._silent_count >= self.silence_frames
        if not (timed_out or ended):
            return None
        return self._finish()

    def flush(self):
        if not self._active:
            self.reset()
            return None
        return self._finish()

    def _finish(self):
        segment = b''.join(self._frames) if self._voice_count >= self.min_voice_frames else None
        self.reset()
        return segment


def normalize_transcript(text):
    text = re.sub(r'\s+', ' ', text).strip()
    if not text:
        return ''
    markers = {
        '[blank_audio]', '[silence]', '[music]', '(silence)', '(music)',
        '（无声）', '（音乐）', '[无声]', '[音乐]',
    }
    if text.lower() in markers:
        return ''
    if not re.search(r'[\w\u3400-\u9fff]', text, flags=re.UNICODE):
        return ''
    return text


class WhisperCliRecognizer:
    def __init__(self, executable, model_path, language='zh', threads=4,
                 no_gpu=True, timeout=60.0):
        self.executable = executable
        self.model_path = model_path
        self.language = language
        self.threads = max(1, int(threads))
        self.no_gpu = bool(no_gpu)
        self.timeout = float(timeout)

    def build_command(self, wav_path, output_prefix):
        command = [
            self.executable,
            '-m', self.model_path,
            '-f', wav_path,
            '-l', self.language,
            '-t', str(self.threads),
            '-nt',
            '-otxt',
            '-of', output_prefix,
        ]
        if self.no_gpu:
            command.append('-ng')
        return command

    def transcribe(self, pcm, sample_rate=16000, channels=1):
        with tempfile.TemporaryDirectory(prefix='desk_whisper_') as temp_dir:
            wav_path = os.path.join(temp_dir, 'utterance.wav')
            output_prefix = os.path.join(temp_dir, 'transcript')
            with wave.open(wav_path, 'wb') as wav_file:
                wav_file.setnchannels(channels)
                wav_file.setsampwidth(2)
                wav_file.setframerate(sample_rate)
                wav_file.writeframes(pcm)
            result = subprocess.run(
                self.build_command(wav_path, output_prefix),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=self.timeout,
                check=False,
                universal_newlines=True,
            )
            if result.returncode != 0:
                raise RuntimeError(
                    'whisper.cpp exited with code %d: %s' %
                    (result.returncode, result.stdout[-500:].strip())
                )
            transcript_path = output_prefix + '.txt'
            if os.path.isfile(transcript_path):
                with open(transcript_path, 'r', encoding='utf-8') as transcript_file:
                    return normalize_transcript(transcript_file.read())
            return normalize_transcript(result.stdout)
