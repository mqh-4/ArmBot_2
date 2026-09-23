#!/usr/bin/env python3
import os
import shutil
import subprocess
import threading
import time

import rospy
from std_msgs.msg import Bool, String

from desk_cleaning.audio import SpeechSegmenter, WhisperCliRecognizer


class UsbWhisperAsrNode:
    def __init__(self):
        self.sample_rate = int(rospy.get_param('~sample_rate', 16000))
        self.channels = int(rospy.get_param('~channels', 1))
        self.frame_ms = int(rospy.get_param('~vad/frame_ms', 30))
        if self.channels != 1:
            raise ValueError('only mono USB microphone input is supported')
        samples_per_frame = self.sample_rate * self.frame_ms
        if samples_per_frame % 1000:
            raise ValueError('sample_rate * frame_ms must be divisible by 1000')
        self.frame_bytes = samples_per_frame // 1000 * self.channels * 2

        self.device = str(rospy.get_param('~device', 'default'))
        self.recorder_executable = self._resolve_executable(
            str(rospy.get_param('~arecord_executable', 'arecord')),
        )
        whisper_executable = str(rospy.get_param('~whisper/executable', '')).strip()
        self.whisper_executable = self._resolve_whisper(whisper_executable)
        configured_model = str(rospy.get_param('~whisper/model_path', '')).strip()
        if not configured_model:
            raise ValueError('~whisper/model_path is required')
        self.model_path = os.path.abspath(os.path.expanduser(configured_model))
        if not os.path.isfile(self.model_path):
            raise ValueError('whisper model not found: %s' % self.model_path)

        self.segmenter = SpeechSegmenter(
            threshold=float(rospy.get_param('~vad/rms_threshold', 550)),
            frame_ms=self.frame_ms,
            start_ms=int(rospy.get_param('~vad/start_ms', 90)),
            silence_ms=int(rospy.get_param('~vad/silence_ms', 800)),
            pre_roll_ms=int(rospy.get_param('~vad/pre_roll_ms', 300)),
            min_speech_ms=int(rospy.get_param('~vad/min_speech_ms', 300)),
            max_speech_ms=int(rospy.get_param('~vad/max_speech_ms', 8000)),
        )
        self.recognizer = WhisperCliRecognizer(
            executable=self.whisper_executable,
            model_path=self.model_path,
            language=str(rospy.get_param('~whisper/language', 'zh')),
            threads=int(rospy.get_param('~whisper/threads', 4)),
            no_gpu=bool(rospy.get_param('~whisper/no_gpu', True)),
            timeout=float(rospy.get_param('~whisper/timeout_sec', 60.0)),
        )
        self.mute_during_tts = bool(rospy.get_param('~mute_during_tts', True))
        self.tts_guard_sec = float(rospy.get_param('~tts_guard_sec', 3.0))
        self.restart_delay_sec = float(rospy.get_param('~restart_delay_sec', 1.0))
        self.min_text_chars = int(rospy.get_param('~min_text_chars', 1))
        self._muted_until = 0.0
        self._mute_lock = threading.Lock()
        self._segmenter_lock = threading.Lock()
        self._recorder = None

        self.text_pub = rospy.Publisher('~text', String, queue_size=10)
        self.listening_pub = rospy.Publisher('~listening', Bool, queue_size=1, latch=True)
        if self.mute_during_tts:
            rospy.Subscriber('/desk_cleaning/speech', String, self._speech_callback, queue_size=10)
        rospy.on_shutdown(self._stop_recorder)
        rospy.loginfo(
            'USB whisper ASR ready (device=%s, model=%s)',
            self.device, self.model_path,
        )

    @staticmethod
    def _resolve_executable(value):
        expanded = os.path.abspath(os.path.expanduser(value)) if os.path.sep in value else value
        resolved = expanded if os.path.isfile(expanded) else shutil.which(value)
        if not resolved:
            raise ValueError('executable not found: %s' % value)
        return resolved

    def _resolve_whisper(self, configured):
        if configured:
            return self._resolve_executable(configured)
        resolved = shutil.which('whisper-cli')
        if resolved:
            return resolved
        raise ValueError('whisper-cli not found; set ~whisper/executable')

    def _speech_callback(self, _message):
        with self._mute_lock:
            self._muted_until = max(self._muted_until, time.monotonic() + self.tts_guard_sec)
        with self._segmenter_lock:
            self.segmenter.reset()

    def _is_muted(self):
        with self._mute_lock:
            return time.monotonic() < self._muted_until

    def _start_recorder(self):
        command = [
            self.recorder_executable,
            '-q', '-D', self.device,
            '-t', 'raw', '-f', 'S16_LE',
            '-r', str(self.sample_rate), '-c', str(self.channels),
        ]
        rospy.loginfo('listening on ALSA device %s', self.device)
        self._recorder = subprocess.Popen(command, stdout=subprocess.PIPE)
        self.listening_pub.publish(Bool(data=True))

    def _stop_recorder(self):
        recorder = self._recorder
        self._recorder = None
        if recorder is None:
            return
        if recorder.poll() is None:
            recorder.terminate()
            try:
                recorder.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                recorder.kill()
                recorder.wait()
        if recorder.stdout:
            recorder.stdout.close()
        self.listening_pub.publish(Bool(data=False))

    def _capture_utterance(self):
        with self._segmenter_lock:
            self.segmenter.reset()
        self._start_recorder()
        try:
            while not rospy.is_shutdown():
                frame = self._recorder.stdout.read(self.frame_bytes)
                if len(frame) != self.frame_bytes:
                    return_code = self._recorder.poll()
                    raise RuntimeError('arecord stopped unexpectedly (code=%s)' % return_code)
                if self._is_muted():
                    with self._segmenter_lock:
                        self.segmenter.reset()
                    continue
                with self._segmenter_lock:
                    segment = self.segmenter.push(frame)
                if segment:
                    return segment
        finally:
            self._stop_recorder()
        return None

    def spin(self):
        while not rospy.is_shutdown():
            try:
                pcm = self._capture_utterance()
                if not pcm or rospy.is_shutdown():
                    continue
                rospy.loginfo('speech detected, running whisper.cpp')
                text = self.recognizer.transcribe(
                    pcm, sample_rate=self.sample_rate, channels=self.channels,
                )
                if len(text) < self.min_text_chars:
                    rospy.loginfo('whisper.cpp returned no usable text')
                    continue
                rospy.loginfo('recognized speech: %s', text)
                self.text_pub.publish(String(data=text))
            except Exception as exc:
                if rospy.is_shutdown():
                    break
                rospy.logerr_throttle(5.0, 'USB ASR error: %s', exc)
                self._stop_recorder()
                rospy.sleep(self.restart_delay_sec)


if __name__ == '__main__':
    rospy.init_node('usb_whisper_asr')
    try:
        UsbWhisperAsrNode().spin()
    except Exception as exc:
        rospy.logfatal('USB whisper ASR initialization failed: %s', exc)
