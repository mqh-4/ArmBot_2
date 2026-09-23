#!/usr/bin/env python3
import os
import struct
import unittest

from desk_cleaning.audio import (
    SpeechSegmenter,
    WhisperCliRecognizer,
    normalize_transcript,
    pcm_rms_s16le,
)


def pcm_frame(amplitude, samples=160):
    return struct.pack('<%dh' % samples, *([amplitude] * samples))


class AudioTests(unittest.TestCase):
    def test_pcm_rms(self):
        self.assertEqual(pcm_rms_s16le(pcm_frame(0)), 0.0)
        self.assertAlmostEqual(pcm_rms_s16le(pcm_frame(1200)), 1200.0)

    def test_segmenter_emits_speech_after_silence(self):
        segmenter = SpeechSegmenter(
            threshold=500, frame_ms=10, start_ms=20, silence_ms=30,
            pre_roll_ms=20, min_speech_ms=20, max_speech_ms=1000,
        )
        frames = [pcm_frame(0), pcm_frame(1000), pcm_frame(1000),
                  pcm_frame(1000), pcm_frame(0), pcm_frame(0), pcm_frame(0)]
        emitted = [segmenter.push(frame) for frame in frames]
        segments = [value for value in emitted if value is not None]
        self.assertEqual(len(segments), 1)
        self.assertGreaterEqual(len(segments[0]), 6 * len(frames[0]))

    def test_segmenter_rejects_short_noise(self):
        segmenter = SpeechSegmenter(
            threshold=500, frame_ms=10, start_ms=10, silence_ms=20,
            pre_roll_ms=10, min_speech_ms=30, max_speech_ms=1000,
        )
        frames = [pcm_frame(1000), pcm_frame(0), pcm_frame(0)]
        self.assertTrue(all(segmenter.push(frame) is None for frame in frames))

    def test_normalize_transcript(self):
        self.assertEqual(normalize_transcript('  清理   桌面\n'), '清理 桌面')
        self.assertEqual(normalize_transcript('[BLANK_AUDIO]'), '')

    def test_whisper_command_has_output_file(self):
        recognizer = WhisperCliRecognizer(
            '/opt/whisper/whisper-cli', '/models/ggml-base.bin', threads=3,
        )
        command = recognizer.build_command('/tmp/input.wav', '/tmp/result')
        self.assertEqual(command[0], '/opt/whisper/whisper-cli')
        self.assertIn('-otxt', command)
        self.assertEqual(command[command.index('-of') + 1], '/tmp/result')
        self.assertEqual(command[command.index('-t') + 1], '3')
        self.assertTrue(command[-1] == '-ng')


if __name__ == '__main__':
    unittest.main()
