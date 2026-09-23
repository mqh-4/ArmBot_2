#!/usr/bin/env python3
import re
import threading

import rospy
from std_msgs.msg import String


class TtsFeedback:
    def __init__(self):
        self.lock = threading.Lock()
        self.hardware = None
        if rospy.get_param('~use_hardware', False):
            from armpi_fpv_common.tts import TTS
            self.hardware = TTS(bus=int(rospy.get_param('~i2c_bus', 1)))
        rospy.Subscriber('/desk_cleaning/speech', String, self.callback, queue_size=20)

    @staticmethod
    def chunks(text, limit=28):
        parts = [part for part in re.split(r'([，。；！？])', text) if part]
        chunks = []
        current = ''
        for part in parts:
            while len(part) > limit:
                if current:
                    chunks.append(current)
                    current = ''
                chunks.append(part[:limit])
                part = part[limit:]
            if len(current) + len(part) > limit and current:
                chunks.append(current)
                current = ''
            current += part
        if current:
            chunks.append(current)
        return chunks

    def callback(self, msg):
        rospy.loginfo('robot says: %s', msg.data)
        if self.hardware is None:
            return
        with self.lock:
            for chunk in self.chunks(msg.data):
                self.hardware.TTSModuleSpeak('[h0][v10]', chunk)
                rospy.sleep(max(0.8, len(chunk) * 0.16))


if __name__ == '__main__':
    rospy.init_node('desk_cleaning_tts')
    TtsFeedback()
    rospy.spin()
