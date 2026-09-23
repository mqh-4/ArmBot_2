#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

from armpi_fpv_common.asr import ASR


class HardwareAsrBridge:
    def __init__(self):
        self.publisher = rospy.Publisher('/desk_cleaning/voice_text', String, queue_size=10)
        self.commands = {
            int(key): value for key, value in rospy.get_param('~commands', {
                '1': '清理桌面', '2': '停止清理', '3': '继续清理',
            }).items()
        }
        self.asr = ASR(bus=int(rospy.get_param('~i2c_bus', 1)))
        if rospy.get_param('~configure_vocabulary', False):
            self.asr.eraseWords()
            self.asr.setMode(int(rospy.get_param('~mode', 1)))
            vocabulary = rospy.get_param('~vocabulary', {})
            for key, words in vocabulary.items():
                self.asr.addWords(int(key), words)

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            result = self.asr.getResult()
            if result in self.commands:
                text = self.commands[result]
                rospy.loginfo('ASR command %s: %s', result, text)
                self.publisher.publish(String(data=text))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('desk_cleaning_hardware_asr')
    try:
        HardwareAsrBridge().spin()
    except Exception as exc:
        rospy.logfatal('hardware ASR initialization failed: %s', exc)
