#!/usr/bin/env python
import rospy
import json
import urllib2
from emotiv_node.msg import *
from std_msgs.msg import Float32

class EmotivWristCtrl:
    def __init__(self):
        rospy.init_node('EmotivWristCtrl')
        self.data_sub = rospy.Subscriber('emotiv/data', Data, self.data_cb)
        self.expressiv_sub = rospy.Subscriber('emotiv/expressiv', Expressiv,
                self.expressiv_cb)

        self.hand_pub = rospy.Publisher('wrist_diff/desired_hand',
                Float32)
        self.angle_pub = rospy.Publisher('wrist_diff/desired_angle',
                Float32)
        self.rotate_pub = rospy.Publisher('wrist_diff/desired_rotation',
                Float32)

        self.enabled = False
        self.rotate = 0
        self.angle = 0

    def data_cb(self, data):
        maxBadContacts = 2
        badContacts = 0
        for contact in data.contactQuality:
            if contact < 3:
                badContacts = badContacts + 1

        self.enabled = badContacts <= maxBadContacts

    def expressiv_cb(self, data):
        pi = 3.1415926535

        if self.enabled:
            if data.lowerFaceActionPower > 0.5:
                if data.lowerFaceActionName == "Clench":
                    rospy.loginfo('Clenching')
                    self.hand_pub.publish(data=data.lowerFaceActionPower)
                elif data.lowerFaceActionName == "Smirk Left":
                    self.rotate = max(self.rotate - 0.1, - pi / 2);
                    self.rotate_pub.publish(data=self.rotate)
                    rospy.loginfo('Smirking Left: %.02f' % self.rotate)
                elif data.lowerFaceActionName == "Smirk Right":
                    self.rotate = min(self.rotate + 0.1, pi / 2);
                    self.rotate_pub.publish(data=self.rotate)
                    rospy.loginfo('Smirking Right: %.02f' % self.rotate)

            if data.upperFaceActionPower > 0.5:
                if data.upperFaceActionName == "Furrow":
                    rospy.loginfo('Furrowing')
                elif data.upperFaceActionName == "Eyebrow":
                    rospy.loginfo('Eyebrow')
                    self.hand_pub.publish(data=0.0)


if __name__ == "__main__":
    try:
        e = EmotivWristCtrl()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
