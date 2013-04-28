#!/usr/bin/env python
import rospy
import json
import urllib2
from emotiv_node.msg import *

class Emotiv:
    def __init__(self):
        rospy.init_node('Emotiv')
        self.data_pub = rospy.Publisher('emotiv/data', Data)
        self.expressiv_pub = rospy.Publisher('emotiv/expressiv', Expressiv)

    def run(self):
        self.rate = rospy.Rate(10)

        prevtime = 0

        while not rospy.is_shutdown():
            try:
                response = urllib2.urlopen('http://localhost:8888')
                j = json.load(response)
                if prevtime != j['time']:
                    prevtime = j['time']

                    self.data_pub.publish(
                            timestamp=rospy.Time.now(),
                            headsetTime=j['time'],
                            battery=j['battery'],
                            wirelessSignal=j['wirelessSignal'],
                            contactQuality=j['contactQuality']
                            )

                    self.expressiv_pub.publish(
                            timestamp=rospy.Time.now(),
                            headsetTime=j['time'],
                            lowerFaceAction=j['expressiv']['lowerFaceAction'],
                            lowerFaceActionName=j['expressiv']['lowerFaceActionName'],
                            lowerFaceActionPower=j['expressiv']['lowerFaceActionPower'],
                            upperFaceAction=j['expressiv']['upperFaceAction'],
                            upperFaceActionName=j['expressiv']['upperFaceActionName'],
                            upperFaceActionPower=j['expressiv']['upperFaceActionPower'],
                            )

            except urllib2.URLError:
                rospy.logwarn('Could not communicate with emotiv server')

            self.rate.sleep()


if __name__ == "__main__":
    try:
        e = Emotiv()
        e.run()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
