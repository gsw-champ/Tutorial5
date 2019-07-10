#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('save_image')
import os
import sys

sys.path.append(os.path.dirname(__file__) + "/../")

import rospy
from scipy.misc import imsave
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

class image_converter():

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("xtion/rgb/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            i = 0
            # for i in range(1):
            image_name = '{}'.format(i) + '.png'
            imsave('image/' + image_name, cv_image)
            # i = i + 1
            print('###!!!')
            rospy.signal_shutdown('quit')

        except CvBridgeError as e:
            print(e)


def main():
    image_converter()
    rospy.init_node('save_image', anonymous=True, disable_signals='quit')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    


if __name__ == '__main__':
    main()
