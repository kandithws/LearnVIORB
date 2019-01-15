#!/usr/bin/env python

import rospy
from image_dir_streamer import ImageDirStreamer
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

DIST_COEFF = np.array([-0.3838652947250899, 0.1089654751817628, 0.004132306170580261, -0.005166631758660408, 0])
K = np.array([ [334.731213795165, 0, 310.4825656641798],  [0, 375.8117645530208, 227.726470515695] ,  [0, 0, 1] ])

def main():
    rospy.init_node('image_dir_streamer_node')
    image_pub = rospy.Publisher('/camera0/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    img_dir = rospy.get_param('~img_dir')
    rate = rospy.get_param('~rate', 30)
    undistort = rospy.get_param('~undistort', False)
    assert(img_dir)
    with ImageDirStreamer(img_dir, rate=rate, queue_size=1) as streamer:
        while not streamer.is_shutdown() and not rospy.is_shutdown():
            img = streamer.get_image()
            if img is not None:
                try:
                    if undistort:
                        img = cv2.undistort(img, K, DIST_COEFF)
                    image_pub.publish(bridge.cv2_to_imgmsg(img, "rgb8"))
                except CvBridgeError as e:
                    print(e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass