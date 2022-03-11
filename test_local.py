#! /usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from utils.defisheye import Defisheye
import time

bridge = CvBridge()


def image_callback(msg):
    start = time.time()
    # np_arr = np.fromstring(ros_data.data, np.uint8)
    # image_np = cv2.imdecode(np_arr, 1)
    image = bridge.compressed_imgmsg_to_cv2(msg)
    print(image)
    cv2.imshow('Frame', image)
    fps = round(1 / (time.time() - start), 1)
    print('FPS:', fps)


rospy.init_node('test_local')
pub_image = rospy.Subscriber('/usb_cam/compressed/compressed_image', CompressedImage, image_callback)
rospy.Rate(60)
rospy.spin()
