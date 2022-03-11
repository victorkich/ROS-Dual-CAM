#! /usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from utils.defisheye import Defisheye
import threading
import time


class TestLocal(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.bridge = CvBridge()
        self.image_right = None
        self.image_left = None
        # rospy.Subscriber('/usb_cam/compressed/image_right', CompressedImage, self.image_right_callback)
        # rospy.Subscriber('/usb_cam/compressed/image_left', CompressedImage, self.image_left_callback)

    def run(self):
        while not finish:
            _ = rospy.Subscriber('/usb_cam/compressed/image_right', CompressedImage, self.image_right_callback)
            _ = rospy.Subscriber('/usb_cam/compressed/image_left', CompressedImage, self.image_left_callback)

    def image_right_callback(self, msg):
        self.image_right = self.bridge.compressed_imgmsg_to_cv2(msg)

    def image_left_callback(self, msg):
        self.image_left = self.bridge.compressed_imgmsg_to_cv2(msg)

    def step(self):
        if self.image_right is None or self.image_left is None:
            return
        start = time.time()
        frame = cv2.hconcat([self.image_left, self.image_right])
        cv2.imshow('Frame', frame)
        fps = round(1 / (time.time() - start), 1)
        print('FPS:', fps)


rospy.init_node('test_local')
finish = False
test_local = TestLocal()
test_local.daemon = True
test_local.start()
key = cv2.waitKey(1)
while key != 'q':
    test_local.step()
    key = cv2.waitKey(1)
    rospy.Rate(60)
    rospy.spin()
finish = True
