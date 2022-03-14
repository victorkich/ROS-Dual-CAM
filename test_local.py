#! /usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from utils.defisheye import Defisheye
import imutils
import time


class TestLocal:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_right = None
        self.image_left = None
        self.defisheye1 = Defisheye(dtype='linear', format='fullframe', fov=180, pfov=120)
        self.defisheye2 = Defisheye(dtype='linear', format='fullframe', fov=180, pfov=120)
        rospy.Subscriber('/usb_cam/compressed/image_right', CompressedImage, self.image_right_callback)
        rospy.Subscriber('/usb_cam/compressed/image_left', CompressedImage, self.image_left_callback)

    def image_right_callback(self, msg):
        self.image_right = self.defisheye1.convert(self.bridge.compressed_imgmsg_to_cv2(msg))

    def image_left_callback(self, msg):
        self.image_left = self.defisheye2.convert(self.bridge.compressed_imgmsg_to_cv2(msg))

    def step(self):
        if self.image_right is None or self.image_left is None:
            return False
        images = [self.image_left, self.image_right]
        stitcher = cv2.createStitcher() if imutils.is_cv3() else cv2.Stitcher_create()
        test = stitcher.stitch(images)
        print('Test:', test)
        #print('Status:', status)
        #print('Frame:', frame)
        # frame = cv2.hconcat([self.image_left, self.image_right])
        #cv2.imshow('Frame', frame)
        return True


rospy.init_node('test_local')
test_local = TestLocal()
key = cv2.waitKey(1)
while key != 'q':
    start = time.time()
    val = test_local.step()
    if not val:
        continue
    key = cv2.waitKey(1)
    fps = round(1 / (time.time() - start), 1)
    print('\rFPS:', fps)
    time.sleep(1/60)
