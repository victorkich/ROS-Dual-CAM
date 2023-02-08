#! /usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from utils.defisheye import Defisheye
from utils import range_finder as rf
import numpy as np
import time
import os
import yaml


class TestLocal:
    def __init__(self):
        self.bridge = CvBridge()
        # self.stitcher = Stitcher()
        # self.stitcher = cv2.Stitcher_create()
        self.image_right = None
        # self.image_left = None
        # self.defisheye1 = Defisheye(dtype='linear', format='fullframe', fov=140, pfov=110)  # 140 110
        self.defisheye2 = Defisheye(dtype='linear', format='fullframe', fov=155, pfov=110)  # 180 80
        rospy.Subscriber('/camera_2/image_raw/compressed', CompressedImage,  self.image_right_callback, tcp_nodelay=True, queue_size=1, buff_size=2**26)
        # rospy.Subscriber('/camera_left/image_raw/compressed', CompressedImage, self.image_left_callback, tcp_nodelay=True, queue_size=1, buff_size=2**26)

    def image_right_callback(self, msg):
        # self.image_right = self.defisheye1.convert(self.bridge.compressed_imgmsg_to_cv2(msg))
        self.image_right = self.bridge.compressed_imgmsg_to_cv2(msg)

    # def image_left_callback(self, msg):
    #    # self.image_left = self.defisheye2.convert(self.bridge.compressed_imgmsg_to_cv2(msg))
    #    self.image_left = self.bridge.compressed_imgmsg_to_cv2(msg)

    def step(self):
        if self.image_right is None:
            return False, None
        # defish_left = self.defisheye1.convert(self.image_left)
        # defish_right = self.defisheye2.convert(self.image_right)
        # frame = self.stitcher.stitch([defish_left, defish_right])
        # (status, frame) = self.stitcher.stitch([defish_left, defish_right])
        # frame = [self.defisheye1.convert(self.image_left), self.defisheye2.convert(self.image_right)]
        # size = self.image_right.shape
        # frame = self.image_right
        # frame = frame[round(size[0]*0.1):round(size[0]*0.9), round(size[1]*0.28):round(size[1]*0.68)]
        # frame = frame[round(size[0]*0.1):round(size[0]*0.8), round(size[1]*0.1):round(size[1]*0.6)]
        frame = self.defisheye2.convert(self.image_right)
        lidar = np.ones(24)
        angle, distance, frame = real_ttb.get_angle_distance(frame, lidar, green_magnitude=1.0)
        print('Angle:', angle, 'Distance:', distance)
        return True, frame


# Loading configs from config.yaml
path = os.path.dirname(os.path.abspath(__file__))
with open(path + '/config.yml', 'r') as ymlfile:
    config = yaml.load(ymlfile, Loader=yaml.FullLoader)

# Recording settings
RECORD = False
if RECORD:
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (720, 720))

real_ttb = rf.RealTtb(config, path, output=(720, 720))
rospy.init_node('test_local')
test_local = TestLocal()
key = cv2.waitKey(1)
while key != ord('q'):
    start = time.time()
    val, frame = test_local.step()
    if not val:
        continue
    if RECORD:
        out.write(frame)

    cv2.imshow('Frame', frame)
    key = cv2.waitKey(1)
    time.sleep(1/70)
    fps = round(1 / (time.time() - start), 1)
    print('\rFPS:', fps)

print("[INFO] cleaning up...")
if RECORD:
    out.release()
cv2.destroyAllWindows()
