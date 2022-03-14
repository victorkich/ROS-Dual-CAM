#! /usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import time


rospy.init_node('camera_processing')
bridge = CvBridge()
pub_image = rospy.Publisher('/usb_cam/compressed/image_left', CompressedImage, tcp_nodelay=True, queue_size=1)
pub_image2 = rospy.Publisher('/usb_cam/compressed/image_right', CompressedImage, tcp_nodelay=True, queue_size=1)
cam1 = cv2.VideoCapture(0, cv2.CAP_V4L)
cam1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cam1.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
cam1.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
cam1.set(cv2.CAP_PROP_FPS, 30)
cam1.set(cv2.CAP_PROP_BUFFERSIZE, 2)
cam2 = cv2.VideoCapture(3, cv2.CAP_V4L)
cam2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cam2.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
cam2.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
cam2.set(cv2.CAP_PROP_FPS, 30)
cam2.set(cv2.CAP_PROP_BUFFERSIZE, 2)

while not rospy.is_shutdown():
    start = time.time()
    ret1, frame1 = cam1.read()
    frame1 = bridge.cv2_to_compressed_imgmsg(frame1)
    pub_image.publish(frame1)
    ret2, frame2 = cam2.read()
    frame2 = bridge.cv2_to_compressed_imgmsg(frame2)
    pub_image2.publish(frame2)
    time.sleep(0.05)
    fps = round(1 / (time.time() - start), 1)
    print('FPS:', fps)
