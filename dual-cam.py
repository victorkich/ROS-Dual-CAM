#! /usr/bin/env python3

import rospy
import cv2
import threading
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.defisheye import Defisheye
import time


class camThread(threading.Thread):
    def __init__(self, camID=0):
        threading.Thread.__init__(self)
        self.camID = camID

    def run(self):
        print("Starting camera:", self.camID)
        camPreview(self.camID)


def camPreview(camID):
    pub_image = rospy.Publisher('/usb_cam/image_raw_{}'.format(camID), Image, queue_size=30)
    cam = cv2.VideoCapture(camID)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    cam.set(cv2.CAP_PROP_FPS, 30)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    if cam.isOpened():
        rval, frame = cam.read()
    else:
        rval = False

    while rval:
        start = time.time()
        rval, frame = cam.read()
        frame = defisheye.convert(frame)
        frame = bridge.cv2_to_imgmsg(frame)
        pub_image.publish(frame)
        fps = round(1 / (time.time() - start), 1)
        print('FPS:', fps)
        key = cv2.waitKey(1)
        if key == 27:  # exit on ESC
            break


rospy.init_node('camera_processing')
bridge = CvBridge()
defisheye = Defisheye(dtype='linear', format='fullframe', fov=100, pfov=90)
# Create two threads as follows
thread1 = camThread(camID=0)
thread2 = camThread(camID=2)
thread1.start()
thread2.start()
