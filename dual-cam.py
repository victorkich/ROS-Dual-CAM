#! /usr/bin/env python3

import rospy
import cv2
import threading
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class camThread(threading.Thread):
    def __init__(self, previewName, camID):
        threading.Thread.__init__(self)
        self.previewName = previewName
        self.camID = camID

    def run(self):
        print("Starting " + self.previewName)
        camPreview(self.previewName, self.camID)


def camPreview(previewName, camID):
    # cv2.namedWindow(previewName)
    pub_image = rospy.Publisher('/usb_cam/image_raw_{}'.format(camID), Image, queue_size=1)
    cam = cv2.VideoCapture(camID)
    if cam.isOpened():
        rval, frame = cam.read()
    else:
        rval = False

    while rval:
        # cv2.imshow(previewName, frame)
        rval, frame = cam.read()
        frame = bridge.cv2_to_imgmsg(frame)
        pub_image.publish(frame)
        #key = cv2.waitKey(20)
        #if key == 27:  # exit on ESC
        #    break
    #cv2.destroyWindow(previewName)


rospy.init_node('cameras')
bridge = CvBridge()
# Create two threads as follows
thread1 = camThread("Camera 1", 0)
thread2 = camThread("Camera 2", 2)
thread1.start()
thread2.start()
