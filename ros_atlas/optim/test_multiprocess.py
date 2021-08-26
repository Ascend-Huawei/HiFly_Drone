"""Playground to test multiprocess.Manager to share memory across Nodes"""
from multiprocessing import Value, Array, Process, Manager
import cv2
import numpy as np
import rospy
import sys

sys.path.append('..')
from core.CameraPublisher import CameraPublisher


# https://docs.python.org/3/library/multiprocessing.html
# Manager object is more flexible than SharedMemory -- but slower

"""Try -- still use ROS node but no publisher/subscriber (then ROS becomes pointless)
Compose multiple nodes in a single-process

# 

"""


def fetch_frame(img_queue):



if __name__ == '__main__':
    CamPub = CameraPublisher()
    print(CamPub)
    rospy.spin()
    with Manager() as manager:
        img_queue = manager.Queue(maxsize=1)
        output_queue = manager.Queue(maxsize=1)
        print(img_queue, output_queue)

        p = Process(target=fetch_frame, args=(img_queue))

