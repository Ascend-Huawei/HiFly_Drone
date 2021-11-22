import acl
import sys
import os
import cv2
import numpy as np
import time
import gc
from queue import Queue


sys.path.append("../../../src")
sys.path.append("../../../src/lib")

from model_processors.FaceDetectionProcessor import ModelProcessor
from utils.tools import load_model_processor

import rospy
import rosbag
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge


if __name__ == "__main__":
    face_detection_np, model_info = load_model_processor("face_detection")
    face_detector = face_detection_np(model_info)
    counter = 0

    rospy.init_node('native_python', anonymous=True)
    pub = rospy.Publisher("/native_python/results", Image, queue_size=1)
    pub_rate = rospy.Rate(30)

    if not cap.isOpened(): print("Error opening video file")

    while cap.isOpened():
        ret, frame = cap.read()

        inf_ret = face_detector.predict(frame)

        imgmsg = CvBridge().cv2_to_imgmsg(inf_ret, encoding="rgb8")
        imgmsg.header.stamp = rospy.Time.now()
        print(f"[{counter}]: Publish reuult to topic: /native_python/results. MessageType::{type(imgmsg)}")
        pub.publish(imgmsg)

        counter += 1 
    
    else:
        print("Video End. Exiting process.")
        sys.exit()

