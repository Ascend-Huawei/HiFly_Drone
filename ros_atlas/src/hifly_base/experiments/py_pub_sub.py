import acl
import sys
import os
import cv2
import numpy as np
import time
import gc
from queue import Queue


sys.path.append("../../../../src")
sys.path.append("../../../../src/lib")

from model_processors.FaceDetectionProcessor import ModelProcessor
from utils.tools import load_model_processor

import rospy
import rosbag
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


def image_callback(img_data):
        """Subscriber callback function triggered upon receiving incoming data from CameraPublisher 
        @params:
            img_data    - imgmsg data from "uav_cam" node
            cam_info    - camera info from "uav_cam" node
        @type:
            img_data    - ROS:SensorMessage.Image
            cam_info    - ROS:SesnorMessage.CameraInfo
        """
        try:
            rgb_img = CvBridge().imgmsg_to_cv2(img_data)
            if not image_queue.full():
                image_queue.put(rgb_img)
        except CvBridgeError as cvb_err:
            raise cvb_err  


if __name__ == "__main__":
    face_detection_np, model_info = load_model_processor("face_detection")
    face_detector = face_detection_np(model_info)

    counter = 0
    image_queue = Queue(maxsize=1)  

    rospy.init_node('native_python', anonymous=True)
    start = time.process_time()
    cam_data_sub = rospy.Subscriber("/tello/cam_data_raw", Image, image_callback, queue_size=1, buff_size=2**24)
    pub = rospy.Publisher("/native_python/results", Image, queue_size=1)
    pub_rate = rospy.Rate(30)

    print(f"Process duration: {time.process_time() - start}")

    while not rospy.is_shutdown():
        if not image_queue.empty():
            image_data = image_queue.get()
            result_img = face_detector.predict(image_data)

            imgmsg = CvBridge().cv2_to_imgmsg(result_img, encoding="rgb8")
            imgmsg.header.stamp = rospy.Time.now()

            print(f"[{counter}]: Publish reuult to topic: /native_python/results. MessageType::{type(imgmsg)}")
            counter += 1
            pub.publish(imgmsg)
            pub_rate.sleep()

        else:
            continue