import acl
import sys
import os
import cv2
import numpy as np
import time
import gc
from queue import Queue
import logging
import pickle

sys.path.append("../../../../src")
sys.path.append("../../../../src/lib")

from model_processors.FaceDetectionProcessor import ModelProcessor
from utils.tools import load_model_processor

import rospy
import rosbag
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from custom_ros_msgs.msg import FloatArray, FloatArrays, FaceDetection
from cv_bridge import CvBridge

image_queue = Queue(maxsize=1)

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
            timestamp = img_data.header.stamp
            rgb_img = CvBridge().imgmsg_to_cv2(img_data)
            if not image_queue.full():
                image_queue.put((rgb_img, timestamp))
        except CvBridgeError as cvb_err:
            raise cvb_err  


if __name__ == "__main__":
    face_detection_np, model_info = load_model_processor("face_detection")
    face_detector = face_detection_np(model_info)
    counter = 0

    rospy.init_node('native_python', anonymous=True)
    cam_data_sub = rospy.Subscriber("/tello/cam_data_raw", Image, image_callback, queue_size=1, buff_size=2**24)
    inf_res_pub = rospy.Publisher("/face_detection/inf_res", FaceDetection, queue_size=1)
    rate = rospy.Rate(30)

    # inf_res = None
    # with open("inf_res.pkl", "rb") as pkl_file:
    #     inf_res = pickle.load(pkl_file)

    # array1 = inf_res[0]
    # array2 = inf_res[1]    
    # array3 = inf_res[2]

    while not rospy.is_shutdown():
        if not image_queue.empty():
            image, timestamp = image_queue.get()
            
            preprocessed = face_detector.preprocess(image)
            inf_res = face_detector.model.execute([preprocessed])

            # Replace with FaceDetection type (publish both image and inf_res in the same message)
            face_detection_msg = FaceDetection()
            
            face_detection_msg.header.stamp = rospy.Time.now()

            face_detection_msg.array1.list = inf_res[0].flatten().tolist()
            face_detection_msg.array2.list = inf_res[1].flatten().tolist()
            face_detection_msg.array3.list = inf_res[2].flatten().tolist()

            face_detection_msg.img = CvBridge().cv2_to_imgmsg(image)

            print(f"[{counter}]: Publish inf_res to topic: /face_detection/inf_res")
            counter += 1
            inf_res_pub.publish(face_detection_msg)
            rate.sleep()

            #########################################################################################
            # inf_msg = FloatArrays()
            # inf_msg.header.stamp = rospy.Time.now()

            # inf_msg.array1.list = inf_res[0].flatten().tolist()
            # inf_msg.array2.list = inf_res[1].flatten().tolist()
            # inf_msg.array3.list = inf_res[2].flatten().tolist()
            
            # print(f"[{counter}]: Publish inf_res to topic: /native_python/inf_res.")
            # counter += 1
            # inf_res_pub.publish(inf_msg)
            # rate.sleep()

        else:
            rospy.loginfo("Image queue is empty. Check image_callback")
            continue