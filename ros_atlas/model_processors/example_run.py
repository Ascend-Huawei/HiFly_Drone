import acl
import sys
import os
import cv2
import time
from queue import Queue


sys.path.append("..")
sys.path.append("../lib")

from model_processors.FaceDetectionProcessor import ModelProcessor
from utils.tools import load_model_processor

import rospy
import rosbag
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge

# image_queue = Queue(maxsize=1)

# def image_callback(img_data):
#         """Subscriber callback function triggered upon receiving incoming data from CameraPublisher 
#         @params:
#             img_data    - imgmsg data from "uav_cam" node
#             cam_info    - camera info from "uav_cam" node
#         @type:
#             img_data    - ROS:SensorMessage.Image
#             cam_info    - ROS:SesnorMessage.CameraInfo
#         """
#         try:
#             rgb_img = CvBridge().imgmsg_to_cv2(img_data)
#             # camera_info_K = np.array(cam_info.K).reshape([3, 3])
#             # camera_info_D = np.array(cam_info.D)
#             # rgb_undist = cv2.undistort(rgb_img, camera_info_K, camera_info_D)
            
#             if not image_queue.full():
#                 image_queue.put(rgb_img)

#         except CvBridgeError as cvb_err:
#             raise cvb_err  


if __name__ == "__main__":

    # Experiment: read from bag
    bag = rosbag.Bag("/home/HwHiAiUser/HiFly_Drone/ros_atlas/src/hifly_base/sample.bag")

    face_detection_np, model_info = load_model_processor("face_detection")
    face_detector = face_detection_np(model_info)

    # 1. Add in ROS Publisher node in Native-Python to see if this makes any difference
    rospy.init_node('native_python', anonymous=True)
    # cam_data_sub = rospy.Subscriber("/tello/cam_data_raw", Image, image_callback, queue_size=1, buff_size=2**24)
    pub = rospy.Publisher("/native_python/results", Image, queue_size=1)
    pub_rate = rospy.Rate(30)

    # 2. Keep Subscriber running but load static video w cv2 for inference
    # cap = cv2.VideoCapture("/home/HwHiAiUser/HiFly_Drone/data/20210809_141017.mp4")
    # fps = cap.get(cv2.CAP_PROP_FPS)
    # print(f"CameraPublisher - Default video read FPS={round(fps, 3)}")

    # if not cap.isOpened():
    #     print("Error opening video file")
    
    counter = 0
    # while cap.isOpened():
    #     ret, frame = cap.read()

    #     inf_ret = face_detector.predict(frame)

    #     imgmsg = CvBridge().cv2_to_imgmsg(inf_ret, encoding="rgb8")
    #     imgmsg.header.stamp = rospy.Time.now()
    #     print(f"[{counter}]: Publish reuult to topic: /native_python/results. MessageType::{type(imgmsg)}")
    #     pub.publish(imgmsg)

        # counter += 1 


    while not rospy.is_shutdown():
        try:
            # Global Queue code block 
            # if not image_queue.empty():
            #     image_data = image_queue.get()
            #     result_img = face_detector.predict(image_data)

            #     imgmsg = CvBridge().cv2_to_imgmsg(result_img, encoding="rgb8")
            #     imgmsg.header.stamp = rospy.Time.now()

            #     print(f"[{counter}]: Publish reuult to topic: /native_python/results. MessageType::{type(imgmsg)}")
            #     counter += 1
            #     pub.publish(imgmsg)
            #     pub_rate.sleep()

            for topic, msg, t in bag.read_messages(topic=["/tello/cam_data_raw"]):
                print(msg)

            else:
                continue

        except KeyboardInterrupt as exception:
            pass

        except rospy.ROSInterruptException:
            pass
    bag.close()