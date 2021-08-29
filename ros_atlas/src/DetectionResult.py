"""DetectionResult
Monitors the Messages published from ImagePublisher and do inference

"""

import sys
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters

sys.path.append("../../src/lib")
sys.path.append("../../src/")

import numpy as np
import cv2

# from model_processors.ObjectDetectionProcessor import ModelProcessor
# from utils.params import params


class DetectionHandler:
    def __init__(self):
        self.img_topic = "/TelloDrone/image_raw"
        self.camera_info_topic = "/TelloDrone/camera_info"

    def image_callback(self, img_msg, cam_info_msg):
        rgb_img = CvBridge().imgmsg_to_cv2(img_msg, desired_enoding="rgb8")
        camera_info_K = np.array(cam_info_msg.K).reshape([3, 3])
        camera_info_D = np.array(cam_info_msg.D)
        rgb_undist = cv2.undistort(rgb_img, camera_info_K, camera_info_D)
        print(f"Obtained rbg_img of type: {type(rgb_undist)}")

    # Read more on message_filter here: http://wiki.ros.org/message_filters
    # message_filter.TimeSynchronizer: https://docs.ros.org/en/api/message_filters/html/python/#message_filters.TimeSynchronizer
    def start_listen(self):
        """Uses message_filter to synchronize Messages from different sources"""
        rospy.init_node("infer_img", anonymous=True)
        image_sub = message_filters.Subscriber(self.img_topic , Image)
        info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo)
        time_synchronizer = message_filters.TimeSyTimeSynchronizer(fs=[image_sub, info_sub], queue_size=10)
        time_synchronizer.registerCallback(self.image_callback)
        rospy.spin()


if __name__ == "__main__":
    detHandler = DetectionHandler()
    detHandler.start_listen()