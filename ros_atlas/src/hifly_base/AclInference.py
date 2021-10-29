import sys
import cv2
import numpy as np
import threading
from queue import Queue

sys.path.append("../../../src")
sys.path.append("../../../src/lib")

import acl
from utils.tools import load_model_processor
from utils.params import params
from atlas_utils.acl_resource import AclResource
from atlas_utils.acl_model import Model

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class AclInferenceNode:
    """AclInference Node
    Listens and makes inference on incoming sensor data (images) from TelloUAV object and publish results to /acl_inference/results topic
    
    @param:inference_rate
    @param:qsize            
    """
    def __init__(self, model_name, inference_rate=3, qsize=5):
        self._inference_rate = inference_rate
        self._qsize = qsize
        self.model_processor = self._load_mp(model_name)

        self._acl_inference_topic = "/acl_inference/results"
        self._cam_data_topic = "/tello/cam_data_raw"
        self._cam_info_topic = "/tello/cam_info"

        self.image_queue = Queue(maxsize=qsize)

    @staticmethod
    def _load_mp(detector_name):
        """Internal method for children class to load specific ModelProcessor
        :param:
            + detector_name - Key name of detection model
        Returns
            A fully initialized ModelProcessor object
        """
        model_processor, mp_info = load_model_processor(detector_name)

        mp = model_processor(mp_info)
        if detector_name == "yolov3":
            mp.tmp_save_fpath = "./tmp/tmp.jpg"
        
        return mp

    @property
    def inference_rate(self):
        return self._inference_rate

    @inference_rate.setter
    def inference_rate(self, new_inference_rate):
        if new_inference_rate < 1:
            raise ValueError("Inference rate cannot be less than 1.")
        else:
            self._inference_rate = new_inference_rate

    @property
    def qsize(self):
        return self._qsize

    @qsize.setter
    def qsize(self, new_qsize):
        if nwe_qsize < 1:
            raise ValueError("Queue size cannot be less than 1.")
        else:
            self._qsize = new_qsize
            self.image_queue = Queue(maxsize=self._qsize)

    def image_callback(self, img_data, cam_info):
        """Subscriber callback function triggered upon receiving incoming data from CameraPublisher 
        @params:
            img_data    - imgmsg data from "uav_cam" node
            cam_info    - camera info from "uav_cam" node
        """
        try:
            rgb_img = CvBridge().imgmsg_to_cv2(img_data)
            camera_info_K = np.array(cam_info.K).reshape([3, 3])
            camera_info_D = np.array(cam_info.D)
            rgb_undist = cv2.undistort(rgb_img, camera_info_K, camera_info_D)
            
            if not self.image_queue.full():
                self.image_queue.put(rgb_img)

        except CvBridgeError as cvb_err:
            raise cvb_err   

    def init_node(self):
        rospy.init_node('acl_inference_node', anonymous=True)
        rospy.loginfo("ACLInference Node initializing...")

        cam_data_sub = message_filters.Subscriber(self._cam_data_topic, Image)
        cam_info_sub = message_filters.Subscriber(self._cam_info_topic, CameraInfo)
        
        time_synchronizer = message_filters.TimeSynchronizer(fs=[cam_data_sub, cam_info_sub], queue_size=30)
        time_synchronizer.registerCallback(self.image_callback)

        pub = rospy.Publisher("/acl_inference/results", Image, queue_size=1)
        pub_rate = rospy.Rate(self.inference_rate)
        return pub, pub_rate, time_synchronizer


if __name__ == "__main__":
    acl_inference_node = AclInferenceNode(model_name="yolov3")
    pub, pub_rate, ts = acl_inference_node.init_node()

    while not rospy.is_shutdown():
        try: 
            if not acl_inference_node.image_queue.empty():
                image_data = acl_inference_node.image_queue.get()
                result_img = acl_inference_node.model_processor.predict(image_data)
                imgmsg = CvBridge().cv2_to_imgmsg(result_img)
                pub.publish(imgmsg)
                print("Published reuult to topic: /acl_inference/results")
                pub_rate.sleep()
            else:
                continue

        except KeyboardInterrupt as exception:
            raise exception
