import sys
import cv2
import numpy as np
import threading
from queue import Queue
import gc
import time

sys.path.append("../../../src")
sys.path.append("../../../src/lib")

import acl
from utils.tools import load_model_processor
from utils.params import params
# from atlas_utils.acl_resource import AclResource
# from atlas_utils.acl_model import Model

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge


class AclInferenceNode:

    def __init__(self, model_name, inference_rate=30, sub_qsize=1):
        """AclInference Node - Listens and makes inference on incoming sensor data (images) from TelloUAV object and publish results to /acl_inference/results topic
        @param:
            model_name      - Name of the supported model (refer to params.py for keynames).  @type:String
            inference_rate  - Inference rate of model_name.                                   @type:Int
            qsize           - Size of global queue for thread-wise reference.                 @type:Int 
        Returns:
            AclInferenceNode instance.
        """
        self._inference_rate = inference_rate
        self._sub_qsize = sub_qsize

        self.model_processor = self._load_mp(model_name)

        self._acl_inference_topic = "/acl_inference/results"
        self._cam_data_topic = "/tello/cam_data_raw"
        self._cam_info_topic = "/tello/cam_info"
        self.image_queue = Queue(maxsize=1)
    
        self.history = dict()

        rospy.on_shutdown(self.shutdown)

    @staticmethod
    def _load_mp(detector_name):
        """Internal method for children class to load specific ModelProcessor
        @param:
            detector_name - Key name of detection model (refer to keys inside params.py)    @type:String 
        Returns
            A fully initialized ModelProcessor object.                                      @type:ModelProcessor
        """
        model_processor, mp_info = load_model_processor(detector_name)

        mp = model_processor(mp_info)
        if detector_name == "yolov3":
            mp.tmp_save_fpath = "./tmp/tmp.jpg"
        return mp

    def image_callback(self, img_data):
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
            # camera_info_K = np.array(cam_info.K).reshape([3, 3])
            # camera_info_D = np.array(cam_info.D)
            # rgb_undist = cv2.undistort(rgb_img, camera_info_K, camera_info_D)
            
            if not self.image_queue.full():
                self.image_queue.put(rgb_img)

        except CvBridgeError as cvb_err:
            raise cvb_err   

    def init_node(self):
        """Node initialization. Set Subscriber(s) and Publisher."""
        rospy.init_node('acl_inference_node', anonymous=True)
        rospy.loginfo("ACLInference Node initializing...")

        self._node_start = time.time()

        cam_data_sub = rospy.Subscriber(self._cam_data_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        # cam_data_sub = message_filters.Subscriber(self._cam_data_topic, Image)
        # cam_info_sub = message_filters.Subscriber(self._cam_info_topic, CameraInfo)
        # time_synchronizer = message_filters.TimeSynchronizer(fs=[cam_data_sub, cam_info_sub], queue_size=1)
        # time_synchronizer.registerCallback(self.image_callback)

        pub = rospy.Publisher("/acl_inference/results", Image, queue_size=1)
        pub_rate = rospy.Rate(self._inference_rate)
        return pub, pub_rate

    def shutdown(self):
        """Shutdown hook"""
        rospy.loginfo("AclInference node shutdown...")
        rospy.loginfo(f"Release resources...")
        
        # program_duration = time.time() - self._node_start
        # average_loop_runtime = self.history["total_loop_time"] / self.counter
        # average_inference_time = self.history["total_inference_time"] / self.counter

        # rospy.loginfo(f"[Including rate.sleep] Published {self.counter} frames in {program_duration}s -> {self.counter / program_duration} FPS.")
        # rospy.loginfo(f"[Excluding rate.sleep] Average time (s) to complete one loop: {average_loop_runtime}")
        # rospy.loginfo(f"[Excluding rate.sleep] Average time (s) to make inference: {average_inference_time}")

        gc.collect()


if __name__ == "__main__":
    acl_inference_node = AclInferenceNode(model_name="face_detection")
    pub, pub_rate = acl_inference_node.init_node()

    acl_inference_node.counter = 0
    acl_inference_node.loop_times = 0

    acl_inference_node.history["total_loop_time"] = 0
    acl_inference_node.history["total_inference_time"] = 0

    while not rospy.is_shutdown():
        try: 
            if not acl_inference_node.image_queue.empty():
                # loop_start = time.process_time()

                image_data = acl_inference_node.image_queue.get()

                inference_start = time.process_time()
                result_img = acl_inference_node.model_processor.predict(image_data)
                acl_inference_node.history["total_inference_time"] += time.process_time() - inference_start 

                imgmsg = CvBridge().cv2_to_imgmsg(result_img, encoding="rgb8")

                # cv_start = time.process_time()
                # imgmsg = CvBridge().cv2_to_imgmsg(image_data, encoding="rgb8")
                # acl_inference_node.history["total_cvbridge"] += time.process_time() - cv_start

                imgmsg.header.stamp = rospy.Time.now()

                print(f"[{acl_inference_node.counter}]: Publish reuult to topic: /acl_inference/results. MessageType::{type(imgmsg)}")
                acl_inference_node.counter += 1
                pub.publish(imgmsg)
                
                # loop_end = time.process_time()
                # loop_duration = loop_end - loop_start
                # acl_inference_node.history["total_loop_time"] += loop_duration
                
                pub_rate.sleep()
            else:
                continue

        except KeyboardInterrupt as exception:
            raise exception
