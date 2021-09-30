import acl
import sys
import cv2
import numpy as np
import rospy

sys.path.append("../../src")
from utils.tools import load_model_processor

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import message_filter

class AclInference:
    """Acl Inference ROS Node
    Node - maintain acl lifecycle (sub to data publisher, initalize acl resources, model loading & execution, pub inference results) 
    Subscribed to ImagePublisher and a Publisher

    +++ NOTE +++: consider format of inference results, need to pass in a format to publisher

    Arguments:
        model   - specify model name

    """
    def __init__(self, model_name):
        self._model_name = model_name
        self._node_name = 'acl_node'
        self._pub_topic = '/acl_node/inference'
        self._cam_data_topic = "/tello/cam_data_raw"
        self._cam_info_topic = "/tello/cam_info"

    def init_acl(self):
        print("ACL: Loading Model...")
        try:
            self.model_processor = load_model_processor(model_name)
        except Exception:
            return Exception

    def image_callback(self, img_data, cam_info):
        """Callback function triggered upon receiving incoming data - 
        
        Arguments:
            img_data    - imgmsg data from "uav_cam" node
            cam_info    - camera info from "uav_cam" node
        
        """
        rgb_img = CvBridge().imgmsg_to_cv2(img_msg, desired_enoding="rgb8")
        camera_info_K = np.array(cam_info_msg.K).reshape([3, 3])
        camera_info_D = np.array(cam_info_msg.D)
        rgb_undist = cv2.undistort(rgb_img, camera_info_K, camera_info_D)
        print(f"Obtained rbg_img of type: {type(rgb_undist)}")

        # run inferences and publish result
        result_img = self.model_processor.predict(rgb_undist)
        self._acl_inference_pub.publish(result_img)
        self._pub_rate.sleep()


    def init_subscriber(self):
        self._cam_data_sub = message_filters.Subscriber(self._cam_data_topic, Image,)
        self._cam_info_sub = message_filters.Subscriber(self._cam_info_topic, CameraInfo,)
        time_synchronizer = message_filters.TimeSyTimeSynchronizer(fs=[self._cam_data_sub, self._cam_info_sub], queue_size=10)
        time_synchronizer.registerCallback(self.image_callback)
        rospy.spin()
    
    def init_publisher(self, inference_rate, qsize):
        self._acl_inference_pub = rospy.Publisher(self._pub_topic, Image, queue_size=qsize)
        self._pub_rate = rospy.Rate(inference_rate)    

    def init(self):
        rospy.init_node('/acl_node/inference', anonymous=True)
        self.init_publisher(inference_rate)
        self.init_subscriber()

