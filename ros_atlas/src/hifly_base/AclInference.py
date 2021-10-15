import acl
import sys
import cv2
import numpy as np
import rospy
sys.path.append("../../../src")
sys.path.append("../../../src/lib")

from utils.tools import load_model_processor
from utils.params import params

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class AclInferenceNode:
    """AclInference Node
    Listens and make inference on incoming sensor data (images) from TelloUAV and publish result to ResultNode 
    
    Custom Message
    """
    def __init__(self, model_name, inference_rate=10, qsize=10):
        self._model_processor = self._load_mp(model_name)
        self._acl_inference_topic = "/acl_inference/results"
        self._cam_data_topic = "/tello/cam_data_raw"
        self._cam_info_topic = "/tello/cam_info"
        self.init(inference_rate, qsize)

    @staticmethod
    def _load_mp(detector_name):
        """Internal method for children class to load specific ModelProcessor
        :param:
            + detector_name - Key name of detection model
        Returns
            A fully initialized ModelProcessor object
        """
        model_processor, mp_info = load_model_processor(detector_name)
        return model_processor(mp_info)


    def image_callback(self, img_data, cam_info_msg):
        """Callback function triggered upon receiving incoming data - 
        
        Arguments:
            img_data    - imgmsg data from "uav_cam" node
            cam_info    - camera info from "uav_cam" node
        """
        rgb_img = CvBridge().imgmsg_to_cv2(img_data)
        camera_info_K = np.array(cam_info_msg.K).reshape([3, 3])
        camera_info_D = np.array(cam_info_msg.D)
        rgb_undist = cv2.undistort(rgb_img, camera_info_K, camera_info_D)
        print(f"Obtained rbg_img of type: {type(rgb_undist)} with shape: {rgb_undist.shape}")

        # [TO REMOVE] save the rgb_undist image and view whether it is correct
        # cv2.imwrite('tmp_subimage.png', rgb_undist)

        # Pass image from topic to inference and then publish result
        result_img = self._model_processor.predict(rgb_undist)
        cv2.imwrite('inferenced.png', result_img)

        self._acl_inference_pub.publish(result_img)
        self._pub_rate.sleep()


    def init_subscriber(self):
        self._cam_data_sub = message_filters.Subscriber(self._cam_data_topic, Image,)
        self._cam_info_sub = message_filters.Subscriber(self._cam_info_topic, CameraInfo,)
        time_synchronizer = message_filters.TimeSynchronizer(fs=[self._cam_data_sub, self._cam_info_sub], queue_size=10)
        time_synchronizer.registerCallback(self.image_callback)
        rospy.spin()
    
    def init_publisher(self, inference_rate, qsize):
        self._acl_inference_pub = rospy.Publisher(self._acl_inference_topic, Image, queue_size=qsize)
        self._pub_rate = rospy.Rate(inference_rate)    

    def init(self, inference_rate, qsize):
        rospy.init_node('acl_inference_node', anonymous=True)
        self.init_publisher(inference_rate, qsize)
        self.init_subscriber()


if __name__ == "__main__":
    # model = input("Select supported model from HiFly (i.e. face_detection): ")
    acl_inferencer = AclInferenceNode(model_name="yolov3")