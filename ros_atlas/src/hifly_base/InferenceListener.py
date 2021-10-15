import acl
import sys
import cv2
import numpy as np
import rospy

sys.path.append("../../../src")

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import message_filter

class ResultNode:
    """Subscriber to AclInference node
    A subscriber node that listens to any inference results boardcasted to the /acl_node/inference topic
    """
    def __init__(self, ):
        rospy.init("acl_listener", anonymous=True)
        self.acl_sub = rospy.Subscriber('/acl_node/inference', Image, self.inference_callback)
        rospy.spin()

    def inference_callback(self):
        rgb_img = CvBridge().imgmsg_to_cv2(img_msg, desired_enoding="rgb8")
        rgb_undist = cv2.undistort(rgb_img, camera_info_K, camera_info_D)
        print(f"Obtained rbg_img of type: {type(rgb_undist)}")
        cv2.imwrite("tmp_inference", rgb_undist)
    
