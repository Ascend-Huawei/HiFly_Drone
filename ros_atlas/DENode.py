import sys
import rospy

sys.path.append("lib/")

import rospy
from base_nodes.BaseInference import BaseInferenceNode
from custom_ros_msgs.msg import AdaBins
from cv_bridge import CvBridge, CvBridgeError

class DepthEstNode(BaseInferenceNode):
    def __init__(self):
        super().__init__()

    def construct_ros_msg(self, model_output, img):
        try:
            detection_msg = AdaBins()
            detection_msg.header.stamp = rospy.Time.now()
            detection_msg.array1.list = model_output[0].flatten().tolist()
            detection_msg.img = CvBridge().cv2_to_imgmsg(img)
            return detection_msg
        except CvBridgeError as err:
            raise err

if __name__ == "__main__":
    de_inference_node = DepthEstNode()
    de_model = de_inference_node.load_model("indoor_depth_estimation")
    de_inference_node.init()
    de_inference_node.run(de_model)
