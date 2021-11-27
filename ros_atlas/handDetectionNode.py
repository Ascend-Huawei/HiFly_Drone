import sys
import rospy
import cv2

sys.path.append("lib/")

from base_nodes.BaseInference import BaseInferenceNode
from custom_ros_msgs.msg import HandDetection
from cv_bridge import CvBridge, CvBridgeError

class HandDetectionNode(BaseInferenceNode):
    def __init__(self):
        super().__init__()

    def construct_ros_msg(self, model_output, img):
        try:
            detection_msg = HandDetection()
            detection_msg.header.stamp = rospy.Time.now()
            detection_msg.array1.list = model_output[0].flatten().tolist()
            detection_msg.array2.list = model_output[1].flatten().tolist()
            detection_msg.array3.list = model_output[2].flatten().tolist()
            detection_msg.array4.list = model_output[3].flatten().tolist()

            detection_msg.img = CvBridge().cv2_to_imgmsg(img)

            return detection_msg
        except CvBridgeError as err:
            raise err
  
if __name__ == "__main__":
    inference_node = HandDetectionNode()
    hd_model = inference_node.load_model("hand_detection")
    inference_node.init()
    inference_node.run(hd_model)