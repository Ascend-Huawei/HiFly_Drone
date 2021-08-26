import sys

sys.path.append("lib/")

import rospy
from base_nodes.BaseInference import BaseInferenceNode
from custom_ros_msgs.msg import FaceDetection
from cv_bridge import CvBridge, CvBridgeError

class FDInferNode(BaseInferenceNode):
    def __init__(self):
        super().__init__()

    def construct_ros_msg(self, model_output, img):
        try:
            detection_msg = FaceDetection()
            detection_msg.header.stamp = rospy.Time.now()
            detection_msg.array1.list = model_output[0].flatten().tolist()
            detection_msg.array2.list = model_output[1].flatten().tolist()
            detection_msg.array3.list = model_output[2].flatten().tolist()
            detection_msg.img = CvBridge().cv2_to_imgmsg(img)
            return detection_msg
        except CvBridgeError as err:
            raise err
  
if __name__ == "__main__":
    import os
    print(f"FDNode pid: {os.getpid()}")
    fd_inference_node = FDInferNode()
    fd_model = fd_inference_node.load_model("face_detection")
    fd_inference_node.init()
    fd_inference_node.run(fd_model)