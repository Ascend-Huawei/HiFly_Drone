import sys
import numpy as np

sys.path.append("lib/")

from base_nodes.BasePostprocessor import Postprocessor

class HandPostprocessNode(Postprocessor):
    def __init__(self):
        super().__init__()
    
    def deconstruct_ros_msg(self, msg):
        array_1 = np.array(msg.array1.list)
        array_2 = np.reshape(np.array(msg.array2.list), (1, 10))
        array_3 = np.reshape(np.array(msg.array3.list), (1, 10))
        array_4 = np.reshape(np.array(msg.array4.list), (1, 10, 4))
        
        return [array_1, array_2, array_3, array_4]

if __name__ == "__main__":
    hand_postprocess_node = HandPostprocessNode()
    hd_processor = hand_postprocess_node.load_processor(model_name="hand_detection")
    hand_postprocess_node.init()
    hand_postprocess_node.run(hd_processor, "rgb8")