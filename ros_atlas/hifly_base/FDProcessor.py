import sys
import numpy as np
sys.path.append("../../")

from ros_atlas.hifly_base.BasePostprocessor import Postprocessor

class FDPostNode(Postprocessor):
    def __init__(self) -> None:
        super().__init__()
    
    def deconstruct_ros_msg(self, msg):
        array_1 = np.reshape(np.array(msg.array1.list), (1, 13, 13, 18))
        array_2 = np.reshape(np.array(msg.array2.list), (1, 26, 26, 18))
        array_3 = np.reshape(np.array(msg.array3.list), (1, 52, 52, 18))
        return [array_1, array_2, array_3]

if __name__ == "__main__":
    fd_postprocess_node = FDPostNode()
    fd_processor = fd_postprocess_node.load_processor("face_detection")
    fd_postprocess_node.init()
    fd_postprocess_node.run(fd_processor)