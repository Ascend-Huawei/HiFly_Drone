import sys
import numpy as np
sys.path.append("../../")

from ros_atlas.hifly_base.BasePostprocessor import Postprocessor


class DEPostNode(Postprocessor):
    def __init__(self) -> None:
        super().__init__()
    
    def deconstruct_ros_msg(self, msg):
        array_1 = np.reshape(np.array(msg.array1.list), (1, 1, 240, 320))
        return [array_1]

if __name__ == "__main__":
    de_postprocess_node = DEPostNode()
    de_processor = de_postprocess_node.load_processor(model_name="indoor_depth_estimation")
    de_postprocess_node.init()
    de_postprocess_node.run(de_processor)