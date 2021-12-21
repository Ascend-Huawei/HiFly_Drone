import sys
import time
import rospy
import numpy as np

sys.path.append("lib/")

from core.BasePostprocessor import Postprocessor
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException, ROSSerializationException, ROSInterruptException

class DEPostNode(Postprocessor):
    def __init__(self) -> None:
        super().__init__()
    
    def deconstruct_ros_msg(self, msg):
        array_1 = np.reshape(np.array(msg.array1.list), (1, 1, 240, 320))
        return [array_1]

    def run(self, processor, img_format):
        while not rospy.is_shutdown():
            st = time.time()
            if not self.message_queue.empty():
                try:
                    message = self.message_queue.get()
                    model_output = self.deconstruct_ros_msg(message)
                    # frame = CvBridge().imgmsg_to_cv2(message.img)

                    postprocessed = processor.postprocess(outputs=model_output)  

                    # Publish postprocess image for visualization
                    postprocessed = CvBridge().cv2_to_imgmsg(postprocessed, img_format)
                    self.postprocess_pub.publish(postprocessed)
                    self.pub_counter += 1
                    rospy.loginfo(f"[{self.pub_counter}]: Published postprocess output to {self._postprocess_topic}")

                    self.postprocess_pub_rate.sleep()                                           
                    self._iteration_times.append(time.time() - st)                                
                    
                except CvBridgeError as err:
                    rospy.logerr("Ran into exception when converting Image type with CvBridge.")
                    raise err
                except ROSSerializationException as err:
                    rospy.logerr("Ran into exception when serializing message for publish. See error below:")
                    raise err
                except ROSException as err:
                    raise err
                except ROSInterruptException as err:
                    rospy.loginfo("ROS Interrupt.")
                    raise err
            else:
                continue

if __name__ == "__main__":
    de_postprocess_node = DEPostNode()
    de_processor = de_postprocess_node.load_processor(model_name="indoor_depth_estimation")
    de_postprocess_node.init()
    try:
        de_postprocess_node.run(de_processor, 'passthrough')
    except KeyboardInterrupt as e:
        rospy.signal_shutdown("Shutting down Postprocessor. Keyboard terminate")