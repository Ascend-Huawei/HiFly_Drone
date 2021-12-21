import sys
import time
import numpy as np
import rospy

sys.path.append("lib/")

from core.BasePostprocessor import Postprocessor
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException, ROSSerializationException, ROSInterruptException

class HandPostprocessNode(Postprocessor):
    def __init__(self):
        super().__init__()
    
    def deconstruct_ros_msg(self, msg):
        array_1 = np.array(msg.array1.list)
        array_2 = np.reshape(np.array(msg.array2.list), (1, 10))
        array_3 = np.reshape(np.array(msg.array3.list), (1, 10))
        array_4 = np.reshape(np.array(msg.array4.list), (1, 10, 4))
        return [array_1, array_2, array_3, array_4]
    
    def run(self, processor, img_format):
        while not rospy.is_shutdown():
            st = time.time()
            if not self.message_queue.empty():
                try:
                    message = self.message_queue.get()
                    model_output = self.deconstruct_ros_msg(message)
                    frame = CvBridge().imgmsg_to_cv2(message.img)

                    postprocessed = processor.postprocess(frame=frame, outputs=model_output)  

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
    hand_postprocess_node = HandPostprocessNode()
    hd_processor = hand_postprocess_node.load_processor(model_name="hand_detection")
    hand_postprocess_node.init()
    try:
        hand_postprocess_node.run(hd_processor, 'rgb8')
    except KeyboardInterrupt as e:
        rospy.signal_shutdown("Shutting down Postprocessor. Keyboard terminate")