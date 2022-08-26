import os
import sys
import time
import rospy
import numpy as np

sys.path.append("lib/")

from core.BasePostprocessor import Postprocessor
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException, ROSSerializationException, ROSInterruptException
from custom_ros_msgs.msg import ProcessVar

class FDPostNode(Postprocessor):
    def __init__(self) -> None:
        super().__init__()
    
    def deconstruct_ros_msg(self, msg):
        array_1 = np.reshape(np.array(msg.array1.list), (1, 13, 13, 18))
        array_2 = np.reshape(np.array(msg.array2.list), (1, 26, 26, 18))
        array_3 = np.reshape(np.array(msg.array3.list), (1, 52, 52, 18))
        return [array_1, array_2, array_3]
    
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
    

    def run_pid(self, processor, img_format='passthrough'):
        while not rospy.is_shutdown():
            if not self.message_queue.empty():
                st = time.time()
                try:
                    message = self.message_queue.get()
                    model_output = self.deconstruct_ros_msg(message)
                    frame = CvBridge().imgmsg_to_cv2(message.img)

                    postprocessed, process_vars = processor.postprocess(frame=frame, outputs=model_output)
                    # Publish postprocess image for visualization
                    postprocessed = CvBridge().cv2_to_imgmsg(postprocessed, img_format)

                    # NEW -- send process_variables to topic, state machine subscribed to see value of process var
                    process_var_pub = rospy.Publisher("/pid_fd/process_vars", ProcessVar, queue_size=1)
                    process_var_msg = ProcessVar()
                    process_var_msg.area = process_vars[0]
                    process_var_msg.cx = process_vars[1]
                    process_var_msg.cy = process_vars[2]

                    self.postprocess_pub.publish(postprocessed)
                    process_var_pub.publish(process_var_msg)

                    rospy.loginfo(f"[{self.pub_counter}] Postprocessed and published to /pid_fd/process_vars.")
                    self.pub_counter += 1
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
                except KeyboardInterrupt as err:
                    rospy.loginfo("ROS Interrupt.")
                    raise err
            else:
                continue
        

if __name__ == "__main__":
    print(f"FDProcessor pid: {os.getpid()}")
    fd_postprocess_node = FDPostNode()
    # fd_processor = fd_postprocess_node.load_processor(model_name="face_detection", expected_image_shape=(360, 480))
    fd_processor = fd_postprocess_node.load_processor(model_name="face_detection", expected_image_shape=(720, 960))
    fd_postprocess_node.init()
    try:
        fd_postprocess_node.run_pid(fd_processor, img_format='rgb8')
    except KeyboardInterrupt as e:
        rospy.signal_shutdown("Shutting down Postprocessor. Keyboard terminate")
