import cv2
from datetime import datetime
import sys
import time
import rospy

sys.path.append("lib/")

from base_nodes.BaseInference import BaseInferenceNode
from custom_ros_msgs.msg import FaceDetection
from rospy.exceptions import ROSException, ROSSerializationException, ROSInitException, ROSInterruptException
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
    
    def run(self, model):
        while not rospy.is_shutdown():
            st = time.time()
            try:
                if not self.image_queue.empty():
                    image = self.image_queue.get()
                    
                    preprocessed = model.preprocess(image)
                    model_output = model.model.execute([preprocessed])
                    print(f'1. Inference time: {time.time() - st}')

                    ros_inference_msg = self.construct_ros_msg(model_output, image)     # ~0.03s
                    self.inference_pub.publish(ros_inference_msg)
                    self.pub_counter += 1
                    print(f"[{self.pub_counter}]: Published model output(s) to topic: {self._inference_topic}")
                    print(f'2. Inference time: {time.time() - st}')


                    # filename = datetime.utcnow().strftime('%M%S%f')
                    # iteration_time = int(filename) - int(loop_start_time)
                    # rospy.loginfo(f'Time spent on 1 iteration of FDNode while-loop: {iteration_time}')
                    # self.iteration_times.append(iteration_time)

                    print(f'3. Inference time: {time.time() - st}')
                    self.inference_pub_rate.sleep()

                    print(f'4. Inference time: {time.time() - st}')
                    self._iteration_times.append(time.time() - st)

                else:
                    continue

            except CvBridgeError as err:
                    rospy.logerr("Ran into exception when converting message type with CvBridge. See error below:")
                    raise err
            except ROSSerializationException as err:
                rospy.logerr("Ran into exception when serializing message for publish. See error below:")
                raise err
            except ROSException as err:
                raise err
            except ROSInterruptException as err:
                rospy.loginfo("ROS Interrupt.")
                raise err

   
if __name__ == "__main__":
    import os
    print(f"FDProcessor pid: {os.getpid()}")
    fd_inference_node = FDInferNode()
    fd_model = fd_inference_node.load_model("face_detection")
    fd_inference_node.init()
    try:
        fd_inference_node.run(fd_model)
    except KeyboardInterrupt as e:
        rospy.signal_shutdown("Shutting down Inference. Keyboard terminate")