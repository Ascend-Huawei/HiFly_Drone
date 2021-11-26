"""
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

from abc import abstractmethod
from queue import Queue
import sys
import numpy as np

sys.path.append("../../")

from ros_atlas.utils.tools import load_model_processor

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException, ROSSerializationException, ROSInitException, ROSInterruptException


class Postprocessor:
    """Postprocessor - Base postprocessing class with abstract method for formating model-dependent message types.
    Functions
        + Handles ROS nodes related operations: Loading corresponding processor, ROS node initialization, message type formatting, faciliitate node communications, shutdown protocol
        + Subscribes to @topic:/acl_inference/<model_name>
        + Loads corresponding Processing module from <model_name>Processor -- Inherits the <model_name>Processor class but does NOT use AclResource to spawn Model.
    
    @params
        expected_img_shape      expected image shape of incoming frame. Default is None (uses tello camera dimensions)  @type: Bool
    
    Returns
        None
    """
    def __init__(self, expected_img_shape=None):
        self.message_queue = Queue(maxsize=5)
        self.expected_img_shape = expected_img_shape
        rospy.on_shutdown(self.shutdown)
    
    def load_processor(self, model_name):
        mp, model_info = load_model_processor(model_name)
        self._model_info = model_info
        self._model_name = model_name
        return mp(params=model_info, process_only=True)
    
    def init(self):
        try:
            rospy.init_node("postprocessor")
            inference_topic = f"/acl_inference/{self._model_name}"
            postprocess_topic = f"/postprocess/{self._model_name}"
            inference_msg_type = self._model_info["pub_message_type"]       # extracts model-specify inference MessageType

            self.inference_sub = rospy.Subscriber(inference_topic, inference_msg_type, self.inference_callback, queue_size=1, buff_size=2**24)
            self.postprocess_pub = rospy.Publisher(postprocess_topic, Image, queue_size=1)
            self.postprocess_pub_rate = rospy.Rate(30)
            self.pub_counter = 0
            rospy.loginfo("Postprocess Node: Publisher & Subscriber initialized.")

        except ROSInitException as err:
            raise err
    
    def inference_callback(self, msg):
        if not self.message_queue.full():
            self.message_queue.put(msg)

    @abstractmethod
    def deconstruct_ros_msg(self, msg):
        """From ROS Message to expected model_output format"""
        pass

    def run(self, processor):
        while not rospy.is_shutdown():
            if not self.message_queue.empty():
                try:
                    message = self.message_queue.get()
                    model_output = self.deconstruct_ros_msg(message)
                    frame = CvBridge().imgmsg_to_cv2(message.img)

                    postprocessed, _ = processor.postprocess(outputs=model_output, frame=frame)
                    rospy.loginfo(f"@postprocess: {type(postprocessed)}")

                    postprocessed = CvBridge().cv2_to_imgmsg(postprocessed, "passthrough")

                    self.postprocess_pub.publish(postprocessed)
                    rospy.loginfo(f"[{self.pub_counter}] Postprocessed and published.")
                    self.pub_counter += 1
                    self.postprocess_pub_rate.sleep()

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
                except KeyboardInterrupt as err:
                    rospy.loginfo("ROS Interrupt.")
            else:
                continue

    def shutdown(self):
        """Shutdown hook"""
        rospy.loginfo("AclInference node shutdown...")
        rospy.loginfo(f"Release resources...")
