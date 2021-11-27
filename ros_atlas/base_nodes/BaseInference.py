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
import sys
import time
from queue import Queue
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException, ROSSerializationException, ROSInitException, ROSInterruptException

from utils.tools import load_model_processor


class BaseInferenceNode:
    def __init__(self):
        """AclInference Node - Listens and makes inference on incoming sensor data (images) from TelloUAV object and publish results to /acl_inference/results topic
        @param:
            model_name      - Name of the supported model (refer to params.py for keynames)   @type:String
            inference_rate  - Inference rate of model_name.                                   @type:Int
            qsize           - Size of global queue for thread-wise reference.                 @type:Int 
        Returns:
            None
        
        TODO: Users need to inherit from AciInference and override the construct_ros_msg method
        """
        self.image_queue = Queue(maxsize=5)
        self.history = dict()
        rospy.on_shutdown(self.shutdown)

    def load_model(self, model_name):
        """Returns an instantiated model based on model_name."""
        mp, model_info = load_model_processor(model_name)
        self._model_info = model_info
        self._model_name = model_name
        return mp(params=model_info)

    def init(self):
        """Node initialization. Set Subscriber(s) and Publisher."""
        try:
            rospy.init_node('acl_inference_node', anonymous=True)
            rospy.loginfo("ACLInference Node initializing...")

            self.history["node_start"] = time.time()
            self._inference_topic = f"/acl_inference/{self._model_name}"
            self._inference_msg_type = self._model_info["pub_message_type"]

            self.cam_data_sub = rospy.Subscriber("/tello/cam_data_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
            self.inference_pub = rospy.Publisher(self._inference_topic, self._inference_msg_type, queue_size=1)
            self.inference_pub_rate = rospy.Rate(30)
            self.pub_counter = 0
            rospy.loginfo("ACLInference Node: Publisher & Subscriber initialized.")
            return
        except ROSInitException as err:
            err

    def image_callback(self, imgmsg):
        """Subscriber callback function triggered upon receiving incoming data from CameraPublisher 
        @params:
            img_data    - imgmsg data from "uav_cam" node
            cam_info    - camera info from "uav_cam" node
        @type:
            img_data    - ROS:SensorMessage.Image
            cam_info    - ROS:SesnorMessage.CameraInfo
        """
        try:
            timestamp = imgmsg.header.stamp
            rgb_img = CvBridge().imgmsg_to_cv2(imgmsg)
            if not self.image_queue.full():
                self.image_queue.put((rgb_img, timestamp))
        except CvBridgeError as err:
            raise err


    @abstractmethod
    def construct_ros_msg(self, model_output, img):
        """From model_output to expected ROS Message format"""
        pass
    
    def run(self, model):
       while not rospy.is_shutdown():
            try:
                if not self.image_queue.empty():
                    image, _ = self.image_queue.get()
                    
                    preprocessed = model.preprocess(image)
                    model_output = model.model.execute([preprocessed])

                    ros_inference_msg = self.construct_ros_msg(model_output, image)

                    print(f"[{self.pub_counter}]: Publish model_output to topic: {self._inference_topic}")
                    self.inference_pub.publish(ros_inference_msg)
                    self.pub_counter += 1
                    self.inference_pub_rate.sleep()
                else:
                    rospy.loginfo("Image queue is empty. Check image_callback")
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
            except KeyboardInterrupt as err:
                rospy.loginfo("ROS Interrupt.")
                raise err



    def shutdown(self):
        """Shutdown hook"""
        rospy.loginfo("AclInference node shutdown...")
        rospy.loginfo(f"Release resources...")
        
        self.history["node_end"] = time.time()