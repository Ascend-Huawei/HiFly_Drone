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

import argparse
import cv2
import numpy as np
import sys
import time
from queue import Queue
import rospy

from sensor_msgs.msg import Image 
from custom_ros_msgs.msg import FloatArray, FloatArrays, FaceDetection
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException, ROSSerializationException, ROSInitException, ROSInterruptException

sys.path.append("../../")
from ros_atlas.utils.tools import load_model_processor


class AclInferenceNode:
    def __init__(self):
        """AclInference Node - Listens and makes inference on incoming sensor data (images) from TelloUAV object and publish results to /acl_inference/results topic
        @param:
            model_name      - Name of the supported model (refer to params.py for keynames)   @type:String
            inference_rate  - Inference rate of model_name.                                   @type:Int
            qsize           - Size of global queue for thread-wise reference.                 @type:Int 
        Returns:
            AclInferenceNode instance.
        
        #TODO: add inference rate to params.py dictionary for EACH model
        """
        self.bridge = CvBridge()
        self.image_queue = Queue(maxsize=5)
        self.history = dict()
        rospy.on_shutdown(self.shutdown)

    def init(self):
        """Node initialization. Set Subscriber(s) and Publisher."""
        rospy.init_node('acl_inference_node', anonymous=True)
        rospy.loginfo("ACLInference Node initializing...")

        self.history["node_start"] = time.time()
        self.cam_data_sub = rospy.Subscriber("/tello/cam_data_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.inference_pub = rospy.Publisher(self.inference_topic, self.inference_msg_type, queue_size=1)
        self.inference_pub_rate = rospy.Rate(30)

        rospy.loginfo("ACLInference Node: Publisher & Subscriber initialized.")
        return 

    def load_model(self, model_name):
        """Returns an instantiated model based on model_name."""
        model_processor, model_info = load_model_processor(model_name)
        self.inference_topic = "/acl_inference/" + model_name
        self.inference_msg_type = model_info["pub_message_type"]
        return model_processor(model_info)

    def image_callback(self, img_data):
        """Subscriber callback function triggered upon receiving incoming data from CameraPublisher 
        @params:
            img_data    - imgmsg data from "uav_cam" node
            cam_info    - camera info from "uav_cam" node
        @type:
            img_data    - ROS:SensorMessage.Image
            cam_info    - ROS:SesnorMessage.CameraInfo
        """
        try:
            timestamp = img_data.header.stamp
            rgb_img = self.bridge.imgmsg_to_cv2(img_data)
            if not self.image_queue.full():
                self.image_queue.put((rgb_img, timestamp))
        except CvBridgeError as err:
            raise err

    def shutdown(self):
        """Shutdown hook"""
        rospy.loginfo("AclInference node shutdown...")
        rospy.loginfo(f"Release resources...")
        
        self.history["node_end"] = time.time()
        # average_loop_runtime = self.history["total_loop_time"] / self.counter
        # average_inference_time = self.history["total_inference_time"] / self.counter

        # rospy.loginfo(f"[Including rate.sleep] Published {self.counter} frames in {program_duration}s -> {self.counter / program_duration} FPS.")
        # rospy.loginfo(f"[Excluding rate.sleep] Average time (s) to complete one loop: {average_loop_runtime}")
        # rospy.loginfo(f"[Excluding rate.sleep] Average time (s) to make inference: {average_inference_time}")



if __name__ == "__main__":
    #TODO: determine Message type based on Model - Also need to create custom Message Type for AclInferenceNode.Publisher for respective models...

    parser = argparse.ArgumentParser(description='AclInference Node.')
    parser.add_argument('-m', '--model', metavar='MODEL', type=str, default='face_detection', help='Model name (refer to params.py)')
    args = parser.parse_args()

    model_name = args.model

    inference_node = AclInferenceNode()
    model = inference_node.load_model(model_name)
    inference_node.init()

    counter = 0

    while not rospy.is_shutdown():
        try:
            if not inference_node.image_queue.empty():
                image, timestamp = inference_node.image_queue.get()
                
                preprocessed = model.preprocess(image)
                model_output = model.model.execute([preprocessed])

                detection_msg = FaceDetection()
                detection_msg.header.stamp = rospy.Time.now()
                detection_msg.array1.list = model_output[0].flatten().tolist()
                detection_msg.array2.list = model_output[1].flatten().tolist()
                detection_msg.array3.list = model_output[2].flatten().tolist()
                detection_msg.img = CvBridge().cv2_to_imgmsg(image)

                print(f"[{counter}]: Publish model_output to topic: {inference_node.inference_topic}")
                counter += 1
                inference_node.inference_pub.publish(detection_msg)
                inference_node.inference_pub_rate.sleep()
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
        except KeyboardInterrupt as err:
            rospy.loginfo("ROS Interrupt.")