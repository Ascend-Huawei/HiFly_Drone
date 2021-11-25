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
import argparse
import cv2
import os
from queue import Queue
import sys
import numpy as np
import time

sys.path.append("../../")

from ros_atlas.utils.tools import load_model_processor
from ros_atlas.utils.params import params

import rospy
import rosnode
from sensor_msgs.msg import Image, CameraInfo
from custom_ros_msgs.msg import FloatArray, FloatArrays, FaceDetection
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException, ROSSerializationException, ROSInitException, ROSInterruptException

"""Postprocessor

+ Subscribes to @topic:/acl_inference/<model_name> where @message_type can be of: FaceDetection, HandDetection, YoloDetection and more...
+ Capable of determining postprocessing procedure based on model used in AclInferencer
+ Postprocessing should NOT involve spawning AclModels (as inference is already done in AclInference)
+ Postprocessor should inherit from individual processor -- Dynamic Inheritance
    " `"dynamic inheritance` is a class that doesn't inherit from any base class in particular,
        but rather chooses to inherit from one of several base classes at instantiation, depending on some parameter."
"""

def format_ros_msg(msg):
    """Reconstructure ROS Message of @msg.type:FloatArrays back to model_output format"""
    array_1 = np.reshape(np.array(msg.array1.list), (1, 13, 13, 18))
    array_2 = np.reshape(np.array(msg.array2.list), (1, 26, 26, 18))
    array_3 = np.reshape(np.array(msg.array3.list), (1, 52, 52, 18))
    return [array_1, array_2, array_3]

def inference_callback(msg):
    global message_queue
    if not message_queue.full():
        try:
            message_queue.put(msg)
        except CvBridgeError as err:
            rospy.logerr("Ran into exception when converting message type with CvBridge. See error below:")
            raise err

def shutdown():
    rospy.loginfo(f"Rospy postprocess node shutdown")


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Postprocess Node')
    parser.add_argument('-m', '--model', type=str, default='face_detection', help='Model name (refer to params.py)')
    parser.add_argument('-s', '--input_shape', type=int, nargs=2, default=None, help='image shape')

    args = parser.parse_args()
    model_name = args.model
    image_shape = args.input_shape

    mp, model_info = load_model_processor(model_name)
    Postprocessor = mp(params=model_info, image_shape=image_shape, load_model=False)
    print("Model loaded but not really")

    message_queue = Queue(maxsize=5)
    bridge = CvBridge()
    counter = 0
    
    rospy.init_node("postprocessor")
    inference_topic = f"/acl_inference/{model_name}"
    postprocess_topic = f"/postprocess/{model_name}"
    inference_msg_type = model_info["pub_message_type"]       # extracts model-specify inference MessageType
    inference_sub = rospy.Subscriber(inference_topic, inference_msg_type, inference_callback, queue_size=1, buff_size=2**24)
    postprocess_pub = rospy.Publisher(postprocess_topic, Image, queue_size=1)
    postprocess_pub_rate = rospy.Rate(30)
    rospy.on_shutdown(shutdown)

    while not rospy.is_shutdown():
        if not message_queue.empty():
            try:
                message = message_queue.get()                  # @msg.type <model_name> ie; FaceDetection
            
                model_output = format_ros_msg(message)
                frame = bridge.imgmsg_to_cv2(message.img)
                postprocessed, yolo_eval_end = Postprocessor.postprocess(frame, model_output)

                postprocessed = bridge.cv2_to_imgmsg(postprocessed, "rgb8")
                postprocess_pub.publish(postprocessed)
                rospy.loginfo(f"[{counter}] Postprocessed and published")
                counter += 1
                postprocess_pub_rate.sleep()
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
        