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
import time
from queue import Queue
from utils.tools import load_model_processor

import rospy
from sensor_msgs.msg import Image
from rospy.exceptions import ROSInitException


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
        self.message_queue = Queue(maxsize=1)
        self.expected_img_shape = expected_img_shape

        # Objects for runtime analysis
        self._stamp_dict = dict()
        self._sub_cb_times = list()
        self._iteration_times = list()
    
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
            inference_msg_type = self._model_info["pub_message_type"]       # extracts model-specific MessageType

            self.inference_sub = rospy.Subscriber(inference_topic, inference_msg_type, self.inference_callback, queue_size=1, buff_size=2**24)
            self.postprocess_pub = rospy.Publisher(postprocess_topic, Image, queue_size=1)
            self.postprocess_pub_rate = rospy.Rate(10) 
            self.pub_counter = 0
            rospy.loginfo("Postprocess Node: Publisher & Subscriber initialized.")

        except ROSInitException as err:
            rospy.logerr(err)
    
    def inference_callback(self, msg):
        msg_publish_time = msg.header.stamp
        cb_start = time.time()  # start timer
        msg_arrival_time = rospy.Time.now()
        self._stamp_dict[msg_publish_time] = msg_arrival_time

        if not self.message_queue.full():
            self.message_queue.put(msg)
            self._sub_cb_times.append(time.time() - cb_start)
        pass

    @abstractmethod
    def deconstruct_ros_msg(self, msg):
        """From ROS Message to expected model_output format"""
        pass

    @abstractmethod
    def run(self, processor, img_format="rgb8"):
        pass
        
    def shutdown(self):
        """Shutdown hook"""
        avg_iteration_time = sum(self._iteration_times) / len(self._iteration_times)
        avg_cb_time = sum(self._sub_cb_times) / len(self._sub_cb_times)

        inf2post_msg_transfer_times = [v.to_sec() - k.to_sec() for k,v in self._stamp_dict.items()]
        inf2post_avg_msg_trasnfer_time = sum(inf2post_msg_transfer_times) / len(inf2post_msg_transfer_times)

        # import pickle
        # pickle.dump(self._stamp_dict, open('inf2post_msg_transfer.pkl', 'wb'))

        rospy.loginfo(f'\nPostprocessor runtime results:')
        rospy.loginfo(f"Average while-iteration time: {round(avg_iteration_time, 5)}s")
        rospy.loginfo(f"Average sub_cb time: {round(avg_cb_time, 5)}s")
        rospy.loginfo(f"Average message transfer time from Inference (publish) -> Postprocess (subscriber cb): {round(inf2post_avg_msg_trasnfer_time, 5)}s")
        rospy.loginfo("Processor node shutdown, release resources...")
