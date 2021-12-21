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
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSInitException

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
        
        Users need to inherit from baseInference and extend the abstractmethods
        """
        self.image_queue = Queue(maxsize=1)

        # time performance metrics
        self._stamp_dict = dict()
        self._sub_cb_times = list()
        self._iteration_times = list()
        
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

            self._inference_topic = f"/acl_inference/{self._model_name}"
            self._inference_msg_type = self._model_info["pub_message_type"]

            self.cam_data_sub = rospy.Subscriber("/tello/cam_data_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
            self.inference_pub = rospy.Publisher(self._inference_topic, self._inference_msg_type, queue_size=1)
            self.inference_pub_rate = rospy.Rate(10)
            self.pub_counter = 0
            rospy.loginfo("ACLInference Node: Publisher & Subscriber initialized.")
        except ROSInitException as err:
            rospy.logerr(err)

    def image_callback(self, imgmsg):
        """Subscriber callback function triggered upon receiving incoming data from CameraPublisher 
        @params:
            img_data    - imgmsg data from "uav_cam" node
            cam_info    - camera info from "uav_cam" node
        @type:
            img_data    - ROS:SensorMessage.Image
            cam_info    - ROS:SesnorMessage.CameraInfo
        """
        msg_publish_time = imgmsg.header.stamp
        msg_arrival_time = rospy.Time.now()
        cb_start = time.time()
        self._stamp_dict[msg_publish_time] = msg_arrival_time
        
        if not self.image_queue.full():
            try:
                rgb_img = CvBridge().imgmsg_to_cv2(imgmsg)
                self.image_queue.put(rgb_img)
                self._sub_cb_times.append(time.time() - cb_start)
            except CvBridgeError as err:
                raise err

    @abstractmethod
    def construct_ros_msg(self, model_output, img):
        """From model_output to expected ROS Message format"""
        pass
    
    @abstractmethod
    def run(self, model):
        pass

    def shutdown(self):
        """Shutdown hook -- computes relevant runtime results for node and shutdown"""
        avg_iteration_time = sum(self._iteration_times) / len(self._iteration_times)
        avg_cb_time = sum(self._sub_cb_times) / len(self._sub_cb_times)

        cam2inf_msg_transfer_times = [v.to_sec() - k.to_sec() for k,v in self._stamp_dict.items()]
        cam2inf_avg_msg_transfer_time = sum(cam2inf_msg_transfer_times) / len(cam2inf_msg_transfer_times)

        # import pickle
        # pickle.dump(self._stamp_dict, open('cam2inf_msg_transfer.pkl', 'wb')) 

        rospy.loginfo(f'\nInference {self._model_name} runtime results:')
        rospy.loginfo(f"Average while-iteration time: {round(avg_iteration_time, 5)}s")
        rospy.loginfo(f"Average sub_cb time: {round(avg_cb_time, 5)}s")
        rospy.loginfo(f"Average message transfer time from CameraPublisher (publish) -> Inference (subscriber cb): {round(cam2inf_avg_msg_transfer_time, 5)}s")
        rospy.loginfo("Inference node shutdown, release resources...")
