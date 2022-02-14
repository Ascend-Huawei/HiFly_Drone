#!/home/HwHiAiUser/miniconda3/envs/ros-noetic/bin/python3

import cv2
import numpy as np
import rospy
import time
from queue import Queue

from utils.tools import load_model_processor

from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException, ROSSerializationException, ROSInitException, ROSInterruptException
from sensor_msgs.msg import Image

class InferenceNode:
    """Preprocess + Inference + Postprocess Node"""
    def __init__(self) -> None:
        rospy.init_node('acl_inference')
        rospy.loginfo("ACLInference Node initialized...")

        self.init()
        
        rospy.on_shutdown(self.shutdown)

    def init(self) -> None:
        """Fetch Node's parameters from ParameterServer. Initialize node and saves params as class attribute
        Return:
            None
        """
        self.params = rospy.get_param('ACLInference')
        print("\n###################################################")
        print('ACLInference Node Parameters')
        print(self.params)
        print("###################################################\n")

        mp, model_info = load_model_processor(self.params['model'])
        self.model_info = model_info
        self.model_name = self.params['model']
        # self.model = mp(params=self.model_info, expected_image_shape=(360, 480))
        self.model = mp(params=self.model_info, expected_image_shape=(180, 240))

        self.image_queue = Queue(maxsize=1)

        self._inference_topic = f"/acl_inference/{self.model_name}"

        rospy.Subscriber("/tello/cam_data_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
        self._inference_pub = rospy.Publisher(self._inference_topic, Image, queue_size=1)
        # self.inference_pub_rate = rospy.Rate(self.params['pub_rate'])
        self._pub_counter = 0

        # time performance metrics
        self._stamp_dict = dict()
        self._sub_cb_times = list()
        self._iteration_times = list()

        rospy.loginfo("ACLInference Node: Publisher & Subscriber initialized.")

        
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

    def run(self):
        """Main loop for inference. Make inference with model on images from CameraPublisher and publish.
        @param:model    - a ModelProcessor object 
        Returns:
            None
        """
        while not rospy.is_shutdown():
            st = time.time()
            try:
                if not self.image_queue.empty():
                    image = self.image_queue.get()
                    result_frame = self.model.predict(image)

                    cv2.imwrite('test_out.png', result_frame)
                    img_msg = CvBridge().cv2_to_imgmsg(result_frame, "rgb8")
                    img_msg.header.stamp = rospy.Time.now()

                    self._inference_pub.publish(img_msg)
                    self._pub_counter += 1
                    print(f"[{self._pub_counter}]: Published model output(s) to topic: {self._inference_topic}")

                    # self.inference_pub_rate.sleep()
                    self._iteration_times.append(time.time() - st)
                else:
                    continue
            except (ROSException, CvBridgeError, ROSSerializationException) as err:
                rospy.logerr("Ran into exception when converting message type with CvBridge. See error below:")
                raise err

    def shutdown(self):
        """Shutdown hook -- computes relevant runtime results for node and shutdown"""
        avg_iteration_time = sum(self._iteration_times) / len(self._iteration_times)
        avg_cb_time = sum(self._sub_cb_times) / len(self._sub_cb_times)

        cam2inf_msg_transfer_times = [v.to_sec() - k.to_sec() for k,v in self._stamp_dict.items()]
        cam2inf_avg_msg_transfer_time = sum(cam2inf_msg_transfer_times) / len(cam2inf_msg_transfer_times)

        rospy.loginfo(f'\nInference {self.model_name} runtime results:')
        rospy.loginfo(f"Average while-iteration time: {round(avg_iteration_time, 5)*1000}ms")
        rospy.loginfo(f"Average sub_cb time: {round(avg_cb_time, 5)*1000}ms")
        rospy.loginfo(f"Average message transfer time from CameraPublisher (publish) -> Inference (subscriber cb): {round(cam2inf_avg_msg_transfer_time, 5)*1000}ms")
        rospy.loginfo("Inference node shutdown, releasing resources...")

        for key in self.model.metric.keys():
            avg = sum(self.model.metric[key]) / len(self.model.metric[key])
            rospy.loginfo(f'Average {key} time: {round(avg, 3)*1000}ms')


    def __repr__(self):
        rep = f'ACLInference {self.model_name} Node'

if __name__ == '__main__':
    inference_node = InferenceNode()
    inference_node.run()
