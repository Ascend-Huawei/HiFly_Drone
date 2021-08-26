"""Playground to test FPS & Latency after merging Inference & Postprocess together"""

import cv2
import numpy as np
import logging
from queue import Queue
import rospy
import rosbag
import sys
import time

sys.path.append('../')
sys.path.append('../lib/')

from utils.tools import load_model_processor
from utils.uav_utils import connect_uav
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException, ROSSerializationException, ROSInitException, ROSInterruptException
from sensor_msgs.msg import Image


class FaceDetectorNode:
    """Fusion of Inference & Postprocess Noed"""
    def __init__(self) -> None:
        # Define Sub/Pub
        self.image_queue = Queue(maxsize=1)
        
         # Initialize time-metrics
        self._stamp_dict = dict()
        self._sub_cb_times = list()
        self._iteration_times = list()
        
        self.init_node()

        rospy.on_shutdown(self.shutdown)

    def load_model(self, model_name):
        mp, mp_info = load_model_processor(model_name)
        self._model_info = mp_info
        self._model_name = model_name
        # return mp(params=mp_info)
        return mp(params=mp_info, expected_image_shape=(360, 480))

    def init_node(self):
        """Node initialization. Set Subscriber(s) and Publisher."""
        try:
            # rospy.init_node('acl_inference_node', anonymous=True)
            rospy.init_node('acl_inference_node', anonymous=True)
            rospy.loginfo("ACLInference Node initializing...")

            self._inference_topic = f"/acl_inference/{self._model_name}"

            self.cam_data_sub = rospy.Subscriber("/tello/cam_data_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
            self.inference_pub = rospy.Publisher(self._inference_topic, Image, queue_size=1)
            self.inference_pub_rate = rospy.Rate(14)
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

    def run(self, model):
        """Main loop for inference. Make inference with model on images from CameraPublisher and publish.
        @param:model    - a ModelProcessor object 
        Returns:
            None
        """
        self.model = model
        while not rospy.is_shutdown():
            st = time.time()
            try:
                if not self.image_queue.empty():
                    image = self.image_queue.get()
                    result_frame = self.model.predict(image)
                    # cv2.imwrite('test_out.png', result_frame)
                    img_msg = CvBridge().cv2_to_imgmsg(result_frame, "rgb8")
                    img_msg.header.stamp = rospy.Time.now()

                    self.inference_pub.publish(img_msg)
                    self.pub_counter += 1
                    print(f"[{self.pub_counter}]: Published model output(s) to topic: {self._inference_topic}")

                    # self.inference_pub_rate.sleep()
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

    def shutdown(self):
        """Shutdown hook -- computes relevant runtime results for node and shutdown"""
        try:
            avg_iteration_time = sum(self._iteration_times) / len(self._iteration_times)
            avg_cb_time = sum(self._sub_cb_times) / len(self._sub_cb_times)

            cam2inf_msg_transfer_times = [v.to_sec() - k.to_sec() for k,v in self._stamp_dict.items()]
            cam2inf_avg_msg_transfer_time = sum(cam2inf_msg_transfer_times) / len(cam2inf_msg_transfer_times)

            rospy.loginfo(f'\nInference {self._model_name} runtime results:')
            rospy.loginfo(f"Average while-iteration time: {round(avg_iteration_time, 5)}s")
            rospy.loginfo(f"Average sub_cb time: {round(avg_cb_time, 5)}s")
            rospy.loginfo(f"Average message transfer time from CameraPublisher (publish) -> Inference (subscriber cb): {round(cam2inf_avg_msg_transfer_time, 5)}s")
            rospy.loginfo("Inference node shutdown, release resources...")

            # Calculate pre,exe,post from ModelProcessor
            
            for key in self.model.metric.keys():
                avg = sum(self.model.metric[key]) / len(self.model.metric[key])
                rospy.loginfo(f'Average {key} time: {avg}')
        except ZeroDivisionError as Exception:
            print('Unable to gather metric, see errors above.')



if __name__ == '__main__':
    fd_node = FaceDetectorNode()
    model = fd_node.load_model('face_detection') 
    fd_node.init()
    try:
        fd_node.run(model)
    except KeyboardInterrupt as e:
        rospy.signal_shutdown("Shutting down Inference. Keyboard terminate")
