import cv2
import argparse
import os
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import sys
import message_filters

# sys.path.append("../../../src")
# sys.path.append("../../../src/lib")

# from utils.uav_utils import connect_uav

class InferenceVisualizer:
    """VisualizerNode to display acl inference results. Subscribes to /acl_inference/result and display as result data comes in.
    """
    def __init__(self, qsize=15):
        self._acl_inference_topic = "/acl_inference/results"
        self.save_counter = 0
        self.qsize = qsize

    def infered_callback(self, img_data):
        """Callback function to process incoming message data. Saves the message data to tmp directory for now.
        @param:msg_data Message data from /acl_inferece/results 
        @type:msg_data ROS:Image
        """
        try:
            rgb_img = CvBridge().imgmsg_to_cv2(img_data)
            fname = os.path.join("./tmp", str(self.save_counter) + ".png")
            cv2.imwrite(fname, rgb_img)
            log_msg = "saved image: {}".format(self.save_counter)
            rospy.loginfo(log_msg)
            self.save_counter += 1

        except CvBridgeError as cvb_err:
            raise cvb_err

        except FileExistsError as err:
            raise err
            print("Remove files in tmp dir and try again")

    def init_node(self):
        """Initializes the subscriber node to listen to the acl_inference topic
        Initializes Subscriber of @type:message_filter.TimeSynchronizer 
        """
        rospy.init_node("visualizer_node", anonymous=True)
        rospy.loginfo("Inference visualizer initializing...")
        img_res_sub = message_filters.Subscriber(self._acl_inference_topic, Image)
        
        time_synchronizer = message_filters.TimeSynchronizer(fs=[img_res_sub], queue_size=self.qsize)
        time_synchronizer.registerCallback(self.infered_callback)
        rospy.spin()

        # pub = rospy.Publisher("/acl_inference/results", Image, queue_size=self.qsize)
        # pub_rate = rospy.Rate(self.inference_rate)
        # return pub, pub_rate, time_synchronizer
    
    def shutdown_handlier(self):
        rospy.loginfo("Inference visualizer shutdown...")
        rospy.loginfo(f"Saved {self.save_counter} images to directory")
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Inference Visualizer ROS Node")
    parser.add_argument("--qsize", '-q', default=30, type=int, help='queue size for subscriber node (default: 30)')
    args = parser.parse_args()

    qsize = args.qsize

    inf_vis = InferenceVisualizer(qsize=qsize)
    inf_vis.init_node()