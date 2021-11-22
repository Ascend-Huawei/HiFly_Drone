import rospy
import rosbag
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import gc
import cv2
import sys
import math
import argparse
sys.path.append("../../../src")
# sys.path.append("../../../src/lib")

#from utils.uav_utils import connect_uav


class CameraPublisher:
    """CameraPublisher node for publishing drone video stream 
    Accepts an initialized UAV object and initializes a ROS Publisher node to forward live-frames
    from drone as message data to /TelloDrone/image_raw Topic.
    @params:
        uav     initialized TelloUAV object.                                        @type:TelloUAV
        fps      rate (Hz) at which to read (and publish) the data from camera      @type:Int 
        qsize    size of outgoing message queue                                     @type:Int
    
    Returns:
        CameraPublisher node.
    """
    def __init__(self, uav=None, fps=30, qsize=1):
        self._uav = uav
        self._qsize = qsize
        self._fps = fps

        rospy.init_node("uav_cam", anonymous=True)
        rospy.loginfo("CameraPublisher initializing...")
        self.cam_data_topic = "/tello/cam_data_raw"
        self.cam_info_topic = "/tello/cam_info"
        self._cam_data_pub = rospy.Publisher(self.cam_data_topic, Image, queue_size=self._qsize)
        self._cam_info_pub = rospy.Publisher(self.cam_info_topic, CameraInfo, queue_size=self._qsize)
        self._rate = rospy.Rate(self._fps)
        self.cvb = CvBridge()

        rospy.on_shutdown(self.shutdown)

    # def build_cam_info(self):
    #     """Builds Tello Cam Info
    #     Read more: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
        
    #     [TODO] - should accept uav.drone_info to build cam_info
    #         uav.drone_info has: cam HxW, drone fov, 
    #     """
    #     cam_info = CameraInfo()
    #     # cam_info.header = None
    #     cam_info.width = 960
    #     cam_info.height = 720
    #     cam_info.distortion_model = 'plumb_bob'
    #     cx, cy = cam_info.width // 2, cam_info.height // 2
    #     tello_fov = 82.6
    #     fx = cam_info.width / (
    #         2.0 * math.tan(tello_fov * math.pi / 360.0))
    #     fy = fx
    #     cam_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    #     cam_info.D = [0, 0, 0, 0, 0]
    #     cam_info.R = [1.0, 0, 0, 0, 1.
    #     cam_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]

    #     return cam_info 

    def start_publish(self):
        # Test: Replace still frame with video
        cap = cv2.VideoCapture("/home/HwHiAiUser/HiFly_Drone/data/20210809_141017.mp4")
        print(f"CameraPublisher - Default video read FPS={round(cap.get(cv2.CAP_PROP_FPS), 3)}")
    
        if not cap.isOpened(): print("Error opening video file")
        
        while not rospy.is_shutdown() and cap.isOpened():
            # image_data = self._uav.get_frame_read().frame
            # image_data = cv2.imread("face_img.jpg")
            ret, image_data = cap.read()
            if image_data is None or not ret:
                print("Frame is None or return code error")
                break

            # [REMOVE] test if reducing resolution can increase performance: halving each dimension
            image_data = cv2.resize(image_data, (0,0), fx = 0.5, fy = 0.5)
            try:
                img_msg = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
                img_msg = self.cvb.cv2_to_imgmsg(img_msg, "rgb8")
                img_msg.header.stamp = rospy.Time.now()

                self._cam_data_pub.publish(img_msg)
                print("Image message & Cam Info published. ImageMessage: {}".format(type(img_msg)))
                self._rate.sleep()

            except CvBridgeError as err:
                print(err)

    def shutdown(self):
        """Shutdown hook"""
        rospy.loginfo("CamerPublisher node shutdown...")
        rospy.loginfo(f"Release resources...")
        gc.collect()


if __name__ == "__main__":
    # uav = connect_uav()
    parser = argparse.ArgumentParser(description="CameraPublisher ROS Node")
    parser.add_argument("--fps", default=30, type=int, help='Camera publisher FPS (default: 30)')
    args = parser.parse_args()
    fps = args.fps

    imgPub = CameraPublisher(fps=fps)
    imgPub.start_publish()
