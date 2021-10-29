import acl
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import sys
import math
import argparse
sys.path.append("../../../src")
# sys.path.append("../../../src/lib")

from utils.uav_utils import connect_uav


class CameraPublisher:
    """CameraPublisher ROS node for publishing drone video stream 
    Accepts an initialized UAV object and initializes a ROS Publisher node to forward live-frames
    from drone as message data to /TelloDrone/image_raw Topic.
    @params:uav     initialized TelloUAV object
    @param:fps      rate (Hz) at which to read (and publish) the data from camera 
    @param:qsize    size of outgoing message queue

    @type:uav       TelloUAV
    @type:fps       int
    @type:qsize     
    """

    def __init__(self, fps=15, qsize=1):
        # self._uav = uav
        self._qsize = qsize
        self._fps = fps
        rospy.init_node("uav_cam", anonymous=True)
        rospy.loginfo("CameraPublisher initializing...")
        self.cam_data_topic = "/tello/cam_data_raw"
        self.cam_info_topic = "/tello/cam_info"
        self._cam_data_pub = rospy.Publisher(self.cam_data_topic, Image, queue_size=1)
        self._cam_info_pub = rospy.Publisher(self.cam_info_topic, CameraInfo, queue_size=1)
        self._rate = rospy.Rate(30)
        self.cvb = CvBridge()

    @property
    def fps(self):
        return self._fps

    @fps.setter
    def fps(self, new_fps):
        if new_fps < 1:
            raise ValueError("FPS cannot be less than 1.")
        else:
            self._fps = new_fps
            self._rate = rospy.Rate(self._fps)

    def build_cam_info(self):
        """Builds Tello Cam Info
        Read more: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
        
        [TODO] - should accept uav.drone_info to build cam_info
            uav.drone_info has: cam HxW, drone fov, 
        """
        cam_info = CameraInfo()
        # cam_info.header = None
        cam_info.width = 960
        cam_info.height = 720
        cam_info.distortion_model = 'plumb_bob'
        cx, cy = cam_info.width // 2, cam_info.height // 2
        tello_fov = 82.6
        fx = cam_info.width / (
            2.0 * math.tan(tello_fov * math.pi / 360.0))
        fy = fx
        cam_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        cam_info.D = [0, 0, 0, 0, 0]
        cam_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        cam_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]
        return cam_info 

    def start_publish(self):
        # self._uav.streamon() 
        
        # Test: Replace still frame with video
        cap = cv2.VideoCapture("/home/HwHiAiUser/HiFly_Drone/data/20210809_141017.mp4")
        if not cap.isOpened():
            print("Error opening video file")
        
        while not rospy.is_shutdown() and cap.isOpened():
            # image_data = self._uav.get_frame_read().frame
            # image_data = cv2.imread("face_img.jpg")
            ret, image_data = cap.read()
            if image_data is None or not ret:
                print("Frame is None or return code error")
                break
            try:
                img_msg = self.cvb.cv2_to_imgmsg(image_data, "passthrough")
                img_msg.header.stamp = rospy.Time.now()

                cam_info_msg = self.build_cam_info()
                cam_info_msg.header.stamp = img_msg.header.stamp

                self._cam_data_pub.publish(img_msg)
                self._cam_info_pub.publish(cam_info_msg)

                print("Image message & Cam Info published. ImageMessage: {}".format(type(img_msg)))
                self._rate.sleep()

            except CvBridgeError as err:
                print(err)


if __name__ == "__main__":
    # uav = connect_uav()
    parser = argparse.ArgumentParser(description="CameraPublisher ROS Node")
    parser.add_argument("--fps", default=15, type=int, help='Camera publisher FPS (default: 15)')
    args = parser.parse_args()

    fps = args.fps

    imgPub = CameraPublisher(fps=fps)
    imgPub.start_publish()