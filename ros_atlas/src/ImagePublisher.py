import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import sys
import math
sys.path.append("../../src")
from utils.uav_utils import connect_uav


class ImagePublisher:
    """class:ImagePublisher
    Accepts a Tello UAV object and initializes a ROS Publisher node to forward live-frames Message from Tello UAV to /TelloDrone/image_raw Topic.
    :params:
        - uav       - instantated TelloUAV object
        - fps       - dynamically adjustable fps to account for different inference rate
        - qsize     - Publisher qsize
    """

    def __init__(self, uav, fps, qsize=10):
        self._uav = uav
        self._qsize = qsize
        self._fps = fps
        rospy.init_node("tello_img", anonymous=True)
        self.img_topic = "/TelloDrone/image_raw"
        self.camera_info_topic = "/TelloDrone/camera_info"
        self._img_pub = rospy.Publisher(self.img_topic, Image, queue_size=self._qsize)
        self._cam_info_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=self._qsize)
        self._rate = rospy.Rate(self._fps)

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

    # def init_publishers(self):
    #     rospy.init_node("tello_img", anonymous=True)
    #     self.img_topic = "/TelloDrone/image_raw"
    #     self.camera_info_topic = "/TelloDrone/camera_info"
    #     self._img_pub = rospy.Publisher(self.img_topic, Image, queue_size=self._qsize)
    #     self._cam_info_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=self._qsize)

    def build_cam_info(self):
        """Builds Tello Cam Info
        Read more: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
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
        self._uav.streamon()
        while not rospy.is_shutdown():
            image_data = self._uav.get_frame_read().frame
            print(type(image_data), image_data.shape)
            if image_data is None:
                print("Frame is None, check TelloUAV stream.")
                break
            try:
                # img_msg = np.frombuffer(image_data.data, dtype=np.uint8)
                # img_msg = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.shape[0], image_data.shape[1], -1)

                img_msg = CvBridge().cv2_to_imgmsg(image_data, "passthrough")
                img_msg.header.stamp = rospy.Time.now()

                cam_info_msg = self.build_cam_info()
                cam_info_msg.header.stamp = img_msg.header.stamp

                self._img_pub.publish(img_msg)
                self._cam_info_pub.publish(cam_info_msg)

            except CvBridgeError as err:
                print(err)
            finally:
                self._rate.sleep()


if __name__ == "__main__":

    uav = connect_uav()
    imgPub = ImagePublisher(uav, 20)
    imgPub.start_publish()
