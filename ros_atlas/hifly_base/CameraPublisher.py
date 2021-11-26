import cv2
import sys
import argparse
sys.path.append("../../")

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException, ROSSerializationException, ROSInitException, ROSInterruptException
from ros_atlas.utils.uav_utils import connect_uav


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
        self._pub_counter = 0

        if self._uav is not None:
            try:
                self._uav.streamon()
                print("UAV Stream on.")
            except Exception as err:
                raise err
        try:
            rospy.init_node("uav_cam")
            rospy.loginfo("initializing CameraPublisher node.")
            self._cam_data_pub = rospy.Publisher("/tello/cam_data_raw", Image, queue_size=1)
            self._rate = rospy.Rate(30)

        except ROSInitException as e:
            rospy.logerr("Ran into exception when initializing uav_cam node.")
            raise e
            
        rospy.on_shutdown(self.shutdown)

    def start_publish(self):
        cap = cv2.VideoCapture("/home/HwHiAiUser/HiFly_Drone/data/20210809_141017.mp4")
        rospy.loginfo(f"CameraPublisher - Default video read FPS={round(cap.get(cv2.CAP_PROP_FPS), 3)}")
        if not cap.isOpened(): 
            rospy.signal_shutdown("Shutting down CameraPuyblisher. Reason: Error opening video file.")
        
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
                # rospy.loginfo(f"Static image shape = {image_data.shape}")
                img_msg = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
                img_msg = CvBridge().cv2_to_imgmsg(img_msg, "rgb8")
                img_msg.header.stamp = rospy.Time.now()

                self._cam_data_pub.publish(img_msg)
                rospy.loginfo(f"[{self._pub_counter}] Published ImageMessage")
                self._pub_counter += 1
                self._rate.sleep()

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

    def start_publish_live(self):
        rospy.loginfo(f"CameraPublisher - Tello Streamon")
        
        while not rospy.is_shutdown():
            image_data = self._uav.get_frame_read().frame
            if image_data is None:
                rospy.loginfo("Frame is None or return code error.")
                break

            # [REMOVE] test if reducing resolution can increase performance: halving each dimension
            image_data = cv2.resize(image_data, (0,0), fx = 0.5, fy = 0.5)

            try:
                # rospy.loginfo(f"Static image shape = {image_data.shape}")
                img_msg = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
                img_msg = CvBridge().cv2_to_imgmsg(img_msg, "rgb8")
                img_msg.header.stamp = rospy.Time.now()

                self._cam_data_pub.publish(img_msg)
                rospy.loginfo(f"[{self._pub_counter}] Published ImageMessage")
                self._pub_counter += 1
                self._rate.sleep()

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

    def shutdown(self):
        """Shutdown hook"""
        rospy.loginfo("CamerPublisher node shutdown...")
        rospy.loginfo(f"Release resources...")
        gc.collect()

if __name__ == "__main__":
    uav = connect_uav()
    parser = argparse.ArgumentParser(description="CameraPublisher ROS Node")
    parser.add_argument("--fps", default=30, type=int, help='Camera publisher FPS (default: 30)')
    args = parser.parse_args()
    fps = args.fps

    imgPub = CameraPublisher(uav=uav, fps=fps)
    imgPub.start_publish_live()
