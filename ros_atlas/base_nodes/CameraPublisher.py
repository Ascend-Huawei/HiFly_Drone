import argparse
import cv2
import sys
import time

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
    def __init__(self, uav=None, fps=10):
        self._uav = uav
        self._pub_counter = 0
        
        # for runtime analysis
        self._iteration_times = []       # to remove after experiment
        
        if self._uav is not None:
            self._uav.streamon()
            print("UAV Stream on.")
        try:
            rospy.init_node("uav_cam")
            rospy.loginfo("initializing CameraPublisher node.")
            self._cam_data_pub = rospy.Publisher("/tello/cam_data_raw", Image, queue_size=1)
            self._rate = rospy.Rate(fps)
        except ROSInitException as e:
            rospy.logerr("Ran into exception when initializing uav_cam node.")
            raise e
        rospy.on_shutdown(self.shutdown)

    def convert_and_pubish(self, image_data) -> None:
        st = time.time()
        try:
            img_msg = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
            img_msg = CvBridge().cv2_to_imgmsg(img_msg, "rgb8")
            img_msg.header.stamp = rospy.Time.now()

            self._cam_data_pub.publish(img_msg)
            rospy.loginfo(f"[{self._pub_counter}] Published ImageMessage")

            self._pub_counter += 1
            self._rate.sleep()

            self._iteration_times.append(time.time()-st)
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

    def start_publish(self, live_feed) -> None:
        cap = None
        if not live_feed:
            cap = cv2.VideoCapture("/home/HwHiAiUser/HiFly_Drone/data/video.avi")
            rospy.loginfo(f"CameraPublisher - Default video read FPS={round(cap.get(cv2.CAP_PROP_FPS), 30)}")
            if not cap.isOpened(): 
                rospy.signal_shutdown("Shutting down CameraPuyblisher. Reason: Error opening video file.")
        
        while not rospy.is_shutdown():
            image_data = self._uav.get_frame_read().frame if live_feed else cap.read()[1]

            if image_data is None:
                rospy.signal_shutdown("Frame is None. Shutdown")
                return

            # [REMOVE] test if reducing resolution can increase performance: halving each dimension
            # image_data = cv2.resize(image_data, (0,0), fx = 0.5, fy = 0.5)
            self.convert_and_pubish(image_data)
            

    def start_publish_live(self) -> None:
        """Main publish-loop from real-drone"""
        rospy.loginfo(f"CameraPublisher - Tello Streamon")
        
        while not rospy.is_shutdown():
            image_data = self._uav.get_frame_read().frame

            if image_data is None:
                rospy.loginfo("Frame is None or return code error.")
                break

            # [REMOVE] test if reducing resolution can increase performance: halving each dimension
            # image_data = cv2.resize(image_data, (0,0), fx = 0.5, fy = 0.5)

            try:
                img_msg = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
                img_msg = CvBridge().cv2_to_imgmsg(img_msg, "rgb8")
                img_msg.header.stamp = rospy.Time.now()

                self._cam_data_pub.publish(img_msg)
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
                raise err
            
    def shutdown(self) -> None:
        """Shutdown hook"""
        # uncomment to report average runtime
        # avg_iteration_time = sum(self._iteration_times) / len(self._iteration_times)
        # rospy.loginfo(f"CameraPublisher Average iteration time: {avg_iteration_time}")
        rospy.loginfo("CamerPublisher node shutdown. Release resources...")

if __name__ == "__main__":
    uav = None

    parser = argparse.ArgumentParser(description="CameraPublisher ROS Node")
    parser.add_argument("--fps", default=10, type=int, help='Camera publisher FPS (default: 30)')
    parser.add_argument("--live-feed", dest='live_feed', action='store_true')
    parser.add_argument("--no-live-feed", dest='live_feed', action='store_false')
    args = parser.parse_args()
    print(args.live_feed)

    if args.live_feed:
        uav = connect_uav()
    try:
        imgPub = CameraPublisher(uav=uav, fps=args.fps)
        imgPub.start_publish(live_feed=args.live_feed)
    except KeyboardInterrupt as e:
        rospy.signal_shutdown("Shutting down CameraPuyblisher. Keyboard terminate")