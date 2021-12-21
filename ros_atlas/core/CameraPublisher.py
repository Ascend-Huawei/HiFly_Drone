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
        self._iteration_times = []
        
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
        # img_msg = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
        try:
            img_msg = CvBridge().cv2_to_imgmsg(image_data, "rgb8")
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
                rospy.signal_shutdown("Frame is None. Shutting down CameraPublisher.")
                return
            
            # ensure image_data.shape==(960, 720) if not live-stream
            if image_data.shape != (960, 720):
                image_data = cv2.resize(image_data, (960, 720))

            # uncomment to resize before publishing (faster runtime) - also need to specify expect_img_size in Postprocessor
            # image_data = cv2.resize(image_data, (0,0), fx = 0.5, fy = 0.5)
            self.convert_and_pubish(image_data)
            
    def shutdown(self) -> None:
        """Shutdown hook"""
        avg_iteration_time = sum(self._iteration_times) / len(self._iteration_times)
        rospy.loginfo(f"CameraPublisher Average iteration time: {round(avg_iteration_time, 5)}")
        rospy.loginfo("CamerPublisher node shutdown. Release resources...")

if __name__ == "__main__":
    uav = None

    parser = argparse.ArgumentParser(description="CameraPublisher ROS Node")
    parser.add_argument("--fps", default=10, type=int, help='Camera publisher FPS (default: 30)')
    parser.add_argument("--live-feed", dest='live_feed', action='store_true', help='Use live-feed from drone')
    parser.add_argument("--no-live-feed", dest='live_feed', action='store_false', help='Run on pre-recorded video')
    args = parser.parse_args()

    if args.live_feed:
        uav = connect_uav()
    try:
        imgPub = CameraPublisher(uav=uav, fps=args.fps)
        imgPub.start_publish(live_feed=args.live_feed)
    except KeyboardInterrupt as e:
        rospy.signal_shutdown("Shutting down CameraPuyblisher. Keyboard terminate")