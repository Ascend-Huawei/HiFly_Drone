#!/home/HwHiAiUser/miniconda3/envs/ros-noetic/bin/python3

import cv2
import time

from utils.uav_utils import connect_uav

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException, ROSSerializationException, ROSInitException, ROSInterruptException


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
    def __init__(self):
        rospy.init_node("uav_cam")
        rospy.loginfo("CameraPublisher node initialized.")
        self.init()
        
        rospy.on_shutdown(self.shutdown)
    
    def init(self) -> None:
        """Fetch Node's parameters from ParameterServer. Initialize node and saves params as class attribute
        Return:
            None
        """
        self.params = rospy.get_param('CameraPublisher')
        print("\n###################################################")
        print('CameraPublisher Parameters')
        print(self.params)
        print("###################################################")

         # for runtime analysis
        self._pub_counter = 0
        self._iteration_times = []

        # Publisher
        self._cam_data_pub = rospy.Publisher(self.params['topic_name'], Image, queue_size=1)
        self._ros_rate = rospy.Rate(self.params['pub_rate'])

        self.Tello = None
        if self.params['use_uav']:
            self.Tello = connect_uav()
            self.Tello.streamon()
            rospy.loginfo("UAV Stream on.")

    def convert_and_pubish(self, image_data) -> None:
        """Converts image data into supported ROS message @type:Image and publishes the converted message to the specified topic in ParameterServer
        Returns:
            None
        """
        st = time.time()
        # img_msg = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
        
        #  NOTE: uncomment to resize before publishing (faster runtime) - if enable, you will also need to specify @param:expect_img_size in InferenceNode
        # image_data = cv2.resize(image_data, (0,0), fx = 0.5, fy = 0.5)    # scale dimension down by 1/2 -> expected_image_size=(360, 480)
        image_data = cv2.resize(image_data, (0,0), fx = 0.25, fy = 0.25)    # scale dimension down by 3/4 -> expected_image_size=(180, 240)
        try:
            img_msg = CvBridge().cv2_to_imgmsg(image_data, "rgb8")
            img_msg.header.stamp = rospy.Time.now()

            self._cam_data_pub.publish(img_msg)
            print(f"[{self._pub_counter}] Published ImageMessage to {self.params['topic_name']}")

            self._pub_counter += 1
            self._ros_rate.sleep()
            self._iteration_times.append(time.time()-st)
        except (CvBridgeError, ROSSerializationException, ROSException, ROSInterruptException) as err:
            raise err

    def start_publish(self) -> None:
        """Main publish loop for publishing image data. Image data can be from VideoCapture or from live drone
        Return:
            None
        """
        cap = None
        if self.Tello is None:
            cap = cv2.VideoCapture(self.params['video_input'])
            rospy.loginfo(f"CameraPublisher - Default video read FPS={round(cap.get(cv2.CAP_PROP_FPS), 30)}")
            if not cap.isOpened(): 
                rospy.signal_shutdown("Shutting down CameraPuyblisher. Reason: Error opening video file.")
        
        while not rospy.is_shutdown():
            # image_data = self.Tello.get_frame_read().frame if self.Tello is not None else cap.read()[1]
            image_data = cap.read()[1] if self.Tello is None else self.Tello.get_frame_read().frame
            if image_data is None:
                rospy.signal_shutdown("Frame is None. Shutting down CameraPublisher.")
                return
            
            # ensure image_data.shape==(960, 720) if not live-stream - for loading from VideoCapture
            if image_data.shape != (960, 720):
                image_data = cv2.resize(image_data, (960, 720))

            self.convert_and_pubish(image_data)
            
    def shutdown(self) -> None:
        """Shutdown hook"""
        avg_iteration_time = sum(self._iteration_times) / len(self._iteration_times)
        rospy.loginfo(f"CameraPublisher Average iteration time: {round(avg_iteration_time, 5)*1000}ms")
        rospy.loginfo("CamerPublisher node shutdown. Release resources...")


if __name__ == "__main__":
    try:
        imgPub = CameraPublisher()
        imgPub.start_publish()
    except KeyboardInterrupt as e:
        rospy.signal_shutdown("Shutting down CameraPuyblisher. Keyboard terminate")
