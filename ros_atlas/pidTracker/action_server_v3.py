from curtsies import Input
import numpy as np
import cv2
import sys
import time
import rospy
from actionlib import *
from queue import Queue

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from custom_ros_action.msg import InitDroneGoal, InitDroneResult, InitDroneAction, MoveAgentAction, MoveAgentGoal
from custom_ros_msgs.msg import ProcessVar
from rospy.exceptions import ROSException, ROSSerializationException, ROSInitException, ROSInterruptException

sys.path.append("../../")
try:
    from src.inference_filters.DecisionFilter import DecisionFilter
    from ros_atlas.utils.uav_utils import connect_uav
except ImportError as err:
    raise err

class PIDActionServer:
    """
    PIDActionServer
    initializes SASs and drone. Callback functions employed by SASs will have access to the drone object.
    """
    def __init__(self, name) -> None:
        self.app_name = name
        self._uav = None
        
        self._init_sas = SimpleActionServer('init_drone', InitDroneAction, execute_cb=self.execute_init_cb, auto_start=False)
        self._pid_sas = SimpleActionServer('main', MoveAgentAction, execute_cb=self.execute_main_cb, auto_start=False)

        # attributes for PID
        self.setpoint_area = (20000, 100000)        # Case-specific, adjustable -- Lower and Upper bound for Forward & Backward Range-of-Motion
        self.setpoint_center = (480, 360)           # Case-specific, adjustable -- Lower and upper bound for xy rotation     
        self.pid_input = [0.1, 0.1, 0.1]            # Case-specific, adjustable -- Values for PID equation
        self._is_search = True

        self._x_err, self._y_err = 0, 0
        self._prev_x_err, self._prev_y_err = 0, 0

        # initialize subscriber (listens to postprocess for process_var info) 
        self.sub = rospy.Subscriber('/pid_fd/process_vars', ProcessVar, self.process_var_sub_cb, queue_size=1, buff_size=2**24)
        self.pub = rospy.Publisher("/tello/cam_data_raw", Image, queue_size=1)
        self.rate = rospy.Rate(30)

        # Result filter
        self.inference_filter = DecisionFilter(fps=10, window=3)

        # start the SASs
        self._init_sas.start()
        self._pid_sas.start()
        rospy.loginfo(f"SASs started. Ready for client.")

    def pid(self, error, prev_error):
        """PID Output signal equation"""
        return self.pid_input[0]*error + self.pid_input[1]*(error-prev_error) + self.pid_input[2]*(error-prev_error)

    def process_var_sub_cb(self, msg):
        """Subscriber callback without filtering results 
        @param:msg      -- Message from PostProcessor node  @type:ProcessVar
        
        Function
            updates bbox area and coordinates of bbox center as well as current and previous xy errors     
        Return
            None
        """
        if msg.area == 0:
            self._is_search = True   # indicates which mode to use
            self._x_err = self._prev_x_err
            self._y_err = self._prev_y_err
            return

        self._is_search = False                              # if area!=0 -> flip flag
        self._area = msg.area
        self._x_err = msg.cx - self.setpoint_center[0]       # rotational err
        self._y_err = self.setpoint_center[1] - msg.cy       # elevation err
        return
    
    def dfilter_sub_cb(self, msg):
        """Filtere-based Subscriber callback using DecisionFilter"""
        sample = None
        if msg.area != 0:
            sample = "Presence"
        filtered_result = self.inference_filter.sample(sample)      # either enqueue (and return a string) or return the majority result
        
        # Case: DecisionFilter is still collecting samples to determine the majority - returns string "MODE_INFERENCE_SAMPLING"
        if filtered_result == "MODE_INFERENCE_SAMPLING":
            pass
        # Case: DecisionFilter returns the majority inference result (Presence) - switch to tracking mode
        elif filtered_result == "Presence":
            self._is_search = False
            self._area = msg.area
            self._x_err = msg.cx - self.setpoint_center[0]       # rotational err
            self._y_err = self.setpoint_center[1] - msg.cy       # elevation err
        else:
            self._is_search = True
            self._x_err = self._prev_x_err
            self._y_err = self._prev_y_err

    def execute_init_cb(self, goal):
        """Optional callback that gets called in a separate thread whenever a new goal is received, 
        allowing users to have blocking callbacks. Adding an execute callback also deactivates the goalCallback."""
        start_time = time.time()

        while self._uav is None:
            now = time.time()
            # Check if preempt has not been requested by Client-side
            if self._init_sas.is_preempt_requested():
                rospy.loginfo(f"{self._action_name} Preempted")
                self._init_sas.set_preempted()

            # Check for how many tries are left
            if now - start_time > 30:
                rospy.loginfo(f"{self._action_name} Aborted")
                self._init_sas.set_aborted()
            try:
                self._uav = connect_uav()
                if self._uav is not None:
                    self.connect_start = time.time()
                    self._uav.streamon()
                    rospy.loginfo("@init_cb: Tello Drone connection established.")
                    self._init_sas.set_succeeded()
            except Exception as err:
                print(f"Trying again... {now-start_time}s remaining.")
                raise err
    
    def execute_start_stream_cb(self, goal):
        pub_counter = 0
        while not rospy.is_shutdown():
            image_data = self._uav.get_frame_read().frame
            if image_data is None:
                rospy.loginfo("Frame is None or return code error.")
                break
            image_data = cv2.resize(image_data, (0,0), fx = 0.5, fy = 0.5)
            try:
                img_msg = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
                img_msg = CvBridge().cv2_to_imgmsg(img_msg, "rgb8")
                self.pub.publish(img_msg)
                pub_counter += 1
                self.rate.sleep()
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
                raise err
    
    def execute_search_cb(self, goal):
        """should only look at self._is_search"""
        rospy.loginfo('Searching...')
        # while self._is_search:
        # Rotate drone here
        # self._con_search_sas.set_aborted()
    
    def execute_track_cb(self, goal):
        """should only look at self._is_search"""
        # while not self._is_search:
        forward_backward_velocity = 0
        left_right_velocity = 0
        up_down_velocity = 0
        yaw_velocity = 0

        # Compensator: calcuate the amount adjustment needed in YAW axis and distance
        # Localization of ToI to the center x-axis - adjusts camera angle
        if self._x_err != 0:
            yaw_velocity = self.pid(self._x_err, self._prev_x_err)
            yaw_velocity = int(np.clip(yaw_velocity, -100, 100))

        # Localization of ToI to the center y-axis - adjust altitude 
        if self._y_err != 0:
            up_down_velocity = 3*self.pid(self._y_err, self._prev_y_err)
            up_down_velocity = int(np.clip(up_down_velocity, -50, 50))

        # Rectify distance between drone and target from bbox area: Adjusts forward and backward motion
        if self._area > self.setpoint_area[0] and self._area < self.setpoint_area[1]:
            forward_backward_velocity = 0
        elif self._area < self.setpoint_area[0]:
            forward_backward_velocity = 20
        elif self._area > self.setpoint_area[1]:
            forward_backward_velocity = -20

        rospy.loginfo(f"[Tracking] Actuator command: {left_right_velocity}, {forward_backward_velocity}, {up_down_velocity}, {yaw_velocity}")
                # Actuator - adjust drone's motion to converge to setpoint by sending rc control commands
                # self._uav.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)
            # self._con_motor_sas.set_aborted()
        # self._con_track_sas.set_aborted()

    def execute_main_cb(self, goal):
        pub_counter = 0
        while not rospy.is_shutdown():
            image_data = self._uav.get_frame_read().frame
            if image_data is None:
                rospy.loginfo("Frame is None or return code error.")
                break
            # image_data = cv2.resize(image_data, (0,0), fx = 0.5, fy = 0.5)
            try:
                image_data = cv2.resize(image_data, (0,0), fx = 0.5, fy = 0.5)
                img_msg = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
                img_msg = CvBridge().cv2_to_imgmsg(img_msg, "rgb8")
                self.pub.publish(img_msg)
                pub_counter += 1

                # listen to subscriber's results
                if self._is_search:
                    self.execute_search_cb(goal)
                else:
                    self.execute_track_cb(goal)

                self.rate.sleep()
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
                raise err


if __name__ == '__main__':
    import os
    print(f"ActionServer pid: {os.getpid()}")
    rospy.init_node('pid_action_server')
    ActionServer = PIDActionServer('pid_tracker')
    rospy.spin()