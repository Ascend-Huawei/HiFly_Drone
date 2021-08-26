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
from custom_ros_action.msg import  InitDroneAction, MoveAgentAction
from custom_ros_msgs.msg import ProcessVar
from rospy.exceptions import ROSException, ROSSerializationException, ROSInterruptException

sys.path.append("../../")
try:
    from src.inference_filters.DecisionFilter import DecisionFilter
    from ros_atlas.utils.uav_utils import connect_uav
except ImportError as err:
    raise err

class PIDActionServer:
    """PIDActionServer
    Initializes SASs and handles requests from ActionClient.
    """
    def __init__(self, name) -> None:
        self.app_name = name
        self._uav = None
        
        # Initialize SimpleActionServers
        self._init_sas = SimpleActionServer('init_drone', InitDroneAction, execute_cb=self.execute_init_cb, auto_start=False)
        self._manual_sas = SimpleActionServer('manual_control', MoveAgentAction, execute_cb=self.execute_manual_cb, auto_start=False)
        self._pid_sas = SimpleActionServer('main', MoveAgentAction, execute_cb=self.execute_main_cb, auto_start=False)

        # Set attributes for PID
        self.setpoint_area = (20000, 100000)        # Case-specific, adjustable -- Lower and Upper bound for Forward & Backward Range-of-Motion
        self.setpoint_center = (480, 360)           # Case-specific, adjustable -- Lower and upper bound for xy rotation     
        self.pid_input = [0.1, 0.1, 0.1]            # Case-specific, adjustable -- Values for PID equation
        self._is_search = True                      # Flag -- updated in self.process_var_sub_cb

        self._x_err, self._y_err = 0, 0
        self._prev_x_err, self._prev_y_err = 0, 0

        # Initialize Subscriber & Publisher
        self.sub = rospy.Subscriber('/pid_fd/process_vars', ProcessVar, self.process_var_sub_cb, queue_size=1, buff_size=2**24)
        self.pub = rospy.Publisher("/tello/cam_data_raw", Image, queue_size=1)
        self.rate = rospy.Rate(10)
        rospy.loginfo(f"Subscriber & Publisher started. Ready for client.")


        # [Optional] Result filter - tune parameters
        self.inference_filter = DecisionFilter(fps=10, window=3)

        # start the SASs
        self._init_sas.start()
        self._manual_sas.start()
        self._pid_sas.start()
        rospy.loginfo(f"SASs started. Ready for client.")

    def pid(self, error, prev_error):
        """PID Output signal equation"""
        return self.pid_input[0]*error + self.pid_input[1]*(error-prev_error) + self.pid_input[2]*(error-prev_error)

    def process_var_sub_cb(self, msg):
        """@topic:/pid_fd/process_vars Subscriber callback.  
        @param:msg      -- Message from PostProcessor node  @type:ProcessVar
        
        Function
            updates bbox_area, bbox_center, current and previous xy errors    
        Return
            None
        """
        # If no area -> Search mode and set current xy error to be  the same as previous xy error
        if msg.area == 0:
            self._is_search = True
            self._x_err = self._prev_x_err
            self._y_err = self._prev_y_err
            return
        # otherwise -> Track mode and calculate the corresponding process variables
        self._is_search = False
        self._area = msg.area
        self._x_err = msg.cx - self.setpoint_center[0]
        self._y_err = self.setpoint_center[1] - msg.cy
        return
    
    def dfilter_sub_cb(self, msg):
        """Filtere-based Subscriber callback, alternative to self.process_var_sub_cb.
        NOTE: Alternative to self.process_var_sub_cb. Uses DecisionFilter (self.inference_filter) to filter out results
        + Should result in more stablity (less mode-switching)
        + But parameters of DecisionFilter needs to be tuned manually by hand to see best results (fps, window)
        """
        sample = None
        if msg.area != 0:
            sample = "Presence"
        filtered_result = self.inference_filter.sample(sample)      # either enqueue (and return a string) or return the majority result
        
        # Case: DecisionFilter is still collecting samples to determine the majority -> returns string "MODE_INFERENCE_SAMPLING"
        if filtered_result == "MODE_INFERENCE_SAMPLING":
            pass
        # Case: DecisionFilter returns the majority inference result (Presence) -> switch to tracking mode
        elif filtered_result == "Presence":
            self._is_search = False
            self._area = msg.area
            self._x_err = msg.cx - self.setpoint_center[0]       # rotational err
            self._y_err = self.setpoint_center[1] - msg.cy       # elevation err
        else:
            self._is_search = True
            self._x_err = self._prev_x_err
            self._y_err = self._prev_y_err

    def execute_init_cb(self, goal) -> None:
        """Callback for @SimpleActionServer:init_drone
        Function:
            Tries to connect to the Tello UAV and start the stream
        @param:goal     positional parameter for SimpleActionServer's callback methods
        Returns:
            None 
        """
        # Goal: Connect
        if goal.type == "connect":
            start_time = time.time()
            while self._uav is None:
                # Case: if preempt has not been requested by Client-side
                if self._init_sas.is_preempt_requested():
                    self._init_sas.set_preempted()

                # Check for how many tries are left
                if time.time() - start_time > 30:
                    rospy.loginfo("Time exceeded 30seconds, aborting action.")
                    self._init_sas.set_aborted()
                try:
                    self._uav = connect_uav()
                    if self._uav is not None:
                        self._uav.streamon()
                        rospy.loginfo("@init_cb: Tello Drone connection established.")
                        self._init_sas.set_succeeded()
                except Exception as err:
                    print(f"Trying again... {time.time()-start_time}s remaining.")
                    continue
        # Goal: Takeoff
        elif goal.type == "takeoff":
            self._uav.takeoff()
            self._init_sas.set_succeeded()
        # Goal: Land
        elif goal.type == "land":
            self._uav.land()
            self._init_sas.set_succeeded()

    def execute_manual_cb(self, goal):
        engage_manual = None
        while True:
            engage_manual = input('Engage manual control? (y/n)').lower()
            if engage_manual not in ['y', 'n']: 
                rospy.logwarn("Unknown input, enter either y or n")
                continue
            else: 
                break
        if engage_manual == 'n':
            rospy.loginfo("Skipping manual control, starting PIDTrack mode")
            self._manual_sas.set_succeeded()
        elif engage_manual == 'y':
            rospy.loginfo("Manual Control engaged:")
            rospy.loginfo("Listen Key: \n \
                            w a s d: move forward/left/back/right \n \
                            q e: rotate \n \
                            Arrow Up/Down: move up/down \
                            l: Land and terminate program \n \
                            k: Exit manual control and resume tracker")
            with Input(keynames='curses') as input_generator:
                for key in input_generator:
                    try:
                        if key == 'w':
                            self._uav.move_forward(30)
                        elif key == 'a':
                            self._uav.move_left(30)
                        elif key == 's':
                            self._uav.move_back(30)
                        elif key == 'd':
                            self._uav.move_right(30)
                        elif key == 'e':
                            self._uav.rotate_clockwise(30)
                        elif key == 'q':
                            self._uav.rotate_counter_clockwise(30)
                        elif key == 'KEY_UP':
                            self._uav.move_up(30)
                        elif key == 'KEY_DOWN':
                            self._uav.move_down(30)
                        elif key == 'l':
                            rospy.loginfo("Abort manual control, proceeding to land.")
                            self._manual_sas.set_aborted()
                        elif key=='k':
                            rospy.loginfo("Exiting manual control, proceeding to PIDTrack mode")
                            self._manual_sas.set_succeeded()
                        else:
                            rospy.logwarn("Unknown key: {}".format(key))

                    except Exception as err:
                        print("something wrong; key: " + key)
                        self._uav.land()
                        raise err
            else:



    def execute_search_cb(self, goal) -> None:
        """Class method for handling search mode
        Function:
            Searches for target by moving (rotate while hover) the drone
        Returns:
            None
        """
        rospy.loginfo('Searching...')
        # Implement search-function here
    
    def execute_track_cb(self, goal) -> None:
        """Class method for handling tracking mode
        Function:
            Executor and Actuator -- calculates velocity in xyz and yaw and move the drone based on the calculations
        Returns:
            None 
        """
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

        # uncomment below to execute Actuator - control drone's motion to converge to setpoint by sending rc control commands
        rospy.loginfo(f"[Tracking] Actuator command: {left_right_velocity}, {forward_backward_velocity}, {up_down_velocity}, {yaw_velocity}")
        self._uav.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)

    def execute_main_cb(self, goal):
        pub_counter = 0
        while not rospy.is_shutdown():
            image_data = self._uav.get_frame_read().frame
            if image_data is None:
                rospy.loginfo("Frame is None or return code error.")
                break
            # image_data = cv2.resize(image_data, (0,0), fx = 0.5, fy = 0.5)
            try:
                img_msg = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
                img_msg = CvBridge().cv2_to_imgmsg(img_msg, "rgb8")
                img_msg.header.stamp = rospy.Time.now()
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
                rospy.loginfo("Keyboard Interrupt, aborting PID state.")
                self._pid_sas.set_aborted()


if __name__ == '__main__':
    import os
    print(f"ActionServer pid: {os.getpid()}")
    rospy.init_node('pid_action_server')
    ActionServer = PIDActionServer('pid_tracker')
    rospy.spin()