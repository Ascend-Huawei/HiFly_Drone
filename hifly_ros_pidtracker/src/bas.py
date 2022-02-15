from curtsies import Input
import numpy as np
import cv2
import sys
import time
import threading
import rospy
import os

from actionlib import *
from queue import Queue

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from custom_ros_action.msg import  InitDroneAction, MoveAgentAction, MoveAgentFeedback, MoveAgentResult
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
    def __init__(self, useSim=False) -> None:

        # self._uav = SimulatorDrone() if useSim else connect_uav()
        if useSim:
            import PX4Controller
            px4 = PX4Controller()
            self._uav = px4
    
        # SimpleActionServers - Refer to system diagram https://github.com/jwillow19/HiFly_Drone/tree/new_ros_pid/ros_atlas/pidTracker
        self._init_sas = SimpleActionServer('init_drone', InitDroneAction, execute_cb=self.execute_init_cb, auto_start=False)
        self._manual_sas = SimpleActionServer('manual_control', MoveAgentAction, execute_cb=self.execute_manual_cb, auto_start=False)
        self._pid_sas = SimpleActionServer('main', MoveAgentAction, execute_cb=self.execute_main_cb, auto_start=False)

        self._search_count = 0
        self.last_track_control = None              # Pointer to last track command - prevent sending the same RC control to drone

        # Initialize Subscriber & Publisher
        # self.sub = rospy.Subscriber('/pid_fd/process_vars', ProcessVar, self.dfilter_sub_cb, queue_size=1, buff_size=2**24)
        # self.sub = rospy.Subscriber('/pid_fd/process_vars', ProcessVar, self.process_var_sub_cb, queue_size=1, buff_size=2**24)
        # self.pub = rospy.Publisher('/tello/cam_data_raw', Image, queue_size=1)
        self.rate = rospy.Rate(10)
        rospy.loginfo(f"Subscriber & Publisher started. Ready for client.")

        # [Optional] Result filter - require parameters tuning
        # self.inference_filter = DecisionFilter(fps=10, window=1)

        # Initialize SASs
        # self._init_sas.start()
        # self._manual_sas.start()
        self._pid_sas.start()
        rospy.loginfo("SASs started. Ready for client.")

        rospy.on_shutdown(self.shutdown)

    
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
        # Case: DecisionFilter returns the majority inference result (Presence) -> switch to Track mode
        elif filtered_result == "Presence":
            self._is_search = False
            self._search_count = 0                                # Reset search timer to 0
            self._area = msg.area
            self._x_err = msg.cx - self.setpoint_center[0]       # rotational err
            self._y_err = self.setpoint_center[1] - msg.cy       # elevation err
        # Case: Sample majority is None -> Switch to Search mode
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
                if time.time() - start_time > 30:
                    rospy.loginfo("Time exceeded 30seconds, aborting action.")
                    self._init_sas.set_aborted()
                try:
                    self._uav = connect_uav()
                    if self._uav is not None:
                        rospy.loginfo("Tello Drone connection established.")
                        self._uav.streamon()
                        if self._uav.get_battery() < 15:
                            rospy.warninfo("Cannot takeoff due to low battery, please replace the battery and try again.")
                            self._init_sas.set_aborted()

                        self._init_sas.set_succeeded()
                except Exception as err:
                    print(f"Trying again... {time.time()-start_time}s elapsed.")
                    continue

        # Goal: Takeoff
        elif goal.type == "takeoff":
            self._uav.takeoff()
            self._uav.send_rc_control(0, 0, 30, 0)
            self._init_sas.set_succeeded()

        # Goal: Land
        elif goal.type == "land":
            self._uav.land()
            self._search_count = 0          # Reset to 0 - in case re-run after LAND

            while True:
                exit_program = input("Drone has landed. Do you wish to run it again (y/n)? ").lower()
                
                if exit_program not in ['y', 'n']:
                    rospy.logwarn("Unsupported input, please enter either y or n")
                    continue
                break
            if exit_program == "n":
                rospy.loginfo("Terminating program.")
                self._init_sas.set_aborted()
            else:
                rospy.loginfo("Re-running program, returning to TAKEOFF state.")
                self._init_sas.set_succeeded()

    def execute_manual_cb(self, goal):
        engage_manual = None
        while True:
            try:
                engage_manual = input('Engage manual control (y/n)? ').lower()
                if engage_manual not in ['y', 'n']: 
                    rospy.logwarn("Unsupported input, enter either y or n")
                    continue
                else:
                    break
            except KeyboardInterrupt:
                self._manual_sas.set_aborted()

        if engage_manual == 'n':
            rospy.loginfo("Skipping manual control. Starting PIDTrack mode")
            self._manual_sas.set_succeeded()
        else:
            rospy.loginfo("Manual Control engaged")
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

    def execute_search_cb(self, goal) -> None:
        """Class method for handling search mode
        Function:
            Searches for target by moving (rotate while hover) the drone
        Returns:
            None
        """
        rospy.loginfo(f'\nSearch count: {self._search_count}')
        self._uav.send_rc_control(0,0,0, 20)
        self._search_count += 1

    def execute_track_cb(self, goal) -> None:
        """Class method for handling tracking mode
        Function:
            Executor and Actuator -- calculates velocity in xyz and yaw and move the drone based on the calculations
        Returns:
            None 
        """
        
        # # Edge-case -- sample majority is Presence but no detection on latest sample
        # if self._x_err == float("-inf") or self._y_err == float("-inf"):
        #     self._x_err = self._prev_x_err
        #     self._y_err = self._prev_y_err
        #     return

        forward_backward_velocity = goal.move_x
        left_right_velocity = goal.move_y
        up_down_velocity = goal.move_z
        yaw_velocity = goal.move_yaw
        print(f'velocities received: {forward_backward_velocity, left_right_velocity, up_down_velocity, yaw_velocity}')
        # if (left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity) != self.last_track_control:
        #     self._uav.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)
        #     self.last_track_control = (left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)      # Update pointer to store most recent track command

    def execute_main_cb(self, goal):

        # search_count ~ 150 equals to one rotation of drone
        if self._search_count > 100:
            rospy.logwarn('Search mode exceeded limit. Aborting PID state and transition to LAND state')
            self._pid_sas.set_aborted()

        feedback = MoveAgentFeedback(feedback='Goal result, in main-loop')
        self._pid_sas.publish_feedback(feedback)
        # image_data = self._uav.get_frame_read().frame
        # if image_data is None:
        #     rospy.logwarn("Frame is None or return code error.")
        #     self._pid_sas.set_aborted()
        
        # try:
            # img_msg = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
            # img_msg = CvBridge().cv2_to_imgmsg(img_msg, "rgb8")
            # img_msg.header.stamp = rospy.Time.now()
            # self.pub.publish(img_msg)
        # if self._is_search is None:
            # rospy.loginfo(f"Waiting for response from Processor")
            # self.rate.sleep()
        rospy.loginfo(goal)
        if not goal.is_track:
            rospy.loginfo('@execute_main_cb.search')
            self.execute_search_cb(goal)
            self.rate.sleep()

        rospy.loginfo('@execute_main_cb.track')
        self.execute_track_cb(goal)
        self.rate.sleep()

        result = MoveAgentResult(result=True)
        self._pid_sas.set_succeeded(result)

        # except (rospy.exceptions.ROSException, CvBridgeError, KeyboardInterrupt, ROSInterruptException):
        #     rospy.logerr("Exception encounterd. Aborting PID state.")
        #     self._pid_sas.set_aborted()
    
    def shutdown(self):
        self._pid_sas.set_aborted()
        

if __name__ == '__main__':
    print(f"ActionServer pid: {os.getpid()}")
    rospy.init_node('pid_action_server', disable_signals=True)
    ActionServer = PIDActionServer('pid_tracker')
    rospy.spin()