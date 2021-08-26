from curtsies import Input
import numpy as np
import cv2
import sys
import time
import rospy
from smach import StateMachine, Concurrence, State
import smach_ros
from actionlib import *
from multiprocessing import Process

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from custom_ros_action.msg import InitDroneGoal, InitDroneResult, InitDroneAction, MoveAgentAction, MoveAgentGoal
from custom_ros_msgs.msg import ProcessVar
from rospy.exceptions import ROSException, ROSSerializationException, ROSInitException, ROSInterruptException

sys.path.append("../../")
try:
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
        """
        SimpleActionServers for Concurrent States
            + self._con_publish_sas (mode_switcher)       -- publish cam_data, and listens for postprocessed results -> update global flag 
            + self._con_motor_sas (motor_executor)      -- switch state into either: switch to SEARCH or TRACK depending on global flag
        """
        self._con_publish_sas =  SimpleActionServer('mode_switcher', InitDroneAction, execute_cb=self.execute_start_stream_cb, auto_start=False)
        self._con_search_sas =  SimpleActionServer('search_executor', MoveAgentAction, execute_cb=self.execute_search_cb, auto_start=False)
        self._con_track_sas =  SimpleActionServer('track_executor', MoveAgentAction, execute_cb=self.execute_track_cb, auto_start=False)
        # attributed for PID_is_search
        self.setpoint_area = (20000, 100000)        # Lower and Upper bound for Forward&Backward Range-of-Motion - can be adjusted
        self._is_search = True

        self._x_err, self._y_err = 0, 0
        self._prev_x_err, self._prev_y_err = 0, 0
        self.forward_backward_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0

        # initialize subscriber (listens to postprocess for process_var info) 
        # **Note -- this logic can be moved into a function, which can get invoked after certain state
        self.sub = rospy.Subscriber('/pid_fd/process_vars', ProcessVar, self.process_var_sub_cb, queue_size=1, buff_size=2**24)

        # start the SASs
        self._init_sas.start()
        self._con_publish_sas.start()
        self._con_search_sas.start()
        self._con_track_sas.start()
        rospy.loginfo(f"SASs started. Ready for client.")

    # PID equation
    def _pid(self, error, prev_error):
        """PID Output signal equation"""
        return self.pid[0]*error + self.pid[1]*(error-prev_error) + self.pid[2]*(error-prev_error)

    def process_var_sub_cb(self, msg):
        """execute_inference_cb's subscriber's callback function"""
        # rospy.loginfo("Inside sub_cb")
        if msg.area == 0:
            self._is_search = True   # indicates which mode to use
            self._x_err = self._prev_x_err
            self._y_err = self._prev_y_err
            return

        self._is_search = False                              # if area!=0 -> flip flag
        self._area = msg.area
        self._x_err = msg.cx - self.setpoint_center[0]       # rotational err
        self._y_err = self.setpoint_center[1] - msg.cy       # elevation err


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
                    rospy.loginfo("@init_cb: Tello Drone connection established.")
                    self._init_sas.set_succeeded()
            except Exception as err:
                print(f"Trying again... {now-start_time}s remaining.")
                raise err
    
    def execute_start_stream_cb(self, goal):               
        pub_counter = 0
        self._uav.streamon()
        cam_data_pub = rospy.Publisher("/tello/cam_data_raw", Image, queue_size=1)
        rate = rospy.Rate(30)
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
                cam_data_pub.publish(img_msg)
                # rospy.loginfo(f"@execute_start_stream_cb: Published {pub_counter} ImageMessage")
                pub_counter += 1
                rate.sleep()
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
        while self._is_search:
            rospy.loginfo(f"@execute_search: searching")
            # Rotate drone here
        self._con_motor_sas.set_aborted()
    
    def execute_track_cb(self, goal):
        """should only look at self._is_search"""
        while not self._is_search:
            rospy.loginfo(f"@execute_track: tracking")

            forward_backward_velocity = 0
            left_right_velocity = 0
            up_down_velocity = 0
            yaw_velocity = 0

            # Compensator: calcuate the amount adjustment needed in YAW axis and distance
            # Localization of ToI to the center x-axis - adjusts camera angle
            if self._x_err != 0:
                yaw_velocity = self._pid(self._x_err, self._prev_x_err)
                yaw_velocity = int(np.clip(yaw_velocity, -100, 100))

            # Localization of ToI to the center y-axis - adjust altitude 
            if self._y_err != 0:
                up_down_velocity = 3*self._pid(self._y_err, self._prev_y_err)
                up_down_velocity = int(np.clip(up_down_velocity, -50, 50))

            # Rectify distance between drone and target from bbox area: Adjusts forward and backward motion
            if self._area > self.setpoint_area[0] and self._area < self.setpoint_area[1]:
                forward_backward_velocity = 0
            elif self._area < self.setpoint_area[0]:
                forward_backward_velocity = 20
            elif self._area > self.setpoint_area[1]:
                forward_backward_velocity = -20

            # Saves run history in a list for serialization
            # if self.save_flight_hist:
            #     history = {
            #         "left_right_velocity": left_right_velocity,
            #         "forward_backward_velocity": forward_backward_velocity, 
            #         "up_down_velocity": up_down_velocity,
            #         "yaw_velocity": yaw_velocity,
            #         "x_err": x_err,
            #         "y_err": y_err,
            #         "pv_bbox_area": area,
            #         "pv_center": center
            #     }
            #     self.history.append(history)

            # Actuator - adjust drone's motion to converge to setpoint by sending rc control commands
            self._uav.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)
        self._con_motor_sas.set_aborted()


if __name__ == '__main__':
    rospy.init_node('pid_action_server')
    ActionServer = PIDActionServer('pid_tracker')