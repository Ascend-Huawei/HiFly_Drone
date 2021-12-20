from curtsies import Input
import cv2
import sys
import time
import rospy
from smach import StateMachine, Concurrence, State
import smach_ros
from actionlib import *
# from actionlib_msgs.msg import *
from multiprocessing import Process

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from custom_ros_action.msg import InitDroneGoal, InitDroneResult, InitDroneAction, MoveAgentAction, MoveAgentGoal
from custom_ros_msgs.msg import ProcessVar
from rospy.exceptions import ROSException, ROSSerializationException, ROSInitException, ROSInterruptException

sys.path.append("../../")
try:
    from ros_atlas.utils.uav_utils import connect_uav
    from ros_atlas.base_nodes.CameraPublisher import CameraPublisher
    from custom_ros_action.msg import InitDroneGoal, InitDroneResult, InitDroneAction, MoveAgentAction, MoveAgentGoal
except ImportError as err:
    raise err

class Test(State):
    def __init__(self, name) -> None:
        self.app_name = name
        self._uav = None
        
        self._init_sas = SimpleActionServer('init_drone', InitDroneAction, execute_cb=self.execute_init_cb, auto_start=False)
        """
        SimpleActionServers for Concurrent States
        + self._con_infer_sas (mode_switcher)       -- calls execute_inference_cb to publish cam_data, and listens for postprocessed results -> update global flag 
        + self._con_motor_sas (motor_executor)      -- switch state into either: switch to SEARCH or TRACK depending on global flag
        """
        self._con_infer_sas =  SimpleActionServer('mode_switcher', InitDroneAction, execute_cb=self.execute_start_stream_cb, auto_start=False)
        # self._con_motor_sas =  SimpleActionServer('motor_executor', MoveAgentAction, execute_cb=self.execute_rcontrol_cb, auto_start=False)
        self._con_search_sas =  SimpleActionServer('search_executor', MoveAgentAction, execute_cb=self.execute_search_cb, auto_start=False)
        self._con_track_sas =  SimpleActionServer('track_executor', MoveAgentAction, execute_cb=self.execute_track_cb, auto_start=False)
        # attributed for PID_is_search
        self.setpoint_area = (20000, 100000)        # Lower and Upper bound for Forward&Backward Range-of-Motion - can be adjusted
        self._is_search = True

        # initialize subscriber (listens to postprocess for process_var info) 
        # **Note -- this logic can be moved into a function, which can get invoked after certain state
        self.sub = rospy.Subscriber('/pid_fd/process_vars', ProcessVar, self.process_var_sub_cb, queue_size=1, buff_size=2**24)

        # start the SASs
        self._init_sas.start()
        self._con_infer_sas.start()
        # self._con_motor_sas.start()
        self._con_search_sas.start()
        self._con_track_sas.start()

    def process_var_sub_cb(self, msg):
        """execute_inference_cb's subscriber's callback function"""
        # rospy.loginfo("Inside sub_cb")
        process_var_bbox_area = msg.area
        process_var_cx = msg.cx
        process_var_cy = msg.cy
        if process_var_bbox_area == 0:
            self._is_search = True   # indicates which mode to use
        else:
            self._is_search = False   # indicates which mode to use
            self._x_err = process_var_cx - self.setpoint_center[0]       # rotational err
            self._y_err = self.setpoint_center[1] - process_var_cy       # elevation err

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

            image_data = cv2.resize(image_data, (0,0), fx = 0.5, fy = 0.5)

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
        
    def execute_rcontrol_cb(self, goal):
        """TODO: perhaps we can decompose this cb to: execute_search_cb & execute_track_cb"""
        # Beware of inf-loop -- what happens if self._is_search gets flipped sometime inside this while loop?
        while not self._is_search:
            # rospy.loginfo(f"@execute_rcontrol_cb: is_Search={self._is_search}")
            # Execute TRACK state logic -- calculate the adjustment needed and send command to control the drone; continue
            pass
        while self._is_search:
            # rospy.loginfo(f"@execute_rcontrol_cb: is_Search={self._is_search}")
            # Execute SEARCH state logic -- send command to control the drone (simple rotational for now); continue
            pass
    
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
            # Rotate drone here
            pass
        self._con_motor_sas.set_aborted()





def main():
    rospy.init_node('pid_tracker')
    server = Test('pid_tracker_server')

    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # Open the container
    with sm:
        StateMachine.add(label='CONNECT', 
                        state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='connect')),
                        transitions={'succeeded': 'CON'},
                        )


        """Concurrent & Nested"""
        sm_concurrent = Concurrence(outcomes=['succeeded', 'aborted'],
                                    default_outcome='succeeded',
                                    outcome_map={'aborted': {'INFER':'aborted'}})

        sm_nested = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with sm_nested:
            StateMachine.add('SEARCH',
                            state=smach_ros.SimpleActionState('search_executor', MoveAgentAction),
                            transitions={'aborted': 'TRACK'}
                            )
            StateMachine.add('TRACK',
                            state=smach_ros.SimpleActionState('track_executor', MoveAgentAction),
                            transitions={'aborted': 'SEARCH'}
                            )
                                    
        # Open the container
        with sm_concurrent:
            Concurrence.add('INFER', state=smach_ros.SimpleActionState('mode_switcher', InitDroneAction))
            Concurrence.add('CONTROL_MOTOR', state=sm_nested)

        StateMachine.add('CON', sm_concurrent,
                               transitions={'succeeded':'CON',
                                            'aborted':'aborted'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
