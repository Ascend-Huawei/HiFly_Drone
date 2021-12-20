from curtsies import Input
import cv2
import sys
import time
import rospy
from smach import StateMachine, Concurrence
import smach_ros
from actionlib import *
from actionlib_msgs.msg import *
from multiprocessing import Process

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from custom_ros_action.msg import InitDroneGoal, InitDroneResult, InitDroneAction, MoveAgentAction, MoveAgentGoal
from custom_ros_msgs.msg import PIDProcessVars 


sys.path.append("../../")
try:
    from ros_atlas.utils.uav_utils import connect_uav
    from ros_atlas.base_nodes.CameraPublisher import CameraPublisher
    from custom_ros_action.msg import InitDroneGoal, InitDroneResult, InitDroneAction, MoveAgentAction, MoveAgentGoal
except ImportError as err:
    raise err

class PIDTrackServer:
    def __init__(self, name) -> None:
        self._action_name = name
        
        # InitDrone Action server -- handles connection, takeoff logic
        self._init_sas = SimpleActionServer('init_drone', InitDroneAction, execute_cb=self.execute_init_cb, auto_start=False)
        self._init_sas_result = InitDroneResult()

        # ManualControl Action Server -- handles manual control
        self._manual_sas = SimpleActionServer('manual_control', MoveAgentAction, execute_cb=self.execute_manual_cb, auto_start=False)
        
        """
        SimpleActionServers for Concurrent States
        + self._con_infer_sas (mode_switcher)       -- calls execute_inference_cb to publish cam_data, and listens for postprocessed results -> update global flag 
        + self._con_motor_sas (motor_executor)      -- switch state into either: switch to SEARCH or TRACK depending on global flag
        """
        self._con_infer_sas =  SimpleActionServer('mode_switcher', execute_cb=self.execute_inference_cb, auto_start=False)
        self._con_motor_sas =  SimpleActionServer('motor_executor', MoveAgentAction, execute_cb=self.execute_rc_control_cb, auto_start=False)
        # attributed for PID
        self.is_search = False
        self._x_err, self._y_err = 0, 0
        self.setpoint_area = (20000, 100000)        # Lower and Upper bound for Forward&Backward Range-of-Motion - can be adjusted

        self._uav = None

        self._init_sas.start()
        self._manual_sas.start()
        self._inference_sas.start()

    def execute_init_cb(self, goal):
        """Optional callback that gets called in a separate thread whenever a new goal is received, 
        allowing users to have blocking callbacks. Adding an execute callback also deactivates the goalCallback."""
        if goal.type == "connect":
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
                    self._init_sas_result.result = False
                    self._init_sas.set_aborted(self._init_sas_result, "Total attempts: 5")
                try:
                    self._uav = connect_uav()
                    if self._uav is not None:
                        self.connect_start = time.time()
                        rospy.loginfo("@init_cb: Tello Drone connection established.")
                        self._init_sas_result.result = True
                        self._init_sas.set_succeeded(self._init_sas_result)
                except Exception as err:
                    print(f"Trying again... {now-start_time}s remaining.")
                    raise err

        elif goal.type == "takeoff":
            try:
                rospy.loginfo("@execute_init_cb: type==takeoff")
                self._uav.takeoff()
                self.takeoff_start = time.time()
                self._init_sas_result.result = True
                self._init_sas.set_succeeded(self._init_sas_result)
                return self._uav
            except Exception as err:
                raise err

        elif goal.type == "land":
            self._uav.land()
            self.land_start = time.time()
            rospy.loginfo(f"@land: time taken from takeoff to land: {round(self.land_start-self.takeoff_start, 3)}")
            self._init_sas_result.result = True
            self._init_sas.set_aborted(self._init_sas_result)

    def execute_manual_cb(self, goal):
        engage_manual = input('Engage manual control? (y/n): ').lower()
        print(engage_manual)
        if engage_manual == 'y':
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
                            self._uav.land()
                            rospy.loginfo("abort manual control")
                            self._manual_sas.set_aborted()
                        elif key=='k':
                            rospy.loginfo("exiting manual control and starting PIDTrack mode")
                            self._manual_sas.set_succeeded()

                        else:
                            print("Unknown key: {}".format(key))
                    except Exception as err:
                        print("something wrong; key: " + key)
                        self._uav.land()
                        raise err
        else:
            rospy.loginfo("Dismiss manual control, starting PIDTrack mode")
            self._manual_sas.set_aborted()

    def execute_inference_cb(self, goal):
        """
        So far -- it seems like the inference needs to be in the same state as search // track
        1. inference cannot be a separate state b/c search and track both continuous need it. 
        2. inference can be thought of as a core subroutine for either search and track.
        3. either way -- drone NEEDS to be in a state where it continuously streams data (inference of data can be done from other node and listen for result)

        """
        def start_cam_pub(uav):
            imgPub = CameraPublisher(uav=uav, fps=30)
            imgPub.start_publish_live()

        def sub_cb(self, msg):
            """execute_inference_cb's subscriber's callback function"""
            process_var_bbox_area, process_var_bbox_center = msg.process_vars
            if process_var_bbox_area == 0 and process_var_bbox_center is None:
                self.is_search = True   # indicates which mode to use
                # self._x_err = prev_x_err
                # self._y_err = prev_y_err
            else:
                self.is_search = False   # indicates which mode to use
                self._x_err = process_var_bbox_center[0] - self.setpoint_center[0]       # rotational err
                self._y_err = self.setpoint_center[1] - process_var_bbox_center[1]       # elevation err

        data_stream_process = Process(target=start_cam_pub, args=(self._uav))
        data_stream_process.start()
        data_stream_process.join()

        postprocess_topic = "/pid_fd/process_vars"
        postprocess_sub = rospy.Subscriber(postprocess_topic, PIDProcessVars, sub_cb, queue_size=1, buff_size=2**24)
        rospy.loginfo(f"@inference_callback: self.is_search={self._is_search}")

    def execute_rcontrol_cb(self, goal):
        """TODO: perhaps we can decompose this cb to: execute_search_cb & execute_track_cb"""
        last_mode = self._is_search
        while not self._is_search:
            # Execute TRACK state logic -- calculate the adjustment needed and send command to control the drone; continue
            continue
        while self._is_search:
            # Execute SEARCH state logic -- send command to control the drone (simple rotational for now); continue
            continue


# http://wiki.ros.org/smach/Tutorials/Simple%20Action%20State
def main():

    rospy.init_node('pid_tracker')

    # Start an Action server
    server = PIDTrackServer('pid_tracker_server')
    rospy.loginfo(f"Node initialized, started PIDTrackServer: {server}")

    # create StateMachine
    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # Add states to the sm
    with sm:
        StateMachine.add(label='CONNECT', 
                        state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='connect')),
                        transitions={'succeeded': 'TAKEOFF'},
                        )

        StateMachine.add(label='TAKEOFF', 
                        state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='takeoff')),
                        transitions={'succeeded': 'MANUAL'},
                        )
        # StateMachine.add(label='LAND',
        #                 state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='land')),
        #                 transitions={'aborted': 'succeeded'},
        #                 remapping={'land_input', 'uav'}
        #                 )

        StateMachine.add(label='MANUAL',
                        state=smach_ros.SimpleActionState('manual_control', MoveAgentAction),
                        transitions={'aborted': 'LAND', 'succeeded':'PID'}
                        )

        """
        Concurrence container with two children states
            @children: INFER        -- continuously publish image to Inference node & subscribe to Postprocess results
                                        + switch flag (i.e.: is_found=False/True) based on results from Postprocess Subscriber 
            @children: sm_nested    -- contains two mutually exclusive sub-states: SEARCH, TRACK

        """
        sm_concurrent = Concurrence(outcomes=['succeeded', 'aborted'],
                                    default_outcome='succeeded',
                                    outcome_map={'end': {'INFER':'aborted'}}
                                    )

        """
        Nested StateMachine -- children of Concurrence container
            @children -- SEARCH  
            @children -- TRACK
        """
        sm_nested = StateMachine(outcomes=['control_ok', 'control_not_ok'])
        with sm_nested:
            StateMachine.add('SEARCH',
                            state=smach_ros.SimpleActionState('motor_executor', MoveAgentAction, goal=MoveAgentGoal(type='search')),
                            transition={'aborted': 'TRACK'}
                            )
            StateMachine.add('TRACK',
                            state=smach_ros.SimpleActionState('motor_executor', MoveAgentAction, goal=MoveAgentGoal(type='track')),
                            transition={'aborted': 'SEARCH'}
                            )

        with sm_concurrent:
            Concurrence.add('INFER', state=smach_ros.SimpleActionState('mode_switcher', InferAction))
            Concurrence.add('CONTROL_MOTOR', state=sm_nested)

        
        # Connect Concurrent State to Top-level SM
        StateMachine.add(label='PID', 
                        state = sm_concurrent,
                        transitions={'succeeded':'PID', 'aborted':'LAND'})

                        
        StateMachine.add(label='LAND',
                        state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='land')),
                        transitions={'aborted': 'succeeded'},
                        remapping={'land_input', 'uav'}
                        )
        
    
    outcome = sm.execute()
    rospy.signal_shutdown('All Done.')
 

if __name__ == "__main__":
    main()
