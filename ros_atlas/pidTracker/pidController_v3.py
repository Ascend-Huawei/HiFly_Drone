from curtsies import Input
import sys
import time
import rospy
from smach import StateMachine, CBState
import smach_ros
from actionlib import *
from actionlib_msgs.msg import *


sys.path.append("../../")
try:
    from ros_atlas.utils.uav_utils import connect_uav, manual_control
    from ros_atlas.base_nodes.CameraPublisher import CameraPublisher
    from custom_ros_action.msg import InitDroneGoal, InitDroneResult, InitDroneFeedback, InitDroneAction, MoveAgentAction, MoveAgentGoal
except ImportError as err:
    raise err

class PIDTrackServer:
    def __init__(self, name) -> None:
        self._action_name = name
        self._init_sas = SimpleActionServer('init_drone', InitDroneAction, execute_cb=self.execute_init_cb, auto_start=False)
        self._init_sas_result = InitDroneResult()

        self._manual_sas = SimpleActionServer('manual_control', MoveAgentAction, execute_cb=self.execute_manual_cb, auto_start=False)
        
        # can probably condense this into one SAS 
        self._inference_sas =  SimpleActionServer('infer', InferAction, execute_cb=self.execute_inference_cb, auto_start=False)
        # self._track_sas =  SimpleActionServer('pid_track', TrackAction, execute_cb=self.execute_track_cb, auto_start=False)

        self._uav = None

        self._init_sas.start()
        self._manual_sas.start()
        self._search_sas.start()

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
                        # self._uav.streamon()
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
        
        # 1. fetch frame from drone's stream -- and publish it to AclInference & subscribe to Postprocessing node
        rospy.loginfo("@inference_cb: ")
        camPub = CameraPublisher(self._uav)
        camPub.start_publish_live()


        # 2. preprocess, execute, postprocess results (bbox coordinates -- center and area)
        #   a. if no bbox area && center -> search mode
        #   b. if object is "presence" (have bbox and center) -> track mode


# http://wiki.ros.org/smach/Tutorials/Simple%20Action%20State
def main():

    rospy.init_node('pid_tracker')

    # Start an Action server
    server = PIDTrackServer('pid_tracker_server')

    rospy.loginfo(f"Node initialized, started PIDTrackServer: {server}")

    # create StateMachine
    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # Add states to the sm
    """
    "Inside a state -- there's a goal you want to achieve through certain actions. 
                        Goals have outcomes: succeeded (action attained goal), aborted (action failed), preempted (action interrupted)"
                        outcomes dictate what to do next.(which state to go to next)"
    
    For example:
        + state:TRIGGER_INIT    -- goal:initialize drone; action:connect_uav
        + state:TRIGGER_SEARCH  -- goal:search for target; action:rotation etc 
        + state:TRIGGER_TRACK -- goal:initialize drone; action:connect_uav. succeed=TRACK_STATIC
        + state:TRIGGER_TRACK_STATIC -- goal:initialize drone; action:connect_uav. succeed=TRACK_CB
        + state:TRIGGER_TRACK_CB -- goal:initialize drone; action:connect_uav. succeed=

    smach_ros.SimpleActionServer(name, Action, execute_cb, auto_start=True)
        + extends ActionServer with goal policy: only one goal is active status

    sm.StateMachine.add(label, state, transitions, remapping)
        - label (string) - The label of the state being added.

        - state - An instance of a class implementing the State interface.
            smach_ros.SimpleActionState(action_name, action_spec, goal, goal_cb, etc)
                + param::action_name(str)
                + param::action (actionlib action msg) -- action to attain goal (connect_uav)
                + param::goal(actionlib goal msg)  -- goal for action (Establish connection between uav)
                + param::goal_key(str)

        - transitions - A dictionary mapping state outcomes to other state labels or container outcomes.
        - remapping - A dictrionary mapping local userdata keys to userdata keys in the container.
    """
    with sm:

        # def goal_callback(userdata, default_goal):
        #     goal.goal = 2,
        #     return goal

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
                        transitions={'aborted': 'LAND', 'succeeded':'LAND'}
                        )

        StateMachine.add(label='LAND',
                        state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='land')),
                        transitions={'aborted': 'succeeded'},
                        remapping={'land_input', 'uav'}
                        )
        
    
    outcome = sm.execute()
    rospy.signal_shutdown('All Done.')
 

if __name__ == "__main__":
    main()
