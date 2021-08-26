import sys
import time
import rospy
from smach import StateMachine, CBState
import smach_ros
from actionlib import *
from actionlib_msgs.msg import *

sys.path.append("../../")
try:
    from ros_atlas.utils.uav_utils import connect_uav
    from custom_ros_action.msg import InitDroneGoal, InitDroneResult, InitDroneFeedback, InitDroneAction, MoveAgentAction, MoveAgentGoal
except ImportError as err:
    raise err


class InitDroneServer:
    """
    @action::InitDrone
    @goal::connect, open_cam, takeoff
    @feedback
    @result
    
    """
    def __init__(self, name) -> None:
        self._action_name = name
        self._feedback = InitDroneFeedback()
        self._result = InitDroneResult()
        self._sas = SimpleActionServer(name, InitDroneAction, execute_cb=self.execute_init_cb, auto_start=False)
        self._sas.start()

        self._uav = None

    def execute_init_cb(self, goal):
        """Optional callback that gets called in a separate thread whenever a new goal is received, 
        allowing users to have blocking callbacks. Adding an execute callback also deactivates the goalCallback."""
        if goal.type == "connect":
            start_time = time.time()

            while self._uav is None:
                now = time.time()

                # Check if preempt has not been requested by Client-side
                if self._sas.is_preempt_requested():
                    rospy.loginfo(f"{self._action_name} Preempted")
                    self._sas.set_preempted()

                # Check for how many tries are left
                if now - start_time > 30:
                    rospy.loginfo(f"{self._action_name} Aborted")
                    self._result.result = False
                    self._sas.set_aborted(self._result, "Total attempts: 5")
                try:
                    self._feedback.feedback = 'Attempting to connect to drone...'
                    self._uav = connect_uav()
                    if self._uav is not None:
                        rospy.loginfo("######$$#%#%@%#%connected")
                        self._uav.streamon()
                        self._feedback.feedback = 'Connect to Tello OK'
                        self._result.result = True
                        self._sas.set_succeeded(self._result)
                except Exception as err:
                    print(f"Trying again... {now-start_time}s remaining.")
                    continue

        elif goal.type == "takeoff":
            # self._sas.publish_feedback(self._feedback)
            try:
                rospy.loginfo("@execute_init_cb: type==takeoff")
                self._uav.takeoff()
                self._result.result = True
                self._sas.set_succeeded(self._result)
                return self._uav
            except Exception as err:
                raise err
                self._sas.set_aborted(self._result.result)

        elif goal.type == "land":
            rospy.loginfo(userdata)

            self._uav.land()
            rospy.loginfo("@execute_init_cb: type==land")
            time.sleep(3)
            self._result.result = True
            self._sas.set_aborted(self._result)

    
class ManualControlServer:
    def __init__(self, name) -> None:
        self._action_name = name
        self._sas = SimpleActionServer('manual_control', MoveAgentAction, execute_cb=self.execute_manual_cb, auto_start=False)
        self._sas.start()

    def execute_manual_cb(self, userdata, goal):
        engage_manual = input('Engage manual control: y/n').lower()
        if engage_manual == 'y':
            rospy.loginfo('Manul control mode. Press WASD for movement, q to exit.')
            exit_manual = False
            valid_commands = ['w', 'a', 's', 'd', 'q']
            while not exit_manual:
                user_input = input('Enter command: ').lower()
                if user_input not in valid_commands:
                    rospy.loginfo('invalid key try again')
                    continue

                if user_input == 'q':
                    rospy.loginfo('Exit key prompted. Aborting manual control mode.')
                    self._sas.set_aborted()
                
                elif user_input == 'w':
                    rospy.loginfo('Move forward.')
                    continue




        rospy.loginfo(f"@execute_manual_cb: {goal}")
        
        self._sas.set_succeeded()
            
        

# http://wiki.ros.org/smach/Tutorials/Simple%20Action%20State
def main():

    rospy.init_node('pid_track')

    # Start an Action server
    init_drone_server = InitDroneServer('init_drone')
    manual_control_server = ManualControlServer('manual_control')

    # rospy.loginfo(f"Node initialized, started InitDroneServer: {server}")

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

        # def goal_callback(userdata, default_goal):MoveAgentAction
        #     goal.goal = 2,
        #     return goal

        StateMachine.add(label='TRIGGER_INIT', 
                        state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='connect')),
                        transitions={'succeeded': 'TAKEOFF'},
                        remapping=None
                        )

        StateMachine.add(label='TAKEOFF', 
                        state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='takeoff')),
                        # transitions={'succeeded': 'MANUAL'},
                        transitions={'succeeded': 'LAND'},
                        remapping={'takeoff_output': 'uav'}
                        )

        StateMachine.add(label='LAND',
                        state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='land')),
                        transitions={'aborted': 'succeeded'},
                        remapping={'land_input', 'uav'}
                        )

        # StateMachine.add(label='MANUAL',
        #                 state=smach_ros.SimpleActionState('manual_control', MoveAgentAction, goal=MoveAgentGoal),
        #                 transitions={'aborted': 'LAND', 'succeeded':'succeeded'},
        #                 remapping={'manual_input': 'uav'}
        #                 )

        # StateMachine.add(label='LAND',
        #                 state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='land')),
        #                 transitions={'aborted': 'succeeded'},
        #                 remapping={'land_input': 'uav'}
        #                 )
        
    
    outcome = sm.execute()
    rospy.signal_shutdown('All Done.')
 

if __name__ == "__main__":

    uav = connect_uav()
    getattr(uav, 'takeoff')()