import sys
import time
import rospy

import smach
from smach import StateMachine, CBState
from smach import user_data

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
            

@smach.cb_interface(input_keys=[], output_keys=['uav'], outcomes=['finished', 'failed'])
def connect_cb(userdata):
    rospy.loginfo('connecting to drone')
    start_time = time.time()
    rospy.loginfo(userdata)
    uav = None
    while uav is None:
        if time.time() - start_time > 30:
            try:
                rospy.loginfo('Trying to connect')
                uav = connect_uav()
                if uav is not None:
                    uav.streamon()
                    rospy.loginfo("Connected drone. Stream on done.")
                    userdata.uav = uav
                    return 'finished'
            except Exception as err:
                print(f"Trying again... {time.time()-start_time}s remaining.")
                raise err
                # continue
    return 'failed'

@smach.cb_interface(input_keys=['uav'], output_keys=[], outcomes=['finished', 'failed'])
def takeoff_cb(userdata):
    rospy.loginfo('takeoff')
    try:
        rospy.loginfo(type(userdata.uav))
        userdata.uav.takeoff()
        return 'finished'
    except Exception as err:
        raise err
    # if userdata.uav is not None:
    #     try:
    #         userdata.uav.takeoff()
    #         return 'finished'
    #     except Exception as err:
    #         raise err

if __name__ == '__main__':
    rospy.init_node('pid_state_machine')

    sm = StateMachine(outcomes=['done'])
    with sm:
        StateMachine.add('CONNECT', CBState(connect_cb), 
                        transitions={'finished': 'TAKEOFF', 'failed': 'done'})
        StateMachine.add('TAKEOFF', CBState(takeoff_cb), 
                        transitions={'finished': 'done', 'failed': 'done'})
    sm.execute()