import os
import rospy
from smach import StateMachine, State
import smach_ros
from custom_ros_action.msg import InitDroneAction, InitDroneGoal, InitDroneResult, InitDroneAction, MoveAgentAction, MoveAgentGoal


if __name__ == '__main__':
    rospy.init_node('pid_action_client')
    print(f"StateMachine Client pid: {os.getpid()}")


    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    with sm:
        StateMachine.add(label='CONNECT', 
                        state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='connect')),
                        transitions={'succeeded': 'PID'},
                        # transitions={'succeeded': 'TAKEOFF'},
        )
        # StateMachine.add(label='TAKEOFF', 
        #                 state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='takeoff')),
        #                 transitions={'succeeded': 'MANUAL'},
        #                 )
        # StateMachine.add(label='MANUAL',
        #                 state=smach_ros.SimpleActionState('manual_control', MoveAgentAction),
        #                 transitions={'aborted': 'LAND', 'succeeded':'PID'}
        #                 )
        StateMachine.add(label='PID',
                        state=smach_ros.SimpleActionState('main', MoveAgentAction),
                        transitions={'succeeded': 'PID', 'aborted':'aborted'}
                        # transitions={'succeeded': 'PID', 'aborted':'LAND'}
        )
        # StateMachine.add(label='LAND', 
        #                 state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='land')),
        #                 transitions={'succeeded': 'aborted'},
        #                 )

    # Execute SMACH plan
    outcome = sm.execute()
      