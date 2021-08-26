import os
import threading
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
                        transitions={'succeeded': 'TAKEOFF', 'aborted':'aborted'},
        )
        StateMachine.add(label='TAKEOFF', 
                        state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='takeoff')),
                        transitions={'succeeded': 'MANUAL'},
                        )
        StateMachine.add(label='MANUAL',
                        state=smach_ros.SimpleActionState('manual_control', MoveAgentAction),
                        transitions={'aborted': 'LAND', 'succeeded':'PID'}
                        )
        StateMachine.add(label='PID',
                        state=smach_ros.SimpleActionState('main', MoveAgentAction),
                        # transitions={'succeeded': 'PID', 'aborted':'aborted'}
                        transitions={'succeeded': 'PID', 'preempted':'LAND', 'aborted':'LAND'}
        )
        StateMachine.add(label='LAND', 
                        state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='land')),
                        transitions={'succeeded': 'TAKEOFF', 'aborted': 'aborted'},
                        )

    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    # Request the container to preempt
    sm.request_preempt()

    # Block until everything is preempted 
    # (you could do something more complicated to get the execution outcome if you want it)
    smach_thread.join()

    rospy.signal_shutdown("Manual shutdown")
