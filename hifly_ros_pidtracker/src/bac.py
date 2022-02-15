import os
import threading
import numpy as np
import rospy
from smach import StateMachine, State
import smach_ros
from actionlib import SimpleActionClient, GoalStatus
from custom_ros_msgs.msg import ProcessVar
from custom_ros_action.msg import InitDroneAction, InitDroneGoal, InitDroneResult, InitDroneAction, MoveAgentAction, MoveAgentGoal, MoveAgentResult

class PIDActionState(State):
    """Custom ACtionState for tracking
    Function:
        Listens for process variables from Subscriber, compute/plan action, and send computed action to Server for execute
    Returns:
        None
    """
    def __init__(self):
        super().__init__(outcomes=['succeeded', 'aborted', 'preempted'])
        self.x_err, self.y_err = 0, 0
        self.prev_x_err, self.prev_y_err = 0, 0

        self.setpoint_area = (20000, 100000)        # Case-specific, tunable -- Lower and Upper bound for Forward & Backward Range-of-Motion
        self.setpoint_center = (480, 360)           # Case-specific, tunable -- Lower and upper bound for xy rotation     
        self.pid_input = [0.1, 0.1, 0.1]            # Case-specific, tunable -- Values for PID equation
        self.area = 0

        self._goal_status = 0

        self.action_client = SimpleActionClient('main', MoveAgentAction)
        self.action_client.wait_for_server()

        # sub = rospy.Subscriber('/pid_fd/process_vars', ProcessVar, self.process_var_sub_cb, queue_size=1, buff_size=2**24)

        rospy.logwarn('Instantiated PIDActionState')

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
        rospy.logwarn('Triggered Subscriber callback')
        # If no area -> Search mode and set current xy error to be  the same as previous xy error
        if msg.area == 0:
            # self._is_search = True
            self.x_err = self.prev_x_err
            self.y_err = self.prev_y_err
            self.is_track = False
        else:
            # otherwise -> Track mode and calculate the corresponding process variables
            # self.is_search = False
            self.is_track = True
            self.search_count = 0                                # Reset search timer to 0
            self.area = msg.area
            self.x_err = msg.cx - self.setpoint_center[0]
            self.y_err = self.setpoint_center[1] - msg.cy

    def feedback_cb(self, *args):
        print(*args)
        rospy.loginfo(f'Feedback received:')

    def execute(self, userdata):
        # TODO: move to init?
        rospy.Subscriber('/pid_fd/process_vars', ProcessVar, self.process_var_sub_cb, queue_size=1, buff_size=2**24)
        rospy.sleep(0.5)

        # Compute goal
        # Edge-case -- sample majority is Presence but no detection on latest sample
        if self.x_err == float("-inf") or self.y_err == float("-inf"):
            self.x_err = self.prev_x_err
            self.y_err = self.prev_y_err
            rospy.loginfo(f'{self.x_err, self.y_err}')
            return

        forward_backward_velocity = 0
        left_right_velocity = 0
        up_down_velocity = 0
        yaw_velocity = 0

        # Compensator: calcuate the amount adjustment needed in YAW axis and distance
        # Localization of ToI to the center x-axis - adjusts camera angle
        if self.x_err != 0:
            yaw_velocity = self.pid(self.x_err, self.prev_x_err)
            yaw_velocity = int(np.clip(yaw_velocity, -50, 50))
            self.prev_x_err = self.x_err

        # Localization of ToI to the center y-axis - adjust altitude 
        if self.y_err != 0:
            up_down_velocity = 3*self.pid(self.y_err, self.prev_y_err)
            up_down_velocity = int(np.clip(up_down_velocity, -50, 50))
            self.prev_y_err = self.y_err

        # Rectify distance between drone and target from bbox area: Adjusts forward and backward motion
        if self.area > self.setpoint_area[0] and self.area < self.setpoint_area[1]:
            forward_backward_velocity = 0
        elif self.area < self.setpoint_area[0]:
            forward_backward_velocity = 20
        else:
            forward_backward_velocity = -20

        # Send goal to Server for execute - Goal: x,y,z, yaw
        goal = MoveAgentGoal()
        goal.move_x = forward_backward_velocity
        goal.move_y = left_right_velocity
        goal.move_z = up_down_velocity
        goal.move_yaw = yaw_velocity
        goal.is_track = self.is_track

        self.action_client.send_goal(goal, feedback_cb=self.feedback_cb) # Might need to replace this with `send_goal` and skip waiting (for faster response)
        rospy.loginfo('Sent MoveAgentGoal')
        self.action_client.wait_for_result()    # TODO: lookup on API to see if we can skip waiting for result
        ret = self.action_client.get_result()

        if ret:
            print('############################')
            print(ret)
            print('############################')
            return 'succeeded'
        else:
            print('############################')
            print(ret)
            print('############################')
            return 'aborted'




if __name__ == '__main__':
    rospy.init_node('pid_action_client')
    print(f"StateMachine Client pid: {os.getpid()}")


    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    # sm.userdata.prev_x_err = 0
    # sm.userdata.prev_y_err = 0
    # sm.userdata.x_err = 0
    # sm.userdata.y_err = 0

    with sm:
        # StateMachine.add(label='CONNECT', 
        #                 state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='connect')),
        #                 transitions={'succeeded': 'TAKEOFF', 'aborted':'aborted'},
        # )
        # StateMachine.add(label='TAKEOFF', 
        #                 state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='takeoff')),
        #                 transitions={'succeeded': 'MANUAL'},
        #                 )
        # StateMachine.add(label='MANUAL',
        #                 state=smach_ros.SimpleActionState('manual_control', MoveAgentAction),
        #                 transitions={'aborted': 'LAND', 'succeeded':'PID'}
        #                 )
        StateMachine.add(label='PID',
                        # state=smach_ros.SimpleActionState('main', MoveAgentAction),
                        state=PIDActionState(),
                        # transitions={'succeeded': 'PID', 'aborted':'aborted'}
                        transitions={'succeeded': 'PID', 'preempted':'LAND', 'aborted':'LAND'},
                        # remapping={
                        #     'prev_x_err': 'initial_x_err',
                        #     'prev_y_err': 'initial_y_err'}
        )
        StateMachine.add(label='LAND', 
                        state=smach_ros.SimpleActionState('init_drone', InitDroneAction, goal=InitDroneGoal(type='land')),
                        # transitions={'succeeded': 'TAKEOFF', 'aborted': 'aborted'},
                        transitions={'succeeded': 'aborted', 'aborted': 'aborted'},
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
