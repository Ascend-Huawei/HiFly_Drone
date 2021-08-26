import rospy
from actionlib import SimpleActionClient

from custom_ros_action.msg import InitDroneAction, InitDroneGoal, MoveAgentAction, MoveAgentGoal

def feedback_cb(msg):
    """msg from Server"""
    rospy.loginfo(f"Message received: {msg}")

def call_manual_control_server(user_input):
    client = SimpleActionClient('manual_control', MoveAgentAction)
    client.wait_for_server()
    goal = MoveAgentGoal()
    if user_input == 'w':
        goal.move_forward(5)
    elif user_input == 'a':
        goal.move_left(5)
    elif user_input == 'd':
        goal.move_backward(5)
    elif user_input == 'd':
        goal.move_right(5)    

    client.send_goal(goal, feedback_cb=feedback_cb)
    client.wait_for_result()
    result = client.get_result()

    print(result)

if __name__ == '__main__':
    try:
        rospy.init_node('manual_client_node')

        control_input = input('WASD for xy movement')
        control_input = control_input.lower()

        valid_inputs = ['w', 'a', 's', 'd']
        if control_input in valid_inputs:
            final = call_manual_control_server(control_input)
            rospy.loginfo(final)
        else:
            rospy.loginfo('Invalid input. Try again.')

    except Exception as err:
        raise err
