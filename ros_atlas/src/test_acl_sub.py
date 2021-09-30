import rospy
from std_msgs.msg import String


def chatter_callback(msg):
	rospy.loginfo(msg.data)

def listener():
	# initialize a node with init_node with name "listener" and unique ID
	rospy.init_node('listener', anonymous=True)
	# create subscriber object (TopicName, MessageType, CallbackFn)
	rospy.Subscriber("chatter", String, chatter_callback)
	# Start listening
	rospy.spin()

if __name__ == "__main__":
	listener() 
