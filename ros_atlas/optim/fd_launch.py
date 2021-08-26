import rospy
import roslaunch


rospy.init_node('tello_face_detection', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/haier/catkin_ws/src/testapi/launch/test_node.launch"])
launch.start()
rospy.loginfo("started")

rospy.sleep(3)
# 3 seconds later
launch.shutdown()