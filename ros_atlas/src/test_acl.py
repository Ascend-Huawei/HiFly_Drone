import sys
import rospy
from std_msgs.msg import String 

sys.path.append("../../src/lib")

from atlas_utils.acl_resource import AclResource
# from atlas_utils.acl_model import Model

def main():
    acl_resource = AclResource()
    acl_resource.init()

    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node("acl_resource", anonymous=True)
    rate = rospy.Rate(1) #1hz (1msg/sec)

    i=0
    while not rospy.is_shutdown():
        print(acl_resource)
        str_msg = "ACL Resource initialized" 
        rospy.loginfo(str_msg)
        pub.publish(str_msg)
        rate.sleep()
        i += 1


if __name__ == "__main__":
	main()

