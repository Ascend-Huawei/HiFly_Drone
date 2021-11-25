import sys
import logging

sys.path.append("../../")

def main():
    logging.basicConfig(filename='ROS_ATLAS_ENV.log', filemode='w', level=logging.DEBUG,
            format='[%(levelname)s]:%(message)s')
    test_rospy()
    test_cv_bridge()
    test_acl_resource()
    test_acl_image()
    logging.info("Testing complete - no exception encountered.")

def test_rospy():
    """Test rospy imports"""
    logging.info("#############################")
    logging.info("Test rospy")
    logging.info("#############################")
    try:
        import rospy
        logging.info("rospy import successful.")
    except Exception as e:
        logging.warning("Ran into exception when importing rospy, see below exception.")
        logging.error(e)
        logging.info("Exiting program...")
        sys.exit()

def test_cv_bridge():
    """Test for CvBridge missing libgcc_s.so.1 library - pthread_cancel error (multithreading with uav instance)"""
    logging.info("#############################")
    logging.info("Test CvBridge")
    logging.info("#############################")
    try:
        from cv_bridge import CvBridge, CvBridgeError
        from utils.uav_utils import connect_uav
        import ctypes
        libgcc_s = ctypes.CDLL('libgcc_s.so.1')
        uav = connect_uav()
        cvb = CvBridge()
        if uav is not None and cvb is not None:
            logging.info('CvBridge successfully instantiated with TelloUAV.')
    except Exception as e:
        loging.warning("Ran into exception in test_cv_bridge(), see below Exception.")
        logging.error(e)
        logging.info("Exiting program...")
        sys.exit()

def test_acl_resource():
    """Test AclResource with ROS"""
    logging.info("#############################")
    logging.info("Test AclResource")
    logging.info("#############################")
    try:    
        from atlas_utils.acl_resource import AclResource
        acl_resource = AclResource()
        acl_resource.init()
        logging.info("Successfully initialized AclResources.")
    except Exception as e:
        logging.warning('Ran into the following exception when trying to initialize AclResources')
        logging.error(e)
        logging.info("Exiting program...")
        sys.exit()

def test_acl_image():
    """Test AclImage (Pillow) dependencies issue"""
    logging.info("#############################")
    logging.info("Test AclResource")
    logging.info("#############################")
    try:
        from atlas_utils.acl_image import AclImage
        logging.info("AclImage -> Pillow import successful.")
    except Exception as e:
        logging.warning("Ran into exception when importing AclImage, see below error.")
        logging.error(e)
        logging.info("Exiting program...")
        sys.exit()

if __name__ == "__main__":
    main()



