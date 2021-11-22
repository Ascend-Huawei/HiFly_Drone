import sys
import cv2
import numpy as np
import gc
import time

sys.path.append("../../../src")
sys.path.append("../../../src/lib")

from utils.tools import load_model_processor

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import argparse


def main():
    # load Model
    detector_name = "face_detection"
    model_processor, mp_info = load_model_processor(detector_name)
    mp = model_processor(mp_info)
    if detector_name == "yolov3":
        mp.tmp_save_fpath = "./tmp/tmp.jpg"
    
    # init node
    rospy.init_node('full_hifly')
    
    # define vars
    bridge = CvBridge()
    cap = cv2.VideoCapture("/home/HwHiAiUser/HiFly_Drone/data/20210809_141017.mp4")
    print(f"CameraPublisher - Default video read FPS={round(cap.get(cv2.CAP_PROP_FPS), 3)}")
    pub = rospy.Publisher("/hifly_base/results", Image, queue_size=1)
    pub_rate = rospy.Rate(30)
    counter = 0

    if not cap.isOpened(): print("Error opening video file")
    
    while not rospy.is_shutdown() and cap.isOpened():
        ret, image_data = cap.read()
        
        if image_data is None or not ret:
            print("Frame is None or return code error")
            return
        
        try:
            image_data = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)   # this is for loading video. image from drone might already be in correct color format
            result_img = mp.predict(image_data)
            image_data = bridge.cv2_to_imgmsg(result_img, encoding="rgb8")
            image_data.header.stamp = rospy.Time.now()

            print(f"[{counter}]: Publish reuult to topic: /hifly/results.")
            counter += 1
            pub.publish(image_data)
            pub_rate.sleep()

        except CvBridgeError as cvbridge_err:
                raise cvbridge_err
        except KeyboardInterrupt as exception:
            raise exception

    else:
        sys.exit()

if __name__ == "__main__":
    main()
