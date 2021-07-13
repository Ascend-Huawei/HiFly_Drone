import concurrent.futures
import time
import sys
import os
import cv2
import logging
import numpy as np
from importlib import import_module

sys.path.append("..")
sys.path.append("../lib")

from utils.uav_utils import connect_uav 
from utils.params import params
from atlas_utils.presenteragent import presenter_channel


SRC_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PRESENTER_SERVER_CONF = os.path.join(SRC_PATH, "uav_presenter_server.conf")

# uav = connect_uav()

chan = presenter_channel.open_channel(PRESENTER_SERVER_CONF)
if chan is None:
    raise Exception("Open presenter channel failed")
    
# uav.streamon()

"""
Closed-Loop Detection + Tracking System
Control relies on feedback from in a closed-loop manner - enable drone to automatically adjust itself without user intervention to detect and track an
object of interest

+ Visual Servoing with Robotics
+ Implementation of PID of object detection

:class:
    DetectorFactory - purely returns the detection results (feedback) given an input frame - swappable detection models
                    - spawns different Detector class based on input parameter
                    - based on params.py to return a fully initialized object
    PIDController  - using proportional integral derivative to enable continuous modulated control
        1. modify to return the coordinates of detected hand - assuming only 1 object-of-interest, we only grab the result from the bbox with highest confidence

"""

class DetectorFactory:

    _detectors = params["task"]["object_detection"]

    @classmethod
    def from_model(cls, detection_model):
        model_info = cls._detectors[detection_model]
        processor = model_info["model_processor"]
        MP = import_module(f"model_processors.{processor}")
        MP = getattr(MP, "ModelProcessor")
        return MP(model_info)

class PIDController:
    def __init__(self, detector):
        self.detector = detector
    
    def get_feedback(self, frame):
        self._model_feedback = self.detector.predict(frame)

    # def _extract_feedback(self, feedback):
    #     """Deconstructs feedback from model to obtain relevant info"""
    #     num_detections = feedback[0][0].astype(np.int)
    #     scores = feedback[2]
    #     boxes = feedback[3]
    #     bbox_num = 0
        
    #     # loop through all the detections and get the confidence and bbox coordinates
    #     for i in range(num_detections):
    #         det_conf = scores[0, i]
    #         det_ymin = boxes[0, i, 0]
    #         det_xmin = boxes[0, i, 1]
    #         det_ymax = boxes[0, i, 2]
    #         det_xmax = boxes[0, i, 3]

    #         bbox_width = det_xmax - det_xmin
    #         bbox_height = det_ymax - det_ymin
    #         # the detection confidence and bbox dimensions must be greater than a minimum value to be a valid detection
    #         if threshold <= det_conf and 1 >= det_conf and bbox_width > 0 and bbox_height > 0:
    #             bbox_num += 1
    #             xmin = int(round(det_xmin * image.shape[1]))
    #             ymin = int(round(det_ymin * image.shape[0]))
    #             xmax = int(round(det_xmax * image.shape[1]))
    #             ymax = int(round(det_ymax * image.shape[0]))
                
    #         else:
    #             continue


factory = DetectorFactory()
ModelProcessor = factory.from_model("hand_detection")


while True:
    frame_org = uav.get_frame_read().frame
    result_img = ModelProcessor.predict(frame_org)

    """ Display inference results and send to presenter channel """
    _, jpeg_image = cv2.imencode('.jpg', result_img)
    jpeg_image = AclImage(jpeg_image, frame_org.shape[0], frame_org.shape[1], jpeg_image.size)
    chan.send_detection_data(frame_org.shape[0], frame_org.shape[1], jpeg_image, [])
