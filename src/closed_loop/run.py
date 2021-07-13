import concurrent.futures
import time
import sys
import os
import cv2
import logging
import numpy as np
from importlib import import_module
from abc import abstractmethod

sys.path.append("..")
sys.path.append("../lib")

from utils.uav_utils import connect_uav 
from utils.params import params
from atlas_utils.presenteragent import presenter_channel
from atlas_utils.acl_image import AclImage


## Parameters ##
SRC_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PRESENTER_SERVER_CONF = os.path.join(SRC_PATH, "uav_presenter_server.conf")
prerecord = "../../data/handGesture.avi"



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

"""
:class: TelloPIDController - Base class 
    Has: Compensator, Setpoint, Actuator and Process Variable
    :input:
        + uav Obejct - for drone interaction 
        + detector ModelProcessor Object - for inference feedback 
    
    :components:
        + bbox_compensator - internal method for calculating distance b/w detected bbox of ToI and central point
        + Set-point        - pre-set attribute based on Tello data stream dimensions
        + Actuator         - internal method to stabilize droen and track ToI based on bbox_compensator's output
                        I.e.: compensator says bbox is far to the right, actuator rotate camera to the right to adjust

        + how frequent? 
        + stream on another thread?
        + Accepts a connected TelloUAV - takeoff and streamon;
            - once stream is stablized run OD, 

:class: TelloPIDController - children class
    + Model specific - override method to extract inference output
    + add :param: uav back to TelloPIDController 

"""
class TelloPIDController:
    detectors = params["task"]["object_detection"]

    def __init__(self, pid,):
        # self.uav = tello_uav
        self.pid = pid
        self.setpoint = None
        self.history = []

    @staticmethod
    def _load_mp(detector_name):
        """Internal method for children class to load predestined MP"""
        model_info = TelloPIDController.detectors[detector_name]
        processor = model_info["model_processor"]
        MP = import_module(f"model_processors.{processor}")
        MP = getattr(MP, "ModelProcessor")
        return MP(model_info)

    def init_Tello(self):
        """
        Initiate closed-loop tracking sequence, drone takeoff and parallelize streaming and control
        Returns
            None
        """
        self.uav.left_right_velocity = 0
        self.uav.forward_backward_velocity = 0
        self.uav.up_down_velocity = 0
        self.uav.yaw_velocity = 0
        print(self.uav.get_battery())
        self.uav.streamoff()
        self.uav.streamon()
        return True

    def fetch_frame(self, width, height):
        frame = self.uav.get_frame_read().frame
        if self.setpoint is None:
            cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
            self.setpoint = (cx, cy)
        return frame

    def _get_feedback(self, frame):
        """Obtains feedback (inference result) from model. 
        Preprocess and execute the model using ModelProcessor.  
        Parameter:
            frame - input frame for inference
        """
        if self.setpoint is None:
            cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
            self.setpoint = (cx, cy)
        preprocessed = self.model_processor.preprocess(frame)
        infer_output = self.model_processor.model.execute([preprocessed, self.model_processor._image_info])
        return infer_output

    @abstractmethod
    def _unpack_feedback(self, inference_info, frame):
        pass

    @abstractmethod
    def track(self, inference_info):
        pass


class FaceTracker(TelloPIDController):
    def __init__(self, pid):
        super().__init__(pid)
        self.model_processor = self._load_mp("face_detection")

class HandTracker(TelloPIDController):
    def __init__(self, pid):
        super().__init__(pid)
        self.model_processor = self._load_mp("hand_detection")

class ObjectTracker(TelloPIDController):
    def __init__(self, pid):
        super().__init__(pid)
        self.model_processor = self._load_mp("yolov3")

    def _get_feedback(self, frame):
        """Obtains feedback (inference result) from model. 
        Preprocess and execute the model using ModelProcessor.  
        Parameter:
            frame - input frame for inference
        """
        if self.setpoint is None:
            cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
            self.setpoint = (cx, cy)
        preprocessed = self.model_processor.preprocess(frame)
        infer_output = self.model_processor.model.execute([preprocessed, self.model_processor._image_info])
        return infer_output
        
    def _unpack_feedback(self, infer_output, frame, toi="person"):
        """ Extract Process Variables from model's inference output info of input frame. The largest bbox of the same ToI label will be marked as ToI
        Parameter:
            infer_output - model's inference result from executing model on a frame
            frame        - input frame for inference  
            toi          - Target-of-Interest 
        Returns
            process_var_center  - Process Variable - ToI's bbox center
            process_var_bbox    - Process Variable - ToI's bbox area
            result_img   - inference result superimposed on original frame
        """
        box_num = infer_output[1][0, 0]
        box_info = infer_output[0].flatten()
        scale = max(frame.shape[1] / self.model_processor._model_width, frame.shape[0] / self.model_processor._model_height)
        
        # Iterate the detected boxes and look for label that matches ToI with the largest bbox area
        process_var_bbox_area = 0
        process_var_bbox_center = None

        for n in range(int(box_num)):
            ids = int(box_info[5 * int(box_num) + n])
            label = self.model_processor.labels[ids]

            if label == toi:
                score = box_info[4 * int(box_num)+n]
                top_left_x = int(box_info[0 * int(box_num)+n] * scale)
                top_left_y = int(box_info[1 * int(box_num)+n] * scale)
                bottom_right_x = int(box_info[2 * int(box_num) + n] * scale)
                bottom_right_y = int(box_info[3 * int(box_num) + n] * scale)
                cx = (top_left_x + bottom_right_x) // 2
                cy = (top_left_y + bottom_right_y) // 2
                center = (cx, cy)
                area = (bottom_right_x - top_left_x) * (bottom_right_y - top_left_y)

                if area > process_var_bbox_area:
                    process_var_bbox_area = area
                    process_var_bbox_center = center

                    cv2.rectangle(frame, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (0,0,255), 2)
                    cv2.circle(frame, center, 1, (0,0,255), -1)
        
        return frame, (process_var_bbox_area, process_var_bbox_center)

    def _pid_controller(self, uav, process_vars, prev_x_err, prev_y_err):
        """Closed-Loop PID ObjTracker (Compensator + Actuator)
        Error is xy error between frame center and process_var_bbox_center
        
        Calculates the Error value from Process variables and compute the require adjustment for the drone 
        Process Variable Area - for calculating the distance between drone and ToI (if it is over 80% of frame, then drone needs to move back)
                    Info obtained from inference bbox
                    + forward and backward motion of drone
        Process Variable center - for calculating how much to adjust the camera angle and altitude of drone
                    x_err: angle rotation
                    y_err: elevation to eye-level
        
        Analysis 1: 
            Drone video stream input frame = ()
        """
        area = process_vars[0]
        center = process_vars[1]
        
        x_err = process_vars[1][0] - self.setpoint[0]       # rotational err
        y_err = process_vars[1][1] - self.setpoint[1]       # elevation err

        # CHECK: magnitude of x_err and y_err; then determine an appropriate range to adjust
        print(x_err, y_err)
        # CHECK: magnitude of bbox of person from drone's feed; then determine a better rom_fb
        print(area)

        self.rom_fb = [10000, 17000]  # lower and upper bound of Forward&Backward Range-of-Motion
        
        # Velocity signals to send to the drone
        forward_backward_velocity = 0
        left_right_velocity = 0
        up_down_velocity = 0
        yaw_velocity = 0

        """Compensator: calcuate the amount adjustment needed"""
    
        # Localization of ToI to the center - adjusts angle
        if x_err != 0:
            yaw_velocity = pid[0]*x_err + pid[1]*(x_err - prev_x_err)
            yaw_velocity = int(np.clip(yaw_velocity, -100, 100))

        # Localization of ToI to be at eye level - adjust altitude 
        if y_err not in range(-10, 10):
            up_down_velocity = pid[0]y_err + pid[1]*(y_err - prev_y_err)
            up_down_velocity = int(np.clip(up_down_velocity, -100, 100))

        # Stablization: forward and backward motion
        if area > self.rom_fb[0] and area < self.rom_fb[1]:
            forward_backward_velocity = 0
        elif area < self.rom_fb[0]:
            forward_backward_velocity = 20
        elif area > self.rom_fb[1]:
            forward_backward_velocity = -20

        """Actuator - adjust drone's motion to converge to setpoint"""
        uav.send_rc_control(forward_backward_velocity, left_right_velocity, up_down_velocity, yaw_velocity)

        history = {
            "forward_backward_velocity": forward_backward_velocity, 
            "left_right_velocity": left_right_velocity,
            "up_down_velocity": up_down_velocity,
            "yaw_velocity": yaw_velocity,
            "x_err": x_err,
            "y_err": y_err
        }
        self.history.append(history)

        # NOTE: not required to return x,y error for next step - x, y error can be obtained from inference center and setpoint
        return x_err, y_err

        def track(self, uav, frame, prev_x_err, prev_y_err, toi="person"):
            infer_output = self._get_feedback(frame,)
            result_img, process_vars = self._unpack_feedback(infer_output, frame, toi)
            x_err, y_err = self._pid_controller(uav, process_vars, prev_x_err, prev_y_err)

    


pid = [0.4, 0.4, 0] 
obj_tracker = ObjectTracker(pid)

cap = cv2.VideoCapture(prerecord)

chan = presenter_channel.open_channel(PRESENTER_SERVER_CONF)
if chan is None:
    raise Exception("Open presenter channel failed")
    
while cap.isOpened():
    _, frame_org = cap.read()
    cv2.waitKey(10)
    if frame_org is None: break

    result_img, process_vars = obj_tracker.track(frame_org)
    print(process_vars)

    _, jpeg_image = cv2.imencode('.jpg', result_img)
    jpeg_image = AclImage(jpeg_image, frame_org.shape[0], frame_org.shape[1], jpeg_image.size)
    chan.send_detection_data(frame_org.shape[0], frame_org.shape[1], jpeg_image, [])

cap.release()

# Isolate - test single image



# For drone stream
# while True:
#     frame_org = uav.get_frame_read().frame
#     result_img = ModelProcessor.predict(frame_org)

#     """ Display inference results and send to presenter channel """
#     _, jpeg_image = cv2.imencode('.jpg', result_img)
#     jpeg_image = AclImage(jpeg_image, frame_org.shape[0], frame_org.shape[1], jpeg_image.size)
#     chan.send_detection_data(frame_org.shape[0], frame_org.shape[1], jpeg_image, [])
