import os
import cv2
import numpy as np
import sys
import time
import _thread
from importlib import import_module

from utils.uav_utils import  manual_control
from atlas_utils.presenteragent import presenter_channel
from atlas_utils.acl_image import AclImage

class LiveRunner:
    """
    Responsibile for starting stream, capturing frame, starting subprocess
    """
    def __init__(self, uav, params, selector):
        self.uav = uav
        self.selector = selector
        self.model_params = params["task"][self.selector.task][self.selector.model]
        self.uav_presenter_conf = params["presenter_server_conf"]

    def init_model_processor(self):
        # Initialize ModuleProcessor based on params[model]
        self._model_processor = import_module(f"model_processors.{self.selector.processor}")
        self._model_processor = getattr(self._model_processor, "ModelProcessor")
        self.model_processor = self._model_processor(self.model_params)

    def init_presenter_channel(self):
        chan = presenter_channel.open_channel(self.uav_presenter_conf)
        if chan is None:
            print("Open presenter channel failed")
            return

    def engage_manual_control(self):
        # start new thread for manual control
        try:
            _thread.start_new_thread(manual_control, (self.uav, ))
        except:
            print("Error: unable to start thread")

    def display_result(self):
        self.init_model_processor()
        self.uav.streamon()
        print("\n##################################################################################")
        print("Opening Presenter Server...")
        chan = presenter_channel.open_channel(self.uav_presenter_conf)
        if chan is None:
            print("Open presenter channel failed")
            return

        self.engage_manual_control()
        
        print("\n############################################################")
        print("Fetching UAV Livestream...")
        while True:
            ## Read one frame from stream ##
            frame_org = self.uav.get_frame_read().frame
            assert frame_org is not None, "Error: Tello video capture failed, frame is None"

            ## Model Prediction ##
            result_img = self.model_processor.predict(frame_org)
            # result_img, action = self.model_processor.predict(frame_org)

            """ Display inference results and send to presenter channel """
            _, jpeg_image = cv2.imencode('.jpg', result_img)
            jpeg_image = AclImage(jpeg_image, frame_org.shape[0], frame_org.shape[1], jpeg_image.size)
            chan.send_detection_data(frame_org.shape[0], frame_org.shape[1], jpeg_image, [])
