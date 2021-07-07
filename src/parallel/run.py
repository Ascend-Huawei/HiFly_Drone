"""
Sandbox for testing parallel inference (Depth Estimation + Object Detection)

Navie implementation:
0. Spawn ModelProcessor instances
1. Thread grabs frame - store to queue for callback afterwards, and send frame to 2 models
2. Once inference is done, send signal to dequeue
3. Result fusion and send to presenter server
"""
import concurrent.futures
from multiprocessing import Process, Queue, Pool
import time
import sys
import os
import cv2
from functools import partial
import logging
import numpy as np
from PIL import Image


sys.path.append("..")
sys.path.append("../lib")

from utils.params import params
# from model_processors.IndoorDepthProcessor import ModelProcessor as DEModelProcessor
from model_processors.ObjectDetectionProcessor import ModelProcessor as ODModelProcessor
from model_processors.HandDetectionProcessor import ModelProcessor as HDModelProcessor
from model_processors.FaceDetectionProcessor import ModelProcessor as FDModelProcessor


"""
1. Instaniate Models then do inference in parallel
    - put submit in queue 
2. Wait until both processes are completed, grab the result and pass to fusion

"""

def initializer(initargs):
    # verify if global vars are initialized in this processor and their values
    if "inited" not in globals():
        print(f"Initializing global flag on processor {os.getpid()}...")
        global inited, mp
        inited, mp = False, None

    # logging.basicConfig(filename='parallel.log', level=logging.DEBUG)
    model_name, frame = initargs
    if not inited:
        print("ModuleProcessor initialization...")
        if model_name == "hand_detection":
            mp = HDModelProcessor(params["task"]["object_detection"]["hand_detection"])
            logging.info(f"HandDetectionMP Initialized.\nParent Process: {os.getppid()}\nProcess ID: {os.getpid()}")
        elif model_name == "object_detection":
            mp = ODModelProcessor(params["task"]["object_detection"]["yolov3"])
            logging.info(f"ObjectDetectionMP Initialized.\nParent Process: {os.getppid()}\nProcess ID: {os.getpid()}")
        elif model_name == "face_detection":
            mp = FDModelProcessor(params["task"]["object_detection"]["face_detection"])
            logging.info(f"FaceDetectionMP Initialized.\nParent Process: {os.getppid()}\nProcess ID: {os.getpid()}")
        else:
            pass
        inited = True
    logging.info(f"MP {mp} already initialized in Processor {os.getpid()}, prepare for inferece...")
    assert mp is not None, f"ModuleProcessor on {os.getpid()} is None. Try again."
    return mp.predict(frame), model_name


class ResultHandler:
    """
    Accept parentID (MainProcessor iD) and models (list) and handles inference results
    TODO: image fusion, pass onto PresenterServer
    """
    data_dir = "../../data/parallel"
    def __init__(self, parentID, models):
        self.pid = parentID
        self.models = models
        self._get_save_path()

    def _output_tracker(self, model_name):
        """
        Set attribute to track the frame count for each model and returns the name of the latest frame to be used for saving 
        """
        attr = model_name + "_frame_count"
        try:
            frame_count = getattr(self, attr)
            setattr(self, attr, frame_count + 1)
            return str(frame_count) + ".png"
        except AttributeError:
            # no attribute - initialize as 0 and return
            setattr(self, attr, 0)
            return str(getattr(self, attr)) + "png"

    def _get_save_path(self):
        self.out_dir = []
        for model in self.models:
            output_dir = os.path.join(self.data_dir, model)
            if output_dir not in self.out_dir:
                self.out_dir.append(output_dir)
                if not os.path.exists(output_dir):
                    os.mkdir(output_dir)
                    print("Created output path: ", output_dir)
                else:
                    print(f"Pre-exist files located in {output_dir}, removing files...")
                    for f in os.listdir(output_dir):
                        os.remove(os.path.join(output_dir, f))

    @staticmethod
    def _is_pil_image(img):
        return isinstance(img, Image.Image)

    @staticmethod
    def _is_numpy_image(img):
        return isinstance(img, np.ndarray) and (img.ndim in [2,3])

    def handle(self, future):
        """
        Unpack results and save image to the corresponding directory - model_name is guarantee to be in self.out_dir
        :param: 
            result - FutureObject of type Typle(inference_output, model_name)
        """
        inference_res, model_name = future.result()
        if not (self._is_pil_image(inference_res) or self._is_numpy_image(inference_res)):
            raise TypeError(f"Inference result should be PIL or ndarray. Got {type(inference_res)} instead")
        
        save_dir = os.path.join(self.data_dir, model_name)
        out_name = self._output_tracker(model_name)
        if isinstance(inference_res, np.ndarray):
            cv2.imwrite(os.path.join(save_dir, out_name), inference_res)


def kill_all(e):
    e.shutdown(wait=True, cancel_futures=False)


## Parameters ##
MAX_WORKERS = 3
models = ["face_detection", "object_detection",]

# A pre-recorded video - need to change this to something else if someone else is using it
cap = cv2.VideoCapture("../../data/handGesture.avi")

res_handler = ResultHandler(os.getpid(), models)


"""
    Using ProcessPoolExecutor to submit asynchronously executions of tasks (initializer). Will set global flag if no globals are set.
    Then initialize the corresponding ModelProcessor based on global flag ONCE. Future calls to the target function will utilize the pre-initialize
    ModelProcessor to make inference in parallel.

    FutureObject encapsulates the asynchronous execution of a callable - returns a tuple if callable (initializer) is ran successfully without raising 
    an Exception.
"""
logging.basicConfig(filename='exec.log', level=logging.DEBUG)
with concurrent.futures.ProcessPoolExecutor(max_workers=MAX_WORKERS) as executor:
    while cap.isOpened():
        _, frame_org = cap.read()
        assert frame_org is not None, "Frame is None"
        args = ((model, frame_org) for model in models)

        # Works
        res = [executor.submit(initializer, (model, frame_org)) for model in models]
        for future in concurrent.futures.as_completed(res):
            if future.exception() is not None:
                print("FutureObject exception encountered, saving to log..")
                logging.exception(future.exception())
            else:
                future.add_done_callback(res_handler.handle)
        
    cap.release()
    time.sleep(5)
    kill_all(executor)



