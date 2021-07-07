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
        if model_name == "hand":
            mp = HDModelProcessor(params["task"]["object_detection"]["hand_detection"])
            logging.info(f"HandDetectionMP Initialized.\nParent Process: {os.getppid()}\nProcess ID: {os.getpid()}")
        elif model_name == "object":
            mp = ODModelProcessor(params["task"]["object_detection"]["yolov3"])
            logging.info(f"ObjectDetectionMP Initialized.\nParent Process: {os.getppid()}\nProcess ID: {os.getpid()}")
        else:
            pass
        inited = True
    logging.info(f"MP {mp} already initialized in Processor {os.getpid()}, prepare for inferece...")
    assert mp is not None, f"ModuleProcessor on {os.getpid()} is None. Try again."
    return mp.predict(frame)


class ResultHandler:
    def __init__(self, parentID, num_models=2):
        self.pid = parentID
        self.num_models = num_models
    
    def save_to(self, ):
        """saves the result (frames - ndarray, images) to the corresponding directory
        require knowledge of the correct directory
        """
        pass
        
# def kill_all(e):
#     e.shutdown(wait=True, cancel_futures=False)

# def inference(args):
#     inited_mp, frame = args
#     return inited_mp.predict(frame)

# def mp_initializer(model_name):
#     if model_name == "hand_detection":
#         return HDModelProcessor(params["task"]["object_detection"]["hand_detection"])
#     elif model_name == "face_detection":
#         return FDModelProcessor(params["task"]["object_detection"]["face_detection"])

# def initmap(executor, mp_initializer, initargs, frame):
#     """initarg = name of model
#     initmap maps each ModelProcessor with the frame - the ModelProcessor is initialized by
#         mp_initializer based on initargs (model_name)
    
#     """
#     return executor.map(partial(mp_initializer, initargs), frame)

## Parameters ##
MAX_WORKERS = 3

### Try to make multiprocess inference on a list of ndarrays
models = ["hand", "object", "object"]
cap = cv2.VideoCapture("../../data/handGesture.avi")

"""
    Each process calls initializer ONCE and returns a unique ModelProcessor object (based on model)
    Child processes uses this ModelProcessor object to iteratively do inference
    want to executor.map(inited_mp.predict, frame) where frame is continuously coming in

"""


logging.basicConfig(filename='exec.log', level=logging.DEBUG)
frame_counter = 0
with concurrent.futures.ProcessPoolExecutor(max_workers=MAX_WORKERS) as executor:
    while cap.isOpened():
        _, frame_org = cap.read()
        assert frame_org is not None, "Frame is None"
        frame_counter += 1
        args = ((model, frame_org) for model in models)

        # Works
        res = [executor.submit(initializer, (model, frame_org)) for model in models]
        for future in concurrent.futures.as_completed(res):
            if future.exception() is not None:
                print("FutureObject exception encountered, saving to log..")
                logging.exception(future.exception())
            else:
                # ResultHandler
                print(type(future.result()))
                

        print(f"Frame: {frame_counter}")

    cap.release()
    time.sleep(5)
    kill_all(executor)



