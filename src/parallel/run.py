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
from model_processors.IndoorDepthProcessor import ModelProcessor as DEModelProcessor
from model_processors.ObjectDetectionProcessor import ModelProcessor as ODModelProcessor
from model_processors.HandDetectionProcessor import ModelProcessor as HDModelProcessor
from model_processors.FaceDetectionProcessor import ModelProcessor as FDModelProcessor
from atlas_utils.presenteragent import presenter_channel
from atlas_utils.acl_image import AclImage

def initializer(initargs):
    # verify if global vars are initialized in this processor and their values
    if "inited" not in globals():
        print(f"Initializing global flag on processor {os.getpid()}...")
        global inited, mp
        inited, mp = False, None

    logging.basicConfig(filename='parallel.log', level=logging.DEBUG)
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
        elif model_name == "depth_estimation":
            mp = DEModelProcessor(params["task"]["depth_estimation"]["indoor_depth_estimation"])
            logging.info(f"DepthEstimationMP Initialized.\nParent Process: {os.getppid()}\nProcess ID: {os.getpid()}")
        else:
            raise Exception("ModelProcessor not initialized, model not supported")
        inited = True
    logging.info(f"MP {mp} already initialized in Processor {os.getpid()}, prepare for inferece...")
    assert mp is not None, f"ModuleProcessor on {os.getpid()} is None. Try again."
    return mp.predict(frame), model_name


class ResultHandler:
    """
    Accept parentID (MainProcessor iD) and models (list) and handles inference results
    """
    data_dir = "../../data/parallel"
    def __init__(self, parentID, models):
        self.pid = parentID
        self.models = models
        self._set_save_path()
        self.frame_count = 0

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

    def _set_save_path(self):
        self.out_dirs = []
        fused_dir = os.path.join(self.data_dir, "fused") 
        self.out_dirs.append(fused_dir)
        for model in self.models:
            output_dir = os.path.join(self.data_dir, model)
            if output_dir not in self.out_dirs:
                self.out_dirs.append(output_dir)

        for dir_name in self.out_dirs:
            try:
                os.mkdir(dir_name)
                print(f"Created output path: {output_dir}")
            except FileExistsError as err:
                print(f"{output_dir} already exists, emptying directory...")
                for f in os.listdir(output_dir):
                        os.remove(os.path.join(output_dir, f))

    @staticmethod
    def _is_pil_image(img):
        return isinstance(img, Image.Image)

    @staticmethod
    def _is_numpy_image(img):
        return isinstance(img, np.ndarray) and (img.ndim in [2,3])

    @staticmethod
    def _hconcat_resize_min(im_list, interpolation=cv2.INTER_CUBIC):
        h_min = min(im.shape[0] for im in im_list)
        im_list_resize = [cv2.resize(im, (int(im.shape[1] * h_min / im.shape[0]), h_min), interpolation=interpolation)
                        for im in im_list]
        return cv2.hconcat(im_list_resize)

    def handle(self, future):
        """
        Unpack FutureObject result and saves image to the corresponding directory - model_name is guaranteed to be in self.out_dir by initialization.
        :param: 
            result - FutureObject of type Typle(inference_output, model_name)
        """
        inference_res, model_name = future.result()
        if not (self._is_pil_image(inference_res) or self._is_numpy_image(inference_res)):
            raise TypeError(f"Inference result should be PIL or ndarray. Got {type(inference_res)} instead.")
            
        save_dir = os.path.join(self.data_dir, model_name)
        out_name = self._output_tracker(model_name)
        if isinstance(inference_res, np.ndarray):
            cv2.imwrite(os.path.join(save_dir, out_name), inference_res)

    def fusion(self, lst, write=False):
        """
        Input is a list of FutureObjects. Fuse the inference image ouptuts together and save them in the fused directory. 
        """
        images = [future.result()[0] for future in lst]
        for image in images:
            if not (self._is_pil_image(image) or self._is_numpy_image(image)):
                raise TypeError(f"Inference result should be PIL or ndarray. Got {type(image)} instead.")

        fused = self._hconcat_resize_min(images)

        if write: 
            save_dir = os.path.join(self.data_dir, "fused")
            frame_name = str(self.frame_count) + ".png"
            frame_out_path = os.path.join(save_dir, frame_name)
            cv2.imwrite(frame_out_path, fuse)
            self.frame_count += 1
        
        return fused

def kill_all(e):
    e.shutdown(wait=True, cancel_futures=False)


## Parameters ##
MAX_WORKERS = 3
models = ["face_detection", "object_detection",]
# models = ["depth_estimation", "object_detection",]
test_capture = "../../data/handGesture.avi" # A pre-recorded video - need to change this to something else if someone else is using it
SRC_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PRESENTER_SERVER_CONF = os.path.join(SRC_PATH, "uav_presenter_server.conf")

"""
DEV LOG

Summary:
    Using ProcessPoolExecutor to submit asynchronously executions of tasks (initializer). Will set global flag if no globals are set.
    Then initialize the corresponding ModelProcessor based on global flag ONCE. Future calls to the target function will utilize the pre-initialize
    ModelProcessor to make inference in parallel.

    FutureObject encapsulates the asynchronous execution of a callable - returns a tuple if callable (initializer) is ran successfully without raising 
    an Exception.

    iF FutureObject has no exception - pass results to ResultHandler to process results (image fusion, write file).

TODO: 
    - Queue implementation to put frames from video stream -> forward different frames to processors for faster computation;
        currently handling frame one-by-one and not utilizing all available processors
    - Compare FPS of: 
        - parallel inference vs single inference without Queue
            + DE & OD with 3 workers ~= 1-2fps
            + FD & OD with 3 workers ~= 7fps
        - parallel inference vs single inference with Queue
        - resource utilization (CPU %)
    - Integration of parallel inference to platform once finalize
"""
cap = cv2.VideoCapture(test_capture)
res_handler = ResultHandler(os.getpid(), models)

chan = presenter_channel.open_channel(PRESENTER_SERVER_CONF)
if chan is None:
    print("Open presenter channel failed")

logging.basicConfig(filename='exec.log', level=logging.DEBUG)
with concurrent.futures.ProcessPoolExecutor(max_workers=MAX_WORKERS) as executor:
    while cap.isOpened():
        _, frame_org = cap.read()
        if frame_org is None:
            print("VideoCapture frame is None. Releasing VidCap and killing all processors in pool...")
            break
        args = ((model, frame_org) for model in models)

        # Works
        res = [executor.submit(initializer, (model, frame_org)) for model in models]
        for future in concurrent.futures.as_completed(res):
            if future.exception() is not None:
                print("FutureObject exception encountered, saving exception to log..")
                logging.exception(future.exception())
            # Uncomment else clause below to save individual model output to model folder
            # else:
            #     future.add_done_callback(res_handler.handle)

        # Uncomment below to save fusion models outputs to fused folder - currently does not support DE fusion due to output dimension mismatch
        fused = res_handler.fusion(res)

        _, jpeg_image = cv2.imencode('.jpg', fused)
        jpeg_image = AclImage(jpeg_image, frame_org.shape[0], frame_org.shape[1], jpeg_image.size)
        chan.send_detection_data(frame_org.shape[0], frame_org.shape[1], jpeg_image, [])


    cap.release()
    time.sleep(5)
    kill_all(executor)



