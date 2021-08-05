"""
Sample script for running parallel inference on a video with two different models (Depth Estimation + Object Detection)

Navie implementation:
0. Spawn ModelProcessor instances
1. Thread grabs frame - store to queue for callback afterwards, and send frame to 2 models
2. Once inference is done, send signal to dequeue
3. Result fusion and send to presenter server

Script Summary:
    Using ProcessPoolExecutor to submit asynchronous executions of tasks (initializer). Set global flag if no globals are set.
    Then initialize the corresponding ModelProcessor based on global flag on first run. Future calls to the target function (initializer) 
    will utilize the pre-initialized ModelProcessor to make inference in parallel.

    FutureObject encapsulates the asynchronous execution of a callable - returns a tuple if callable (initializer) is ran successfully without raising 
    an Exception.

    iF FutureObject has no exception - pass results to ResultHandler to process results (image fusion, write file).
"""

import concurrent.futures
import time
import sys
import os
import cv2
import logging
import numpy as np
from PIL import Image
import argparse

sys.path.append("..")
sys.path.append("../lib")

from utils.tools import init_presenter_server, load_model_processor
from atlas_utils.acl_image import AclImage

def initializer(initargs):
    """Main initialization and inference target function to be used by ProcessPoolExecutor
    Initialize a ModelProcessor on each AI CPU and make inference. Each processor has global flags inited and mp 
    to check if a ModelProcessor is initiated and its memory address.
    :params:
        initargs - Tuple(model_name: str, frame: np.ndarray) 
    """
    # verify if global vars are initialized in this processor and their values
    if "inited" not in globals():
        print(f"Initializing global flag on processor {os.getpid()}...")
        global inited, mp
        inited, mp = False, None

    model_name, frame = initargs
    if not inited:
        print("ModuleProcessor initialization...")
        mp, model_params = load_model_processor(model_name)
        mp = mp(model_params)
        logging.info(f"{model_name} ModelProcessor Initialized.\tParent Process: {os.getppid()}\tProcess ID: {os.getpid()}")
        inited = True
    logging.info(f"MP {mp} already initialized in Processor {os.getpid()}, prepare for inferece...")
    assert mp is not None, f"ModuleProcessor on {os.getpid()} is None. Try again."
    return mp.predict(frame), model_name

class ResultHandler:
    """Accepts parentID (MainProcessor iD) and models (list) and handles inference results
    Either perform frame fusion or saves individual inference results to respective subdirectory under data/parallel
    """
    data_dir = "../../data/parallel"
    def __init__(self, parentID, models):
        self.pid = parentID
        self.models = models
        self._set_save_path()
        self.frame_count = 0
        if not os.path.exists(self.data_dir):
            os.mkdir(data_dir)

    def _output_tracker(self, model_name):
        """Set attribute to track the frame count for each model and returns the name of the latest frame to be used for saving"""
        attr = model_name + "_frame_count"
        try:
            frame_count = getattr(self, attr)
            setattr(self, attr, frame_count + 1)
            return str(frame_count) + ".png"
        except AttributeError: # no attribute error - initialize as 0 and return
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
                print(f"Created output path: {dir_name}")
            except FileExistsError as err:
                print(f"{dir_name} already exists, emptying directory...")
                for f in os.listdir(dir_name):
                        os.remove(os.path.join(dir_name, f))

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
        """Unpack FutureObject result and saves image to the corresponding directory - model_name is guaranteed to be in self.out_dir by initialization.
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
        """Fuses inference results into one image and save them in the fused directory
        :param:
            lst - List of FutureObjects
            write - Boolean, write and save to path if True
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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Parallel Inference setting")
    parser.add_argument("--models", nargs="+", help="Two supported models", default=['yolov3', 'indoor_depth_estimation'])
    parser.add_argument("--num_processors", type=int, help="Number of Processors", default=3)
    parser.add_argument("--vid_in", type=str, help="Path to video input", required=True)
    parser.add_argument("--fuse", type=bool, help="Perform side-by-side image fusion and forward to Presenter Server if True", default=True)
    args = parser.parse_args()

    models, num_processors, vid_in, do_fusion = args.models, args.num_processors, args.vid_in, args.fuse

    chan = init_presenter_server()
    cap = cv2.VideoCapture(vid_in)
    
    res_handler = ResultHandler(os.getpid(), models)

    logging.basicConfig(filename='exec.log', level=logging.DEBUG)
    with concurrent.futures.ProcessPoolExecutor(max_workers=num_processors) as executor:
        while cap.isOpened():
            _, frame_org = cap.read()
            cv2.waitKey(10)
            if frame_org is None:
                print("VideoCapture frame is None. Releasing VidCap and killing all processors in pool...")
                break
            args = ((model, frame_org) for model in models)

            res = [executor.submit(initializer, (model, frame_org)) for model in models]
            for future in concurrent.futures.as_completed(res):
                if future.exception() is not None:
                    print("FutureObject exception encountered, saving exception to log..")
                    logging.exception(future.exception())
                if not do_fusion:
                    future.add_done_callback(res_handler.handle)
                    
            if do_fusion:
                fused = res_handler.fusion(res, write=False)
                _, jpeg_image = cv2.imencode('.jpg', fused)
                jpeg_image = AclImage(jpeg_image, frame_org.shape[0], frame_org.shape[1], jpeg_image.size)
                chan.send_detection_data(frame_org.shape[0], frame_org.shape[1], jpeg_image, [])

        cap.release()
        time.sleep(1)
        kill_all(executor)