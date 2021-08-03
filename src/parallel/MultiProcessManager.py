"""
Sandbox for testing parallel inference (Depth Estimation + Object Detection)

Navie implementation:
0. Spawn ModelProcessor instances
1. Thread grabs frame - store to queue for callback afterwards, and send frame to 2 models
2. Once inference is done, send signal to dequeue
3. Result fusion and send to presenter server
"""
import concurrent.futures
import multiprocessing as mp
from multiprocessing import Process, Queue, Pool
import time
import sys
import os
import cv2
import logging
import numpy as np

sys.path.append("..")
sys.path.append("../lib")

from utils.uav_utils import connect_uav 
from utils.params import params
from utils.tools import load_model_processor

from model_processors.BaseProcessor import BaseProcessor

# from model_processors.IndoorDepthProcessor import ModelProcessor as DEModelProcessor
# from model_processors.ObjectDetectionProcessor import ModelProcessor as ODModelProcessor
# from model_processors.HandDetectionProcessor import ModelProcessor as HDModelProcessor
from model_processors.FaceDetectionProcessor import ModelProcessor as FDModelProcessor

from atlas_utils.presenteragent import presenter_channel
from atlas_utils.acl_image import AclImage


"""
1. Instaniate Models then do inference in parallel
    - put submit in queue 
2. Wait until both processes are completed, grab the result and pass to fusion

"""
class MultiProcessManager(object):
    def __init__(self, chan, uav, model_name, num_infer_processes=1, fps=15) -> None:
        super().__init__()
        
        self.manager = mp.Manager()
        self._input_q = self.manager.Queue()
        self._output_d = self.manager.dict()
        self._cond = self.manager.Condition()
        self._frame_id = 0
        self._running_inference = mp.Value("b", True)
        self._num_infer_processes = num_infer_processes
        self._fps = fps
        self.model_name = model_name
        self.chan = chan
        self.uav = uav

    @property
    def num_infer_processes(self):
        return self._num_infer_processes
    
    @property
    def fps(self):
        return self._fps

    @num_infer_processes.setter
    def num_infer_processes(self, num_processes):
        if num_processes <= 0:
            raise ValueError("Number of Processors must be at least 1!")
        else:
            self._num_infer_processes = num_processes

    @fps.setter
    def fps(self, new_fps):
        if new_fps <= 0:
            raise ValueError("FPS must be at least 1!")
        else:
            self._fps = new_fps
    
    def start(self):
        self.infer_processes = [mp.Process(target=self.inference_loop) for _ in range(self._num_infer_processes)]
        for p in self.infer_processes:
            p.start()

        self.feed_fps(fps=self._fps)

    def stop(self):
        self._running_inference.value = False
        for p in self.infer_processes:
            p.join()
        self.feed_process.join()
        
    def feed(self):
        '''Feed based on inference rate (frame are captured with same interval)'''
        frame_org = self.uav.get_frame_read().frame

        self._input_q.put({
            'frame_id': self._frame_id, 
            'input': frame_org
        })

        self._frame_id += 1

    def feed_fps(self, fps=15):
        sleep_time = 0 if fps >= 30 else (1/fps - 1/30)
        while self._running_inference.value:
            self.feed()
            time.sleep(sleep_time)
              
    def inference_loop(self):
        """Main Inference Loop to be done on separate processors
        Initialize models on each AI CPUs. Dequeue inference results from Queue and forward to Presenter Server 
        """
        model_processor, model_params = load_model_processor(self.model_name)
        model_processor = model_processor(model_params)
        
        logging.info(f"{self.model_name} Model Processor Initialized.\tParent Process: {os.getppid()}\tProcess ID: {os.getpid()}")
        
        st = time.time()
        flag = True
        while self._running_inference.value:
            if len(self._output_d) > 200:
                time.sleep(.5)
                if flag:
                    size = len(self._output_d)
                    et = time.time() - st
                    fps = size / et
                    print(f"Total time for {size:.2f} outputs", et, f"{fps:.2f} fps") 
                    flag = False
                continue
            try:
                sample = self._input_q.get(timeout=1)
                if sample['input'] is not None:
                    output = model_processor.predict(sample['input'])
                    assert output is not None, "Error: Output is None!!!!"
                    print(f"From pid {mp.current_process().pid}, Running Successfully: frame: {sample['frame_id']}")
                    # callback: sent to presenter server
                    if self.chan is not None:
                        _, output = cv2.imencode('.jpg', output)
                        output = AclImage(output, 720, 960, output.size)
                        self.chan.send_detection_data(720, 960, output, [])
                    else: # save to output dict
                        self._output_d[sample['frame_id']] = output
                        
                    with self._cond:
                        self._cond.notify()
                else:
                    print(sample)
                    raise Exception("Invalid sample")
            except:
                print("queue is Empty")
        logging.info(f"inference loop id {mp.current_process().pid} ends")

