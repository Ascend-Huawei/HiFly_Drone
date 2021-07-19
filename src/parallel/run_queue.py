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


"""
1. Instaniate Models then do inference in parallel
    - put submit in queue 
2. Wait until both processes are completed, grab the result and pass to fusion

"""
class MultiProcessManager(object):
    def __init__(self, num_infer_processes=1, input_q_size=10) -> None:
        super().__init__()
        
        self.manager = mp.Manager()
        self.input_q = self.manager.Queue()
        self.output_d = self.manager.dict()
        self.cond = self.manager.Condition()
        self.frame_id = 0
        self.running_inference = mp.Value("b", True)
        
        self.cap = cv2.VideoCapture("/home/HwHiAiUser/projects/atlas-track/inputs/MOT17-11-SDP-raw.webm")
        
        for _ in range(input_q_size):
            _, frame = self.cap.read()
            self.input_q.put({
                'frame_id': self.frame_id, 
                'input': frame
            })
            self.frame_id += 1
            
        processes = [mp.Process(target=self.inference_loop, args=(self.cond, ))
                     for _ in range(num_infer_processes)]
        
        for p in processes:
            p.start()
            
        while self.running_inference.value:
            self.feed_input()
            print("output size:", len(self.output_d))
        # self.smart_feed()
        
        for p in processes:
            p.join()
    
    def smart_feed(self):
        '''
        feed based on inference speed (frame are captured with same interval)
        '''
        # cap_interval = 0.1 # default 10fps?
        initial_qsize = self.input_q.qsize()
        st = time.time()
        # calculate cap interval:
        while self.input_q.qsize() > 5:
            continue
        total_time = time.time() - st # small
        cap_interval = total_time / (initial_qsize - self.input_q.qsize()) - 0.1
        
        # get feed time
        feed_time = time.time()
        ret, frame = self.cap.read()
        if not ret: # end or dirty
            self.running_inference.value = False
        self.input_q.put({
            'frame_id': self.frame_id, 
            'input': frame
        })
        self.frame_id += 1
        feed_time = time.time() - feed_time
        print(feed_time)
        cap_interval = cap_interval - feed_time
        print("###################")
        print("feed rate: ", 1/cap_interval, cap_interval)
        print("###################")
        
        while self.running_inference.value:
            
            feed_time = time.time()
            ret, frame = self.cap.read()
            if not ret: # end or dirty
                self.running_inference.value = False
            self.input_q.put({
                'frame_id': self.frame_id, 
                'input': frame
            })
            self.frame_id += 1
            feed_time = time.time() - feed_time
            
            time.sleep(cap_interval)
                
    def feed_input(self):
        with self.cond:
            self.cond.wait()
            ret, frame = self.cap.read()
            if not ret: # end or dirty
                self.running_inference.value = False
            self.input_q.put({
                'frame_id': self.frame_id, 
                'input': frame
            })
            self.frame_id += 1
        
    def inference_loop(self, cond):
        model_processor = DEModelProcessor(params["task"]["depth_estimation"]["indoor_depth_estimation"]) # 1.7 fps to 4.1fps
        # model_processor = ODModelProcessor(params["task"]["object_detection"]["yolov3"]) # same speed 7-8 fps (limited by dvpp resource?)
        # model_processor = HDModelProcessor(params["task"]["object_detection"]["hand_detection"]) # same speed 22 fps 
        # model_processor = FDModelProcessor(params["task"]["object_detection"]["face_detection"]) # 9fps to 17fps
        
        logging.info(f"Depth Estimation Model Processor Initialized.      Parent Process: {os.getppid()}      Process ID: {os.getpid()}")
        st = time.time()
        flag = True
        while self.running_inference.value:
            if len(self.output_d) > 20:
                time.sleep(.5)
                if flag:
                    # two processes: Total time for 30 outputs 15.4 ~ 16.7
                    # 4 processes: Total time for 33 outputs 8.7
                    size = len(self.output_d)
                    et = time.time() - st
                    fps = size / et
                    print(f"Total time for {size:.2f} outputs", et, f"{fps:.2f} fps") 
                    flag = False
                continue
            try:
                sample = self.input_q.get(timeout=1)
                if sample['input'] is not None:
                    # print(sample['input'].shape)
                    # print(sample['input'])
                    output = model_processor.predict(sample['input'])
                    assert output is not None, "Error: Output is None!!!!"
                    print(f"From pid {mp.current_process().pid}, Running Successfully: frame: {sample['frame_id']}")
                    self.output_d[sample['frame_id']] = output
                    with cond:
                        cond.notify()
                else:
                    print(sample)
                    raise Exception("Invalid sample")
            except:
                print("queue is Empty")
        logging.info(f"inference loop id {mp.current_process().pid} ends")

logging.basicConfig(filename='exec.log', level=logging.DEBUG)
mp_manager = MultiProcessManager(num_infer_processes=4)
