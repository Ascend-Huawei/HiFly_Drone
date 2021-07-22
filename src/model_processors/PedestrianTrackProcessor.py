"""
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import cv2
import os
from model_processors.track_utils import dataloader
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

from model_processors.BaseProcessor import BaseProcessor

from track_utils.dataloader import LoadVideo, LoadImages, letterbox
from track_utils.multitracker import JDETracker
from track_utils.timer import Timer
from track_utils import visualization as vis
from argparse import Namespace

def mkdir_if_missing(d):
    """
    create directory if not exist
    """
    if not os.path.exists(d):
        os.makedirs(d)

class ModelProcessor(BaseProcessor):
    def __init__(self, params):
        super().__init__(params)
        
        self.args = Namespace(**params['args'])
        # print(self.args, type(self.args)); raise Exception
        result_root = self.args.output_root if self.args.output_root != '' else '.'
        mkdir_if_missing(result_root)
        
        # dir for output images; default: outputs/'VideoFileName'
        video_name = os.path.basename(self.args.input_video).replace(' ', '_').split('.')[0]
        self.save_dir = os.path.join(result_root, video_name)  
        mkdir_if_missing(self.save_dir)
          
        # setup dataloader, use LoadVideo or LoadImages
        # self.dataloader = LoadVideo(self.args.input_video, (1088, 608))
        # print(self.args.input_video)
        # print(len(self.dataloader)); raise Exception
        # result_filename = os.path.join(result_root, 'results.txt')
        # frame_rate = self.dataloader.frame_rate
        
        self.tracker = JDETracker(self.args)
        self.timer = Timer()
        

    def predict(self, frame):
        self.timer.tic()
        
        preprocessed = self.preprocess(frame)
        outputs = self.model.execute(preprocessed)
        result = self.postprocess(outputs, frame)
        return result
        # print("Results will be saved at {}".format(self.save_dir))

        # self.
        # for frame_id, (count, img, img0) in enumerate(self.dataloader):
        #     if frame_id % 20 == 0 and frame_id != 0:
        #         print('Processing frame {} ({:.2f} fps)'.format(frame_id, 1. / max(1e-5, self.timer.average_time)))

        #     # run tracking, start tracking timer 
        #     self.timer.tic()

        #     # list of Tracklet; see multitracker.STrack
        #     online_targets = self.tracker.update(np.array([img]), img0)

        #     # prepare for drawing, get all bbox and id
        #     online_tlwhs = []
        #     online_ids = []
        #     for t in online_targets:
        #         tlwh = t.tlwh
        #         tid = t.track_id
        #         vertical = tlwh[2] / tlwh[3] > 1.6
        #         if tlwh[2] * tlwh[3] > self.args.min_box_area and not vertical:
        #             online_tlwhs.append(tlwh)
        #             online_ids.append(tid)
        #     self.timer.toc()
        #     print(online_ids, online_tlwhs)
        #     # draw bbox and id
        #     online_im = vis.plot_tracking(img0, online_tlwhs, online_ids, frame_id=frame_id,
        #                                     fps=1. / self.timer.average_time)
        #     cv2.imwrite(os.path.join(self.save_dir, '{:05d}.jpg'.format(frame_id)), online_im)


        # return result

    def preprocess(self, img0):
        """preprocess frame from drone"""
        img, _, _, _ = letterbox(img0)
        img = np.expand_dims(img, axis=0)
        return img.astype(np.float32).copy()    
        
    def postprocess(self, outputs, img0):
        """postprocess output from model"""
        online_targets = self.tracker.update(outputs)
        
        # prepare for drawing, get all bbox and id
        online_tlwhs = []
        online_ids = []
        for t in online_targets:
            tlwh = t.tlwh
            tid = t.track_id
            vertical = tlwh[2] / tlwh[3] > 1.6
            if tlwh[2] * tlwh[3] > self.args.min_box_area and not vertical:
                online_tlwhs.append(tlwh)
                online_ids.append(tid)
        self.timer.toc()
        # print(online_ids, online_tlwhs)
        # draw bbox and id
        online_im = vis.plot_tracking(img0, online_tlwhs, online_ids, frame_id=self.tracker.frame_id,
                                        fps=1. / self.timer.average_time)
        
        return online_im