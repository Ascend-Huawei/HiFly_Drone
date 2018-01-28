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

import os
import cv2
import numpy as np
import sys
from queue import Queue
from model_processors.BaseProcessor import BaseProcessor

from atlas_utils.acl_dvpp import Dvpp
from atlas_utils.acl_image import AclImage



class ModelProcessor(BaseProcessor):
    
    labels = ["person",
        "bicycle", "car", "motorbike", "aeroplane",
        "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench",
        "bird", "cat", "dog", "horse", "sheep", "cow", "elephant",
        "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
        "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
        "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog",
        "pizza", "donut", "cake", "chair", "sofa", "potted plant", "bed", "dining table",
        "toilet", "TV monitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
        "scissors", "teddy bear", "hair drier", "toothbrush"]

    def __init__(self, params):
        # Initialize Parent class BaseProcessor
        super().__init__(params)
        self._dvpp = Dvpp(self._acl_resource)
        self._tmp_file = "../../data/tmp.jpg"
        self._image_info = self.construct_image_info()
        self.q = Queue(maxsize=16)

    def predict(self, frame):
        preprocessed = self.preprocess(frame)
        self.q.put(preprocessed)
        if self.q.full():
            self.batch = [self.q.get() for _ in range(self.q.qsize())]
            self.batch = np.stack(self.batch, axis=0)
            infer_output = self.model.execute([self.batch])
        # result = self.postprocess(infer_output, frame)
            return infer_output

    def preprocess(self, frame):
        """preprocess frame from drone"""
        img = np.array(frame, dtype='float32', copy=True)
        img_resized = cv2.resize(img, (self._model_width, self._model_height), interpolation=cv2.INTER_CUBIC)
        # img_resized = cv2.resize(img, (self._model_width, self._model_height))
        return img_resized

    def postprocess(self, infer_output, origin_img):
        """
        postprocess
        :param infer_output - model inference execution output
        :param origin_img   - original image
        :param image_file   - image path
        returns mutated origin_img as output
        """
        box_num = infer_output[0][0]
        label = infer_output[3][0]
        boxes = infer_output[2][0]
        scores = infer_output[1][0]
        print(f"# Boxes: {box_num}, Highest Score: {scores[0]}, Most confident label: {ModelProcessor.labels[int(label[0])]}")

        for i in range(box_num):
            score = scores[0, i]
            print(score)
            top_left_x = boxes[i, 0] * self._model_width
            top_left_y = boxes[i, 1] * self._model_height
            bottom_right_x = boxes[i, 2] * self._model_width
            bootom_right_y = boxes[i, 3] * self._model_height
            cv2.rectangle(origin_img, (top_left_x, top_left_y),  (bottom_right_x, bottom_right_y), (255, 0, 0), 4)
        
        return origin_img

    def construct_image_info(self):
        """construct image info"""
        image_info = np.array([self._model_width, self._model_height, 
                            self._model_width, self._model_height], 
                            dtype = np.float32) 
        return image_info