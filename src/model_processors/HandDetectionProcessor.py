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
from model_processors.BaseProcessor import BaseProcessor

class ModelProcessor(BaseProcessor):
    """acl model wrapper"""
    def __init__(self, params):
        super().__init__(params)


    def predict(self, img_original):
        """run predict"""
        #preprocess image to get 'model_input'
        model_input = self.preprocess(img_original)

        # execute model inference
        result = self.model.execute([model_input]) 
        return result 

    def preprocess(self, img_original):
        """
        preprocessing: resize image to model required size
        """
        image = cv2.resize(img_original, (300,300))
        image = image.astype(np.uint8).copy()
        return image

    def postprocessing(self, image, resultList, threshold=0.3):
        # """
        # draw the bounding boxes for all detected hands with confidence greater than a set threshold
        # """
        num_detections = resultList[0][0].astype(np.int)
        scores = resultList[2]
        boxes = resultList[3]
        bbox_num = 0
    
        # loop through all the detections and get the confidence and bbox coordinates
        for i in range(num_detections):
            det_conf = scores[0, i]
            det_ymin = boxes[0, i, 0]
            det_xmin = boxes[0, i, 1]
            det_ymax = boxes[0, i, 2]
            det_xmax = boxes[0, i, 3]

            bbox_width = det_xmax - det_xmin
            bbox_height = det_ymax - det_ymin
            # the detection confidence and bbox dimensions must be greater than a minimum value to be a valid detection
            if threshold <= det_conf and 1 >= det_conf and bbox_width > 0 and bbox_height > 0:
                bbox_num += 1
                xmin = int(round(det_xmin * image.shape[1]))
                ymin = int(round(det_ymin * image.shape[0]))
                xmax = int(round(det_xmax * image.shape[1]))
                ymax = int(round(det_ymax * image.shape[0]))
                
                cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            else:
                continue

        # print("detected bbox num:", bbox_num)
        return image