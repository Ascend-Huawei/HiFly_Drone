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
from PIL import Image, ImageDraw, ImageFont
from model_processors.BaseProcessor import BaseProcessor

from atlas_utils.acl_dvpp import Dvpp
from atlas_utils.acl_image import AclImage


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

class ModelProcessor(BaseProcessor):
    def __init__(self, params):
        # Initialize Parent class BaseProcessor
        super().__init__(params)
        self._dvpp = Dvpp(self._acl_resource)
        self._tmp_file = "../../data/tmp.jpg"
        self._image_info = self.construct_image_info()

    def predict(self, frame):
        preprocessed = self.preprocess(frame)
        result = self.model.execute([preprocessed, self._image_info])
        result = self.postprocess(result, frame)
        return result


    def preprocess(self, frame):
        """preprocess frame from drone"""
        cv2.imwrite(self._tmp_file, frame)
        self._acl_image = AclImage(self._tmp_file)
        image_input = self._acl_image.copy_to_dvpp()
        yuv_image = self._dvpp.jpegd(image_input)
        resized_image = self._dvpp.crop_and_paste(yuv_image, self._acl_image.width, self._acl_image.height,\
            self._model_width, self._model_height)
        return resized_image

    def postprocess(self, infer_output, origin_img):
        """
        postprocess
        :param infer_output - model inference execution output
        :param origin_img   - original image
        :param image_file   - image path
        returns mutated origin_img as output
        """
        box_num = infer_output[1][0, 0]
        box_info = infer_output[0].flatten()
        scale = max(origin_img.shape[1] / self._model_width, origin_img.shape[0] / self._model_height)
        
        origin_img = Image.fromarray(origin_img)
        draw = ImageDraw.Draw(origin_img)
        font = ImageFont.load_default()
        for n in range(int(box_num)):
            ids = int(box_info[5 * int(box_num) + n])
            label = labels[ids]
            score = box_info[4 * int(box_num)+n]
            top_left_x = box_info[0 * int(box_num)+n] * scale
            top_left_y = box_info[1 * int(box_num)+n] * scale
            bottom_right_x = box_info[2 * int(box_num) + n] * scale
            bottom_right_y = box_info[3 * int(box_num) + n] * scale
            # print(" % s: class % d, box % d % d % d % d, score % f" % (
                # label, ids, top_left_x, top_left_y, 
                # bottom_right_x, bottom_right_y, score))
            draw.line([(top_left_x, top_left_y), (bottom_right_x, top_left_y), (bottom_right_x, bottom_right_y), \
            (top_left_x, bottom_right_y), (top_left_x, top_left_y)], fill=(0, 200, 100), width=3)
            draw.text((top_left_x, top_left_y), label, font=font, fill=255)

        return np.array(origin_img)

    def construct_image_info(self):
        """construct image info"""
        image_info = np.array([self._model_width, self._model_height, 
                            self._model_width, self._model_height], 
                            dtype = np.float32) 
        return image_info