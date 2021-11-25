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
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

from model_processors.BaseProcessor import BaseProcessor


class ModelProcessor(BaseProcessor):
    def __init__(self, params):
        super().__init__(params)
        
        # parameters for poseprocessing
        self.ih, self.iw = (params['camera_height'], params['camera_width'])
        

    def predict(self, frame):
        preprocessed = self.preprocess(frame)
        outputs = self.model.execute([preprocessed])
        result = self.postprocess(outputs)
        return result

    def preprocess(self, img):
        """preprocess frame from drone"""
        # preprocessing: resize and paste input image to a new image with size 416*416
        img = cv2.resize(img, (640, 480), interpolation = cv2.INTER_AREA) / 255.
        img = img.transpose((2, 0, 1))
        # normalize
        mean = [0.485, 0.456, 0.406]
        std = [0.229, 0.224, 0.225]
        for channel in range(3):
            img[channel] = (img[channel] - mean[channel]) / std[channel]
        img = np.expand_dims(img, axis=0)
        return img.astype(np.float32).copy()
        
    def postprocess(self, outputs):
        """postprocess frame from drone"""
        min_depth = 1e-3
        max_depth = 10
        final = np.clip(outputs[0], min_depth, max_depth)
        final[final < min_depth] = min_depth
        final[final > max_depth] = max_depth
        final[np.isinf(final)] = max_depth
        final[np.isnan(final)] = min_depth
        final = cv2.resize(final.squeeze(), (self.iw, self.ih), interpolation = cv2.INTER_AREA)

        # TODO: find better way instead of save & load
        # matplotlib 默认的十色环：”C0”, “C1”, ……，”C9”
        parent_id = os.getppid()
        d = f"tmp/{parent_id}"
        if not os.path.exists(d):
            os.makedirs(d)
        filename = f"tmp/{parent_id}/tmp_{os.getpid()}.jpg"
        plt.imsave(filename, final)
        return cv2.imread(filename)
  