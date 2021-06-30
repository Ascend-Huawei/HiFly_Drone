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

from atlas_utils.acl_dvpp import Dvpp
from atlas_utils.acl_image import AclImage


class ModelProcessor(BaseProcessor):

    gesture_categories = [
        '0',
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
        '7',
        '8',
        '9',
        'left',
        'ok',
        'right',
        'rock',
        'finger heart',
        'praise',
        'prayer',
        'stop',
        'Give the middle finger',
        'bow',
        'No gesture'
    ]

    def __init__(self, params):
        super().__init__(params)
        self._dvpp = Dvpp(self._acl_resource)
        self._tmp_file = "../data/gesture_yuv/tmp.jpg"

    """Try with default ACLImage and DVPP implementation - then implement CV2 and image memory implement if time permits"""
    def preprocess(self, image):
        image_dvpp = image.copy_to_dvpp()
        yuv_image = self._dvpp.jpegd(image_dvpp)
        resized_image = self._dvpp.resize(yuv_image, self._model_width, self._model_height)
        return resized_image

    def postprocess(self, infer_output, image_file):
        data = infer_output[0]
        vals = data.flatten()
        top_k = vals.argsort()[-1:-2:-1]
        for n in top_k:
            object_class = self.get_gesture_categories(n)
        if len(top_k):
            object_class = self.get_gesture_categories(top_k[0])
            output_path = os.path.join(os.path.join(SRC_PATH, "../outputs"), os.path.basename(image_file))
            origin_img = Image.open(image_file)
            draw = ImageDraw.Draw(origin_img)
            font = ImageFont.load_default()
            draw.text((10, 50), object_class, font=font, fill=255)
            origin_img.save(output_path)
    
    def predict(self, frame):
        cv2.imwrite(self._tmp_file, frame)
        self._acl_image = AclImage(self._tmp_file)
        # image = AclImage(image_file)
        # resized_image = self.preprocess(image)
        resized_image = self.preprocess(self._acl_image)

        # result = self.inference([resized_image, ])
        result = self.model.execute([preprocessed, self._image_info])
        gesture.postprocess(result, image_file)


    def get_gesture_categories(self, gesture_id):
        if gesture_id >= len(gesture_categories):
            return "unknown"
        else:
            return gesture_categories[gesture_id]