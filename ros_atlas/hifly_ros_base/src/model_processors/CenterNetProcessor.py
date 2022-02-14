import cv2
import numpy as np
import time

from model_processors.BaseProcessor import BaseProcessor


class ModelProcessor(BaseProcessor):
    def __init__(self, params, expected_image_shape=None, process_only=False):
        super().__init__(params=params, process_only=process_only)
        
        # parameters for preprocessing
        self.ih, self.iw = (params['camera_height'], params['camera_width'])
        self.h, self.w = params['model_height'], params['model_width']
        self.scale = min(self.w / self.iw, self.h / self.ih)
        self.nw = int(self.iw * self.scale)
        self.nh = int(self.ih * self.scale)

        # parameters for postprocessing
        self.image_shape = expected_image_shape if expected_image_shape is not None else [params['camera_height'], params['camera_width']]
        self.model_shape = [self.h, self.w]
        self.num_classes = 1
        self.anchors = self.get_anchors()
        
    def predict(self, frame):
        st = time.time()
        preprocessed = self.preprocess(frame)
        print(f'Total time to preprocess: {time.time() - st}')

        st = time.time()
        outputs = self.model.execute([preprocessed])
        print(f'Total time to execute: {time.time() - st}')

        # result = self.postprocess(frame, outputs)
        return outputs

    def preprocess(self, image):
        # image = generator.load_image(i)
        # path = os.path.join(self.data_dir, 'JPEGImages', self.image_names[image_index] + self.image_extension)

        # default parameters
        flip_test = True

        src_image = image.copy()
        c = np.array([image.shape[1] / 2., image.shape[0] / 2.], dtype=np.float32)
        s = max(image.shape[0], image.shape[1]) * 1.0

        tgt_w = 512
        tgt_h = 512
        # to change...
        image = self.preprocess_image(image, c, s, tgt_w=tgt_w, tgt_h=tgt_h)
        
        if flip_test:
            flipped_image = image[:, ::-1]
            inputs = np.stack([image, flipped_image], axis=0)
        else:
            inputs = np.expand_dims(image, axis=0)
        
        input_path = sys.argv[1]

        # inputs.tofile(os.path.join(input_path, generator.image_names[i]+"bin"))
        return inputs

        
    def postprocess(self, outputs, frame):
        # TO REMOVE ###########################################
        process_var_bbox_area = 0
        cx, cy = float("-inf"), float("-inf")
        # TO REMOVE ###########################################

        box_axis, box_score = yolo_eval(outputs, self.anchors, self.num_classes, self.image_shape)
        nparryList, boxList = get_box_img(frame, box_axis)

        if len(nparryList) > 0:
            try:
            # for box in boxList:  # should be box = boxList[0] -- single box
                box = boxList[0]
                area = (box[1] - box[0]) * (box[3] - box[2])
                if area > process_var_bbox_area:
                    cx = (box[0] + box[1]) // 2
                    cy = (box[2] + box[3]) // 2
                    process_var_bbox_area = area
                cv2.rectangle(frame, (box[0], box[2]),  (box[1], box[3]), (255, 0, 0), 4) 
            except KeyError as e:
                print(e)
                
        return frame, (process_var_bbox_area, cx, cy) 

    # From CenterNet
    def preprocess_image(self, image, c, s, tgt_w, tgt_h):
        trans_input = get_affine_transform(c, s, (tgt_w, tgt_h))
        image = cv2.warpAffine(image, trans_input, (tgt_w, tgt_h), flags=cv2.INTER_LINEAR)
        image = image.astype(np.float32)

        image[..., 0] -= 103.939
        image[..., 1] -= 116.779
        image[..., 2] -= 123.68

        return image

    def get_affine_transform(center, scale, output_size, rot=0., inv=False):
        if not isinstance(scale, np.ndarray) and not isinstance(scale, list) and not isinstance(scale, tuple):
            scale = np.array([scale, scale], dtype=np.float32)

        if not isinstance(output_size, np.ndarray) and not isinstance(output_size, list) and not isinstance(output_size,
                                                                                                            tuple):
            output_size = np.array([output_size, output_size], dtype=np.float32)

        scale_tmp = scale
        src_w = scale_tmp[0]
        src_h = scale_tmp[1]
        dst_w = output_size[0]
        dst_h = output_size[1]

        rot_rad = np.pi * rot / 180
        src_dir = get_dir([0, src_h * -0.5], rot_rad)
        dst_dir = np.array([0, dst_h * -0.5], np.float32)

        src = np.zeros((3, 2), dtype=np.float32)
        dst = np.zeros((3, 2), dtype=np.float32)
        src[0, :] = center
        src[1, :] = center + src_dir
        dst[0, :] = [dst_w * 0.5, dst_h * 0.5]
        dst[1, :] = np.array([dst_w * 0.5, dst_h * 0.5], np.float32) + dst_dir
        src[2:, :] = get_3rd_point(src[0, :], src[1, :])
        dst[2:, :] = get_3rd_point(dst[0, :], dst[1, :])

        if inv:
            trans = cv2.getAffineTransform(np.float32(dst), np.float32(src))
        else:
            trans = cv2.getAffineTransform(np.float32(src), np.float32(dst))

        return trans