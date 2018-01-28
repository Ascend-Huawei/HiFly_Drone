import sys
import os
import numpy as np
import pickle
import cv2
import argparse

sys.path.append("..")
sys.path.append("../lib")

from model_processors.SSDMP import ModelProcessor
from utils.params import params
from utils.tools import load_model_processor
from utils.uav_utils import connect_uav

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

ssd_mb_processor, ssdmb_info = load_model_processor('ssd_mobilenetv1')
SSDMP = ssd_mb_processor(ssdmb_info)

def run_vid_feed(vid_path):
    """Extract frame-by-frame from video and send to MP for prediction"""
    if not os.path.exists(vid_path):
        raise FileNotFoundError("Video not found at provided path, try again")
    cap = cv2.VideoCapture(vid_path)
    if not cap.isOpened(): raise Exception("video capture not open")
    while cap.isOpened():
        _, frame = cap.read()
        if frame is not None:
            infer_output = SSDMP.predict(frame)
            if infer_output is None:
                continue
            box_num = infer_output[0][0]
            label = infer_output[3][0]
            boxes = infer_output[2][0]
            scores = infer_output[1][0]
            print(f"# Boxes: {int(box_num)}, Highest Score: {scores[0]}, Most confident label: {labels[int(label[0])]}")

def run_drone_feed():
    uav = connect_uav()
    uav.streamoff()
    uav.streamon()

    while True:
        frame = uav.get_frame_read().frame
        if frame is not None:
            infer_output = SSDMP.predict(frame)
            if infer_output is None:
                continue
            box_num = infer_output[0][0]
            label = infer_output[3][0]
            boxes = infer_output[2][0]
            scores = infer_output[1][0]
            print(f"# Boxes: {int(box_num)}, Highest Score: {scores[0]}, Most confident label: {labels[int(label[0])]}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", type=str, default=None, help="Input path to video, uses Drone livefeed if None")
    args = parser.parse_args()

    if args.input is None:
        run_drone_feed()
    else:
        run_vid_feed(args.input)

