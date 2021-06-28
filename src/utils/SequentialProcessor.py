"""
A sequential processor designed to handle serialized data stream from a source. 
Usage should depend on the application. For example, hand gesture application can afford asynchronous processing.
This sequential processor uses adaptive sampling rate for video (decrease sampling rate when small frame change and increase sampling rate upon drastic frame change), thus reducing load on edge device during stationary, unchanging duration in videos.

Paper: Real-time Video Inference on Edge Devices via Adaptive Model Streaming

Intended workflow:
Data Stream ---> SP (adaptive sampling + enqueue) 
                  |
                  | (dequeue)
                  |
                  |---> Model Inference ---> Deterministic Finite Automaton

Parameters:
    + adaptive_threshold (int): threshold for adaptive rate (Sample frame if AR > threshold)
    + cross_frame_differential (str): metric for calculating adaptive rate (MSE, RMSE,) 
    + interval for frame capture (preset sampling interval, may further reduces resources - but should be application-specific. default=None)

"""

import numpy as np
import cv2
import time
import os


if not os.path.exists("../../data/handGesture"):
    os.mkdir("../../data/handGesture")
    print("Created directory to store unit test frames")

### UNIT TEST - Run adaptive sampling on a pre-recorder video a hand gestures ###
def frameCapture():
    # define a video capture object
    vid = cv2.VideoCapture(0)
    if not vid.isOpened():
        raise Exception("Unable to read camera feed")
    frameNr = 0
    while True:
        # Capture the video frame by frame
        ret, frame = vid.read()
        if ret:
            cv2.imwrite(f"../../data/handGesture/frame_{frameNr}.jpg", frame)
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Stop frame capture...")
                break
        else:
            print("Ran into unexpected error, cannot read video")
            break
        frameNr = frameNr+1
    vid.release()
    print("Resource released.")

def vidCapture():
    # define a video capture object
    vid = cv2.VideoCapture(0)
    if vid.isOpened() == False:
        raise Exception("Unable to read camera feed")
    out = cv2.VideoWriter("../../data/handGesture.avi", cv2.VideoWriter_fourcc('M','J','P','G'), 20, (120,120))
    while True:
        # Capture the video frame by frame
        ret, frame = vid.read()
        if ret:
            out.write(frame)
            cv2.imshow('frame', frame)
            # the 'q' button is set as the quitting button you may use any desired button of your choice
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break
    # After the loop release the cap object
    vid.release()
    out.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

def vid2frame():
    if not os.path.exists("../../data/handGesture.avi"):
        raise Exception("Video does not exist")
    if not os.path.exists("../../data/output"):
        os.mkdir("../../data/output")
        print("Video frame output path created")
    capture = cv2.VideoCapture("../../data/handGesture.avi")
    frameNr = 0
    while True:
        success, frame = capture.read()
        if success:
            cv2.imwrite(f"../../data/output/frame_{frameNr}.jpg", frame)
        else:
            break
        frameNr = frameNr+1
    capture.release()

### Adaptive Sequential Processor class ###

class SequentialProcessor:
    def __init__(self, adaptive_threshold, cross_frame_differential, ):
        pass

    def adaptive_score(self):
        pass

    def _enqueue(self):
        pass

    def _dequeue(self):
        pass
    

