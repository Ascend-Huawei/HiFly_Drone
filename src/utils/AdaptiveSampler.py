import numpy as np
import cv2
import time
import os



### UNIT TEST - Run adaptive sampling on a pre-recorder video a hand gestures ###

    
def frameCapture():
    """Capture a video and save it frame-by-frame"""
    vid = cv2.VideoCapture(0)
    if not vid.isOpened():
        raise Exception("Unable to read camera feed")
    frameNr = 0
    while True:
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
    """Capture a video and save it as video format"""
    vid = cv2.VideoCapture(0)
    if not vid.isOpened():
        raise Exception("Unable to read camera feed")

    frame_width = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))
    size = (frame_width, frame_height)
    print(size)

    out = cv2.VideoWriter("../../data/handGesture.avi", cv2.VideoWriter_fourcc('M','J','P','G'), 20, size)
    while True:
        ret, frame = vid.read()
        if ret:
            out.write(frame)
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break
    vid.release()
    out.release()
    cv2.destroyAllWindows()

def vid2frame():
    """Split video into frames"""
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

"""
### Adaptive Sequential Processor class ###

A sequential processor designed to handle serialized data stream from a source. 
Usage should depend on the application. For example, hand gesture application can afford asynchronous processing.
This sequential processor uses adaptive sampling rate for video (decrease sampling rate when small frame change and increase sampling rate upon drastic frame change), thus reducing load on edge device during stationary, unchanging duration in videos.

Paper: Real-time Video Inference on Edge Devices via Adaptive Model Streaming

Intended workflow:
1. Video stream from drone (djitellopy grabs video frame-by-frame) 
    Input to AdaptiveProcessor should be numpy frame
2. Start AdaptiveProcessor after a delay (if it starts when the video start then )
Data Stream (Video) ---> SP (adaptive sampling + enqueue) 
                    |
                    | (dequeue)
                    |
                    |---> Model Inference ---> Deterministic Finite Automaton

Parameters:
    + vid/frames: 
    + adaptive_threshold (int): threshold for adaptive rate (Sample frame if AR > threshold)
    + cross_frame_differential (str): metric for calculating adaptive rate (MSE, RMSE,) 
    + interval for frame capture (preset sampling interval, may further reduces resources - but should be application-specific. default=None)

"""

class AdaptiveSampler:
    def __init__(self, adaptive_threshold, cross_frame_differential, ):
        pass

    def adaptive_score(self):
        pass

    def _enqueue(self):
        pass

    def _dequeue(self):
        pass
    

if __name__ == '__main__':

    frames_dir = "../../data/handGesture"
    if not os.path.exists(frames_dir):
        os.mkdir(frames_dir)
        print("Created directory to store unit test frames")

    # 1. Create a sample video
    # frameCapture()
    vidCapture()

    # 2. Test sampler and see which frames it selects
    # 2.1. Emulate an incoming video stream and run sampler on it