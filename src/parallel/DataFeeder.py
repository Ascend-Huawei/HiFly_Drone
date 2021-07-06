import numpy as np
import os
import cv2
from queue import Queue
import threading
import concurrent.futures

"""
### DataFeeder ###

Threading: enqueue every x frames and dequeue every x frames
Input: frame
"""

class Feeder:
    def __init__(self, base_fps,):
        self.base_fps = base_fps
        self.max_queue =  self.base_fps ## change
        self.q = Queue(maxsize=self.max_queue)

    def start_threads(self, frame):
        thread = threading.Thread(name="frame_queue", target=self.enqueue_frame, args=[frame])
        thread.daemon = True
        thread.start()

    def _enqueue(self, frame):
        """require more work on "when" to queue"""
        if not self.q.full():
            frame_dict = {
                "frame": frame,
                "depth": False,
                "object": False 
            }
            self.q.put(frame_dict)
        else:
            ## Come up with a way to handle max queue (pop latest frame and enqueue?)
            pass

    def fuse(self, frameA, frameB):
        """fuse frameA and frameB together and dequeue the frame from self.q"""
        
        pass

# Testing frames in video
# if __name__ == '__main__':
#     cap = cv2.VideoCapture("../../data/handGesture.avi")

#     scheduler = Scheduler()
#     # batches = []

#     # emulate video stream from drone
#     while cap.isOpened():
#         success, frame = cap.read()

#         # schedule the enqueue method to execute and returns a future object
#         scheduler.start_threads(frame)
#         # with concurrent.futures.ThreadPoolExecutor() as executor:
#         #     t1 = executor.submit(scheduler.enqueue_frame, frame)
#             # if t1.result() is not None:
#             #     batches.append(t1.result())

#         if success:
#             cv2.imshow('frame', frame)
#             cv2.waitKey(10)
#             if 0xFF == ord('q'):
#                 break
#         else:
#             break
        
#     cap.release()
#     batches = np.array(scheduler.batches)
#     print(batches.shape)
#     for batch in scheduler.batches:
#         print(batch.shape)