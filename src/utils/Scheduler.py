import numpy as np
import os
import cv2
from queue import Queue
import threading
import concurrent.futures

"""
### Queueing Scheduler ###
Takes in user-specified parameters (frames per second) and queue the batch of frames sequentially to a Queue. Then dequeue each batch to model for inference. Scheduler will run in the background to continuously queue the incoming frames and dequeue.

Threading: enqueue every x frames and dequeue every x frames
Input: frame
"""

class Scheduler:
    def __init__(self, batch_size=30):
        self.batch_size = batch_size
        self.max_queue = batch_size * 3
        self.q = Queue(maxsize=self.max_queue)
        pass

    def start_threads(self, frame):
        thread = threading.Thread(name="background_queue", target=self.enqueue_frame, args=[frame])
        thread.daemon = True
        thread.start()

    def batch_dequeue(self):
        """ Dequeue a batch of frames and offload them to model for inference """
        batch = [None for _ in range(self.batch_size)]
        for ind, _ in enumerate(batch):
            batch[ind] = self.q.get()
        return batch

    def enqueue_frame(self, frame):
        if self.q.qsize() < self.max_queue:
            # print("enqueue frame...")
            self.q.put(frame)
        else:
            print("Reached max queue size. dequeue a batch...")
            batch = self.batch_dequeue()
            return batch

if __name__ == '__main__':
    cap = cv2.VideoCapture("../../data/handGesture.avi")

    scheduler = Scheduler()
    batches = []

    # emulate video stream from drone
    while cap.isOpened():
        success, frame = cap.read()

        # schedule the enqueue method to execute and returns a future object
        # scheduler.start_threads(frame)
        with concurrent.futures.ThreadPoolExecutor() as executor:
            t1 = executor.submit(scheduler.enqueue_frame, frame)
            # print(t1.result())
            if t1.result() is not None:
                # pass frame batch to model - save to 
                batches.append(t1.result())

        if success:
            cv2.imshow('frame', frame)
            cv2.waitKey(10)
            if 0xFF == ord('q'):
                break
        else:
            break
        
    cap.release()
    batches = np.array(batches)
    print(batches.shape)
    # print(batches)