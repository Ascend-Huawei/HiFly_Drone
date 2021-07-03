from queue import Queue
import concurrent.futures
import threading
import numpy as np

"""
A general window-based filter to prune volatile results in a period of frames

Queue implementation (i.e: first 10 result, get most reoccuring inference result, release and output mode result)
parameters:
    window                  - length of a window in seconds (ms) (int)
    batch_inference_rate    - inference time for a batch in millisecond (ms) (float)
        ** preset for now - gesture_yuv ~ 1.40ms per frame **
    batch_size              - model batch size (int)

Queue size is determined by how many frames are processed over the specified window. I.e. for a window of 3 seconds,
if the batch inference rate is 1.5ms for <batch_size> images, then:
    1. inference rate per image -> batch_inference_rate / batch_siz
    2. window / inference_rate_per_image = number of frames within the window

Distiller should queue the inference results - no need for threading
After <window> seconds, distiller will dequeue, return most reoccuring result, and queue the next batch

Display frames are handled by ModelProcessor, WindowDistiller returns the most reoccuring result

"""

class InferenceDistiller:
    def __init__(self, window=3, batch_inference_rate=1.5, batch_size=1,):
        self.batch_size = batch_size
        self.batch_inference_rate = batch_inference_rate
        self.window = window
        self.inference_rate_per_frame = self.batch_inference_rate / self.batch_size # ms
        # self.max_qsize = int(self.window / self.inference_rate_per_frame * 1000)
        # self.q = Queue(maxsize=self.max_qsize)
        self.inference_tracker = dict()
        self.mode_inference = None

        ### Assume worst case of 4fps and best case 7fps - redo calculations for frames under this assumption
        self.fps = 4
        self.max_qsize = self.fps * self.window
        self.q = Queue(maxsize=self.max_qsize) 
        print(f"Max size: {self.max_qsize}")

    # def start_thread(self, result):
    #     """initialize worker thread"""
    #     threading.Thread(daemon=True, target=worker, args=[result])).start()

    def sample(self, result):
        if not self.q.full():
            self._enqueue(result)
        else:
            # get the mode result, then release
            self.mode_inference = self._release()
            print(f"Mode inference result: {self.mode_inference}")
            return self.mode_inference
        
    def _enqueue(self, result):
        """put inference result to the Queue when there is room"""
        self.q.put(result)
        # create dynamic hashtable of inference result. Key=Inference Output, Value=occurance
        # each enqueue will update the hashtable
        if result in self.inference_tracker:
            self.inference_tracker[result] += 1
        else:
            print(f"Detected new gesture: {result}")
            self.inference_tracker[result] = 1

            
    
    # def _batch_dequeue(self):
    #     """release all elements in the queue and make room for frames in next window"""
    #     # empty self.q, return none
    #     with self.q.mutex:
    #         self.q.queue.clear()

    def _release(self):
        "Release elements in the queue and get the key from inference_tracker with the highest value"
        # for item in range(self.max_qsize):
        #     self.q.get_nowait()
        distilled_result = max(self.inference_tracker, key=self.inference_tracker.get)
        print("Release: ", self.inference_tracker)
        print("Max queue size reached, release elements in queue for next batch, reset hash table...")
        with self.q.mutex:
            self.q.queue.clear()
        self.inference_tracker = dict()
        return distilled_result

    

test = InferenceDistiller()
### Unit Test
### Enqueue correctly given inference result ###
### Dequeue (Release elements correctly) - queue empty after release ###
### output is correct (most inference - highest value count) & hash table resets accordingly ###