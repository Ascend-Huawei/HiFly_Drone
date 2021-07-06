"""
Sandbox for testing parallel inference (Depth Estimation + Object Detection)

Navie implementation:
0. Spawn ModelProcessor instances
1. Thread grabs frame - store to queue for callback afterwards, and send frame to 2 models
2. Once inference is done, send signal to dequeue
3. Result fusion and send to presenter server
"""

from utils.params import params
from model_processor.IndoorDepthProcessor import ModelProcessor as DEModelProcessor
from model_processor.ObjectDetectionProcessor import ModelProcessor as ODModelProcessor
from DataFeeder import Feeder
from RunLive import LiveRunner

de_fps = 3
feeder = Feeder(3)

demp = DEModelProcessor(params["task"]["depth_estimation"]["indoor_depth_estimation"])
odmp = ODModelProcessor(params["task"]["object_detection"]["yolov3"])

while True:
    frame_org = self.uav.get_frame_read().frame
    assert frame_org is not None, "Error: Tello video capture failed, frame is None"

    feeder.start_threads(frame_org)
    od_result = odmp.predict(frame_org)
    de_result = demp.predict(frame_org)
    signal = True if od_result and de_result else None

    if signal:
        fused_output = feeder.fuse(od_result, de_result)

        _, jpeg_image = cv2.imencode('.jpg', fused_output)
        jpeg_image = AclImage(jpeg_image, frame_org.shape[0], frame_org.shape[1], jpeg_image.size)
        chan.send_detection_data(frame_org.shape[0], frame_org.shape[1], jpeg_image, [])