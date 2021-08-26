import cv2
import logging
import sys
import time

sys.path.append('../')
sys.path.append('../lib')
from utils.tools import load_model_processor

"""start image -> inference -> postprocess -> result end"""
TEST_VIDEO_PATH = '/home/HwHiAiUser/HiFly_Drone/data/video.avi'

def main():
    # logging.basicConfig(level=logging.DEBUG, filename='example.log', filemode='w', )

    # Initialize FDModelProcessor
    mp, mp_info = load_model_processor('face_detection')
    model = mp(mp_info)

    # Fetch the first frame
    cap = cv2.VideoCapture(TEST_VIDEO_PATH)
    print(f"CameraPublisher - Default video read FPS={round(cap.get(cv2.CAP_PROP_FPS), 30)}")
    assert cap.isOpened(), 'VideoCapture not open'
    _, frame = cap.read()
    print(f'Obtained input')
    assert frame is not None, 'Frame is None'
    print(frame.shape)
    
    # start pipeline
    st = time.time()
    print(f'Passing input to pipeline')
    output_frame, _ = model.predict(frame)
    print(f'End of pipeline. Total time taken: {time.time() - st}')



if __name__ == '__main__':
    main()