import sys
import os

def validate_paths():
    """define paths to project, samples and home directory. Check if they exists"""
    SRC_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    PROJECT_DIR = os.path.dirname(SRC_PATH)
    MODEL_PATH = os.path.join(PROJECT_DIR, "model")
    PRESENTER_SERVER_SH = os.path.join(SRC_PATH, "lib/server/run_presenter_server.sh")
    PRESENTER_SERVER_CONF = os.path.join(SRC_PATH, "uav_presenter_server.conf")
    
    paths = {
        "SRC_PATH": SRC_PATH,
        "MODEL_PATH": MODEL_PATH,
        "PRESENTER_SERVER_SH": PRESENTER_SERVER_SH,
        "PRESENTER_SERVER_CONF": PRESENTER_SERVER_CONF
    }
    for k, p in paths.items():
        if not os.path.exists(p):
            # print("No such path: {}={}".format(k, p))
            raise Exception("No such path: {}={}".format(k, p))
    print("System paths all good")
    return paths

paths = validate_paths()

# Following Ascend Sample format: sample>python>level2-simple-inference
# with categories: classification, object detection, segmentation, recommendation, nlp, other
# where each task is a hash table with Model:configuration as key:val pair

params = {
    "task": {
        "classification": {

        },
        "object_detection": {
            "yolov3": {
                "model_width": 416,
                "model_height": 416,
                "model_path": os.path.join(paths["MODEL_PATH"], "yolov3.om"),
                # "model_processor": os.path.join(paths["SRC_PATH"], "ObjectDetectionProcessor.py"),
                "model_processor": "ObjectDetectionProcessor",
                "live_runner": "run_live_obj_detection",
                "model_info": "https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/yolov3/ATC_yolov3_caffe_AE"
            },
            "hand_detection": {
                "model_width": 300,
                "model_height": 300,
                "model_path": os.path.join(paths["MODEL_PATH"], "Hand_detection.om"),
                "model_processor": "HandDetectionProcessor",
                "live_runner": "run_live_hand_detection",
                "model_info": "<Example URL>"
            },
            "face_detection": {
                "model_width": 416,
                "model_height": 416,
                "model_path": os.path.join(paths["MODEL_PATH"], "face_detection.om"),
                "model_processor": "FaceDetectionProcessor",
                "live_runner": "run_live_face_detection",
                "model_info": "<Example URL>",
                "camera_width": 960,
                "camera_height": 720,
            }
        },
        "segmentation": {

        },
        "other": {

        }
    },
    "presenter_server_sh": paths["PRESENTER_SERVER_SH"],
    "presenter_server_conf": paths["PRESENTER_SERVER_CONF"]
    
}
