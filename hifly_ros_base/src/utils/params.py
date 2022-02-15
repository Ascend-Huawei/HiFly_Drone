import os


def validate_paths():
    """define paths to project, samples and home directory. Check if they exists"""
    ROS_ATLAS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    PROJECT_DIR = os.path.dirname(ROS_ATLAS)
    MODEL_PATH = os.path.join(PROJECT_DIR, "model")
    
    paths = {
        "ROS_ATLAS": ROS_ATLAS,
        "MODEL_PATH": MODEL_PATH,
    }
    for k, p in paths.items():
        if not os.path.exists(p):
            raise Exception("No such path: {}={}".format(k, p))
    return paths

paths = validate_paths()

# Following Ascend Sample format: sample>python>level2-simple-inference
# with categories: classification, object detection, segmentation, recommendation, nlp, other
# where each task is a hash table with Model:configuration as key:val pair

params = {
    "task": {
        "classification": {
            "gesture_yuv": {
                "model_width": 256,
                "model_height": 224,
                "model_path": os.path.join(paths["MODEL_PATH"], "gesture_yuv.om"),
                "model_processor": "HandGestureProcessor",
                "model_info": "https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/gesture_recognition/ATC_gesture_recognition_Caffe_AE"
            },

        },
        "object_detection": {
            "yolov3": {
                "model_width": 416,
                "model_height": 416,
                "model_path": os.path.join(paths["MODEL_PATH"], "yolov3.om"),
                "model_processor": "ObjectDetectionProcessor",
                "model_info": "https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/yolov3/ATC_yolov3_caffe_AE"
            },
            "hand_detection": {
                "model_width": 300,
                "model_height": 300,
                "model_path": os.path.join(paths["MODEL_PATH"], "hand_detection.om"),
                "model_processor": "HandDetectionProcessor",
                "model_info": "https://gitee.com/ascend/samples/tree/master/python/contrib/hand_detection_Gitee",
            },
            "face_detection": {
                "model_width": 416,
                "model_height": 416,
                "model_path": os.path.join(paths["MODEL_PATH"], "face_detection.om"),
                "model_processor": "FaceDetectionProcessor",
                "model_info": "WHENET FaceDetector",
                "camera_width": 960,
                "camera_height": 720
            },
            "centernet": {
                "model_width": 512,
                "model_height": 512,
                "model_path": os.path.join(paths["MODEL_PATH"], "Centernet_2batch_input_fp16_output_fp32.om"),
                "model_processor": "CenterNetProcessor",
            }

        },
        "object_tracking":{
            "pedestrian_tracking": {
                "model_width": 1088,
                "model_height": 608,
                "model_path": os.path.join(paths["MODEL_PATH"], "mot_v2.om"),
                "model_processor": "PedestrianTrackProcessor",
                "live_runner": "run_live_pedestrian_track",
                
                "args": {
                    "conf_thres": 0.35,
                    "track_buffer": 30,
                    "min_box_area": 100,
                    "K": 100,
                    "input_video": os.path.join(paths["ROS_ATLAS"], "../data/london.mp4"),
                    "output_root": os.path.join(paths["ROS_ATLAS"], "../outputs"),
                    "output_type": "images",
                    "mean": [0.408, 0.447, 0.470],
                    "std": [0.289, 0.274, 0.278],
                    "down_ratio": 4,
                    "num_classes": 1,
                    "inp_shape": [608, 1088],
                    "img0_shape": [720, 960]
                },
            },
        },
        "segmentation": {

        },
        "depth_estimation":{
            "indoor_depth_estimation": {
                "model_width": 640,
                "model_height": 480,
                "model_path": os.path.join(paths["MODEL_PATH"], "indoor_depth_estimation.om"),
                "model_processor": "IndoorDepthProcessor",
                "model_info": "https://github.com/shariqfarooq123/AdaBins",
                "camera_width": 960,
                "camera_height": 720,
            },
        },
        "other": {

        },
    },
}
