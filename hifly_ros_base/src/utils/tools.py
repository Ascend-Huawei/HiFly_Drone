import os
from importlib import import_module
import rosbag
import rospy
import sys

# sys.path.append("../../")
# sys.path.append("..")       # for load_model_processor -- model_processor subpackage needs to be included in the namespace
from utils.params import params

def validate_paths():
    """define paths to project, samples and home directory. Check if they exists"""
    SRC_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    print(SRC_PATH)
    print(os.path.dirname(os.path.abspath(__file__)))

    PROJECT_DIR = os.path.dirname(SRC_PATH)
    MODEL_PATH = os.path.join(PROJECT_DIR, "model")
    
    paths = {
        "SRC_PATH": SRC_PATH,
        "MODEL_PATH": MODEL_PATH,
    }
    for k, p in paths.items():
        if not os.path.exists(p):
            # print("No such path: {}={}".format(k, p))
            raise Exception("No such path: {}={}".format(k, p))
    print("System paths all good")
    return paths

def load_model_processor(model_name, display_info=True):
        """Loads ModelProcessor
        :param:
            + detector_name - Key name of detection model, i.e. face_detection, indoor_depth_estimation
        Returns
            A ModelProcessor class and its parameters for initialization
        """
        MP = None
        for task, task_models in params['task'].items():
            if model_name not in task_models:
                continue
            
            model_info = task_models[model_name]
            processor = model_info["model_processor"]

            MP = import_module(f"model_processors.{processor}")
            MP = getattr(MP, "ModelProcessor")

            # Display model info
            if display_info:
                print(f"\n")
                print("################# Model Info #################")
                print(f"Model Info: {model_name}")
                for key, val in model_info.items():
                    print(f"{key}: {val}")
                print("############################################\n")

            return MP, model_info

        if MP is None:
            raise Exception("Model name not found in params, see params.py for supported models.")

def get_acl_rt_mem_info():
    """Prints Acl runtime memory info"""
    for i in range(10):
        free, total, ret = acl.rt.get_mem_info(i)
        print(f"({i}) free, totoal, ret: ", free, total, ret)

def read_bag_ts(bag_name) -> None:
    """Reads a ROSBag object and analyze the average time(stamp) between data.
    Args:
        bag_name (str)  name of the rosbag
    Returns:
        Nnoe
    """
    rospy.init_node('read_bag')
    time_between_frames = []
    bag = rosbag.Bag(bag_name)
  
    prev = None
    for topic, msg, timestamp in bag.read_messages(topics=[]):
        if prev is None: 
            prev = timestamp.to_sec()
            continue
        try:
            diff = timestamp.to_sec() - prev
            time_between_frames.append(diff)
            prev = timestamp.to_sec()
        except TypeError as e:
            print(prev())
            diff = timestamp - prev()
            time_between_frames.append(diff)
            prev = timestamp.to_sec()
            
    avg_frame_out_time = round(sum(time_between_frames) / len(time_between_frames), 3)
    print(f'Average time to publish result frame: {avg_frame_out_time}s ')
    print(f'FPS = {1 / avg_frame_out_time}')

    rospy.signal_shutdown('Read bag complete')