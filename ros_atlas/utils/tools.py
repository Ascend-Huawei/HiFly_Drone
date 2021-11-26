import acl
import sys
import os
from importlib import import_module

sys.path.append("..")
sys.path.append("../lib")
sys.path.append("../../")

from ros_atlas.utils.params import params
from ros_atlas.lib.atlas_utils.acl_image import AclImage

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
