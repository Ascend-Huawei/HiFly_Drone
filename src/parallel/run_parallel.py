
import sys
import argparse

sys.path.append("..")
sys.path.append("../lib")

from MultiProcessManager import MultiProcessManager
from utils.tools import init_presenter_server
from utils.uav_utils import connect_uav

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Multiprocessing Inference setting")
    parser.add_argument("--fps", type=int, default=15, help="Inference FPS")
    parser.add_argument("--num_processors", type=int, default=3, help="Number of Inference Processors")
    parser.add_argument("--model_name", type=str, required=True, help="Number of Inference Processors")

    args = parser.parse_args()

    uav = connect_uav()
    uav.streamon()

    chan = init_presenter_server()

    mp_manager = MultiProcessManager(chan=chan, uav=uav, model_name=args.model_name, num_infer_processes=args.num_processors, fps=args.fps)
    mp_manager.start()
    


