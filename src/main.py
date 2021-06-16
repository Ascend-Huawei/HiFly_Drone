"""
Oject-Detection Module:
NOTE - manual movement, get stream from drone, preprocess the stream (possible in real time?), detect object in the scene
"""

import sys
import os
import time

from utils.uav_utils import connect_uav 
from utils.params import params
from utils.ModuleSelector import ModuleSelector
from utils.RunLive import LiveRunner

if __name__ == '__main__':
    # Get user input and choose module
    selector = ModuleSelector(params)
    selector.user_input()
    
    print(f"Preparing {selector.model} for {selector.task}...")
    print("Importing relevant ModelProcessor and LiveRunner")

    # run the presenter_server script
    # run_presenter_server_sh = params["presenter_server_sh"] 
    # os.system(f"bash {run_presenter_server_sh} uav_presenter_server.conf")

    # Connect to Tello drone
    uav = connect_uav()

    runner = LiveRunner(uav, params, selector)
    # needed to fully connect to presenter server?
    time.sleep(10)
    runner.display_result()

