"""
Closed-Loop Detection + Tracking System
Control relies on feedback from in a closed-loop manner - enable drone to automatically adjust itself without user intervention to detect and track an
object of interest

+ Visual Servoing with Robotics with object detection

"""
import time
import sys
import os
import cv2
from importlib import import_module
from datetime import datetime
import argparse

sys.path.append("..")
sys.path.append("../lib")
sys.path.append("../model_processors")

from utils.params import params
from atlas_utils.presenteragent import presenter_channel
from atlas_utils.acl_image import AclImage

def init_presenter_server():
    SRC_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    PRESENTER_SERVER_CONF = os.path.join(SRC_PATH, "uav_presenter_server.conf")
    chan = presenter_channel.open_channel(PRESENTER_SERVER_CONF)
    if chan is None:
        raise Exception("Open presenter channel failed")
    return chan

def gen_save_name(tracker):
    now = datetime.now()
    save_name = now.strftime("%m%d%Y_%H%M%S")
    return f"PID_{tracker}_{save_name}"

def select_tracker(tracker_name):
    """Imports corresponding PIDTracker based on tracker_name"""
    tmp_dict = {
        'face': 'closed_loop.PIDFaceTracker',
        'person': 'closed_loop.PIDPersonTracker'
    }
    if tracker_name not in tmp_dict:
        raise Exception("Tracker not supported, see list of supported objects for tracking")
    tracker = import_module(tmp_dict[tracker_name])
    tracker = getattr(tracker, "PIDTracker")
    return tracker

def parser():
    parser = argparse.ArgumentParser(description="Tello UAV PID-Tracker Setting")
    parser.add_argument("--pid", nargs="+", help="PID List", default=[0.1, 0.1, 0.1])
    parser.add_argument("--flight_name", help="Flight run name", default=None)
    parser.add_argument("--save_flight", "-s", type=bool, help="Save flight statistics to pkl if True", default=False)
    parser.add_argument("--use_ps",  type=bool, help="Forward flight video to Presenter Server if True", default=False)
    parser.add_argument("--duration", "-d", type=int, help="Flight duration (in seconds)", default=120)
    parser.add_argument("--tracker", "-t", help="Tracker name", default="face")

    args = parser.parse_args()
    return args

def send_to_presenter_server(chan, frame_org, result_img) -> None:
    _, jpeg_image = cv2.imencode('.jpg', result_img)
    jpeg_image = AclImage(jpeg_image, frame_org.shape[0], frame_org.shape[1], jpeg_image.size)
    chan.send_detection_data(frame_org.shape[0], frame_org.shape[1], jpeg_image, [])

if __name__ == "__main__":
    args = parser()

    x_err, y_err = 0, 0
    tookoff = False
    flight_end = False
    timeout = time.time() + args.duration
    pid = [float(val) for val in args.pid]

    Tracker = select_tracker(args.tracker)
    tracker = Tracker(pid, save_flight_hist=args.save_flight)
    tracker.init_uav()
    
    if args.use_ps:
        chan = init_presenter_server()

    while not flight_end:
        try:
            if not tookoff:
                tookoff = True
                tracker.uav.takeoff()
                tracker.uav.move_up(70)
            
            if time.time() > timeout:        
                tracker.uav.land()
                tracker.uav.streamoff()
                flight_end = True
            
            frame_org = tracker.fetch_frame()
            x_err, y_err, result_img = tracker.run_state_machine(frame_org, x_err, y_err)

            if args.use_ps:
                send_to_presenter_server(chan, frame_org, result_img)

        except (KeyboardInterrupt, Exception) as e:
            tracker.uav.land()
            tracker.uav.streamoff()
            break

    if args.save_flight:
        save_file = gen_save_name(args.tracker) if args.flight_name is None else args.flight_name
        print(save_file)
        tracker.save_hist(save_file)

