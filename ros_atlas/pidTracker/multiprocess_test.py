import sys
from multiprocessing import Pool, Process
import subprocess

sys.path.append("../..")
from ros_atlas.utils.uav_utils import connect_uav
from ros_atlas.base_nodes.CameraPublisher import CameraPublisher


def spawn_cam_pub_subprocess(device):
    imgPub = CameraPublisher(uav=device)
    imgPub.start_publish_live()

if __name__ == "__main__":
    uav = connect_uav()
    
    p = Process(target=spawn_cam_pub_subprocess, args=(uav,)) 
    p.start()   # streamon failed
    p.join()

    uav.streamon() # streamon ok