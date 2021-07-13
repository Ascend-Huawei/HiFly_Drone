import numpy as np
import matplotlib.pyplot as plt
import pickle
import glob
import argparse
import os



def parse_file(filename):
    test_run_list= None
    with open(filename, "rb") as f:
        test_run_list = pickle.load(f)

    # Deconstruct the list - extract:
    # NOTE: test_run_list is a List(Dicts) 
    #   1. pv_center and create a 3D plot with setpoint;  x_err overtime, yaw_velocity over time
    #   2. pv_bbox_area and create a 2D plot with setpoint; forward_backward_velocity over time
        
    x_err_lst = []
    yaw_lst = []
    pv_center_lst = []
    pv_bbox_area_lst = []

    for history in test_run_list:
        x_err_lst.append(history["x_err"])
        yaw_lst.append(history["yaw_velocity"])        
        pv_center_lst.append(history["pv_center"])        
        pv_bbox_area_lst.append(history["pv_bbox_area"])        

    return x_err_lst, yaw_lst, pv_center_lst, pv_bbox_area_lst


def plot(test_run, y, y_name):
    # Set point
        
    time = np.arange(0, len(y))
    fig, ax = plt.subplots()
    plt.plot(time, y, label=f"{y_name}")
    if y_name == "PV Center":
        plt.axhline(y=360, color="r", linestyle="-", label="Setpoint (Center): y")
        plt.axhline(y=480, color="r", linestyle="-", label="Setpoint (Center): x")
    elif y_name == "PV Bbox Area":
        plt.axhline(y=100000, color="r", linestyle="-", label="Setpoint (BBox Area) Lower threshold")
        plt.axhline(y=200000, color="r", linestyle="-", label="Setpoint (BBox Area) Upper threshold")
    elif y_name == "X error" or y_name == "Y error":
        plt.axhline(y=0, color="r", linestyle="-", label="Setpoint")

    plt.title(f"Test run {test_run}")
    plt.xlabel("Time (s)")
    plt.ylabel(y_name)
    plt.legend()
    plt.savefig(f"test_run/test_run_{test_run}_{y_name}")


def get_latest_run(dir_name):
    dir_list = os.listdir(dir_name) 
    fname = [int(filename.split(".")[0].split("_")[-1]) for filename in dir_list]
    fname.sort(key=lambda x: int(x))
    return fname[-1]


    
if __name__ == "__main__":
    latest_run = get_latest_run("test_run")

    # parser = argparse.ArgumentParser()
    # parser.add_argument("--rn", type=str, help="Run name to plot", required=True)
    # args = parser.parse_args()

    filename = f"test_run/test_run_{latest_run}.pkl"

    x_err_lst, yaw_lst, pv_center_lst, pv_bbox_area_lst = parse_file(filename)
    plot(str(latest_run), pv_center_lst, "PV Center")
    plot(str(latest_run), pv_bbox_area_lst, "PV Bbox Area")
    plot(str(latest_run), x_err_lst, "X error")
    plot(str(latest_run), yaw_lst, "YAW Velocity")
    print("Plots generated, exiting...")