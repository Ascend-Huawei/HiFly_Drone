# ROS Atlas HiFly
Introducing the Ascend Eco-Platform for Intelligent UAVs enabled by the Atlas 200 DK and DJI Tello (now with ROS!).


### Table-of-Contents
[ROS Conda Environment Installation](#install-ros-on-atlass-200-dk)<br>
[(Optional) ROS Docker Installation](#[optional]-install-ros-docker)<br>
[How to Run](#how-to-run-the-core-ROS-Nodes)

## Install RoboStack on Atlas 200 DK
`RoboStack` is a pre-built Conda environment for any ROS distributions. See their repo for more info: [RoboStack/ros-noetic](https://github.com/RoboStack/ros-noetic).<br>

Please ensure you have `conda` installed before following the steps below.


1. Create a ros-noetic-desktop conda environment (replace `<env_name>` with your environment name)<br>
 `conda create -n <env_name> ros-noetic-desktop -c conda-forge -c robostack`
2. Activate the created conda environment<br>
 `conda activate <env_name>`
3. Verify the environment is working by running the MasterNode<br>
 `roscore`<br>
> You should see the standard `roscore` output on your terminal, otherwise you should refer to `RoboStack/ros-noetic`'s README for a more detailed installation guide
<hr>


## [Optional] Install ROS-Docker on PC/Laptop (for visualization) 

For visualization purposes, we may run a ROS GUI (rqt) from inside a Docker container on an external machine that listens to the topics on the Atlas 200 DK. 
To do so, pull the official `ros:noetic` image from the `osrf` DockerHub repository on the host machine (your laptop or desktop)
```
docker pull osrf/ros:noetic-desktop-full
```
<hr>

## Setting up dependent ROS packages

> **Prerequisite - Compile custom ROS messages** used in the project by copying `HiFly_Drone/ros_atlas/catkin_ws/src/custom_ros_msg/` to your catkin workspace and compile with `catkin_make` to create the `custom_ros_msg` ROS package. Refer to this guide: [Creating a ROS msg](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) for more details.

0. Login to Atlas 200 DK from PC (Refer to this guide on how to setup and access). _(Note: it is required to use VScode with Remote-SSH extension to login remotely, otherwise you might not get the video stream to display on your PC.)_
1. On the Atlas 200 DK, git clone this repository <br>
    `git clone https://github.com/Ascend-Huawei/HiFly_Drone.git`
2. Navigate to the project directory<br>
    `cd HiFly_Drone/ros_atlas`<br>
2. Copy the `catkin_ws` directory to your local catkin workspace. _(Note: the local catkin workspace should have been created in the **Prerequisite** step, otherwise refer to the guide under the prerequisite step.)_ <br>
    `cp -r ./catkin_ws/src/ ~/catkin_ws/src/`
3. Navigate to your local catkin workspace _(the catkin_ws in this example is lcoated in the home directory)_ <br>
    `cd ~/catkin_ws`
4. Compile the source code into ROS packages with `catkin_make` <br>
    `catkin_make`
5. Add the workspace to your ROS environment by sourcing the generated setup file <br>
    `source devel/setup.bash`

> Please refer to [this documentation from ROS](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) for more resources on how to create a ROS package, 


### Demo: Running the Face Detection pipeline on images from the drone
`CameraPublisher`, `BaseInference`, `BaseProcessor` are located in:
`~/HiFly_Drone/ros_atlas/base_nodes/`

1. Start MasterNode in Terminal 1: run `roscore`
2. Start face-detection inference node in Terminal 2: `python3 FDNode.py`
3. Start face-detection post-processing node in Terminal 3: `python3 FDProcessor.py`
4. Start the CameraPublisher once the other nodes are ready in Terminal 4: 
    - to run with drone’s live feed: `python3 CameraPublisher.py --live-feed`
    - to run on a static video: `python3 CameraPubilsher.py —no-live-feed`
        - also, replace `@CameraPublish.line76` with your pre-recorded video’s file path
5. [Optional] Open a docker visualization GUI on your local machine:
    - Refer to the section [(Optional) ROS Docker Installation](#install-ros-docker).
    ```
    # On the host, create a container from the native osrf/ros:noetic image. Specify environment variables and bind-mount volume (this command mounts (shares) the host's x11 unix socket)
	docker run -it --rm --net=host \
	--env="DISPLAY" \
	--env="ROS_MASTER_URI=http://192.168.1.2:11311" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--name noetic-x11-newest \
	osrf/ros:noetic-desktop-full

    # On the host, open another terminal and enable x11-unix-socket connection from the container you just created.
        export containerId=$(docker ps -l -q)
	xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId` # disable access control for Xserver so other clients can join
   
    # Inside the container, run rqt to open the GUI
    	rqt
    ```
## Code Implementation
A brief summary on how the core nodes are implemented and how to extend them for your own application. The core nodes are located under `HiFly_Drone/ros_atlas/core`

|   File   |         Description           |
|:--------:|:-----------------------------:|
| `CameraPublisher.py`   | Main script to publish livestream from drone to the `/tello/cam_data_raw` topic |
| `BaseInference.py`     | Parent class to be inherited by models class for inference. Listens to `/tello/cam_data_raw`, make inference, and publish the inference results to `/acl_inferece/<model_name>`|
| `BasePostprocessor.py` | Parent class for postprocessing the inference results. Listens to `/acl_inference/<model_name>`, postprocess, and publish the final results to `/postprocess/<model_name>` |
