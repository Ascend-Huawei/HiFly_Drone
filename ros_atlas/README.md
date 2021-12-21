# ROS Atlas HiFly
Introducing the Ascend Eco-Platform for Intelligent UAVs enabled by the Atlas 200 DK and DJI Tello (now with ROS!).


### Table-of-Contents
[ROS Conda Environment Installation](#install-ros-on-atlass-200-dk)<br>
[(Optional) ROS Docker Installation](#install-ros-docker)<br>
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


## Install ROS-Docker

> NOTE: This step is optional. It allows you to visualize the data on an external machine by running a ROS environment inside a Docker container. 
To do so, pull the official `ros:noetic` image from the `osrf` DockerHub repository.
```
##### On Host-side ######
# Pull the image
docker pull osrf/ros:noetic-desktop-full

# check current docker images, verify image pulled successfully
docker image ls

# Create a container from the native osrf/ros:noetic image and specify env and bind-mount volume
# (this command mounts (shares) the host's x11 unix socket
docker run -it --rm --net=host \
--env="DISPLAY" \
--env="ROS_MASTER_URI=http://192.168.1.2:11311" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--name noetic-x11-newest \
osrf/ros:noetic-desktop-full

# although x11-unix-socket is shared, we need to enable connection to it from 
# the container. We do this by exporting the recently started container's ID as a variable
export containerId=$(docker ps -l -q)

# then we disable access control for Xserver for that specific docker container using its ID (which we saved earlier)
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId` # disable access control for Xserver so other clients can join


###### Inside the Container ########
	apt update 
	source /ros_entrypoint.sh # or you might need to source setup.bash under /opt/noetic
	# install libraries/packages Ie: git, ping, arp-scan, python3-pip, python3-venv
	# apt install python3-pip git arp-scan iputil etc...

```

## How to run the core ROS Nodes

### Preparation step: 
Compile the custom ROS message types used in the project by copying `HiFly_Drone/ros_atlas/catkin_ws/src/custom_ros_msg/` to your catkin workspace and then compiling with `catkin_make` to create the `custom_ros_msg` ROS package.

1. Navigate to the project directory<br>
    `cd HiFly_Drone/ros_atlas`<br>
2. Copy the `catkin_ws` directory to your local catkin workspace<br>
    `cp -r ./catkin_ws/src/ ~/catkin_ws/src/`
3. Navigate to your local catkin workspace _(the catkin_ws in this example is lcoated in the home directory)_ <br>
    `cd ~/catkin_ws`
4. Compile the source code into ROS packages with `catkin_make`<br>
    `catkin_make`
5. Add the workspace to your ROS environment by sourcing the generated setup file
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
        - also, replace @CameraPublish.line76 with your pre-recorded video’s file path
5. [Optional] Open a docker visualization GUI on your local machine: and start `rqt`
    - Refer to the section [(Optional) ROS Docker Installation](#install-ros-docker) to see how to do this.
