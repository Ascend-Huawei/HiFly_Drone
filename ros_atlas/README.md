# ROS Atlas HiFly
Introducing the Ascend Eco-Platform for Intelligent UAVs enabled by the Atlas 200 DK and DJI Tello (now with ROS!).


## 🤖 Install RoboStack on Atlas 200 DK
`RoboStack` is a pre-built Conda environment for any ROS distributions. See their repo for more info: [RoboStack/ros-noetic](https://github.com/RoboStack/ros-noetic).<br>

#### 🐍 Install Conda
Pleas visit the [official link (Miniconda Installers)](https://docs.conda.io/en/latest/miniconda.html) and choose `Miniconda3 Linux-aarch64 64-bit` to install the compatible version on the 200 DK.

#### Create Robotstack conda environment on Atlas 200 DK
1. Create a ros-noetic-desktop conda environment (replace `<env_name>` with your environment name)<br>
 `conda create -n <env_name> ros-noetic-desktop -c conda-forge -c robostack`
2. Activate the created conda environment<br>
 `conda activate <env_name>`
3. Install the python dependencies with `pip` in the conda environment <br>
	```pip3 install -r requirements.txt -y```
	> NOTE: Ensure conda is using `pip` within its environment and not using the global `pip`. One may check with `which pip` inside the conda env. 
4. Add the following lines to the `~/.bashrc` file and save the changes<br>
	```
	export ROS_MASTER_URI=http://192.168.1.2:11311
	export ROS_IP=192.168.1.2
	
	# The following commands are OPTIONAL 
	source ~/catkin_ws/devel/setup.bash
	conda activate <env-name>
	```
	> 👏 **NOTE**: The optional commands automatically sources the catkin setup file and activates the ROS conda environment. If you added the optional commands in the `.bashrc` then you do not have to manually run `source ~/catkin_ws/devel/setup.bash` or `conda activate <env-name>` anymore whenever you open a new terminal.
	
5. Verify the environment is working by running the MasterNode<br>
 `roscore`<br>
	> You should see the standard `roscore` output on your terminal, otherwise you should refer to `RoboStack/ros-noetic`'s README for a more detailed installation guide
<hr>


## 🐳 Install ROS-Docker on PC

For visualization purposes, we will run a ROS GUI (rqt) from inside a Docker container on the PC that listens to the ROS topics from the Atlas 200 DK. 
To do so, pull the official `ros:noetic` image from the `osrf` DockerHub repository on the host machine (your laptop or desktop)
```
docker pull osrf/ros:noetic-desktop-full
```
<hr>

## Setting up ROS packages
The following steps are required to compile the ROS messages used in this project. Refer to [Creating a ROS msg](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) for more details on how to create a ROS message.

0. Login to Atlas 200 DK from PC (Refer to this guide on how to setup and access). _(Note: it is required to use VScode with Remote-SSH extension to login remotely, otherwise you might not get the video stream to display on your PC.)_
1. Activate the ros-conda environment and create a catkin workspace in the home directory <br>
    ```
    cd
    conda activate ros-noetic
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    ```
    Refer to [Creating a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) for more details on how to create catkin workspace.
3. On the Atlas 200 DK, return to the home directory and git clone this repository <br>
    ```
    cd
    git clone https://github.com/Ascend-Huawei/HiFly_Drone.git
    ```
2. Navigate to the project directory<br>
    `cd HiFly_Drone/ros_atlas`<br>
2. Copy the contents under `ros_atlas` directory to your local catkin workspace <br>
    `cp -r * ~/catkin_ws/src/`
3. Navigate to your local catkin workspace _(the catkin_ws in this example is lcoated in the home directory)_ <br>
    `cd ~/catkin_ws`
4. Compile the source code into ROS packages with `catkin_make` <br>
    `catkin_make`
5. Add the workspace to your ROS environment by sourcing the generated setup file <br>
    `source devel/setup.bash`

> Please refer to [this documentation from ROS](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) for more resources on how to create a ROS package, 
<hr>

## Run Core pipeline with Face Detection 
This is a simple demonstration on how to run the pipeline with a FaceDetection model on livestreamed images from the drone. 
Before we begin, **ensure the Atlas 200 DK is connected to the drone before you run the pipeline**. In this example, we will use `roslaunch` to expedite the process of running multiple nodes in one application (ROS-package). Alternatively, you may also use `rosrun` to run any particular node within a ROS package.


1. **On the Atlas 200 DK**, Ensure you have followed the steps in [Setting up ROS packages](#setting-up-ros-packages) to compile and source the HiFly ROS Base project.
	
2. Open a terminal on the Atlas 200 DK and use roslaunch to run the base pipeline with a face-detection model.  <br>
	```
	# to run pipeline on a video file without using the drone
	roslaunch hifly_ros_base base_pipeline.launch use_uav:=False vid_in:=/path/to/a/video/file model:=face_detection
	
	# to run pipeline on livestream drone data (ensure drone is connected to the 200DK)
	roslaunch hifly_ros_base base_pipeline.launch model:=face_detection
	```
	
3. **[Optional] On your local machine (your laptop or desktop)**, open a docker visualization GUI <br>
	1. **On the host** Create a temporary container from the native `osrf/ros:noetic` image. Specify the environment variables and bind-mount volume (this command mounts (shares) the host's x11 unix socket)<br>
		```
		docker run -it --rm --net=host \
		--env="DISPLAY" \
		--env="ROS_MASTER_URI=http://192.168.1.2:11311" \
		--env="QT_X11_NO_MITSHM=1" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--name noetic-x11-newest \
		osrf/ros:noetic-desktop-full
		```
	2. **On the host** open another terminal and enable x11-unix-socket connection from the container you just created by disabling access control for Xserver so other clients can join. <br>
		```
		export containerId=$(docker ps -l -q)
		xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`	
		```
   	3. **Within the container** (back to the first terminal) run `rqt` to open the GUI <br>

<hr>

## Code Implementation
A brief summary on how the core nodes are implemented and how to extend them for your own application. The core nodes are located under `HiFly_Drone/ros_atlas/core`

![ROS Core Nodes](https://github.com/Ascend-Huawei/HiFly_Drone/blob/main/.github/images/ros_integration.png)
|   File   |         Description           |
|:--------:|:-----------------------------:|
| `CameraPublisher.py`   | Main script to publish livestream from drone to the `/tello/cam_data_raw` topic |
| `ACLInference.py` | Base class for running ACL inference models on Image data from `@topic:/tello/cam_data_raw`. Publishes the infered results to `@topic:/acl_inference/<model_name>`|
## Project Extension
To add your own inference module to this project, you need:

1. An offline model stored in the `model` subdirectory (you can view the projects in [Ascend Samples](https://gitee.com/ascend/samples) for more ideas or use the Ascend Tensor Compiler [ATC] to convert .caffe or .pb models into .om models).

2. Register your offline model's info and metadata to the dictionary in `ros_atlas/utils/params.py`. For example, to add a YOLO face detector module for object detection, you would add a dictionary item under `object_detection`. The dictionary is required to have the following keys to run successfully: `model_width`, `model_height`, `model_path`, `model_processor`
    ```python
    "object_detection": {
        "face_detection": {
                "model_width": 416,
                "model_height": 416,
                "model_path": os.path.join(paths["MODEL_PATH"], "face_detection.om"),
                "model_processor": "FaceDetectionProcessor",
                "model_info": "<model description>",
            }
    }
    ```
    > Note that you may also pass in other parameters in the dictionary for later uses by deconstructing them in the `params` argument in your `Processor` class

3. Write a custom `<ModelName>Processor` class in `ros_atlas/model_processor/` (inherited from `BaseProcessor.py`) to take care of the offline model's inputs and outputs by overriding the preprocess and postprocess methods.
4. Provide the name of the model as an argument of the `roslaunch` command. For example: `roslaunch hifly_ros_base base_pipeline.launch model:=<your model>`
