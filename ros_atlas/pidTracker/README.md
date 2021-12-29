PID Tello-Tracker is a pseudo-tracking algorithm that is based on a PID Controller using object detection inference results from the Atlas 200 DK as continuous feedback to determine the state of the drone. 

**For more information, refer to the [PID Tracker Wiki Page](https://github.com/Ascend-Huawei/HiFly_Drone/wiki/Closed-Loop-PID-Tracker)**

### What it is:
- A PID Face Tracker written in Python3.7.5
- Mainly based on [ROS smach](http://wiki.ros.org/smach) and [ROS actionlib](http://wiki.ros.org/actionlib)
- AI-enabled tracking


## Run a FaceDetection-based PID Tracker
1. **On the Atlas 200 DK**, start the MasterNode with <br>
	`roscore`
  
2. Open a second terminal on the Atlas 200 DK and run the face-detection inference node under the `ros_atlas` directory <br>
	```
	source ~/catkin_ws/devel/setup.bash
	cd ~/HiFly_Drone/ros_atlas
	python3 FDNode.py
	```
3. Open a third terminal on the Atlas 200 DK and run the postprocessing node for face-detection under the `ros_atlas` directory <br>
	```
	source ~/catkin_ws/devel/setup.bash
	python3 FDProcessor.py
	```
4. Open a fourth terminal on the Atlas 200 DK and run the PIDActionServer under `ros_atlas/pidTracker/` directory <br> 
	```
	source ~/catkin_ws/devel/setup.bash
	cd ~/HiFly_Drone/ros_atlas/pidTracker/
	python3 action_server_final.py
	```
 
5. Open a fifth terminal on the Atlas 200 DK and run the StateMachineClient under `ros_atlas/pidTracker/` directory <br> 
	```
	source ~/catkin_ws/devel/setup.bash
	cd ~/HiFly_Drone/ros_atlas/pidTracker/
	python3 action_client_final.py
	```
	
6. **On your local machine (your laptop or desktop)**, open a docker visualization GUI <br>
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

## Code Description
The ROS version of the PID Tracker has two main components: ActionServer and StateMachineClient. A state machine is used to plan out transitions of states based on the outcome of said states. 
While the ActionServer takes care most of the logic to achieve the goal(s) set out by the various states. The `action_server_final.py`   

|   File   |         Description           |
|:--------:|:-----------------------------:|
| `action_server_final.py`          | ROS ActionServer that response to the different requests from various states coming from the client side. Mainly consists of several `SimpleActionServer` (SAS), each SAS answers to their state(s). |
| `action_client_final.py`          | State Machine that plans the transition of states for the PID Tracking application. The state machine uses `SimpleActionState` to makes request(s) to their corresponding SAS |
