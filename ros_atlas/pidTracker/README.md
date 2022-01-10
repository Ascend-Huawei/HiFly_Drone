# ROS PID Face-Tracker
PID Tello-Tracker is a pseudo-tracking algorithm that is based on a PID Controller using object detection inference results from the Atlas 200 DK as continuous feedback to determine the state of the drone. 

**For more information, refer to the [PID Tracker Wiki Page](https://github.com/Ascend-Huawei/HiFly_Drone/wiki/Closed-Loop-PID-Tracker)**

#### What it is:
- A PID Face Tracker written in Python3.7.5
- Mainly based on [ROS smach](http://wiki.ros.org/smach) and [ROS actionlib](http://wiki.ros.org/actionlib)
- AI and Edge-enabled tracking

#### System Diagram

1. Each `SimpleActionState` in StateMachineClient (left) makes a request to their corresponding `SimpleActionState` in PIDActionServer (middle)
2. Upon receving a request from `SimpleActionState`, the `SimpleActionServer` executes its callback function to achieve its goal and send a response to the State (fail or success)
	- Callback functions may publish and subscribe to other nodes for tasks like: Inference and Postprocessing
3. Upon receving a response from `SimpleActionServer`, the StateMachineClient transitions to its next state based on the outcome of the previous State.

![ROS PID Tracker](https://github.com/jwillow19/HiFly_Drone/blob/main/.github/images/PID_SM.png)

#### Description of States in PIDTrack
|   States   |         Description           |
|:----------:|:-----------------------------:|
| `CONNECT`  | Connects to the drone. If successful, transition to `TAKEOFF`, else aborted |
| `TAKEOFF`  | Takeoff once the drone is connected. If successful, transition to `MANUAL`, else aborted |
| `MANUAL`   | Enable users to control the drone with the keyboard. Server will prompt for user input. If user entered "y" to prompt, server will engage in manual control loop. Inside the control-loop, user can enter "k" to kill the loop and move on to `PID` state or enter "l" to move on to `LAND` state. If user entered "n" to prompt, transition to `PID` state  |
| `PID`      | Executes the PID control-loop until the user terminates with keyboard interrupt. At which point, it will transition to the `LAND` state |
| `LAND`     | Lands the drone and abort the state machine |

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
4. Open a fourth terminal on the Atlas 200 DK and run the PIDActionServer under the `ros_atlas/pidTracker/` directory <br> 
	```
	source ~/catkin_ws/devel/setup.bash
	cd ~/HiFly_Drone/ros_atlas/pidTracker/
	python3 action_server_final.py
	```
 
5. Open a fifth terminal on the Atlas 200 DK and run the StateMachineClient under the `ros_atlas/pidTracker/` directory <br> 
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
