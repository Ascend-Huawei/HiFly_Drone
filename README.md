# Atlas 200 DK x DJI Tello Ryze
Introducing the Ascend Eco-Platform for Intelligent UAVs - Enabled by the Atlas 200 DK and DJI Tello to achieve real-time deep learning solutions and fast prototyping to developers.
This project was created while keeping in mind of modularity and fast prototype development. We hope to build ready-to-use modularized capabilities for developers interested in taking deep learning to the realm of UAVs.

### Useful Links
[Official Atlas 200 DK Developer Kit](https://support.huaweicloud.com/intl/en-us/environment-deployment-Atlas200DK202/atlased_04_0029.html "Atlas 200 DK Upgrade")<br>
[Ascend Samples](https://gitee.com/ascend/samples) <br>
[🐶 Ascend ModelZoo](https://www.hiascend.com/en/software/modelzoo)<br>
[📹 HiFly Introduction YouTube Video](https://youtu.be/zZQy9RBLlEo)<br>
[🚁 HiFly PID Tracker Video](https://www.youtube.com/watch?v=Gix8SquAKGM&t=2s)<br>

### Hardware Requirements
- [Atlas 200 DK](https://e.huawei.com/ph/products/cloud-computing-dc/atlas/atlas-200)
- [DJI Tello Ryze](https://www.ryzerobotics.com/tello)
- Wireless Router (TP-Link TL-WR902AC)

### 📓 HiFly Wiki
- [RoadMap](https://github.com/Ascend-Huawei/HiFly_Drone/wiki/Roadmap)
- [How to Contribute](https://github.com/Ascend-Huawei/HiFly_Drone/wiki/How-to-Contribute)
- [Project Ideas](https://github.com/Ascend-Huawei/HiFly_Drone/wiki/Requested-Extensions)
- [TP Link Wireless Router Setup](https://github.com/Ascend-Huawei/HiFly_Drone/wiki/TP-Link-Wireless-Router-Setup)
- [Project: Parallel Inference](https://github.com/Ascend-Huawei/HiFly_Drone/wiki/Multiprocess-Inference)
- [Project: PID Tracker](https://github.com/Ascend-Huawei/HiFly_Drone/wiki/Closed-Loop-PID-Tracker)

### Extension Projects
- [Gesture Controlled Drone](https://github.com/Ascend-Huawei/gesture-controlled-drone)

<hr>

## ROS implementation
The following is for simple Python Implementation. For ROS implementation, please refer to [this guide](https://github.com/Ascend-Huawei/HiFly_Drone/tree/ros_atlas)

## Table of Content
[Installation](#installation)<br>
[How to run the project](#how-to-run-a-simple-project)<br>
[Code Implementation and how to extend](#code-implementation)<br>
[Available Modules](#available-modules)

## Installation
1. Login to Atlas 200 DK from PC (Refer to this [guide](https://www.notion.so/hiascend/Atlas-200-DK-Setup-Guide-070b907c3c124381bdd6721618b81ef8) on how to setup and access). `Note`, it is required to use `VScode` with `Remote-SSH` extension to login remotely, otherwise you might not get the video stream to display on your PC.
2. On Atlas 200 DK, git clone this repo 
    (No internet access? Just try connecting Atlas 200 Dk to a router with Ethernet Cable. For details, check [official document](https://support.huaweicloud.com/intl/en-us/environment-deployment-Atlas200DK1012/atlased_04_0012.html)) 

    `git clone  https://github.com/Ascend-Huawei/HiFly_Drone.git`

3. Navigate to the project directory: 

    `cd HiFly_Drone`

4. Create and activate python virutal environment: 

    `python3 -m venv hifly && source hifly/bin/activate`

5. Install the required dependencies to run this project:

    `pip3 install -r requirements.txt`

## How to run a simple project
This section covers how to run inference on UAV's camera livefeed. 

### Prerequisite
Knowing how to build an AI inference application on Ascend AI processor is required for following steps. Bascially, you will need to firstly get an inference offline model (`.om` file) by converting a pretrained deep learning model (TensorFlow/PyTorch/MindSpore/Caffe) using the `Ascend Tensor Compiler (ATC)` tool. The APIs to run the om model is `AscendCL`. A quick guide and experiment can be found [here](https://www.notion.so/hiascend/Public-Huawei-Ascend-Community-0308233b512e4a55b261dd00e1ad565b). 

1. Get the model: refer to the [list of supported modules](#available-modules) below and get the model you wish to run. Once downloaded, rename the '.om' and store the file inside the `models` subdirectory. 

2. Turn on DJI Tello and connect it to the 200 DK via a wireless router. For more details, please refer to the [TP Link Wireless Router Setup Guide](https://github.com/Ascend-Huawei/HiFly_Drone/wiki/TP-Link-Wireless-Router-Setup).
    >Note that you are not limited to only TPLink Wireless Routers 

3. Once the offline model is in place and the UAV is connected to the 200 DK. Activate the virtual environment and navigate to the project's `src` subdirectory:

    1. To prepare the presenter server to view live video footage

        `bash lib/server/run_presenter_server.sh uav_presenter_server.conf`
    
    2. Run real-time inference

        `python3 main.py`

4. Running the above will prompt a manual on the terminal. Select the task and models you wish to use. The inference result can be found at http://127.0.0.1:7007.

    Click [here](#code-implementation) to learn more about what the program is doing in the back

## How to run face tracking project
1. Download the [YOLO Face Detector](https://gitee.com/ascend/samples/tree/master/python/contrib/head_pose_picture) (`yolo_model.om`), rename it as `face_detection.om`, and store the file inside the `models` subdirectory. 
2. Follow this [guide](src/pid_controllers) to run the PIDtracker. (Set `--use_ps=True` for live stream on PC)

## Code Implementation

The modularity feature of this project is enabled by the `ModuleSelector`. It takes the user's specification and dynamically load the corresponding  `Processor`  to handle real-time inference    

|   File   |         Description           |
|:--------:|:-----------------------------:|
| `main.py`| Main script to invoke the interface and prompts user to select a model for inference      |
| `ModuleSelector.py`     | `ModuleSelector` Python class that takes the user's specifications and decides which task-specific `Processor` to load for inference |
| `BaseProcessor.py`      | Parent Processor class responsible for initialization of ACL resources and Model parameters. Each inference model have their respective `Processor` (child of `BaseProcessor`) for pre-and-post processing |
| `params.py`             | Hash table storing information for each task (which offline model to use and its corresponding Processor)  

### Project Extension
To add your own inference module to this project, you need:

1. An offline model stored in the `model` subdirectory (you can view the projects in [Ascend Samples](https://gitee.com/ascend/samples) for more ideas or use the Ascend Tensor Compiler [ATC] to convert .caffe or .pb models into .om models).

2. Write a custom `Processor` class (inherited from `BaseProcessor`) to take care of the offline model's inputs and outputs by overriding the preprocess and postprocess methods.

3. Register the model's info to the dictionary in `src/utils/params.py` such that the `ModuleSelector` will know which Processor and model to load during selection

    For example, to add a YOLO face detector module for object detection, you would add another dictionary item inside object_detection. The dictionary is required to have the following keys to run the `ModuleSelector` and `Processor`: `model_width`, `model_height`, `model_path`, `model_processor`
    ```python
    "object_detection": {
        "face_detection": {
                "model_width": 416,
                "model_height": 416,
                "model_path": os.path.join(paths["MODEL_PATH"], "face_detection.om"),
                "model_processor": "FaceDetectionProcessor",
                "model_info": "<model description>",
                "camera_width": "<optional parameters>"960,
                "camera_height": 720,
            }
    }
    ```
    > Note that you may also pass in other parameters in the dictionary for later uses by deconstructing them in the `params` argument in your `Processor` class

## Available Modules
A list of integrated modules.
|   Model   |         Rename the OM            | Terminal Manual
|:--------:|:-----------------------------:|:-----------------------------:|
| [YOLO Face Detector](https://gitee.com/ascend/samples/tree/master/python/contrib/head_pose_picture)|`face_detection.om`|`object_detection`->`face_detection`|
| [YOLOv3 Object Detection](https://gitee.com/ascend/samples/tree/master/python/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture) |`yolov3.om`|`object_detection`->`object_detection`|
| [Hand Detection](https://gitee.com/ascend/samples/tree/master/python/contrib/hand_detection_Gitee)|`hand_detection.om`|`object_detection`->`hand_detection`|
| [Indoor Depth Estimation](https://gitee.com/HardysJin/samples/tree/master/python/contrib/indoor_depth_estimation_picture)|`indoor_depth_estimation.om`|`depth_estimation`->`indoor_depth_estimation`|


