# Multiprocess Inference Manager

Multiprocess Inference Manager enables the Atlas 200 DK to perform asynchronous inference on Tello-UAV live video stream over several AI CPUs, thus improving overall inference and feedback time.  

**For more information, refer to the [MultiProcessManager Wiki Page](https://github.com/Ascend-Huawei/HiFly_Drone/wiki/Multiprocess-Inference)**

### What it is:
- Written in Python 3.7.5
- uses `djitellopy` to extract video live stream from Tello UAV and the built-in `multiprocessing` Python library
- A Process Manager keeping track of incoming frames and distributing frames to available AI CPUs accordingly for faster inference and live-feedback on Presenter Server. 

## Run Async-Inference using MultiProcessManager
Assuming you have already gone through the main installation steps from the main [README](https://github.com/Ascend-Huawei/HiFly_Drone/tree/main) - to run this project, you will:
1. Turn on the Tello drone and connect your 200DK board to it via wireless router
2. Activate the virtual environment in the project directiory `~/HiFly_Drone` <br>
    `source venv/bin/activate`

3. Initialize Presenter Server<br>
    `cd src && bash lib/server/run_presenter_server.sh uav_presenter_server.conf`
    
4. Switch into the `parallel` directory from `src/`<br>
    `cd parallel`
    
5. In the terminal, run <br>
    `python3 run_async.py --model_name <model_name>`
    
`run_async.py` uses the `MultiProcessManager` to run asynchronous inference on several processors for one model to speed up feedback rate. The script accepts three optional arguments, see below table for the description of each optional argument.
|   Arguments             |         Description           |
|:-----------------------:|:-----------------------------:|
| `--model_name`          | Name of the inference module, i.e.: indoor_depth_estimation |
| `--fps`                 | Module's inference rate |
| `--num_processors`      | Number of Processors (AICPUs) to be used (up to 8) |


> NOTE: Refer to `utils/params.py` to see the list of supported inference modules for `--model_name`. In addition, `--fps` is value that varies depending on the inference module being used.

## Run Parallel Inference
Follow steps 1-4 as in Async-Inference and replace step 5 with the following:<br>
    `python3 run_parallel.py --models <model_1> <modeL_2>`
   
`run_parallel.py` uses `concurrent.futures` to make inference using two different models in parallel on a video without speed up. See below table for the description of each optional argument.
|   Arguments             |         Description           |
|:-----------------------:|:-----------------------------:|
| `--models`              | The name of the two models to be ran in parallel, i.e.: indoor_depth_estimation and yolov3 |
| `--num_processors`      | Number of Processors (AICPUs) to be used (up to 8) |
| `--vid_in`              | Path to the video input |
| `--fuse`                | Perform result fusion and send to presenter server if True, otherwise the frame-by-frame results are saved in individual data directories |


> NOTE: Refer to `utils/params.py` to see the list of supported inference modules for `--models`.



## Code Description
Multi-inference on different AICPU is handled by the `MultiProcessManager` class. `MultiProcessManager` grabs live video frames from the Tello UAV, store them into a Queue and 
pass the frames onto different AICPU for inference based on user-defined configuration (`fps` and `num_processors`). The resulting frames are dequeued and forwarded to the Presenter Server for live-feedback. This implementation is shown to be able to improve inference and feedback rate for several supported modules.


|   File   |         Description           |
|:--------:|:-----------------------------:|
| `run_async.py`           | Main script to run inference asynchronously for one model |
| `run_parallel.py`        | Example script to show how to run inference on two different modules in parallel with `concurrent.futures` |
| `MultiProcessManager.py` | `MultiProcessManager` Python class for `run_async.py` with methods to put frames to queue, start inference loop to send to presenter server |
