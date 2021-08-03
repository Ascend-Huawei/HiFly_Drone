# Multiprocess Inference Manager

Multiprocess Inference Manager enables the Atlas 200 DK to perform asynchronous inference on Tello-UAV live video stream over several AI CPUs, thus improving overall inference and feedback time.  

**For more information, refer to the [PID Tracker Wiki Page](https://github.com/Ascend-Huawei/HiFly_Drone/wiki/Closed-Loop-PID-Tracker)**

### What it is:
- Written in Python 3.7.5
- uses the built-in `multiprocessing` Python library
- Uses djitellopy wrapper to extract video live stream 
- A Process Manager keeping track of incoming frames and distributing frames to available AI CPUs accordingly for faster inference and live-feedback on Presenter Server.


## Run Multiprocess-Inference-Manager
Assuming you have already gone through the main installation steps from the main [README](https://github.com/Ascend-Huawei/HiFly_Drone/tree/main) - to run this project, you will:
1. Turn on the Tello drone and connect your 200DK board to it via wireless router
2. Activate the virtual environment in the project directiory `~/HiFly_Drone` <br>
    `source venv/bin/activate`

3. Initialize Presenter Server<br>
    `cd src && bash lib/server/run_presenter_server.sh uav_presenter_server.conf`
    
4. Switch into the `parallel` directory from `src/`<br>
    `cd parallel`
    
5. In the terminal, run <br>
    `python3 run_parallel.py --model_name <model_name>`
    
The `run_parallel.py` accepts three optional arguments, see below table for the description of each optional argument.
|   Arguments             |         Description           |
|:-----------------------:|:-----------------------------:|
| `--model_name`          | Name of the inference module, i.e.: indoor_depth_estimation |
| `--fps`                 | Module's inference rate |
| `--num_processors`      | Number of Processors (AICPUs) to be used (up to 8) |


> NOTE: Refer to `utils/params.py` to see the list of supported inference modules for `--model_name`. In addition, `--fps` is value that varies depending on the inference module being used.


## Code Description
Multi-inference on different AICPU is handled by the `MultiProcessManager` class. `MultiProcessManager` grabs live video frames from the Tello UAV, store them into a Queue and 
pass the frames onto different AICPU for inference based on user-defined configuration (`fps` and `num_processors`). The resulting frames are dequeued and forwarded to the Presenter
Server for live-feedback. This implementation is shown to be able to improve inference and feedback rate for several supported modules.


|   File   |         Description           |
|:--------:|:-----------------------------:|
| `run_parallel.py`          | Main script to run asynchronous inference for one model |
| `MultiProcessManager.py` | `MultiProcessManager` Python class with methods to pass frames to queue, start inference loop to send to presenter server |
| `run.py` | Example script to show how to run inference on two different modules in parallel with `concurrent.futures` |
