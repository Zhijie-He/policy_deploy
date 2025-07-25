# policy_deploy

## Dependencies
Ensure the following packages and libraries are installed:
- [LibTorch](https://pytorch.org/get-started/locally/) (PyTorch C++ API)
- pkg-config: `sudo apt install pkg-config`
- glfw3: `sudo apt install libglfw3-dev`
- yaml-cpp: `sudo apt install libyaml-cpp-dev`

## Environment Variables
Before building the project, make sure to set the following environment variables to point to the correct library paths:
```sh
# Set the LibTorch directory
export LIBTORCH_DIR=/path/to/libtorch
# Set the TensorRT directory
export TENSORRT_DIR=/path/to/TensorRT
# Set the cuda directory
export CUDA_DIR=/path/to/cuda
```

## Key Action
w	  Walk forward
a	  Move left
s	  Walk backward
d	  Move right
q	  Turn left
e	  Turn right

y	  Move to transfer pose (typically used before starting policy)
u	  Move to default home pose and start policy
x	  🚨 Emergency stop — immediately halts motion

## Executables Overview
1. robot_sim_sync.cpp
```sh
Usage:
  robot sim sync [OPTION...]

  -c, --config arg  Config name (e.g., mdl)
  -h, --help        Print usage
```
`./robot_sim_sync -c mdl`

2. run.cpp

Main control entry, supporting two execution modes:
```sh
Usage:
  run [OPTION...]

  -m, --mode arg       Mode: sim2mujoco or sim2real
  -c, --config arg     Config name: mdl
      --headless       Run in headless mode (no GUI)
  -d, --device arg     Device to use: cpu or cuda (default: cpu)
  -n, --net arg        Network interface name for sim2real (default: "")
      --engine arg     Inference engine type: libtorch | tensorrt (default: 
                       libtorch)
      --precision arg  Inference precision: fp32 | fp16 | int8 (default: 
                       fp32)
  -h, --help           Show help
```

Mode 1: Sim2Mujoco (Simulation Deployment)

Deploys the trained policy into a MuJoCo-based simulated environment.
`./run -m sim2mujoco -c mdl -d cpu --headless `

Available robot model options:
- mdl

Mode 2: Sim2Real (Real Robot Deployment)

Deploys the policy onto the real mdl robot and controls it via DDS.
` ./run -m sim2real -c mdl -net [net_interface] -d cuda`

Parameters:
- mdl: Currently, only this robot configuration is supported for real deployment.
- net_interface: Network interface used for communication, e.g., eno1, wlan0. currently not used in reality.

It is recommended to run simulate_robot alongside in this mode to simulate control command input.

3. simulate_robot.cpp
`./simulate_robot -c mdl`


4. start_low_state — Custom LowState Aggregator
Subscribes to IMU and motor state topics, merges them into a unified LowState message, and republishes it over DDS. Required in real robot mode.
`./start_low_state -c mdl`
