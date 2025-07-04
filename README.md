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

## Executables Overview
1. check_G1Robot_input.cpp

A standalone tool to verify whether the G1 robot's state data received via DDS matches the MuJoCo simulation data.

Useful for debugging and data consistency checks.
`./check_G1Robot_input`

2. g1.cpp

Main control entry, supporting two execution modes:
```sh
Usage:
  g1 [OPTION...]

  -m, --mode arg       Mode: sim2mujoco or sim2real
  -c, --config arg     Config name: g1_unitree | g1_eman | mdl
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
`./g1 -m sim2mujoco -c g1_eman  -d cpu --headless `

Available robot model options:
- g1_unitree
- g1_eman
- mdl

Mode 2: Sim2Real (Real Robot Deployment)

Deploys the policy onto the real G1 robot and controls it via DDS.
` ./g1 -m sim2real -c g1_eman -net [net_interface] -d cuda`

Parameters:
- g1_eman: Currently, only this robot configuration is supported for real deployment.
- net_interface: Network interface used for communication, e.g., eno1, wlan0.

It is recommended to run simulate_robot alongside in this mode to simulate control command input.

3. simulate_robot.cpp

Simulates policy outputs and sends command data over DDS, typically used to test sim2real deployments without actual control code.
`./simulate_robot`

4. deploy_mujoco.cpp

Quickly deploys a trained policy into a MuJoCo simulation for testing or visualization.
```sh
Usage:
  deploy_mujoco [OPTION...]

  -c, --config arg  Config name (e.g., g1_unitree, g1_eman, h1, h1_2)
      --headless    Run in headless mode (no GUI)
  -d, --device arg  Device to use: cpu or cuda (default: cpu)
  -h, --help        Show help
```

Used for test in the non-linux system, e.g. MacOS.


