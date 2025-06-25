# policy_deploy

## Dependencies
Ensure the following packages and libraries are installed:
<!-- - [MuJoCo](https://github.com/google-deepmind/mujoco/releases) (for physics simulation) -->
- [LibTorch](https://pytorch.org/get-started/locally/) (PyTorch C++ API)
- glfw3: `sudo apt install libglfw3-dev`
- yaml-cpp: `sudo apt install libyaml-cpp-dev`
- pkg-config: `sudo apt install pkg-config`

## Environment Variables
Before building the project, make sure to set the following environment variables to point to the correct library paths:
```sh
# Set the LibTorch directory
export LIBTORCH_DIR=/path/to/libtorch
```

## Executables Overview
1. check_G1Robot_input.cpp

A standalone tool to verify whether the G1 robot's state data received via DDS matches the MuJoCo simulation data.

Useful for debugging and data consistency checks.
`./check_G1Robot_input`


2. g1.cpp

Main control entry, supporting two execution modes:

Mode 1: Sim2Mujoco (Simulation Deployment)

Deploys the trained policy into a MuJoCo-based simulated environment.
`./g1 sim2mujoco g1_unitree cuda`

Available robot model options:
- g1_unitree
- g1_eman
- h1
- h1_2

Mode 2: Sim2Real (Real Robot Deployment)

Deploys the policy onto the real G1 robot and controls it via DDS.
`./g1 sim2real g1_eman cpu net_interface`

Parameters:
- g1_eman: Currently, only this robot configuration is supported for real deployment.
- net_interface: Network interface used for communication, e.g., eno1, wlan0.

It is recommended to run simulate_robot alongside in this mode to simulate control command input.

3. simulate_robot.cpp

Simulates policy outputs and sends command data over DDS, typically used to test sim2real deployments without actual control code.
`./simulate_robot`

4. deploy_mujoco.cpp

Quickly deploys a trained policy into a MuJoCo simulation for testing or visualization.
`./deploy_mujoco`

Used for test in the non-linux system, e.g. MacOS.


