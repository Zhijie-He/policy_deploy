# policy_deploy

## Dependencies
Ensure the following packages and libraries are installed:
- [MuJoCo](https://github.com/google-deepmind/mujoco/releases) (for physics simulation)
- [LibTorch](https://pytorch.org/get-started/locally/) (PyTorch C++ API)
- Eigen3: `sudo apt install libeigen3-dev`
- yaml-cpp: `sudo apt install libyaml-cpp-dev`
- pkg-config: `sudo apt install pkg-config`

## Environment Variables
Before building the project, make sure to set the following environment variables to point to the correct library paths:
```sh
# Set the LibTorch directory
export LIBTORCH_DIR=/path/to/libtorch

# Set the MuJoCo directory
export MUJOCO_DIR=/path/to/mujoco

# [Optional] Set the Unitree SDK 2 directory 
export UNITREE_SDK2_DIR=/path/to/unitree_sdk2
```
