# Workflow
Starting point...
[https://nvidia-isaac-ros.github.io/reference_workflows/isaac_perceptor/run_perceptor_in_sim.html](https://nvidia-isaac-ros.github.io/reference_workflows/isaac_perceptor/run_perceptor_in_sim.html)
[https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment](https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment)
[https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)
[https://nvidia-isaac-ros.github.io/robots/nova_carter/getting_started.html#nova-carter-dev-setup](https://nvidia-isaac-ros.github.io/robots/nova_carter/getting_started.html#nova-carter-dev-setup)

## Start the IsaacROSDev Container (from the workspace...)
./src/isaac_ros_common/scripts/run_dev.sh

## Add the following packages....

### Isaac ROS repos...
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth.git
git clone https://github.com/NVIDIA-ISAAC-ROS/nova_carter.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_jetson.git
git vlone https://github.com/NVIDIA-ISAAC-ROS/isaac_perceptor.git

### Application Repos...
git clone https://github.com/robosoft-ai/sm_isaac_perceptor_1.git
git clone https://github.com/robosoft-ai/rrt_exploration.git

### SMACC Repos
git clone https://github.com/robosoft-ai/SMACC2.git
git clone https://github.com/rapyuta-robotics/UE_msgs.git

(Todo: remove ue_msgs dependencies!)

## Add key dependencies...

### Install Nvblox From Debian...
sudo apt-get install -y ros-humble-isaac-ros-nvblox && \
rosdep update && \
rosdep install isaac_ros_nvblox

#### Download nvblox assets
sudo apt-get install -y curl tar

NGC_ORG="nvidia"
NGC_TEAM="isaac"
NGC_RESOURCE="isaac_ros_assets"
NGC_VERSION="isaac_ros_nvblox"
NGC_FILENAME="quickstart.tar.gz"

REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/versions/$NGC_VERSION/files/$NGC_FILENAME"


mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/${NGC_VERSION} && \
    curl -LO --request GET "${REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets/${NGC_VERSION} && \
    rm ${NGC_FILENAME}


### Install LTTng-UST...
sudo apt-get install lttng-tools
sudo apt-get install lttng-modules-dkms
sudo apt-get install liblttng-ust-dev

## Build Workspace
colcon build --symlink-install 
source workspaces/isaac_ros-dev/install/setup.bash

## Launch Application
source install/setup.bash 
ros2 launch sm_isaac_perceptor_1 sm_isaac_perceptor_1_launch.py 

