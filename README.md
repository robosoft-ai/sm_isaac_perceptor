# Workflow
## Setting up an IsaacROSDev Container
[https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment](https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment)  
[https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)  
[https://nvidia-isaac-ros.github.io/robots/nova_carter/getting_started.html#nova-carter-dev-setup](https://nvidia-isaac-ros.github.io/robots/nova_carter/getting_started.html#nova-carter-dev-setup)  

## Reference Example
[https://nvidia-isaac-ros.github.io/reference_workflows/isaac_perceptor/run_perceptor_in_sim.html](https://nvidia-isaac-ros.github.io/reference_workflows/isaac_perceptor/run_perceptor_in_sim.html)  

## Let's Get Started
We begin by cloning isaac_ros_common to the src folder of our local workspace. My local workspace is ~/workspace/humble_ws
 ```
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/nova_carter.git  
 ```
## Start the IsaacROSDev Container (from the workspace...)
 ```
./src/isaac_ros_common/scripts/run_dev.sh  -d  ~/workspace/humble_ws/
 ```
Then run ls in the terminal to confirm that isaac_ros-dev is set to your host host workspace.  
or this one from inside workspace/src/isaac_ros_common/scripts. The -d command explicity sets the workspace to workspaces/isaac_ros-dev inside the IsaacROSDev container.
```
./run_dev.sh -d  ~/workspace/humble_ws/
```
## Installations onto the Container....

### Install LTTng-UST...
We'll need this for SMACC later...  
 ```
sudo apt-get update  
sudo apt-get install lttng-tools  
sudo apt-get install lttng-modules-dkms  
sudo apt-get install liblttng-ust-dev  
 ```
### Install Jetson Stats
 ```
sudo apt-get install -y ros-humble-isaac-ros-jetson-stats
 ```
### Use rosdep to install Nova Carter bringup dependencies...
 ```
rosdep update
rosdep install -i -r --from-paths ${ISAAC_ROS_WS}/src/nova_carter/nova_carter_bringup/ --rosdistro humble -y
 ```
### Install Nvblox From Debian...
 ```
sudo apt-get install -y ros-humble-isaac-ros-nvblox && \
rosdep update && \
rosdep install isaac_ros_nvblox
 ```
#### Download nvblox assets
 ```
sudo apt-get install -y curl tar
 ```
Set variables for isaac_ros_assets workspace folder...
 ```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
NGC_RESOURCE="isaac_ros_assets"
NGC_VERSION="isaac_ros_nvblox"
NGC_FILENAME="quickstart.tar.gz"

REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/versions/$NGC_VERSION/files/$NGC_FILENAME"
 ```
Create isaac_ros_assets workspace folder...
 ```
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/${NGC_VERSION} && \
    curl -LO --request GET "${REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets/${NGC_VERSION} && \
    rm ${NGC_FILENAME}
 ```
#### Download the isacc_ros_ess and isaac_ros_peoplesemsegnet models into isaac_ros_assets (takes a while)
Source setup.bash since the packages are already installed...   
```
source /opt/ros/humble/setup.bash
```
Set env variable so you don't have to manually accept every EULA...  
```   
export ISAAC_ROS_ACCEPT_EULA=1
 ```
Run the shell scripts  
```   
ros2 run isaac_ros_ess_models_install install_ess_models.sh
ros2 run isaac_ros_peoplesemseg_models_install install_peoplesemsegnet_vanilla.sh
ros2 run isaac_ros_peoplesemseg_models_install install_peoplesemsegnet_shuffleseg.sh
 ```

## Assemble the Workspace

### Isaac ROS repos...
 ```
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git  
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git  
 ```
### Application Repos...  
 ```
git clone https://github.com/robosoft-ai/SMACC2.git  
git clone https://github.com/robosoft-ai/sm_isaac_perceptor_1.git  
git clone https://github.com/robosoft-ai/rrt_exploration.git  
 ```

## Add key dependencies...
 ```
sudo apt update
rosdep update
rosdep install -i -r --from-paths ${ISAAC_ROS_WS}/src/nova_carter/nova_carter_bringup/ --rosdistro humble -y
 ```
 ```
sudo apt-get install -y ros-humble-isaac-ros-peoplesemseg-models-install ros-humble-isaac-ros-ess-models-install
source /opt/ros/humble/setup.bash
```
Install the required assets into isaac_ros_assets: (Takes a while)

Set env variable so you don't have to manually accept every EULA...  
```   
export ISAAC_ROS_ACCEPT_EULA=1
 ```
Run the shell scripts  
```   
ros2 run isaac_ros_ess_models_install install_ess_models.sh
ros2 run isaac_ros_peoplesemseg_models_install install_peoplesemsegnet_vanilla.sh
ros2 run isaac_ros_peoplesemseg_models_install install_peoplesemsegnet_shuffleseg.sh
 ```


## Build Workspace
 ```
colcon build --symlink-install
 ```
 ```
source workspaces/isaac_ros-dev/install/setup.bash  
 ```

## Launch Application
 ```
source install/setup.bash   
ros2 launch sm_isaac_perceptor_1 sm_isaac_perceptor_1_launch.py 
 ```
Reference Application Launch Command...
 ```
ros2 launch nova_carter_bringup navigation.launch.py \
mode:=simulation run_rviz:=True
 ```
