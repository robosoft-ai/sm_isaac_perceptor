# Workflow
## Setting up an IsaacROSDev Container
[https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment](https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment)  
[https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)  
[https://nvidia-isaac-ros.github.io/robots/nova_carter/getting_started.html#nova-carter-dev-setup](https://nvidia-isaac-ros.github.io/robots/nova_carter/getting_started.html#nova-carter-dev-setup)  

## Reference Example
[https://nvidia-isaac-ros.github.io/reference_workflows/isaac_perceptor/run_perceptor_in_sim.html](https://nvidia-isaac-ros.github.io/reference_workflows/isaac_perceptor/run_perceptor_in_sim.html)  

## Let's Get Started
We begin by cloning isaac_ros_common and nova_carter repos to the src folder of our local workspace. My local workspace is ~/workspace/humble_ws
 ```
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/nova_carter.git  
 ```
## Start the IsaacROSDev Container (from the workspace...)
 ```
./src/isaac_ros_common/scripts/run_dev.sh  -d  ~/workspace/humble_ws/
 ```
Then run ls in the terminal to confirm that isaac_ros-dev is set to your host host workspace.    
  
Alternatively, you can run this one from inside workspace/src/isaac_ros_common/scripts. The -d command explicity sets the workspace to workspaces/isaac_ros-dev inside the IsaacROSDev container.
```
./run_dev.sh -d  ~/workspace/humble_ws/
```
## Installations onto the Container....
First, lets get updated... 
 ```
sudo apt-get update  
rosdep update
 ```
We'll need curl too to for later when we download assets into the isaac_ros_assets folder...
 ```
sudo apt-get install -y curl tar
 ```
### Install LTTng-UST...
We'll need this for SMACC later...  
 ```
sudo apt-get install -y lttng-tools  
sudo apt-get install -y lttng-modules-dkms  
sudo apt-get install -y liblttng-ust-dev  
 ```
### Install Jetson Stats
 ```
sudo apt-get install -y ros-humble-isaac-ros-jetson-stats
 ```
### Use rosdep to install Nova Carter bringup dependencies...
 ```
rosdep install -i -r --from-paths ${ISAAC_ROS_WS}/src/nova_carter/nova_carter_bringup/ --rosdistro humble -y
 ```
### Install Nvblox From Debian...
 ```
sudo apt-get install -y ros-humble-isaac-ros-nvblox && \
rosdep install isaac_ros_nvblox
 ```
#### Download the nvblox assets

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
#### Download the isacc_ros_ess and isaac_ros_peoplesemsegnet models into the isaac_ros_assets folder (takes a while)
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
### Install isaac_ros_object_detection and other perception packages from Debian... (optional)
We'll start with pointcloud_to_laserscan...  
 ```
sudo apt-get install -y ros-humble-pointcloud-to-laserscan
 ```
#### Install isaac_ros_detectnet
Then we'll get into isaac_ros_object_detection, starting with detectnet
 ```
sudo apt-get install -y ros-humble-isaac-ros-detectnet 
 ```
#### Download the isaac_ros_detectnet assets

Set variables for isaac_ros_assets workspace folder...
 ```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
NGC_RESOURCE="isaac_ros_assets"
NGC_VERSION="isaac_ros_detectnet"
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
#### Install isaac_ros_rtdetr
 ```
sudo apt-get install -y ros-humble-isaac-ros-rtdetr
 ```
#### Download the isaac_ros_rtdetr assets

Set variables for isaac_ros_assets workspace folder...
 ```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
NGC_RESOURCE="isaac_ros_assets"
NGC_VERSION="isaac_ros_rtdetr"
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
#### Install isaac_ros_yolov8
 ```
 sudo apt-get install -y ros-humble-isaac-ros-yolov8
 ```
#### Download the isaac_ros_yolov8 assets

Set variables for isaac_ros_assets workspace folder...
 ```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
NGC_RESOURCE="isaac_ros_assets"
NGC_VERSION="isaac_ros_yolov8"
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
#### Configure the isaac_ros_yolov8 assets
 Download any model you choose from Ultralytics YOLOv8. Nvidia's examples use YOLOv8s.
 ```
cd Downloads && \
   wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.pt
 ```
You need to convert the PyTorch model (.pt) to a general ONNX model (.onnx). This ONNX model will be converted to a TensorRT engine file and used with the Isaac ROS TensorRT node for inference. 

First install ultralytics and onnx via pip:
 ```
pip3 install ultralytics
pip3 install onnx
 ```
Then convert the model from a .pt file to a .onnx model using ultralytics.   
Start a python shell...  
 ```
python3
 ```
Then within the shell, export the model:
 ```
>> from ultralytics import YOLO
>> model = YOLO('yolov8s.pt')
>> model.export(format='onnx')
 ```
Exit the interactive python shell and copy the generated .onnx model into the the isaac_ros_assets folder 
 ```
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8
cp yolov8s.onnx ${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8
 ```
#### Install isaac_ros_image_pipeline
 ```
 sudo apt-get install -y ros-humble-isaac-ros-depth-image-proc
 sudo apt-get install -y ros-humble-isaac-ros-gxf-extensions
 sudo apt-get install -y ros-humble-isaac-ros-image-pipeline
 ```
These should already be installed but if you just want to make sure..  
 ```
 sudo apt-get install -y ros-humble-isaac-ros-image-proc
 sudo apt-get install -y ros-humble-isaac-ros-stereo-image-proc
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

## Build Workspace
```
source /opt/ros/humble/setup.bash
```
```
rosdep update
rosdep install --ignore-src --from-paths src -y -r
 ```
 ```
colcon build --symlink-install
 ```

## Launch Application
Source the workspace...  
 ```
source install/setup.bash
 ```
 ```
ros2 launch sm_isaac_perceptor_1 sm_isaac_perceptor_1_launch.py 
 ```
Reference Application Launch Command...
 ```
ros2 launch nova_carter_bringup navigation.launch.py \
mode:=simulation run_rviz:=True
 ```
