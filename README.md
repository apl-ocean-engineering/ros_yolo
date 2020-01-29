# ros_yolo

## Overview
ROS wrapper around a PyTorch implementation of the YOLO network [here](https://github.com/apl-ocean-engineering/pytorchYolo). 

## Installation
This package is a thin wrapper around the core Pytorch Yolo Python package.   
### Python package installation
Refer to the readme of the [Python Package](https://github.com/apl-ocean-engineering/pytorchYolo) for installation. 

### Wraper Installation
The underlying PyTorch YOLO code uses Python3, and therefore this ROS wrapper must utilize a Python3 catkin workspace and Python3 cv_bridge. We use [catkin tools](https://catkin-tools.readthedocs.io/en/latest/) for building.  

Intructions, [originating here](https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674), to build this package for python3 are:  

1. Install core python3 packages
	- $ sudo apt-get install python3-pip python3-yaml
	- $ sudo pip3 install rospkg catkin_pkg
	- $ sudo apt-get install python-catkin-tools python3-dev python3-numpy
2. Create a new catkin_ws and configure for Python3 ($ mkdir -p <ws_name>/src)
	- $ cd <ws_name>
	- $ catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
	- $ catkin config --install
3. Clone the vision_opencv ROS repo (to build in Python 3). *Instructions for ROS melodic, untested for other distros*
	- $ cd <ws_name>/src
	- $ git clone -b melodic https://github.com/ros-perception/vision_opencv.git
4. Build and source the workspace
	- $ cd <ws_name>
	- $ catkin build
	- source <ws_name>/install/setup.bash --extend

## Configuration download
Example weights and configuration can be found [here](https://drive.google.com/drive/folders/1VOEoOOTOrzb-vwieegfKXBICpTeckB2F?usp=sharing).

## Running
Very simple node (yolo.py). *You must specify the location of the your .data configuration file using the --data_cfg flag*. See the [Python Package](https://github.com/apl-ocean-engineering/pytorchYolo) for documentation. 

## Example with Kinect
1. Download example data with weights
2. Install the [Python Package](https://github.com/apl-ocean-engineering/pytorchYolo).
3. Edit the YOLO.data file (<path_to_pytorchYolo_python_package>/cfg/YOLO.data) to point to the YOLO classes .txt file, cfg file, and weight file. 
2. Launch the kinect
	- $ roscore
	- $ roslaunch openni_launch openni.launch
3. Run the ros node, pointing to the .data file
	- $ ./yolo.py --data_cfg <path_to_pytorchYolo_python_package>/cfg/YOLO.data

4. Move camera around and see detection!


## Demo

See Demo video under video/



	
