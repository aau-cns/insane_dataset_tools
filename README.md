![INSANE Logo](./resources/insane_logo.png)

# Introduction

[![arXiv](https://img.shields.io/badge/arXiv-10.48550/ARXIV.2210.09114-b31b1b.svg)](https://arxiv.org/abs/2210.09114) [![License](https://img.shields.io/badge/License-AAUCNS-green.svg)](./LICENSE)

The INSANE data set is a multi-sensor cross-domain UAV data set (18 sensors) with accurate and absolute 6 DoF ground truth. The scenarios include indoor flights in a controlled environment with motion capture ground truth, outdoor-to-indoor transition flights with continuous ground truth, and extensive coverage of Mars analog data with the same vehicle. Mars analog data includes segments with various ground structures, cliff flight over, and cliff-wall traversing trajectories for mapping.

This data set is ideal for testing novel algorithms with real-world sensor data and corresponding effects such as sensor degradation. Dedicated raw data for customized sensor calibration routines and vibration data for vehicle integrity tests are provided.

**Preprint** available at: https://arxiv.org/abs/2210.09114

**Data set** available on our webpage: https://www.aau.at/intelligente-systemtechnologien/control-of-networked-systems/datasets/

**Data set Features:**

- 6 DoF absolute ground~truth with centimeter and sub-degree accuracy 1-sigma for outdoor data sets.
- Indoor trajectories with motion capture ground truth (6 DoF millimeter and sub-degree accuracy) for the initial proof of algorithms.
- Outdoor to indoor transition trajectories with continuous absolute ground truth.
- Trajectories in a Mars analog desert environment for Mars-Helicopter analog setups, including various ground structures, cliff flight over, and cliff-wall traversing trajectories for mapping.
- Vehicle and sensor integrity, including intrinsic information such as static IMU data and RPM-correlated vibration data.
- Real-world sensor effects and degradation posed by individual scenarios.
- Initialization sequences for VIO algorithms.
- Inter-sensor calibrations in pre-calculated form and raw calibration data sequences for custom calibration routines.

The data which you can download from our website is ready to use. However, if you want to customize individual components, then all necessary steps are described below.

**In the following, we provide scripts and their description for:**

- Sensor calibration
- Data export of the original ROS bag files
- Ground-truth generation and transition segment alignment
- and sensor CSV data to ROS bag exports 

## Dependencies

The majority of the scripts are Matlab files and do not require external dependencies. A few post-processing scripts are written in Python and require additional packages and a dedicated catkin workspace for custom ROS messages.

### System Dependencies

Dependencies can be installed as follows:

```sh
apt install python3 python3-pip python3-catkin-tools ros-noetic-tf ros-noetic-cv-bridge ros-noetic-image-transport
pip3 install -U rospkg numpy tqdm scipy py3rosmsgs pycryptodomex
```

If you want to use a docker container, please also install the following dependencies for CV2:

```sh
apt-get install ffmpeg libsm6 libxext6
```

### ROS Dependencies

The data set uses two non-standard ROS messages `MotorSpeed.msg` and `TagDistance.msg`. Both message definitions are located in `catkin_ws/src/insane_msgs`. These messages have to be build for the export script and further usage.

```sh
$ cd catkin_ws
$ catkin init
$ catkin build fiducial_msgs insane_msgs
$ source ./devel/setup.bash
```

#### Additional steps for non-Python 3 ROS installs (<Noetic)

If you are using a ROS version below Noetic, then the following steps are required to build **cv bridge** module against Python3

Create a dedicated catkin workspace:
```sh
mkdir -p ~/cvbridge_build_ws/src && cd ~/cvbridge_build_ws/src && git clone -b noetic https://github.com/ros-perception/vision_opencv.git
```
Adapt the following lines in the `CMakeLists.txt` of the `ros-perception/vision_opencv` package:

```cmake
# modification at Line 11, changing:
find_package(Boost REQUIRED python37)
# to
find_package(Boost REQUIRED python-py35)
```

Compile against the Python 3 library:
```sh
$ catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.5m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.5m.so
$ catkin build
$source ~/cvbridge_build_ws/devel/setup.bash
```

# Export CSV and PNG to ROS Bag

If you downloaded the exported CSV and PNG files from our website and would like to use the data as a rosbag file, then you can use the following python script `post_scripts/csv2bag.py`. This requires the installation of dependencies and custom ros messages as described [here](#dependencies).

Run the script inside a folder which only contains the data for one dataset e.g., as shown below:

```
mars_4
├── mars_4_sensors
├── mars_4_nav_cam
└── mars_4_stereo_cam
```

The script will scan the directory for the expected CSV files and generate a bagfile `insane_dataset.bag`.

# Contact

For further information, please contact [Christian Brommer](mailto:christian.brommer@aau.at)

# License

This software is made available to the public to use (_source-available_), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the [LICENSE](./LICENSE) file. No license in patents is granted.

## Usage for academic purposes

If you use this software in an academic research setting, please cite the corresponding paper and consult the [LICENSE](./LICENSE) file for a detailed explanation.

```latex
@article{brommer_insane_2022,
	title     = {{INSANE}: Cross-Domain {UAV} Data Sets with Increased Number of Sensors for developing
	             Advanced and Novel Estimators},
	author    = {Brommer, Christian and Fornasier, Alessandro and Scheiber, Martin and 
	             Delaune, Jeff and Brockers, Roland and Steinbrener, Jan and Weiss, Stephan},
	year      = 2022,
	month     = oct,
	publisher = {arXiv},
	doi       = {10.48550/arXiv.2210.09114},
	url       = {http://arxiv.org/abs/2210.09114},
	note      = {arXiv:2210.09114 [cs]},
	keywords  = {Computer Science - Robotics}
}
```