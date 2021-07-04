# ros_pyparrot
ROS Driver for pyparrot (Work in progress)

![ROS Pyparrot Artwork](https://toton95.github.io/assets/img/posts/ros-pyparrot_7.jpg)

This driver was created to be used along with Dr. Amy McGovern's [pyparrot](https://github.com/amymcgovern/pyparrot) driver. This project contains a simple interface to control the position of a supported Parrot vehicle through a ROS environment. 

## Credit

If you find this code useful, please consider to cite us as:

BibTeX:

```BibTeX
@INPROCEEDINGS{9024528,
  author={I. {Rubio Scola} and G. A. {Guijarro Reyes} and L. R. {Garcia Carrillo} and J. {Hespanha} and J. {Xie}},
  booktitle={2019 IEEE Globecom Workshops (GC Wkshps)}, 
  title={Translational Model Identification and Robust Control for the Parrot Mambo UAS Multicopter}, 
  year={2019},
  volume={},
  number={},
  pages={1-6},
  doi={10.1109/GCWkshps45667.2019.9024528}}

```

### Current supported vehicles
+ Mambo

### Current features
+ Basic control using the ROS environment
+ Cannon topic provided (Mambo)
+ Auto take-off topic provided
+ Up to 7 vehicles can be controlled by one bluetooth 4.0 controller
+ Launch files to fly multiple vehicles are provided

### Requirements
+ ROS Melodic
+ Bluetooth 4.0 controller
+ Python 3
+ Bluepy `pip3 install bluepy`
+ ROSPKG `pip3 install rospkg`
+ Zeroconf `pip3 install zeroconf`
+ pyparrot `pip3 install pyparrot`

### Installation 
+ Use the following command in your preferred catkin workspace 
```
$ git clone https://github.com/TOTON95/ros_pyparrot.git
```
+ Build it
```
$ catkin_make
```

### Usage
+ In the root directory of your preferred catkin workspace run 
```
$ source devel\setup.bash
```
+ Launch one of the files
```
$ roslaunch ros_pyparrot <name_of_desired_launch_file>
```

## Disclaimer 

### Use the driver at your own risk, the creators of this code are NOT responsible in any way in case of accidents or damage, against users and/or objects.

