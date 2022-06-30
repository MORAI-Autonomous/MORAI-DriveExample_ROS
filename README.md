[![MORAILog](./docs/MORAI_Logo.png)](https://www.morai.ai)
===
# MORAI - Drive example (ROS)

First step to enjoy the `MORAI Sim: Drive` with ROS.

This example support Ubuntu 18.04 or later
```
./
├── morai_msgs                   # [ROS msgs] MORAI Simulator ROS message set
└── morai_standard               # [Simulator Example] MORAI Sim: Drive example project
     ├── launch                    # example launch files
     ├── rviz                      # rviz preset configuration file
     └── scripts                   # example script files
          ├── autonomous_driving     # [Autonomous Driving] autonomous driving module
          ├── network                # ROS network connection
          └── main.py                # [Entry] example excuter
```

These example contains the below list.
  - Trajectory following lateral control
  - Smart(adaptive) Cruise Control
  - ROS communication

# Requirement

- ROS1 desktop-full >= melodic

- python >= 2.7

# Installation

Install packages which basically need

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src && catkin_init_workspace
$ git clone https://github.com/MORAI-Autonomous/MORAI-DriveExample_ROS.git
$ cd MORAI-DriveExample_ROS
$ git submodule update --init --recursive
$ sudo chmod -R a+x morai_standard/
$ find -name 'requirements.txt' | xargs -L 1 sudo pip install -U -r
$ cd ~/catkin_ws
$ rosdep install --from-paths . --ignore-src -r -y
$ catkin_make
$ source devel/setup.bash
```

# Usage

Enjoy the example which follow the trajectory with smart cruise control.
```
$ roslaunch morai_standard morai_standard.launch
```

# License
- MORAI Drive Example license info:  [Drive Example License](./docs/License.md)
- MORAI Autonomous Driving license info: [Autonomous Driving License](./morai_standard/scripts/autonomous_driving/docs/License.md)
- MGeo Module license info: [MGeo module License](./morai_standard/scripts/autonomous_driving/mgeo/lib/mgeo/docs/License.md)

