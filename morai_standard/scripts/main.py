#!/usr/bin/env python
# -*- coding: utf-8 -*-
from network.ros_manager import RosManager
from autonomous_driving.autonomous_driving import AutonomousDriving


def main():
    autonomous_driving = AutonomousDriving()
    ros_manager = RosManager(autonomous_driving)
    ros_manager.execute()


if __name__ == '__main__':
    main()
