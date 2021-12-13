#!/usr/bin/env python
# -*- coding: utf-8 -*-
from abc import ABCMeta, abstractmethod
from autonomous_driving.vehicle_state import VehicleState


class Manager(ABCMeta('ABC', (object,), {'__slots__': ()})):
    def __init__(self, autonomous_driving):
        self.autonomous_driving = autonomous_driving
        self.is_shutdown = True
        self.is_received = False

        self.vehicle_state = VehicleState()
        self.object_info_list = []
        self.traffic_light = []

    @abstractmethod
    def set_protocol(self):
        pass

    @abstractmethod
    def check_protocol(self):
        pass

    @abstractmethod
    def send_data(self):
        pass

    def execute(self):
        print("start simulation")
        self.set_protocol()
        self.check_protocol()

        while not self.is_shutdown:
            self.check_protocol()
            if self.is_received:
                control_input, local_path = self.autonomous_driving.execute(
                    self.vehicle_state, self.object_info_list, self.traffic_light
                )
                self.send_data(control_input, local_path)

        print("end simulation")
