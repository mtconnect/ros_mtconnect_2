#!/usr/bin/python
from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import os, sys

import functools, time, re, copy, uuid
import requests, urllib2
import xml.etree.ElementTree as ET

from threading import Thread

from inputConveyor import inputConveyor
from cnc import cnc
from robot import Robot
from robot_interface import RobotInterface
from buffer import Buffer
from cmm import cmm

class cell:

    def __init__(self):

        self.cell_part_quality = None
        self.current_part = None
        self.cycle_count = 0
        self.initiate_cnc('localhost',7896)

        #Comment the robot initiation when simulating/executing with RViz
        self.initiate_robot('localhost',7996)
        self.initiate_buffer('localhost',7696)
        self.initiate_cmm('localhost',7596)
        self.initiate_inputConveyor('localhost',7796)

    def cell_part(self, value = None, current_part = None, cycle_count = None):
        #For InputConveyor Interaction: part_quality and cycle count; IMTS
        if value:
            self.cell_part_quality = value
            return self.cell_part_quality

        elif cycle_count == True:
            self.cycle_count = self.cycle_count + 1
            print ("Cycle count: ", self.cycle_count, ".")

        elif current_part == True:
            return self.current_part

        elif current_part == "reset":
            self.current_part = "reset"

            #Pause after completion
            thread = Thread(target = self.reset_all)
            thread.daemon =True
            thread.start()

        elif current_part:
            self.current_part = current_part
            return self.current_part

        else:
            return self.cell_part_quality

    def part_arrival(self):
        self.inputConveyor.superstate.cycle_count = 0
        self.inputConveyor.superstate.current_part = None
        self.inputConveyor.superstate.enable()

    def initiate_inputConveyor(self,host,port):
        self.inputConveyor = inputConveyor(host,port,cell_part=self.cell_part)
        self.inputConveyor.superstate.load_time_limit(600)
        self.inputConveyor.superstate.unload_time_limit(600)

    def initiate_cnc(self,host,port,sim = True):
        self.cnc = cnc(host,port,sim)
        self.cnc.superstate.load_time_limit(900)
        self.cnc.superstate.unload_time_limit(900)

    def initiate_robot(self,host,port, sim = True):
        self.robot = Robot(host,port,RobotInterface(), sim = sim)
        self.robot.superstate.material_load_interface.superstate.simulated_duration = 600
        self.robot.superstate.material_unload_interface.superstate.simulated_duration = 600
        self.robot.superstate.enable()

    def initiate_buffer(self,host,port):
        self.buffer = Buffer(host,port)
        self.buffer.superstate.load_time_limit(600)
        self.buffer.superstate.unload_time_limit(600)

    def initiate_cmm(self,host,port, sim = True):
        self.cmm = cmm(host,port,sim = sim, cell_part=self.cell_part)
        self.cmm.superstate.load_time_limit(900)
        self.cmm.superstate.unload_time_limit(900)

    def reset_device(self,device = None):
        if device:

            for k,v in device.superstate.lp.iteritems():
                device.superstate.lp[k]._response = None
            device.superstate.master_tasks = {}
            device.superstate.coordinator = None
            device.superstate.collaborator = None
            device.superstate.adapter.removeAllAsset('Task')
            device.superstate.set_priority()
            device.superstate.initiate_interfaces()
            device.superstate.events = []
            device.superstate.initiate_pull_thread()

            print (device.superstate.device_uuid," reset.")

    def reset_all(self):

        if self.current_part == "reset":
            time.sleep(3)
            nsx = urllib2.urlopen("http://localhost:5000/current").read()
            nsr = ET.fromstring(nsx)
            ns = nsr[0].attrib['nextSequence']

            self.inputConveyor.superstate.nextsequence = ns
            self.buffer.superstate.nextsequence = ns
            self.cnc.superstate.nextsequence = ns
            self.cmm.superstate.nextsequence = ns

            self.reset_device(self.cnc)
            self.reset_device(self.buffer)
            self.reset_device(self.cmm)
            self.reset_device(self.inputConveyor)


            self.cnc.superstate.enable()
            self.cmm.superstate.enable()
            self.cnc.superstate.initiate_cnc_client()
            self.cmm.superstate.initiate_cmm_client()
            self.buffer.superstate.enable()
            time.sleep(7)

            if self.cycle_count <1000:

                #self.part_arrival() #Uncomment for continuous cycles
                pass

            else:
                print ("Restart setup!")

            self.current_part = None

if __name__ == "__main__":
    machine_cell = cell()
    machine_cell.cnc.superstate.enable()
    machine_cell.cmm.superstate.enable()
    machine_cell.buffer.superstate.enable()
    time.sleep(10)
    machine_cell.part_arrival()
