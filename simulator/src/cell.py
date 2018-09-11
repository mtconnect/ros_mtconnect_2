#!/usr/bin/python

import os, sys
sys.path.insert(0,os.getcwd()+'\\utils')

from material import *
from door import *
from chuck import *
from coordinator import *
from collaborator import *
from hurco_bridge import *
from cmm_bridge import *
from robot_interface import RobotInterface
from SocketServer import ThreadingMixIn, TCPServer, BaseRequestHandler
import threading
import socket
import uuid
from datetime import datetime
import re
import struct, gc
from mtconnect_adapter import Adapter
from long_pull import LongPull
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from archetypeToInstance import archetypeToInstance
from from_long_pull import from_long_pull, from_long_pull_asset

from transitions.extensions import HierarchicalMachine as Machine
from transitions.extensions.nesting import NestedState
from threading import Timer, Thread
import functools, time, re, copy, uuid
import requests, urllib2
import xml.etree.ElementTree as ET

from inputConveyor import inputConveyor
from cnc import cnc
from robot import Robot
from buffer import Buffer
from cmm import cmm
from outputConveyor import outputConveyor

class cell(object):

    def __init__(self):

        self.cell_part_quality = None
        self.current_part = None
        self.cycle_count = 0
        self.initiate_cnc('localhost',7897 ,False)
        #self.initiate_robot('localhost',7996)
        self.initiate_buffer('localhost',7697)
        self.initiate_cmm('localhost',7597,False)
        self.initiate_inputConveyor('localhost',7797)
        #self.initiate_outputConveyor('localhost',7496)

    def cell_part(self, value = None, current_part = None, cycle_count = None):
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
            self.reset_all()

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
        self.inputConveyor.create_statemachine()
        self.inputConveyor.superstate.load_time_limit(600)
        self.inputConveyor.superstate.unload_time_limit(600)
        #self.inputConveyor.superstate.enable()

    def initiate_cnc(self,host,port,sim = True):
        self.cnc = cnc(host,port,sim)
        self.cnc.create_statemachine()
        self.cnc.superstate.load_time_limit(900)
        self.cnc.superstate.unload_time_limit(900)
        #self.cnc.superstate.enable()

    def initiate_robot(self,host,port, sim = True):
        self.robot = Robot(host,port,RobotInterface(), sim = sim)
        self.robot.superstate.material_load_interface.superstate.simulated_duration = 600
        self.robot.superstate.material_unload_interface.superstate.simulated_duration = 600
        self.robot.superstate.enable()

    def initiate_buffer(self,host,port):
        self.buffer = Buffer(host,port)
        self.buffer.create_statemachine()
        self.buffer.superstate.load_time_limit(600)
        self.buffer.superstate.unload_time_limit(600)
        #self.buffer.superstate.enable()

    def initiate_cmm(self,host,port, sim = True):
        self.cmm = cmm(host,port,sim = sim, cell_part=self.cell_part)
        self.cmm.create_statemachine()
        self.cmm.superstate.load_time_limit(900)
        self.cmm.superstate.unload_time_limit(900)
        #self.cmm.superstate.enable()
        
    def initiate_outputConveyor(self,host,port):
        self.outputConveyor = outputConveyor(host,port)
        self.outputConveyor.create_statemachine()
        self.outputConveyor.superstate.load_time_limit(200)
        self.outputConveyor.superstate.unload_time_limit(200)
        self.outputConveyor.superstate.enable()

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
            #device.superstate.adapter.stop()
            #time.sleep(2)

            print (device.superstate.deviceUuid," reset.")

    def reset_all(self):
        if self.current_part == "reset":
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

            if self.cycle_count <4:

                self.part_arrival()

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
