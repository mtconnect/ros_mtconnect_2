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

        self.initiate_cnc('localhost',7896)
        #self.initiate_robot('localhost',7996)
        self.initiate_buffer('localhost',7696)
        self.initiate_cmm('localhost',7596)
        self.initiate_inputConveyor('localhost',7796)
        #self.initiate_outputConveyor('localhost',7496)

    def cell_part(self, value = None):
        if value: self.cell_part_quality = value

        return self.cell_part_quality

    def part_arrival(self):
        self.inputConveyor.superstate.enable()

    def initiate_inputConveyor(self,host,port):
        self.inputConveyor = inputConveyor(host,port,cell_part=self.cell_part)
        self.inputConveyor.create_statemachine()
        self.inputConveyor.superstate.load_time_limit(200)
        self.inputConveyor.superstate.unload_time_limit(200)
        #self.inputConveyor.superstate.enable()

    def initiate_cnc(self,host,port,sim = True):
        self.cnc = cnc(host,port,sim)
        self.cnc.create_statemachine()
        self.cnc.superstate.load_time_limit(600)
        self.cnc.superstate.unload_time_limit(600)
        self.cnc.superstate.enable()

    def initiate_robot(self,host,port, sim = True):
        self.robot = Robot(host,port,RobotInterface(), sim = sim)
        self.robot.superstate.material_load_interface.superstate.simulated_duration = 40
        self.robot.superstate.material_unload_interface.superstate.simulated_duration = 40
        self.robot.superstate.enable()

    def initiate_buffer(self,host,port):
        self.buffer = Buffer(host,port)
        self.buffer.create_statemachine()
        self.buffer.superstate.load_time_limit(200)
        self.buffer.superstate.unload_time_limit(200)
        self.buffer.superstate.enable()

    def initiate_cmm(self,host,port, sim = True):
        self.cmm = cmm(host,port,sim = sim, cell_part=self.cell_part)
        self.cmm.create_statemachine()
        self.cmm.superstate.load_time_limit(200)
        self.cmm.superstate.unload_time_limit(200)
        self.cmm.superstate.enable()
        
    def initiate_outputConveyor(self,host,port):
        self.outputConveyor = outputConveyor(host,port)
        self.outputConveyor.create_statemachine()
        self.outputConveyor.superstate.load_time_limit(200)
        self.outputConveyor.superstate.unload_time_limit(200)
        self.outputConveyor.superstate.enable()
        

if __name__ == "__main__":
    machine_cell = cell()
    time.sleep(10)
    machine_cell.part_arrival()
