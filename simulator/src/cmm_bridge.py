import json
import array
import collections
import logging
import threading
import time
import urllib3
import requests
import socket
import sys
import ctypes

programStatus = { 'NotStarted' : 'Uninitialized',
                  'Running' : 'Started',
                  'Good' : 'Completed Good Part',
                  'Bad' : 'Completed Bad Part',
                  'ReWork' : 'Completed ReWork Part'
                 }
cmmStatus = { 'Ready' : 'CMM Ready',
              'NotParked' : 'CMM Not Parked',
              'InError' : 'CMM In Error',
              'Manual' : 'CMM in Manual',
              'SpeedZero' : 'CMM Speed Zero',
              'MotorsOff' : 'CMM Motors OFF'
             }

cmmEvent = collections.namedtuple('cmmEvent',
    [
     'startProgramA',
     'startProgramB',
     'startProgramC',
     'getDims',
     'getStatus',
     'getCmmState'])

taskcmm = cmmEvent
taskcmm.startProgramA = '/run_program?1/'
taskcmm.startProgramB = '/run_program?2/'
taskcmm.startProgramC = '/run_program?3/'
taskcmm.getDims = '/send_dims/'
taskcmm.getStatus = '/send_status/'
taskcmm.getCmmState = '/send_status2/'

class hexagonClient(object):

    def __init__(self, url, localPort, remotePort):

        self.url = url
        self.localPort = localPort
        self.remotePort = remotePort
        self.protocol = None
        self.headers = None
        self.urlString = None
        self.connection = None
        self.programStatus = 'Uninitialized'
        self.cmmStatus = 'CMM In Error'

    def connect(self):
        try:
            self.protocol = 'http'
            self.urlString = self.protocol + '://'+self.url+':' + str(self.localPort)
            return True
        except Exception as e:
            print str(e)
            return False

    def send_data(self, data):
        result = requests.post(data)
        return result

    def load_run_pgm(self, pgm):
        flagState = False
        if pgm == taskcmm.getCmmState:
            flagState = True
            pgm = taskcmm.getStatus
        response = self.send_data(self.urlString + pgm)
        #print response.text

        if pgm == taskcmm.getDims:
            return response.text # send the JSON Dimensions
            # example:
            #    {
            #        "LOC1.X" :  {
            #            "DimType" : "X Location",
            #            "DimNominal" : "2.687500",
            #            "DimMeasure" : "2.687500",
            #            "DimUTol" : "0.002000",
            #            "DimLTol" : "0.002000",
            #            "OutOfTol" : "InTolerance"
            #            }
            #    }
        elif pgm == taskcmm.getStatus:
            # parse the JSON and send the value
            # i.e.: {
            #           "ErrorString": "NA",
            #           "ManualAuto": "Manual",
            #           "MeasurementGood": "ReWork",
            #           "State": "Running",
            #           "IsParked": "False",
            #           "RunningState": "FinishBad",
            #           "MotorsOn": "On",
            #           "Speed": "0.0"
            #        }
            vals = json.loads(response.text)

            if vals['RunningState'] == 'Running':
                self.programStatus = programStatus['Running']
            elif vals['RunningState'] == 'Paused':
                self.programStatus = programStatus['NotStarted']
            elif vals['RunningState'] == 'Error':
                self.programStatus = programStatus['NotStarted']
            else:
                if vals['MeasurementGood'] == 'ReWork':
                    self.programStatus = programStatus['ReWork']
                elif vals['MeasurementGood'] == 'False':
                    self.programStatus = programStatus['Bad']
                else:
                    self.programStatus = programStatus['Good']

            if vals['IsParked'].upper() == 'FALSE':
                self.cmmStatus = cmmStatus['NotParked']
                # make sure everything is ok before stating it is ready
            elif vals['RunningState'].upper() == 'RUNNING':
                self.cmmStatus = cmmStatus['NotParked']
            elif vals['State'].upper() == 'READY' and vals['ManualAuto'].upper() == 'AUTO' and vals['MotorsOn'].upper() == 'ON' and vals['Speed'] != '0.0' and vals['RunningState'].upper() != 'ERROR':
                self.cmmStatus = cmmStatus['Ready']
            elif vals['State'].upper() == 'BUSY':
                self.cmmStatus = cmmStatus['NotParked']
            else:
                self.cmmStatus = cmmStatus['InError']
        else: #  running the programs
            # look at the return values
            if 'running program' in response.text:
                self.cmmStatus = cmmStatus['NotParked']
                self.programStatus = programStatus['Running']
            else:
                self.programStatus = programStatus['NotStarted']

        # tell them the program status
        if flagState:
            return self.cmmStatus
        else:
            return self.programStatus
if __name__ == '__main__':
    cmm = hexagonClient('192.168.1.41', 5000, 5000)
    cmm.connect()
    #task_execution = cmm.load_run_pgm(taskcmm.getDims)
    #print task_execution
    status = cmm.load_run_pgm(taskcmm.getStatus)
    print status
    state = cmm.load_run_pgm(taskcmm.getCmmState)
    print state
    task_execution = cmm.load_run_pgm(taskcmm.startProgramA)
    #time.sleep(0.1)
    print task_execution
