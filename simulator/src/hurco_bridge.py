import requests
import json
import array
import collections
import logging
import threading
import time
import urllib3

programStatus = ('Uninitialized', 'Started', 'Completed Successful', 'Completed Error', 'Completed Abort')
cncEvent = collections.namedtuple('cncEvent', ['openChuck','openDoor','closeChuck','closeDoor','toolChange','cycle'])

tasks = cncEvent
tasks.openChuck = 'D:\open_chuck.FNC'
tasks.closeChuck = 'D:\close_chuck.FNC'
tasks.openDoor = 'D:\open_door.FNC'
tasks.closeDoor = 'D:\close_door.FNC'
tasks.toolChange = 'D:\AMT RH-2.HWM'
tasks.cycle = 'D:\FRAME.HWM'

local_vendor = "0025"
local_passwd = "5twOdw/SXwnvLAJg+QbSpDCV2tkak9WaaLHN9AxAs2sKiZYQxQQ2Whwp0JC0WM+LnooyOjxTFmvD2G1/fVsi8g=="

class RestAPIException(Exception):
  def __init__( self, ErrorMsg ):
    self.errorMsg = ErrorMsg
    Exception.__init__(self, 'RestAPI Exception Error: %s' % ErrorMsg)

  def GetErrorMsg(self):
    return self.errorMsg

class hurcoClient(object):

    def __init__(self, url, port):

        self.url = url
        self.port = port
        self.protocol = None
        self.headers = None
        self.urlString = None
        self.connection = None

        self.connection = self.connect()


    def connect(self):
        try:
            self.protocol = 'http'
            self.urlString = self.protocol + '://'+self.url+':' + str(self.port)
            
            result = requests.post(self.urlString + '/AuthService/Connect', data = '{"username": "0025", \
                "password": "5twOdw/SXwnvLAJg+QbSpDCV2tkak9WaaLHN9AxAs2sKiZYQxQQ2Whwp0JC0WM+LnooyOjxTFmvD2G1/fVsi8g=="}', verify=False)

            tokenValue = result.json()['token']
            
            self.headers = {'token': tokenValue}

            return True
            
        except RestAPIException as exc:
            print exc.GetErrorMsg()
            return False

    def load_run_pgm(self, pgm):
        
        result = requests.get(self.urlString + '/DataService/Bulk/SID_WINMAX_BULK_RCRID', headers=self.headers, verify=False)

        rcrJson = result.json()
        rcrJson['bulk']['BulkStruct']['dwCmdId'] = 42   # Load program command ID
        rcrJson['bulk']['BulkStruct']['dValue'][0] = 0  # close all other loaded programs (0 = no, 1 = yes)
        rcrJson['bulk']['BulkStruct']['dValue'][1] = 2  # queue program to run after loading (0 = no, 1 = yes)
        rcrJson['bulk']['BulkStruct']['dValue'][2] = 1  # skip reload if program is already loaded (0 = force reload, 1 = only load if not already loaded)
        rcrJson['bulk']['BulkStruct']['dValue'][3] = 0
        rcrJson['bulk']['BulkStruct']['dValue'][4] = 0
        rcrJson['bulk']['BulkStruct']['dValue'][5] = 0
        # Clear out string section that holds path/filename to load
        sValueLen = len(rcrJson['bulk']['BulkStruct']['sValue'])
        for i in range(0, sValueLen):
            rcrJson['bulk']['BulkStruct']['sValue'][i] = 0
            
        byteFileName = bytearray(pgm, 'ascii')
        i = 0
        for c in list(byteFileName):
            rcrJson['bulk']['BulkStruct']['sValue'][i] = c
            i += 1

        result = requests.put(self.urlString + '/DataService/Bulk/SID_WINMAX_BULK_RCRID', headers=self.headers, data=json.dumps(rcrJson), verify=False)
        bulk_status_code = result.status_code

        result = requests.get(self.urlString + '/DataService/Integer/SID_RT_WAITING_ON_REMOTE_PROGRAM_START', headers=self.headers, verify=False)

        while int(result.json()) != 1:
            result = requests.get(self.urlString + '/DataService/Integer/SID_RT_WAITING_ON_REMOTE_PROGRAM_START', headers=self.headers, verify=False)
            pass

        result = requests.put(self.urlString + '/DataService/Double/SID_RT_START_CYCLE_BUTTON', headers=self.headers, data='{"data":1.0}', verify=False)        
        
        """
        result = requests.get(self.urlString + '/DataService/Integer/SID_RT_DESKTOP_RUN_IN_PROGRESS', headers=self.headers, verify=False)

        
        
        if bulk_status_code == 200:
            
            while result.json() != 1:
                result = requests.get(self.urlString + '/DataService/Integer/SID_RT_DESKTOP_RUN_IN_PROGRESS', headers=self.headers, verify=False)
                pass

            while result.json() != 0:
                result = requests.get(self.urlString + '/DataService/Integer/SID_RT_DESKTOP_RUN_IN_PROGRESS', headers=self.headers, verify=False)

            return True

        else:
            raise RestAPIException('Remote Cmd Request Result: %s', result.text)
        """
        result = requests.get(self.urlString + '/DataService/Integer/SID_CURRENT_PROGRAM_STATUS', headers=self.headers, verify = False)
        a=result.json()
        while result.json() == a:
          result = requests.get(self.urlString + '/DataService/Integer/SID_CURRENT_PROGRAM_STATUS', headers=self.headers, verify = False)
          #print "Program is running"

        b = result.json()
          
        while int(result.json()) == b:
          result = requests.get(self.urlString + '/DataService/Integer/SID_CURRENT_PROGRAM_STATUS', headers=self.headers, verify = False)
        
        return True
      
if __name__ == '__main__':
    cnc = hurcoClient('192.168.1.42',4503)
    print ("Connection Successfull")
    task_execution = cnc.load_run_pgm(tasks.openDoor)
    print (task_execution)
    time.sleep(2)
    task_execution = cnc.load_run_pgm(tasks.closeChuck)
    print (task_execution)
    time.sleep(2)
    task_execution = cnc.load_run_pgm(tasks.closeDoor)



