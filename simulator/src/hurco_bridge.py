import requests
import json
import array
import collections
import logging
import threading
import time
import urllib3

programStatus = ('Uninitialized', 'Started', 'Completed Successful', 'Completed Error', 'Completed Abort')
cncEvent = collections.namedtuple('cncEvent', ['openChuck','openDoor','closeChuck','closeDoor','cycle'])

tasks = cncEvent
tasks.openChuck = 'C:\Program Files (x86)\Hurco\DS WinMax Mill\Samples\intro-29.HWM'
tasks.closeChuck = 'C:\Program Files (x86)\Hurco\DS WinMax Mill\Samples\intro-29.HWM'
tasks.openDoor = 'C:\Program Files (x86)\Hurco\DS WinMax Mill\Samples\intro-29.HWM'
tasks.closeDoor = 'C:\Program Files (x86)\Hurco\DS WinMax Mill\Samples\intro-29.HWM'
tasks.cycle = 'C:\Program Files (x86)\Hurco\DS WinMax Mill\Samples\intro-05.HWM'

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
        rcrJson['bulk']['BulkStruct']['dValue'][1] = 1  # queue program to run after loading (0 = no, 1 = yes)
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

if __name__ == '__main__':
    cnc = hurcoClient('localhost',4503)
    task_execution = cnc.load_run_pgm(tasks.cycle)
    if task_execution:
        print "Successful Execution"
    else:
        print "Error!"



