import json
import time
import http.client
from datetime import datetime
import pytz

class HttpConnection():

    def __init__(self, logger, address, auth, api_prefix):
        self.logger = logger
        self.api_prefix = api_prefix
        self.http_headers = {
            "Accept-Language":"en-EN", 
            "Authorization":auth,
            "Content-Type":"application/json"}
        try:
            self.connection = http.client.HTTPConnection(host=address, timeout=5)
        except Exception as e:
            self.logger.warn('Creation of http connection failed')
            self.logger.warn(str(e))

    def __del__(self):
        if self.isValid():
            self.connection.close()

    def isValid(self):
        return not self.connection is None

    def get(self, path):
        if not self.isValid():
            self.connection.connect()
        self.connection.request("GET", self.api_prefix+path, headers = self.http_headers)
        resp = self.connection.getresponse() 
        if resp.status <200 or resp.status>=300:
            self.logger.warn("GET failed with status {} and reason: {}".format(resp.status, resp.reason))
        return resp

    def post(self, path, body):
        self.connection.request("POST", self.api_prefix+path,body=body, headers=self.http_headers)
        resp = self.connection.getresponse()
        if resp.status <200 or resp.status>=300:
            self.logger.warn("POST failed with status {} and reason: {}".format(resp.status, resp.reason))
        return json.loads(resp.read())

    def put(self, path, body):
        self.connection.request("PUT", self.api_prefix+path,body=body, headers=self.http_headers)
        resp = self.connection.getresponse()
        #self.logger.info(resp.read())
        if resp.status <200 or resp.status>=300:
            self.logger.warn("POST failed with status {} and reason: {}".format(resp.status, resp.reason))
        return json.loads(resp.read())
    
    def putNoResponse(self, path, body):
        self.connection.request("PUT", self.api_prefix+path,body=body, headers=self.http_headers)        

class MirRestAPI():

    def __init__(self, logger, hostname, auth = ""):
        self.logger = logger
        if hostname is not None:
            address = hostname + ":80"
        # else:
        #     address="192.168.12.20:80"
        self.http = HttpConnection(logger, address, auth, "/api/v2.0.0")

    def close(self):
        self.http.__del__()
        self.logger.info("REST API: Connection closed")

    def isConnected(self, print=True):
        if not self.http.isValid():
            self.logger.warn('REST API: Http-Connection is not valid')
            return False
        try:
            self.http.connection.connect()
            self.http.connection.close()
            if print:
                self.logger.info("REST API: Connected!")
        except Exception as e:
            if print:
                self.logger.warn('REST API: Attempt to connect failed: ' + str(e))
            return False
        return True

    def isAvailable(self):
        status = json.dumps(self.getStatus())
        if "service_unavailable" in status:
            return False
        else:
            return True
    
    def waitForAvailable(self):
        while True:
            if self.isConnected(print=False):
                if self.isAvailable():
                    self.logger.info('REST API: available')
                    break
                else:
                    self.logger.info('REST API: unavailable... waiting')
                    time.sleep(1)

    def getStatus(self):
        response = self.http.get("/status")
        return json.loads(response.read())
    
    def getStateId(self):
        status = self.getStatus()
        state_id = status["state_id"]
        return state_id
    
    """ Choices are: {3, 4, 11}, State: {Ready, Pause, Manualcontrol}
    """
    def setStateId(self, stateId):
        return self.http.put("/status", json.dumps({'state_id': stateId}))
    
    def isReady(self):
        status = self.getStatus()
        if status["state_id"] != 3: # 3=Ready, 4=Pause, 11=Manualcontrol
            self.logger.warn("MIR currently occupied. System state: {}".format(status["state_text"]))
            return False
        else:
            return True
    
    def getAllSettings(self, advanced=False, listGroups=False):
        if advanced:
            response = self.http.get("/settings/advanced")
        elif listGroups:
            response = self.http.get("/setting_groups")
        else:
            response = self.http.get("/settings")
        return json.loads(response.read())
    
    def getGroupSettings(self, groupID):
        response = self.http.get("/setting_groups/" + groupID + "/settings")
        return json.loads(response.read())
    
    def setSetting(self, settingID, settingData):
        return self.http.put("/setting", json.dumps({settingID: settingData}))
    
    def syncTime(self):
        tz_str = "Europe/Berlin"
        timezone = pytz.timezone(tz_str)
        timeobj = datetime.now(timezone)
        self.logger.info("REST API: Set Timezone to " + tz_str)

        dT = timeobj.strftime("%Y-%m-%dT%X")
        response = 'REST API: '
        try:
            response += str(self.http.put("/status", json.dumps({'datetime': dT})))
        except Exception as e:
            if str(e) == "timed out":
                # setting datetime over REST API seems not to be intended
                # that's why there is no response accompanying the PUT request,
                # therefore a time out occurs, however time has been set correctly
                response += "Set datetime to " + dT + " in timezone " + tz_str
                self.logger.info("REST API: Setting time Mir triggers emergency stop, please unlock.")
                self.logger.info(response)
                
                # this is needed, because a timeset restarts the restAPI
                self.waitForAvailable()
                
                return response
        response += " Error setting datetime"
        return response
        
    
    def getDistanceStatistics(self):
        response = self.http.get("/statistics/distance")
        return json.loads(response.read())

    def getPositions(self):
        response = self.http.get("/positions")
        return json.loads(response.read())

    def getPoseGuid(self, pos_name):
        positions = self.getPositions()
        return next((pos["guid"] for pos in positions if pos["name"]==pos_name), None)

    def getMissions(self):
        response = self.http.get("/missions")
        return json.loads(response.read())

    def getMissionGuid(self, mission_name):
        missions = self.getMissions()
        return next((mis["guid"] for mis in missions if mis["name"]==mission_name), None)

    def getSounds(self):
        response = self.http.get("/sounds")
        return json.loads(response.read())

    def moveTo(self, position, mission="MoveTo"):
        mis_guid = self.getMissionGuid(mission)
        pos_guid = self.getPoseGuid(position)

        for (var, txt, name) in zip((mis_guid, pos_guid),("Mission", "Position"),(mission, position)):
            if var is None:
                self.logger.warn("No {} named {} available on MIR - Aborting MoveTo".format(txt,name))
                return

        body = json.dumps({
            "mission_id": mis_guid,
            "message": "Externally scheduled mission from the MIR Python Client",
            "parameters": [{
                    "value": pos_guid,
                    "input_name": "target"
            }]})

        data = self.http.post("/mission_queue", body)
        self.logger.info("Mission scheduled for execution under id {}".format(data["id"]))

        while data["state"] != "Done":
            resp = self.http.get("/mission_queue/{}".format(data["id"]))
            data = json.loads(resp.read())
            if data["state"] == "Error":
                self.logger.warn("Mission failed as robot is in error")
                return
            self.logger.info(data["state"])
            time.sleep(2)

        self.logger.info("Mission executed successfully")