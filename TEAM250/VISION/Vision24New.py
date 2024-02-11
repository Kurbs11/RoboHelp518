# Currently expects networktable server on fauxbot.

# $Revision: 1.2 $
# $Date: 2024-01-19 10:11:51-05 $
# $Author: animation $
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from math import pi,atan2,asin,sqrt, sin, cos
import json
import time
import sys
import cv2
import numpy as np
import apriltag

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from ntcore import NetworkTableInstance, EventFlags

#FudgeFactor = 1.20 #these are now retrieved from get_camera_parameters()
#FudgeOffset = 0.55

# TAG-centric coordinates. X is right, Y is up, Z points out of screen
CORNERS   = np.array([[-4.0,  4.0, 0.0],   # Start at NW corner and proceed 
                      [ 4.0,  4.0, 0.0],   # clockwise. This is the order
                      [ 4.0, -4.0, 0.0],   # returned by Detector.
                      [-4.0, -4.0, 0.0]])  
# Sweetspot axes: Increasing X moves the sweetspot to the tag's left and camera's right. 
# Increasing Y move the sweetspot up. Increasing Z moves the sweetspot forward from the tag
# along the normal to the tag face.
SWEETSPOT=np.array(([[12.0, 0.0, 72.0]])*17)
SWEETSPOT=SWEETSPOT.reshape(17,3)

# Waypoints are like SWEETSPOTS but aren't tied to a Tag. They're used in Auton. (Not implemented yet)
#Likely to be used for Note starting positions on centerline
RedWaypoints    = [ 1, 2, 3, 4, 5 ]
BlueWaypoints   = [ 1, 2, 3, 4, 5 ]

#Tag locations are all with respect to an origin in the red source zone
TAG_LOCATION= [[593.68,   9.68, 53.38,  2.09], [637.21,  34.79, 53.38,  2.09], [652.73, 196.17, 57.13,  3.14]
, [652.73, 218.42, 57.13,  3.14], [578.77, 323.00, 53.38,  4.71], [ 72.50, 323.00, 53.38,  4.71]
, [ -1.50, 218.42, 57.13,  0.00], [ -1.50, 196.17, 57.13,  0.00], [ 14.02,  34.79, 53.38,  1.05]
, [ 57.54,   9.68, 53.38,  1.05], [468.69, 146.19, 52.00,  5.24], [468.69, 177.10, 52.00,  1.05]
, [441.74, 161.62, 52.00,  3.14], [209.48, 161.62, 52.00,  0.00], [182.73, 177.10, 52.00,  2.09], [182.73, 146.19, 52.00,  4.19]]
BEARINGS=[(999.0,999.0)]*17 #Will be added to closest function for offscreen tag detection
ID_to_Game_Element = { #L/R Based on if camera is looking at the object
    1: "Blue Source Right",
    2: "Blue Source Left",
    3: "Red Speaker Right",
    4: "Red Speaker Middle",
    5: "Red Amp",
    6: "Blue Amp",
    7: "Blue Speaker Middle",
    8: "Blue Speaker Left",
    9: "Red Source Right",
    10 : "Red Source Left",
    11: "Red Stage Left",
    12: "Red Stage Right",
    13: "Red Center Stage",
    14: "Blue Center Stage",
    15: "Blue Stage Left",
    16: "Blue Stage Right"
}


options  = apriltag.DetectorOptions(
           families="tag36h11"
           ,nthreads      = 4
           ,quad_decimate = 2
           ,quad_blur     = 0.0
           ,refine_edges  = True
           ,refine_decode = True
           ,refine_pose   = False
           ,quad_contours = True
           )
detector = apriltag.Detector(options)

configFile = "/boot/frc.json"

class CameraConfig: pass     # Is this just to initialize CameraConfig?

team            = None
server          = False
cameraConfigs   = []
switchedCameraConfigs = []
cameras         = []
AllTags         = [ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 ]
RedTags = [3,4,5,9,10,11,12,13]
BlueTags = [1,2,6,7,8,14,15,16]
AllianceTags    = [] #May not be a necessary array
CamPos          = np.array([0.0, 0.0, 0.0])
CamPos          = CamPos.reshape(3,1)
CamRange        = 0.0
SWSPos          = CamPos #SweetSpot Position
Alliance        = 'none'
current_cmd     = 'none'
cmd             = ''

class Tag(object) :
    def __init__(self, id, ntinst):

        self.Tagid = id
        self.ntinst = ntinst
        self.tagtable = self.ntinst.getTable("Tag_{:02d}".format(id)) 
        self.config_Rng = self.tagtable.getDoubleTopic("Rng").publish()
        self.config_Hdg = self.tagtable.getDoubleTopic("Hdg").publish()
        self.Rng = 999 #default number (placeholder)
        self.Hdg = 999
    def update_Rng(self,new_Rng): 
        self.Rng = new_Rng #Updates the value 
        self.config_Rng.set(self.Rng) # Updates the network table with the new value (Should make pushData() defunct)
    def update_Hdg(self,new_Hdg):
        self.Hdg = new_Hdg
        self.config_Hdg.set(self.Hdg)


def rotate(point, origin, angle): #Point is a tuple of the camera's XZ-WorldLocation 
    #Origin is a tuple of the closest tag's XZ World-Location, Angle is the tags offset
    ox, oy = origin # ox goes to the origin tuple's first element (x), oy for 2nd
    px, py = point
    qx = ox + cos(angle) * (px - ox) - sin(angle) * (py - oy)
    qy = oy + sin(angle) * (px - ox) + cos(angle) * (py - oy)
    return qx, qy

def Set_Tag_List(cmd,Alliance): #Command is from Drivestation
    if cmd == 'E':
        return AllTags
    if Alliance == 'RED':
        if cmd == 'A': #A for Auton
            pass                 # <--- fix this
        elif cmd == 'M': #M for Amp :0
            return [5]
        elif cmd == 'R': #R for Source ;0
            return [9,10]
        elif cmd == 'S': #S for Speaker 
            return [3,4]
        elif cmd == '':
            return []
    elif Alliance == 'BLUE':
        if cmd == 'A':
            pass                 # <--- fix this
        elif cmd == 'M':
            return [6]
        elif cmd == 'R':
            return [1,2]
        elif cmd == 'S':
            return [7,8]
        elif cmd == '':
            return []


def get_camera_parameters():
    param_file = '/home/pi/Camera_B_800x600_parameters.json'
    #param_file = '/home/pi/Camera_B_640x480_parameters.json'
    Pfile      = open(param_file,'r')
    j          = json.load(Pfile)
    Pfile.close()
    return j["width"], j["height"], np.array(j["mtx"]), np.array(j["dist"])


def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    camera = UsbCamera(config.name, config.path)
    server = CameraServer.startAutomaticCapture(camera=camera)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.addSwitchedCamera(config.name)

    def listener(event):
        data = event.data
        if data is not None:
            value = data.value.value()
            if isinstance(value, int):
                if value >= 0 and value < len(cameras):
                    server.setSource(cameras[value])
            elif isinstance(value, float):
                i = int(value)
                if i >= 0 and i < len(cameras):
                    server.setSource(cameras[i])
            elif isinstance(value, str):
                for i in range(len(cameraConfigs)):
                    if value == cameraConfigs[i].name:
                        server.setSource(cameras[i])
                        break

    NetworkTableInstance.getDefault().addListener(
        NetworkTableInstance.getDefault().getEntry(config.key),
        EventFlags.kImmediate | EventFlags.kValueAll,
        listener)

    return server




def ClosestTag(tags, ID_to_Game_Element, Closest_ID, Closest_Rng, Closest_Hdg):
    #Import from notebook
    if len(tags) == 0:
        Closest_ID.set("None In View") #The assumption is the tables can take in strings
        Closest_Rng.set(-1) #Obvious garbage value
        return
    
    closest = [] #Will be a 2-element array containing the most recent closest id and its range
    closest.append(ID_to_Game_Element[tags[0].Tagid])
    closest.append(tags[0].Rng)
    closest.append(tags[0].Hdg)
    for tag in tags:
        if tag.Rng < closest[1]:
            closest[0] = ID_to_Game_Element[tag.Tagid] #Closest location's id is input into dictionary to output the name of the game object 
            closest[1] = tag.Rng
            closest[2] = tag.Hdg 
    Closest_ID.set(closest[0]) #Updates the network table's location to the new closest one
    Closest_Rng.set(closest[1])
    Closest_Hdg.set(closest[2])

    return 
def OneTagToAll(Input_Tag,Tags,TAG_LOCATIONS): #May take in Bearings... TBD (May get deleted entirely)
    #Using the closest tag in view as an input, we can calculate the cameras distance to all the other tags 
    #If through this process, there is a tag out of frame closer to the camera, we will update it to be the new closest
    for Tag in Tags:
        if Tag.Tagid != Input_Tag.Tagid:
            pass
    pass


if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    print ("TADA!")
    #IP='fauxbot'
    IP='192.168.1.46'

    width, height, USBcam_mtx, USBcam_dist, FudgeFactor, FudgeOffset = get_camera_parameters()
    CORNERS = (CORNERS - FudgeOffset) / FudgeFactor

    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("Vision24")
        #ntinst.setServerTeam(team)
        ntinst.setServer(IP)
        #ntinst.startDSClient()

    # start cameras
    # work around wpilibsuite/allwpilib#5055
    CameraServer.setSize(CameraServer.kSize160x120)
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(height, width, 1), dtype=np.uint8)
    input_stream = CameraServer.getVideo()
    output_stream = CameraServer.putVideo('Vision 2024', width, height)

    ##Initialization of network tables for Tags1-16 is now handled in the tag object construction. 
    DS_tbl = ntinst.getTable("DS")
    subDS  = DS_tbl.getStringTopic("Cmd").subscribe(' ') #Since we are only Reading in Driverstation entries, we Subscribe
    subAlliance = DS_tbl.getStringTopic("Alliance").subscribe(' ')

    Closest_tag = ntinst.getTable("Closest Tag")
    Closest_ID = Closest_tag.getStringTopic("Location").publish() #Topic has input based on ID to game-element map
    Closest_Rng = Closest_tag.getDoubleTopic("Rng").publish()
    Closest_Hdg = Closest_tag.getDoubleTopic("Hdg").publish()

    CamPos_tbl = ntinst.getTable("CamPos") #These are for the cameras field coordinates
    pubCamWorldX = CamPos_tbl.getDoubleTopic("X").publish()
    pubCamWorldX = CamPos_tbl.getDoubleTopic("X").publish()
    pubCamWorldY = CamPos_tbl.getDoubleTopic("Y").publish()
    pubCamTag_id =  CamPos_tbl.getDoubleTopic("Tag_ID").publish()
    
     # wait for Drive Station
    while Alliance not in ['RED','BLUE']:
        Alliance = subAlliance.get()
        time.sleep(0.1) #Delays program execution(Re-looping) by .1s
    while current_cmd == 'none':
        current_cmd = subDS.get('none')
    print ("Alliance:",Alliance,"  Cmd:",current_cmd)

    #Create 16 tag objects (This is an alternative way to the 48 lines of code up above)
    tags = []
    for i in range(16): 
        current_tag = Tag(i+1, ntinst)
        current_tag.Tagid
        current_tag.config_Rng #Configuring the network table slots for current tag's heading and range
        current_tag.config_Hdg
        tags.append(current_tag) #Our tag system goes from 1-16, so this 0-15 loop needs minor offset 

    ###CameraFieldCoordinates = [-1,-1,-1] #Will be used to give exact position of the camera based on fields coordinate system
    # loop forever
    while True:
        current_cmd = subDS.get() #get() retrieves data from subscribed topic
        if current_cmd != cmd: #If the command has changed, update!
            cmd = current_cmd
            Tag_List = Set_Tag_List(cmd,Alliance)

        frame_time, input_img = input_stream.grabFrame(img)
        output_img = np.copy(input_img)
        gray       = cv2.cvtColor (input_img, cv2.COLOR_BGR2GRAY)
        results    = detector.detect(gray)
        for r in results:
            if (r.tag_id in AllTags): #If the detected tag is one of the 16 on the field...
                ret, rvecs, tvecs = cv2.solvePnP(CORNERS,r.corners,USBcam_mtx,USBcam_dist,cv2.SOLVEPNP_IPPE_SQUARE)
                ZYX,jac     = cv2.Rodrigues(rvecs)
                #Rng         = cv2.Rodrigues(rvecs)[0]
                Hdg         = atan2(tvecs[0],tvecs[2])*180/pi
                CamPos      = -np.matrix(ZYX).T * np.matrix(tvecs)
                CamRange    = (sqrt(CamPos[0]**2+CamPos[2]**2)) #Since our Y-value is never changing, we are no longer factoring it for Rng
                

                #find the existing tag object with the tag_id in results 
                current_tag = tags[r.tag_id -1]
                current_tag.update_Rng(CamRange) #now that update_Rng is a class method, the tag object's Rng variable gets updated
                current_tag.update_Hdg(Hdg)
                SWSPos[0]   = CamPos[0] - SWEETSPOT[r.tag_id][0] #X, Y, and Z Coordinates w/ respect to Sweetspot of interst
                SWSPos[1]   = CamPos[1] - SWEETSPOT[r.tag_id][1]
                SWSPos[2]   = CamPos[2] - SWEETSPOT[r.tag_id][2]
                SWSRng      = sqrt(SWSPos[0]**2+SWSPos[2]**2)
                SWSHdg      = atan2(SWSPos[0],SWSPos[2])*180/pi
                #Also look into potentially making these Arrays just one

                WorldX = TAG_LOCATION[r.tag_id][0]+CamPos[2]
                WorldY = TAG_LOCATION[r.tag_id][1]+CamPos[0] #Concern: Is world Y-axis a different plane than CamY-axis?
                WorldX,WorldY = rotate ((WorldX,WorldY),(TAG_LOCATION[r.tag_id][0],TAG_LOCATION[r.tag_id][1]),TAG_LOCATION[r.tag_id][3])

                pubCamWorldX.set(WorldX)
                pubCamWorldY.set(WorldY) 
                pubCamTag_id.set(r.tag_id)
        ClosestTag(tags,ID_to_Game_Element, Closest_ID, Closest_Rng, Closest_Hdg) #Currently only evaluates after all the detected april tags have been added to tables
        
        output_stream.putFrame(output_img)
    #Note for next time: polishing of SweetspotV0 code needs to still be tested on PI