# Currently expects networktable server on fauxbot.

# $Revision: 1.3 $
# $Date: 2024-03-03 07:15:31-05 $
# $Author: animation $
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

# (Note about coordinates: We follow the convention used in the official field
# drawings - X axis is the wall OPPOSITE the Amps, Y axis is the Blue Alliance
# wall.
# Note about rotation values: We picked NORTH to be the +Y direction. Internal
# calculations are in radians, published values are in degrees between +/- 180.
# Positive degrees are toward the EAST (+X direction), negative are toward the
# WEST (-X direction), like a compass.)

# What's going on here?
# The robot needs to know how to get from where it is to where it wants to go.
# "Where it wants to go" is a pre-defined HOTSPOT that is an absolute field X,Y
# position and a rotation.
# The camera is constantly looking for any and all Apriltags. They are used to
# update the robot's location data - an array (BEARINGS) that tells it where it
# is (heading & range) in relation to each HOTSPOT.
#
#
#
#

from math import pi,atan2,asin,sqrt, sin, cos, degrees 
import json
import time
import sys
import cv2
import numpy as np
import apriltag
import subprocess

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from ntcore import NetworkTableInstance, EventFlags

#FudgeFactor = 1.20 #these are now retrieved from get_camera_parameters()
#FudgeOffset = 0.55

# TAG-centric coordinates. X is right, Y is up, Z points out of screen
CORNERS   = np.array([[-4.0,  4.0, 0.0],   # Start at NW corner and proceed 
                      [ 4.0,  4.0, 0.0],   # clockwise. This is the order
                      [ 4.0, -4.0, 0.0],   # returned by Detector.
                      [-4.0, -4.0, 0.0]])  
# Hotspot axes: Increasing X moves the hotspot to the tag's left and camera's right. 
# Increasing Y move the hotspot up. Increasing Z moves the hotspot forward from the tag
# along the normal to the tag face.
HOTSPOT=np.array(([[12.0, 0.0, 72.0]])*17)
HOTSPOT=HOTSPOT.reshape(17,3)

# Hotspots identify places on the field the robot wants to know about. These can
# be tags or spikes or something like a waypoint. In addition to location data
# Hotspots have orientation data: robot's rotation wrt NORTH. Hotspots are an
# evolution of Sweetspots with several differences.
# 1) Their X,Y values are relative to the field.
# 2) They're no longer associated with a tag.
# 3) There are more Hotspots than Sweetspots.
# 4) The Z values are gone and replaced by rotation values. They're in radians.
#
# Right now there are 32 Hotspots. The first 16 are for the tags, the rest are
# for the spikes.

#All in ID_to_Game_Element dictionary order
HotSpots = np.array( [[999.00, 999.00,999.00]
,[583.29,  15.68, 150.00]
,[626.82,  40.79, 150.00]
,[604.73, 208.17,  90.00]
,[604.73, 230.42,  90.00]
,[590.77, 323.00,   0.00]
,[ 84.50, 323.00,   0.00]
,[ 46.50, 206.42, -90.00]
,[ 46.50, 184.17, -90.00]
,[  3.63,  28.79,-150.00]
,[ 47.15,   3.68,-150.00]
,[458.30, 152.19, 150.00]
,[479.08, 183.10,  30.00]
,[441.74, 173.62,  90.00]
,[209.48, 149.62, -90.00]
,[193.12, 171.10, -30.00]
,[172.34, 140.19,-150.00]
,[116,275.64, 0]
,[116, 218.64, 0]
,[116, 161.64, 0]
,[326.6, 293.64, 0]
,[326.6, 227.64, 0]
,[326.6, 161.64, 0]
,[326.6, 95.64, 0]
,[326.6, 29.64, 0]
,[537.2, 161.64, 0]
,[537.2, 218.64,0]
,[537.2, 275.64, 0]
,[326.6, 293.64, 0]
,[326.6, 227.64, 0]
,[326.6, 161.64, 0]
,[326.6, 95.64, 0]
,[326.6, 29.64, 0]])


#Tag locations are all with respect to an origin in the red source zone
TAG_LOCATION= [[999.0, 999.0, 999.0]
, [593.68,   9.68, 53.38,  2.09]
, [637.21,  34.79, 53.38,  2.09]
, [652.73, 196.17, 57.13,  3.14]
, [652.73, 218.42, 57.13,  3.14]
, [578.77, 323.00, 53.38,  4.71]
, [ 72.50, 323.00, 53.38,  4.71]
, [ -1.50, 218.42, 57.13,  0.00]
, [ -1.50, 196.17, 57.13,  0.00]
, [ 14.02,  34.79, 53.38,  1.05]
, [ 57.54,   9.68, 53.38,  1.05]
, [468.69, 146.19, 52.00,  5.24]
, [468.69, 177.10, 52.00,  1.05]
, [441.74, 161.62, 52.00,  3.14]
, [209.48, 161.62, 52.00,  0.00]
, [182.73, 177.10, 52.00,  2.09]
, [182.73, 146.19, 52.00,  4.19]]
RED_SPIKES   =np.array([(528.0,161.64),(528.0,218.64),(528.0,275.64)])
BLUE_SPIKES  =np.array([(114.0,161.64),(114.0,218.64),(114.0,275.64)])
PURPLE_SPIKES=np.array([(326.0,29.64),(326.0,95.64),(326.0,161.64),(326.0,227.64),(326.0,293.64)]) #Centerline spikes
BEARINGS=np.array([(999.0,999.0)]*33) #Will be added to closest function for offscreen tag detection
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
    10: "Red Source Left",
    11: "Red Stage Left",
    12: "Red Stage Right",
    13: "Red Stage Center",
    14: "Blue Stage Center",
    15: "Blue Stage Left",
    16: "Blue Stage Right",
    17: "Blue Spike Left",
    18: "Blue Spike Middle", 
    19: "Blue Spike Right",
    20: "Blue Centerline Spike 1", #Centerline spikes are listed 1-5 starting farthest from scoring table side (per request)
    21: "Blue Centerline Spike 2",
    22: "Blue Centerline Spike 3",
    23: "Blue Centerline Spike 4",
    24: "Blue Centerline Spike 5",
    25: "Red Spike Left",
    26: "Red Spike Middle", 
    27: "Red Spike Right",
    28: "Red Centerline Spike 1", #Hotspots for centerline spikes may differ per alliance despite spikes physically being in same spot
    29: "Red Centerline Spike 2", #This is to allow for different strategies of approach like dragging all spikes onto your side
    30: "Red Centerline Spike 3",
    31: "Red Centerline Spike 4",
    32: "Red Centerline Spike 5",

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
WorldX          = 999
WorldY          = 999
Hotspot_Ctr     = 0

class Tag(object) :
    def __init__(self, id, ntinst):
        
        self.Tagid = id
        self.ntinst = ntinst
        #self.tagtable = self.ntinst.getTable("Tag_{:02d}".format(id))  
        self.tagtable = self.ntinst.getTable(ID_to_Game_Element[id]) #Table name now tells you game object
        #IDs 17-32 are not for tags, but for spikes, which are where the notes are located at the start of match
        #Spikes possess all of the other information tags do, except we can't directly measure their distance and need the info of a tag in view

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
        #May need to add a new attribute for necessary rotation (once directly in hotspot heading=0 so you can't tell your orientation)
        self.config_Rot = self.tagtable.getDoubleTopic("Rot").publish()
        self.Rot = 361 #Degree representation (may need to be radians, also placeholder value)
    def update_Rot(self,new_Rot): 
        self.Rot = new_Rot 
        self.config_Rot.set(self.Rot)    
        #Note: Spikes wont be detected by OpenCV so their values will need to be updated in a separate function

def rotate(point, origin, angle): #Point is a tuple of the camera's XZ-WorldLocation 
    #Origin is a tuple of the closest tag's XZ World-Location, Angle is the tags offset
    ox, oy = origin # ox goes to the origin tuple's first element (x), oy for 2nd
    px, py = point
    qx = ox + cos(angle) * (px - ox) - sin(angle) * (py - oy)
    qy = oy + sin(angle) * (px - ox) + cos(angle) * (py - oy)
    return qx, qy

def Set_Tag_List(cmd,Alliance): #Command is from Drivestation
    return AllTags
    #if cmd == 'E':
    #    return AllTags
    #if Alliance == 'RED':
    #    if cmd == 'A': #A for Auton
    #        pass                 # <--- fix this
    #    elif cmd == 'M': #M for Amp :0
    #        return [5]
    #    elif cmd == 'R': #R for Source ;0
    #        return [9,10]
    #    elif cmd == 'S': #S for Speaker 
    #        return [3,4]
    #    elif cmd == '':
    #        return []
    #elif Alliance == 'BLUE':
    #    if cmd == 'A':
    #        pass                 # <--- fix this
    #    elif cmd == 'M':
    #        return [6]
    #    elif cmd == 'R':
    #        return [1,2]
    #    elif cmd == 'S':
    #        return [7,8]
    #    elif cmd == '':
    #        return []

def register_Hotspot(X, Y, HdgRad, Ctr): #Used to acquire accurate values of hotspots based on robots position
# X & Y are relateive to the field (world)
# Rot is relative to NORTH
    if X == 999 or Y == 999: #Will be 999 before it reads its first tag
        return()
    Rot=999
    subprocess.run(["sudo","mount","-o","remount","rw","/"])
    SSfile=open('Sweetspot.dat','a') 
    SSfile.write('{0:5.3f} {1:5.3f} {2:5.3f}\n'.format(float(X),float(Y),float(Rot)))
    if Ctr == 1:
        SSfile.write ('======================\n')
    SSfile.close
    subprocess.run(["sudo","mount","-o","remount","ro","/"])
    return ()

def get_camera_parameters():
    param_file = '/home/pi/Camera_Matrix.json'
    #param_file = '/home/pi/Camera_B_640x480_parameters.json'
    Pfile      = open(param_file,'r')
    j          = json.load(Pfile)
    Pfile.close()
    return j["width"], j["height"], np.array(j["mtx"]), np.array(j["dist"]), j["FudgeFactor"], j["FudgeOffset"]


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
        Closest_ID.set("None In View") # Table topics can also take in strings
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

HEADING  = [999.0]*33
RANGE    = [999.0]*33

"""
BuildWorld takes in the camera's World X & WorldY, its heading wrt to the
reference tag and the reference tag id. It populates the BEARINGS table with the
headings and ranges of the other 15 tags.
THE SPIKES HAVE BEEN ADDED. This brings thr total to 32.
"""
def BuildWorld (WX, WY, HdgRad, RefTag):
    # New method of knowing position with respect to all tags by only reading one
    # Calculate intermediate angles relative to X-axis. Adjust for heading later.
    # A2R: angle between RefTag and X-axis at camera
    # A2K: angle between Tag K and X-axis at camera

    deltaX = TAG_LOCATION[RefTag][0] - WX
    deltaY = TAG_LOCATION[RefTag][1] - WY
    A2R = atan2(deltaY,deltaX)

    

    for K in range(1,33):
        deltaX = HotSpots[K][0] - WX
        deltaY = HotSpots[K][1] - WY
        A2K = atan2(deltaY,deltaX)
        H2K = A2K - A2R - HdgRad
        if H2K > pi:
           H2K = H2K - 2*pi
        BEARINGS[K][0] = -degrees(H2K)
        BEARINGS[K][1] = sqrt(deltaX**2 + deltaY**2)


if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    print ("TADA!")
    #IP='fauxbot'
    IP='169.254.27.134'
    #IP='192.168.1.46'

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
    img = np.zeros(shape=(height, width, 3), dtype=np.uint8)
    input_stream = CameraServer.getVideo()
    output_stream = CameraServer.putVideo('Vision 2024', width, height)

    ##Initialization of network tables for Tags1-16 is now handled in the tag object construction. 

    # Robot programmers want everything, no filters. So this is only used to register hotspots.
    ## This DS_tbl will be published by the Driver Station laptop in competition.
    ## For now, it's published by a Raspberry Pi.
    DS_tbl = ntinst.getTable("DS")
    subDS  = DS_tbl.getStringTopic("Cmd").subscribe(' ') #Since we are only Reading in Driverstation entries, we Subscribe
    subAlliance = DS_tbl.getStringTopic("Alliance").subscribe(' ')

    # Headings_tbl & Ranges_tbl are for development use. They're not needed in
    # competition.
    Headings_tbl = ntinst.getTable("Headings")
    pubHeadings  = Headings_tbl.getDoubleArrayTopic("WorldHdg").publish()
    Ranges_tbl   = ntinst.getTable("Ranges")
    pubRanges    = Ranges_tbl.getDoubleArrayTopic("WorldRng").publish()


    Closest_tag = ntinst.getTable("Closest Tag")
    Closest_ID = Closest_tag.getStringTopic("Location").publish() #Topic has input based on ID to game-element map
    Closest_Rng = Closest_tag.getDoubleTopic("Rng").publish()
    Closest_Hdg = Closest_tag.getDoubleTopic("Hdg").publish()

    CamPos_tbl = ntinst.getTable("CamPos") #These are for the camera's field coordinates
    pubCamWorldX = CamPos_tbl.getDoubleTopic("X").publish()
    pubCamWorldY = CamPos_tbl.getDoubleTopic("Y").publish()
    pubCamWorldR = CamPos_tbl.getDoubleTopic("Rot").publish()
    pubCamTag_id =  CamPos_tbl.getDoubleTopic("Tag_ID").publish()
    
    # Don't wait for Drive Station. The programmers want everything, all the time
    ## wait for Drive Station
    #while Alliance not in ['RED','BLUE']:
    #    Alliance = subAlliance.get()
    #    time.sleep(0.1) #Delays program execution(Re-looping) by .1s
    #while current_cmd == 'none':
    #    current_cmd = subDS.get('none')
    #print ("Alliance:",Alliance,"  Cmd:",current_cmd)

    #Create 32 tag objects (This is an alternative way to the 96 lines of code up above)
    tags = []
    for i in range(32): 
        current_tag = Tag(i+1, ntinst)
        current_tag.Tagid
        current_tag.config_Rng #Configuring the network table slots for current tag's heading and range
        current_tag.config_Hdg
        tags.append(current_tag) #Our tag system goes from 1-16, so this 0-15 loop needs minor offset 
        #The other 16 "tag" objects represent the spikes

    ###CameraFieldCoordinates = [-1,-1,-1] #Will be used to give exact position of the camera based on fields coordinate system
    # loop forever
    while True:
        current_cmd = subDS.get() #get() retrieves data from subscribed topic
        
        if current_cmd != cmd: #If the command has changed, update!
            cmd = current_cmd
            if cmd == "W":
                Hotspot_Ctr = 10
                #register_Hotspot(WorldX, WorldY,-180)
            else:
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
                HdgRad      = atan2(tvecs[0],tvecs[2])
                Hdg         = HdgRad * 180/pi
                CamPos      = -np.matrix(ZYX).T * np.matrix(tvecs)
                CamRange    = (sqrt(CamPos[0]**2+CamPos[2]**2)) #Since our Y-value is never changing, we are no longer factoring it for Rng
                

                #find the existing tag object with the tag_id in results 
                current_tag = tags[r.tag_id -1]
                current_tag.update_Rng(CamRange) #now that update_Rng is a class method, the tag object's Rng variable gets updated
                current_tag.update_Hdg(Hdg)
                SWSPos[0]   = CamPos[0] - HOTSPOT[r.tag_id][0] #X, Y, and Z Coordinates w/ respect to Hotspot of interst
                SWSPos[1]   = CamPos[1] - HOTSPOT[r.tag_id][1]
                SWSPos[2]   = CamPos[2] - HOTSPOT[r.tag_id][2]
                SWSRng      = sqrt(SWSPos[0]**2+SWSPos[2]**2)
                SWSHdg      = atan2(SWSPos[0],SWSPos[2])*180/pi
                #Also look into potentially making these Arrays just one

                WorldX = TAG_LOCATION[r.tag_id][0]+CamPos[2]
                WorldY = TAG_LOCATION[r.tag_id][1]+CamPos[0] #Concern: Is world Y-axis a different plane than CamY-axis?
                WorldX,WorldY = rotate ((WorldX,WorldY),(TAG_LOCATION[r.tag_id][0],TAG_LOCATION[r.tag_id][1]),TAG_LOCATION[r.tag_id][3])
                pubCamWorldX.set(WorldX) 
                pubCamWorldY.set(WorldY) 
                # determine camera's rotation wrt NORTH
                deltaX = TAG_LOCATION[r.tag_id][0] - WorldX
                deltaY = TAG_LOCATION[r.tag_id][1] - WorldY
                A2N = atan2(deltaY,deltaX) + HdgRad - pi/2
                if A2N < -pi:
                    A2N = A2N + 2*pi
                print ("A2N: ",-degrees(A2N))
                pubCamWorldR.set(-degrees(A2N))

                pubCamTag_id.set(r.tag_id)
                BEARINGS[r.tag_id][0] = round(Hdg,2)
                BEARINGS[r.tag_id][1] = round(CamRange,2)
                BuildWorld (WorldX, WorldY, HdgRad, r.tag_id) #Consider decreasing scope
                print('TAG HEADING   RANGE')
                #This new part is updated displaying the bearings information in the network tables 
                for i in range (1,33):
                    tags[i].update_Rng(BEARINGS[i][1]) #The network table data associated with this hotspot (tag/spike) is changed to match bearings array 
                    tags[i].update_Hdg(BEARINGS[i][0]) 
                    tags[i].update_Rot(HotSpots[i][2])
                    print ('{0:2d} {1:8.2f} {2:8.2f}'.format(i,BEARINGS[i][0],BEARINGS[i][1]))

                print (' ')
                pubHeadings.set(HEADING)
                pubRanges.set(RANGE)
                if Hotspot_Ctr > 0:
                    register_Hotspot (WorldX, WorldY, HdgRad, Hotspot_Ctr)
                    Hotspot_Ctr -= 1

        ClosestTag(tags,ID_to_Game_Element, Closest_ID, Closest_Rng, Closest_Hdg) #Currently only evaluates after all the detected april tags have been added to tables
        
        output_stream.putFrame(output_img)
