# Currently expects networktable server on fauxbot.

# $Revision: 1.2 $
# $Date: 2024-01-19 10:11:51-05 $
# $Author: animation $
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from math import pi,atan2,asin,sqrt
import json
import time
import sys
import cv2
import numpy as np
import apriltag

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from ntcore import NetworkTableInstance, EventFlags

i = 3

FudgeFactor = 1.20
FudgeOffset = 0.55

# TAG-centric coordinates. X is right, Y is up, Z points out of screen
CORNERS   = np.array([[-4.0,  4.0, 0.0],   # Start at NW corner and proceed 
                      [ 4.0,  4.0, 0.0],   # clockwise. This is the order
                      [ 4.0, -4.0, 0.0],   # returned by Detector.
                      [-4.0, -4.0, 0.0]])  
# Sweetspot axes: Increasing X moves the sweetspot to the tag's left and camera's right. 
# Increasing Y move the sweetspot up. Increasing Z moves the sweetspot forward from the tag
# along the normal to the tag face.
#SWEETSPOT = np.array([0.0, 0.0, -30.0])
SWEETSPOT = np.array([0.0, 0.0, 0.0])

TAG_LOCATION=[(-99,-99,-99),(593.68,9.68,53.38),(637.21,34.79,53.38),(652.73,196.17,57.13),(652.73,218.42,57.13),(578.77,323.00,53.38),(72.50,323.00,53.38),(1.50,218.42,57.13),(1.50,196.17,57.13),(14.02,34.79,53.38),(57.54,9.68,53.38),(468.69,146.19,52.00),(468.69,177.10,52.00),(441.74,161.62,52.00),(209.48,161.62,52.00),(182.73,177.10,52.00),(182.73,146.19,52.00)]
BEARINGS=[(-99.0,-99.0)]*17
Red_Tags = [3,4,5,9,10,11,12,13]
Blue_Tags = [1,2,6,7,8,14,15,16]
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
AllianceTags    = [ 1, 2, 3, 4, 5, 6, 7, 8 ]
CamPos          = np.array([0.0, 0.0, 0.0])
CamPos          = CamPos.reshape(3,1)
CamRange        = 0.0

def get_camera_parameters():
    param_file = '/home/pi/Camera_B_800x600_parameters.json'
    #param_file = '/home/pi/Camera_B_640x480_parameters.json'
    Pfile      = open(param_file,'r')
    j          = json.load(Pfile)
    Pfile.close()
    return j["width"], j["height"], np.array(j["mtx"]), np.array(j["dist"])

class Tag(object, id) :
    def __init__(self, id):

        self.Tagid = id
        ntinst = NetworkTableInstance.getDefault() #I believe necessary for accessing Tables
        self.tagtable = ntinst.getTable("Tag_{:02d}".format(id)) #Concern is that these values aren't being adjusted
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

def pushData(TagID,Rng,Hdg):
    if TagID == 1:
        pub1H.set(Hdg)
        pub1R.set(Rng)
        return True
    elif TagID == 2:
        pub2H.set(Hdg)
        pub2R.set(Rng)
        return True
    elif TagID == 3:
        pub3H.set(Hdg)
        pub3R.set(Rng)
        return True
    elif TagID == 4:
        pub4H.set(Hdg)
        pub4R.set(Rng)
        return True
    elif TagID == 5:
        pub5H.set(Hdg)
        pub5R.set(Rng)
        return True
    elif TagID == 6:
        pub6H.set(Hdg)
        pub6R.set(Rng)
        return True
    elif TagID == 7:
        pub7H.set(Hdg)
        pub7R.set(Rng)
        return True
    elif TagID == 8:
        pub8H.set(Hdg)
        pub8R.set(Rng)
        return True
    elif TagID == 9:
        pub9H.set(Hdg)
        pub9R.set(Rng)
        return True
    elif TagID == 10:
        pub10H.set(Hdg)
        pub10R.set(Rng)
        return True
    elif TagID == 11:
        pub11H.set(Hdg)
        pub11R.set(Rng)
        return True
    elif TagID == 12:
        pub12H.set(Hdg)
        pub12R.set(Rng)
        return True
    elif TagID == 13:
        pub13H.set(Hdg)
        pub13R.set(Rng)
        return True
    elif TagID == 14:
        pub14H.set(Hdg)
        pub14R.set(Rng)
        return True
    elif TagID == 15:
        pub15H.set(Hdg)
        pub15R.set(Rng)
        return True
    elif TagID == 16:
        pub16H.set(Hdg)
        pub16R.set(Rng)
        return True

    return False
    
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

def setTag(tags, tag_id, Hdg, Rng): #Should be redundant after class design

    for tag in tags:
        if tag_id == tag.id:
            tag.Hdg.set(Hdg)
            tag.Rng.set(Rng)



def ClosestTag(tags, ID_to_Game_Element, Closest_Object, Closest_Rng, Closest_Hdg):
    #Import from notebook
    if len(tags) == 0:
        Closest_Object.set("None In View") #The assumption is the tables can take in strings
        Closest_Rng.set(-1) #Obvious garbage value
        return
    
    closest = [] #Will be a 3-element array containing the most recent closest id, its range, and its heading
    closest.append(tags[0].Tagid)
    closest.append(tags[0].Rng)
    closest.append(tags[0].Hdg)
    for tag in tags:
        if tag.Rng < closest[1]:
            closest[0] = ID_to_Game_Element[tag.Tagid] #Closest location's id is input into dictionary to output the name of the game object 
            closest[1] = tag.Rng
            closest[2] = tag.Hdg 
    Closest_Object.set(closest[0]) #Updates the network table's location to the new closest one
    Closest_Rng.set(closest[1])
    Closest_Hdg.set(closest[2])

    return 

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    print ("TADA!")
    IP='fauxbot'
    #IP='192.168.1.41'

    width, height, USBcam_mtx, USBcam_dist = get_camera_parameters()
    #sampleRequested,sampleCount,tagDistance,tagRotation = check_sample_request()   # <------

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
        table = ntinst.getTable("data")
        pub1 = table.getDoubleTopic("1").publish()
        pub2 = table.getDoubleTopic("2").publish()

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

    ##Initialization of network tables for Tags1-16 is now handled in the tag object construction. Requires testing before deletion.
    Tag01_tbl = ntinst.getTable("Tag_01") #network table instance-> creates the tabel for Tag1
    pub1H = Tag01_tbl.getDoubleTopic("Hdg").publish()
    pub1R = Tag01_tbl.getDoubleTopic("Rng").publish()

    Tag02_tbl = ntinst.getTable("Tag_02")
    pub2H = Tag02_tbl.getDoubleTopic("Hdg").publish()
    pub2R = Tag02_tbl.getDoubleTopic("Rng").publish()

    Tag03_tbl = ntinst.getTable("Tag_03")
    pub3H = Tag03_tbl.getDoubleTopic("Hdg").publish()
    pub3R = Tag03_tbl.getDoubleTopic("Rng").publish()

    Tag04_tbl = ntinst.getTable("Tag_04")
    pub4H = Tag04_tbl.getDoubleTopic("Hdg").publish()
    pub4R = Tag04_tbl.getDoubleTopic("Rng").publish()

    Tag05_tbl = ntinst.getTable("Tag_05")
    pub5H = Tag05_tbl.getDoubleTopic("Hdg").publish()
    pub5R = Tag05_tbl.getDoubleTopic("Rng").publish()

    Tag06_tbl = ntinst.getTable("Tag_06")
    pub6H = Tag06_tbl.getDoubleTopic("Hdg").publish()
    pub6R = Tag06_tbl.getDoubleTopic("Rng").publish()

    Tag07_tbl = ntinst.getTable("Tag_07")
    pub7H = Tag07_tbl.getDoubleTopic("Hdg").publish()
    pub7R = Tag07_tbl.getDoubleTopic("Rng").publish()

    Tag08_tbl = ntinst.getTable("Tag_08")
    pub8H = Tag08_tbl.getDoubleTopic("Hdg").publish()
    pub8R = Tag08_tbl.getDoubleTopic("Rng").publish()

    Tag09_tbl = ntinst.getTable("Tag_09")
    pub9H = Tag09_tbl.getDoubleTopic("Hdg").publish()
    pub9R = Tag09_tbl.getDoubleTopic("Rng").publish()

    Tag10_tbl = ntinst.getTable("Tag_10")
    pub10H = Tag10_tbl.getDoubleTopic("Hdg").publish()
    pub10R = Tag10_tbl.getDoubleTopic("Rng").publish()

    Tag11_tbl = ntinst.getTable("Tag_11")
    pub11H = Tag11_tbl.getDoubleTopic("Hdg").publish()
    pub11R = Tag11_tbl.getDoubleTopic("Rng").publish()

    Tag12_tbl = ntinst.getTable("Tag_12")
    pub12H = Tag12_tbl.getDoubleTopic("Hdg").publish()
    pub12R = Tag12_tbl.getDoubleTopic("Rng").publish()

    Tag13_tbl = ntinst.getTable("Tag_13")
    pub13H = Tag13_tbl.getDoubleTopic("Hdg").publish()
    pub13R = Tag13_tbl.getDoubleTopic("Rng").publish()

    Tag14_tbl = ntinst.getTable("Tag_14")
    pub14H = Tag14_tbl.getDoubleTopic("Hdg").publish()
    pub14R = Tag14_tbl.getDoubleTopic("Rng").publish()

    Tag15_tbl = ntinst.getTable("Tag_15")
    pub15H = Tag15_tbl.getDoubleTopic("Hdg").publish()
    pub15R = Tag15_tbl.getDoubleTopic("Rng").publish()

    Tag16_tbl = ntinst.getTable("Tag_16")
    pub16H = Tag16_tbl.getDoubleTopic("Hdg").publish()
    pub16R = Tag16_tbl.getDoubleTopic("Rng").publish()

    #Currently calculated based on Closest Rng of Tag in view.
    #Goal is to allow distance from one tag to be able to tell distance from all
    Closest_tag = ntinst.getTable("Closest Tag") 
    Closest_Object = Closest_tag.getDoubleTopic("Location").publish() #Maybe change into actual game element
    Closest_Rng = Closest_tag.getDoubleTopic("Rng").publish()
    Closest_Hdg = Closest_tag.getDoubleTopic("Hdg").publish()



    #Create 16 tag objects (This is an alternative way to the 48 lines of code up above)
    tags = []
    for i in range(16):
        current_tag = Tag(i+1)
        current_tag.Tagid
        current_tag.config_Rng #Configuring the network table slots for current tag's heading and range
        current_tag.config_Hdg
        tags.append(current_tag) #Our tag system goes from 1-16, so this 0-15 loop needs minor offset 


    # loop forever
    while True:
        frame_time, input_img = input_stream.grabFrame(img)
        output_img = np.copy(input_img)
        gray       = cv2.cvtColor (input_img, cv2.COLOR_BGR2GRAY)
        results    = detector.detect(gray)
        for r in results:
            if (r.tag_id in AllTags): #If the detected tag is one of the 16 on the field...
                ret, rvecs, tvecs = cv2.solvePnP(CORNERS-SWEETSPOT,r.corners,USBcam_mtx,USBcam_dist,cv2.SOLVEPNP_IPPE_SQUARE)
                ZYX,jac     = cv2.Rodrigues(rvecs)
                #Rng         = cv2.Rodrigues(rvecs)[0]
                Hdg         = atan2(tvecs[0],tvecs[2])*180/pi
                CamPos      = -np.matrix(ZYX).T * np.matrix(tvecs)
                CamRange    = (sqrt(CamPos[0]**2+CamPos[1]**2+CamPos[2]**2)-FudgeOffset)/FudgeFactor
                
                #setTag(tags, r.tag_id, Hdg, CamRange) #With this new class design, this function should now be unnecessary

                #find the existing tag object with the tag_id in results 
                current_tag = tags[r.tag_id +1]
                current_tag.update_Rng(CamRange) #now that update_Rng is a class method, the tag object's Rng variable gets updated
                current_tag.update_Hdg(Hdg)
                

                if not pushData (r.tag_id,CamRange,Hdg): #pushData updates the Hdgs and Rngs values in the network tables
                    #However, now the update functions handle this. Also, due to the current_tag subscripter [], 
                    #it should always only update detected tags, removing the need for this check.
                    print (r.tag_id," failed.")
        ClosestTag(tags,ID_to_Game_Element, Closest_Object, Closest_Rng, Closest_Hdg) #Currently only evaluates after all the detected april tags have been added to tables

        output_stream.putFrame(output_img)
