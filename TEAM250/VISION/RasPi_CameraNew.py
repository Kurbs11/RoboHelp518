# $Revision: 2.8 $
# $Date: 2024-01-12 23:33:45-05 $
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

Red        = (0,0,255)
Orange     = (0,127,255)
Yellow     = (32,255,255)
Green      = (0,255,0)
Blue       = (255,0,0)
White      = (255,255,255)
Black      = (0,0,0)

FudgeFactor = 1.20 #Range and Heading control calculations were previously off by this multiple.
FudgeOffset = 0.55

# TAG-centric coordinates. X is right, Y is up, Z points out of screen
CORNERS   = np.array([[-4.0,  4.0, 0.0],   # Start at NW corner and proceed 
                      [ 4.0,  4.0, 0.0],   # clockwise. This is the order
                      [ 4.0, -4.0, 0.0],   # returned by Detector.
                      [-4.0, -4.0, 0.0]])  
OFFSET_1L = np.array([0.0, -15.0, 20.0])
HEIGHT1   = np.array([0.0, 10.0, 0.0])
XSECTION  = np.array([[ 4.0, 0.0,  4.0],
                      [ 4.0, 0.0, -4.0],
                      [-4.0, 0.0, -4.0],
                      [-4.0, 0.0,  4.0]])
LBASE_1   = OFFSET_1L + XSECTION


def draw_box(img,base,top,color,weight): #For the green cube displayed above targets.
    # bottom
    # last param (16) is for anti-alias line type LINE_AA
    cv2.line (img, base[0], base[1], color, weight, 16)
    cv2.line (img, base[1], base[2], color, weight, 16)
    cv2.line (img, base[2], base[3], color, weight, 16)
    cv2.line (img, base[3], base[0], color, weight, 16)
    # top
    # last param (16) is for anti-alias line type LINE_AA
    cv2.line (img, top[0], top[1], color, weight, 16)
    cv2.line (img, top[1], top[2], color, weight, 16)
    cv2.line (img, top[2], top[3], color, weight, 16)
    cv2.line (img, top[3], top[0], color, weight, 16)
    # sides
    # last param (16) is for anti-alias line type LINE_AA
    cv2.line (img, base[0], top[0], color, weight, 16)
    cv2.line (img, base[1], top[1], color, weight, 16)
    cv2.line (img, base[2], top[2], color, weight, 16)
    cv2.line (img, base[3], top[3], color, weight, 16)

options  = apriltag.DetectorOptions(
           families="tag36h11" #Type of tags FRC Crescendo uses.
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

team            = None #Will be 250 when synced at competition.
server          = False #For multi-camera system.
cameraConfigs   = []
switchedCameraConfigs = []
cameras         = []
AllianceTags    = [ 1, 2, 3, 4, 5, 6, 7, 8 ] #Although there are 16 Tags on field, there are 8 per alliance. 
#Need to verify if we get alliance color input provided from FRC
CamPos          = np.array([0.0, 0.0, 0.0]) 
CamPos          = CamPos.reshape(3,1)
CamRange        = 0.0
NSamples        = 0

def get_camera_parameters(): #All camera parameters are parsed from a JSON file.
    param_file = '/home/pi/Camera_B_800x600_parameters.json'
    #param_file = '/home/pi/Camera_B_640x480_parameters.json'
    Pfile      = open(param_file,'r')
    j          = json.load(Pfile)
    Pfile.close()
    return j["width"], j["height"], np.array(j["mtx"]), np.array(j["dist"])

def check_sample_request():
    with open('/home/pi/sampleCtl.json','r') as f:
        try:
            j = json.load(f)
            if j["sampleSwitch"] == "True":
                print ("Sample requested...")
                try:
                    RWcheck = open('/home/pi/writeable','w')
                    RWcheck.close()
                    return True,j["sampleCount"],j["tagDistance"],j["tagRotation"]
                except:
                    print ('File system is not writeable. No samples will be collected.')
                    return False, -1, -1, -1
            return False, -1, -1, -1
        except:
            return False, -1, -1, -1

def rot2eul(R): #For additional Heading information. (Not currently used)
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError: #Thought to represent a mismatch of JSON's Key-Value.
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

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    print ("TADA!")
    IP='Fauxbot'

    width, height, USBcam_mtx, USBcam_dist = get_camera_parameters()
    print (width,height)
    print (USBcam_mtx[0,2],USBcam_mtx[1,2])
    Camera_Center =  (int(USBcam_mtx[0,2]+0.5),int(USBcam_mtx[1,2]+0.5))
    sampleRequested,sampleCount,tagDistance,tagRotation = check_sample_request()
    #if sampleRequested:
    #    print ("Sample requested...")

    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("wpilibpi")
        ntinst.setServerTeam(team)
        #ntinst.startDSClient()
        table = ninst.getTable("data")
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

    # loop forever
    while True:
        frame_time, input_img = input_stream.grabFrame(img)
        output_img = np.copy(input_img)
        gray        = cv2.cvtColor (input_img, cv2.COLOR_BGR2GRAY)
        cv2.line    (output_img, (0,0), (width,0), Blue, 5)
        cv2.line    (output_img, (0,0), (0,height), Blue, 5)
        cv2.line    (output_img, (width,0), (width,height), Blue, 5)
        cv2.line    (output_img, (0,height), (width,height), Blue, 5)
        cv2.line    (output_img, (0,int(height/2)), (width,int(height/2)), White, 2)
        cv2.line    (output_img, (int(width/2),0), (int(width/2),height), Green, 2)
        cv2.circle  (output_img, Camera_Center, 15, Green, 5)
        results     = detector.detect(gray) #return list of tags in frame
        for r in results:
            if (r.tag_id in AllianceTags):
                # extract the bounding box (x, y)-coordinates for the AprilTag
                # and convert each of the (x, y)-coordinate pairs to integers
                (ptA, ptB, ptC, ptD) = r.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                label_color = (0, 0, 255)
                cv2.line    (output_img, ptA, ptB, Red, 6)
                cv2.line    (output_img, ptB, ptC, Red, 6)
                cv2.line    (output_img, ptC, ptD, Red, 6)
                cv2.line    (output_img, ptD, ptA, Red, 6)
                # draw the center (x, y)-coordinates of the AprilTag
                (cX, cY)    = (int(r.center[0]), int(r.center[1]))
                cv2.circle  (output_img, (cX, cY), 5, Red, -1)
                ## draw the tag family on the frame
                tagId       = str(r.tag_id)
                #tagCount    += 1
                ret, rvecs, tvecs = cv2.solvePnP(CORNERS,r.corners,USBcam_mtx,USBcam_dist,cv2.SOLVEPNP_IPPE_SQUARE)
                ZYX,jac    = cv2.Rodrigues(rvecs)
                #R          = cv2.Rodrigues(rvecs)[0]
                #roll       = 180*atan2(-R[2][1], R[2][2])/pi
                #pitch      = 180*asin(R[2][0])/pi
                #yaw        = 180*atan2(-R[1][0], R[0][0])/pi
                Rng         = cv2.Rodrigues(rvecs)[0]
                Hdg         = atan2(tvecs[0],tvecs[2])*180/pi
                CamPos   = -np.matrix(ZYX).T * np.matrix(tvecs)
                CamRange = (sqrt(CamPos[0]**2+CamPos[1]**2+CamPos[2]**2)-FudgeOffset)/FudgeFactor
                cv2.putText (output_img, "RNG = "+str(int(CamRange+0.5)),(200,100),
                             cv2.FONT_HERSHEY_SIMPLEX, 2.5, label_color, 6)
                cv2.putText (output_img, "HDG = "+str(int(Hdg)),(200,200),
                             cv2.FONT_HERSHEY_SIMPLEX, 2.5, label_color, 6)
                tvecs = tvecs + ([[0.0], [-5.0], [20.0]])
                bottom, jac = cv2.projectPoints(LBASE_1,rvecs,tvecs,USBcam_mtx,USBcam_dist)
                top,    jac = cv2.projectPoints(LBASE_1+HEIGHT1,rvecs,tvecs,USBcam_mtx,USBcam_dist)
                draw_box(output_img,bottom.reshape(4,2).astype(int),
                               top.reshape(4,2).astype(int),Green,2)

                if sampleRequested and sampleCount >= 0:
                    if NSamples <= sampleCount:
                        print (tagId, cX, cY)
                        print ('T ',tvecs[0],tvecs[1],tvecs[2]) 
                        print ('R ',rvecs[0],rvecs[1],rvecs[2]) 
                        ZYX,jac = cv2.Rodrigues(rvecs)
                        R       = cv2.Rodrigues(rvecs)[0]
                        print ('Rodrigues:\n',R)
                        print('ZYX:\n{0:6.5f}, {1:6.5f}, {2:6.5f}'.format(ZYX[0,0],ZYX[0,1],ZYX[0,2]))
                        print('{0:6.5f}, {1:6.5f}, {2:6.5f}'.format(ZYX[1,0],ZYX[1,1],ZYX[1,2]))
                        print('{0:6.5f}, {1:6.5f}, {2:6.5f}'.format(ZYX[2,0],ZYX[2,1],ZYX[2,2]))
                        print ('Cam ',CamPos[0],CamPos[1],CamPos[2])
                        # The following file open/close is likely creating a lot of needless 
                        # overhead but I'm trying to prevent data loss upon program termination.
                        Cam_file=open('CamSample.dat','a')
                        tvec_file=open('tvecSample.dat','a')
                        rvec_file=open('rvecSample.dat','a')
                        tvec_file.write('{0:d} {1:5.3f} {2:5.3f} {3:5.3f}\n'.format(tagDistance, *tvecs[0],*tvecs[1],*tvecs[2]))
                        rvec_file.write('{0:d} {1:5.3f} {2:5.3f} {3:5.3f}\n'.format(tagDistance, *rvecs[0],*rvecs[1],*rvecs[2]))
                        Cam_file.write('{0:d} {1:5.3f} {2:5.3f} {3:5.3f}\n'.format(tagDistance, float(CamPos[0]),float(CamPos[1]),float(CamPos[2])))
                        Cam_file.close
                        tvec_file.close
                        rvec_file.close
                        NSamples += 1

        output_stream.putFrame(output_img)

        
