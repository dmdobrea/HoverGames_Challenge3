import os
import sys
import cv2

def checkVideoDevices (inputFile, camNo, isIt_NavQPlus, frameWidth, frameHeight, verb):
    if inputFile != 'None':
        #=======================================================================
        # Based on the input grab a reference to the video file
        if os.path.isfile(inputFile):
            if verb == 1:
                print("[INFO_VS] : Opening input video file: {}".format(inputFile))
            vs = cv2.VideoCapture(inputFile)
        else:
            if verb == 1:
                print("[ERR._VS] : The file {} does not exist! The program will be terminated.".format(inputFile))
            sys.exit()
    else:
        if verb == 1:
            print("[INFO_VS] : Opening input from CAM...")
        if isIt_NavQPlus == 0:
            # it is RPi
            videoPath = "/dev/video{}".format(int(camNo)*2)
            if not os.path.exists(videoPath):           #os.path.isfile - check it is a regular file
                if verb == 1:
                    print( "[ERR._VS] : The camera /dev/video{} is not connected to RPi! The program will be terminated.".format(int(camNo)*2) )
                sys.exit()
            
            vs = cv2.VideoCapture(int(camNo)*2)
            if not vs.isOpened():
                if verb == 1:
                    print( "[ERR._VS] : I can't open the camera /dev/video{} connected to RPi! The program will be terminated.".format(int(camNo)*2) )
                sys.exit()
                
            vs.set(cv2.CAP_PROP_FRAME_WIDTH,  frameWidth)
            vs.set(cv2.CAP_PROP_FRAME_HEIGHT, frameHeight)
            if frameWidth != vs.get(cv2.CAP_PROP_FRAME_WIDTH) or frameHeight != vs.get(cv2.CAP_PROP_FRAME_HEIGHT):
                if verb == 1:
                    print( "[ERR._VS] : Unable to set your resolution. Frame resolution set to: {} x {}".format(cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT) )
                
        elif isIt_NavQPlus == 1:
            # it is NavQ+
            if not os.path.exists('/dev/video3') and not os.path.exists('/dev/video4'):
                if verb == 1:
                    print( "[ERR._VS] : The camera /dev/video{} is not connected to NavQ+! The program will be terminated.".format(int(camNo) + 3) )
                sys.exit()
            
            if int(camNo) == 0:    
                videoCapString = 'v4l2src device={:s} ! video/x-raw,framerate={:d}/1,width={:d},height={:d} ! appsink'.format('/dev/video3', 30, frameWidth, frameHeight )
            elif int(camNo) == 1:
                videoCapString = 'v4l2src device={:s} ! video/x-raw,framerate={:d}/1,width={:d},height={:d} ! appsink'.format('/dev/video4', 30, frameWidth, frameHeight )
            else:
                if verb == 1:
                    print( "[ERR._VS] : At this point only camera /dev/video3 & 5 are accepted in NavQ+! The program will be terminated." )
                sys.exit()
                
            vs = cv2.VideoCapture(videoCapString, cv2.CAP_GSTREAMER)
            if not vs.isOpened():
                if verb == 1:
                    print( "[ERR._VS] : I can't open the NavQ+ camera /dev/video{}! The program will be terminated.".format(int(camNo) +3) ) 
                sys.exit()
        else:
            if verb == 1:
                print( "[ERR._VS] : You select an unknow development board! The program will be terminated.".format(int(camNo) + 3) )
            sys.exit()
    return vs
