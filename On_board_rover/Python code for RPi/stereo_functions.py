# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# Created By  : Vistasp Edulji
# Created Date: 21/05/22
# version ='1.0'
# ---------------------------------------------------------------------------
"""Pyhton script for autonoumous rover using computer vision and inertial methods. The following script stores different functions for stereo object detection used by the rover"""
# ---------------------------------------------------------------------------

#Required Libraries

import cv2 as cv
import numpy as np

def Imcap(cam_port: int,size: tuple = 0 ):
    """Function that captures image and realeases the buffer. Necessary on systems like the RPI for multi-camera capture."""
    cap = cv.VideoCapture(cam_port)

    if size != 0:
        cap.set(cv.CAP_PROP_FRAME_WIDTH,size[1])
        cap.set(cv.CAP_PROP_FRAME_HEIGHT,size[0])

    success,img = cap.read()
    cap.release()
    return img if success else "IMFAIL"

def ImgZoneAvg(img: cv,corner1: tuple, corner2: tuple):
    """Function that returns the percentage of non-zero pixel values in given image region"""
    sum=0; count=0; wcount=0                                                                            # Function variables
    xRange = range(corner1[0],corner2[0])                                                               # Cycle through rows
    yRange = range(corner1[1],corner2[1])                                                               # Cycle through columns

    for x in xRange:
        for y in yRange:
            if img[x,y] > 0:
                wcount+=1
            count+=1

    return float(wcount)/float(count)                                                                   # Retrun percentage of non-zero pixels

def ImMask(img: cv,thresh: np.uint8):
    """Function that masks the image based on threshold value"""
    img = cv.cvtColor(img,cv.COLOR_BGR2HSV)[:,:,1]
    cols = np.size(img)
    img=img.reshape((1,cols))
    for x in img:
            line = x

    i=0
    for x in line:
        if x<=thresh:
            line[i]=0
        i+=1

    img=line
    imsize = (102240,230400,409920,921600,2073600,307200,480000,691200,786432,1228800,1470000,1555200,24300,76800)
    imshape = ((240,426),(360,640),(480,854),(720,1280),(1080,1920),(480,640),(800,600),(720,960),(768,1024),(960,1280),(1050,1400),(1440,1080),(180,135),(240,320))
    i = imsize.index(img.size)
    return img.reshape(imshape[i])                                                                      # Return masked image


def ImgDiparity(ImgL: cv, ImgR: cv):
    """Function to coumpute disparity"""
    img = np.subtract(ImgL,ImgR)
    img = img.astype(np.uint8)
    cols = np.size(img)
    img=img.reshape((1,cols))
    for x in img:
            line = x

    i=0
    for x in line:
        if x==0:
            line[i]=0
        else:
            line[i]=255
        i+=1

    img=line
    imsize = (102240,230400,409920,921600,2073600,307200,480000,691200,786432,1228800,1470000,1555200,24300,76800)
    imshape = ((240,426),(360,640),(480,854),(720,1280),(1080,1920),(480,640),(800,600),(720,960),(768,1024),(960,1280),(1050,1400),(1440,1080),(180,135),(240,320))
    i = imsize.index(img.size)
    return img.reshape(imshape[i])                                                                      # Return computed disparity from images

def GetSteerBool(disparity: cv,corner1: tuple,corner2: tuple,threshold: int):
    """Function that returns a boolean value if object is detected"""
    mean = ImgZoneAvg(disparity,corner1,corner2)                                                        # Get Mean Value of pixels from the zone
    if mean > threshold:                                                                                # Check if value greater than threshold (object in path)
        return True
    else:
        return False

def GetSteerDir(disparity: cv,I1C1: tuple,I1C2: tuple,I2C1: tuple,I2C2: tuple,threshold: int):
    """Function that checks which side is clear"""
    meanL = ImgZoneAvg(disparity,I1C1,I1C2)                                                             # Compute mean for left side of disparity map
    meanR = ImgZoneAvg(disparity,I2C1,I2C2)                                                             # Compute mean for right side of diparity map

    if meanL < meanR and meanL < threshold:                                                             # Check which side of disparity map and return accordingly
        return 1
    elif meanR < meanL and meanR < threshold:
        return -1
    else:
        return 0

def MainSteroOut(CamL: int,CamR: int):
    """Function that returns the values necessary to make decission based on SV"""

    imgL = Imcap(cam_port = CamL)
    imgR = Imcap(cam_port = CamR)

    imgL = ImMask(imgL,70)
    imgR = ImMask(imgR,70)

    disparity = ImgDiparity(imgL,imgR)
    threshold = 0.55

    status = GetSteerBool(disparity,(0,120),(400,560),threshold)

    if status:
        return status, GetSteerDir(disparity,(0,0),(400,110),(0,570),(400,640),threshold)
    else:
        return status, 0

def TestSteroOut(imgL: cv,imgR: cv):
    """Function that returns the values necessary to make decission based on SV. Remove this function if deploying on headless systems"""

    imgL = ImMask(imgL,70)
    imgR = ImMask(imgR,70)

    disparity = ImgDiparity(imgL,imgR)
    threshold = 0.55

    status = GetSteerBool(disparity,(0,120),(400,560),threshold)

    if status:
        return status, GetSteerDir(disparity,(0,0),(400,110),(0,570),(400,640),threshold), disparity
    else:
        return status, 0, disparity



