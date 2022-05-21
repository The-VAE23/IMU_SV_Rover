# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# Created By  : Vistasp Edulji
# Created Date: 11/05/22
# version ='1.0'
# ---------------------------------------------------------------------------
"""Pyhton script for autonoumous rover using computer vision and inertial methods. The following script stores different functions called by the main script"""
# ---------------------------------------------------------------------------

# Required Libraries
import serial
import time
import numpy as np
from math import *

PI = 3.141592654

def printSerial(Aobj: serial):
    """Function to read and print serial buffer from Arduino object"""
    x=Aobj.inWaiting()
    while x>0:
        print(Aobj.readline().decode('utf-8'))
        x=Aobj.inWaiting()


def writeSerial(Aobj: serial,CMD):
    """Function to write serial data to Arduino object."""
    try:
        Aobj.write(bytes(str(CMD),'utf-8'))
    except:
        print("Serial Write operation failed for Arduino"+str(Aobj))

def readSerial(Aobj: serial):
    """Function to read data from Arduino object"""
    data = Aobj.readline()
    return data

def getIMU(IMU_board: serial,period: float):
    """Function that reads IMU value and calculates the distance travelled in forward direction"""
    writeSerial(IMU_board,1)
    time.sleep(0.5)
    data = readSerial(IMU_board).decode('utf_8').split(',')
    dist = float(data[0])*period*period
    HDG = 360-float(data[1])
    pitch = float(data[2])
    roll = float(data[3])
    return dist,HDG,pitch,roll

def getVecfromDist(dist: float,HDG: float):
    """Function that converts polar to cartesian"""
    return np.array([[dist*cos(HDG*PI/180)],[dist*sin(HDG*PI/180)]])

def getDistfromVec(vector: np.array):
    """Function that converts cartesian to polar"""
    sq=np.power(vector[0],2)+np.power(vector[1],2)
    HDG = atan2(float(vector[1]),float(vector[0]))
    return sqrt(float(sq)) , HDG*180/PI if HDG*180/PI >=0 else HDG*180/PI+360

def giveDriveCMD(dist: float,st_angle: float,drive_mode: int,Driving_Board: serial):
    """Function the sends the CMD to the bot to execute the driving"""
    if drive_mode == 0:
        CMDs = ["30","0"]
        print(CMDs)
        for x in CMDs:
            writeSerial(Driving_Board,x)
    else:
        CMDs = ["",""]
        st_angle = 20 if st_angle > 20 else st_angle
        st_angle = -20 if st_angle < -20 else st_angle
        CMDs[0]= "3"+str(st_angle)
        if dist < 1:
            speed = 5
        elif dist > 1 and dist < 10:
            speed = 7
        else:
            speed = 10
        CMDs[1] = "1"+str(drive_mode)+str(speed)
        print(CMDs)
        for x in CMDs:
            writeSerial(Driving_Board,x)

def sendIMGdata(IMG: np.array,AR: serial):
    """Function used to send img data over RF"""
    writeSerial(AR,2)
    IMG = IMG.reshape((1,np.size(IMG)))
    i=0
    data=0
    while i < IMG.size:
        x = AR.inWaiting()
        if x>0:
            data = readSerial(AR).decode('utf-8')
        else:
            data = "0"
        if int(data) == 1:
            if IMG.size - i > 7:
                string = str(IMG[0,i])+','+str(IMG[0,i+1])+','+str(IMG[0,i+2])+','+str(IMG[0,i+3])+','+str(IMG[0,i+4])+','+str(IMG[0,i+5])+','+str(IMG[0,i+6])+','+str(IMG[0,i+7])
                writeSerial(AR,string)
            elif IMG.size - i == 7:
                string = str(IMG[0,i])+','+str(IMG[0,i+1])+','+str(IMG[0,i+2])+','+str(IMG[0,i+3])+','+str(IMG[0,i+4])+','+str(IMG[0,i+5])+','+str(IMG[0,i+6])
                writeSerial(AR,string)
            elif IMG.size - i == 6:
                string = str(IMG[0,i])+','+str(IMG[0,i+1])+','+str(IMG[0,i+2])+','+str(IMG[0,i+3])+','+str(IMG[0,i+4])+','+str(IMG[0,i+5])
                writeSerial(AR,string)
            elif IMG.size - i == 5:
                string = str(IMG[0,i])+','+str(IMG[0,i+1])+','+str(IMG[0,i+2])+','+str(IMG[0,i+3])+','+str(IMG[0,i+4])
                writeSerial(AR,string)
            elif IMG.size - i == 4:
                string = str(IMG[0,i])+','+str(IMG[0,i+1])+','+str(IMG[0,i+2])+','+str(IMG[0,i+3])
                writeSerial(AR,string)
            elif IMG.size - i == 3:
                string = str(IMG[0,i])+','+str(IMG[0,i+1])+','+str(IMG[0,i+2])
                writeSerial(AR,string)
            elif IMG.size - i == 2:
                string = str(IMG[0,i])+','+str(IMG[0,i+1])
                writeSerial(AR,string)
            elif IMG.size - i == 1:
                string = str(IMG[0,i])
                writeSerial(AR,string)
            i+=8
            i=i if i < IMG.size else IMG.size
            print(str(i)+'/'+str(IMG.size))
    time.sleep(1)
    writeSerial(AR,"e")