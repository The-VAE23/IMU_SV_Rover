# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# Created By  : Vistasp Edulji
# Created Date: 02/04/22
# version ='0.8'
# ---------------------------------------------------------------------------
"""Pyhton script for autonoumous rover using computer vision and inertial methods. The following script controls and interprets data from the remote"""
# ---------------------------------------------------------------------------

# Required Libraries
import os
import csv
import keyboard
import cv2
from datetime import datetime
from functions import time,np,serial,writeSerial, readSerial, printSerial

# Required variables
M_auto = False                                                                          # Boolean that represents operating mode
rowhead = ["Date and Time","Temperature","Humidity","Latitude","Longiude","Heading"]    # Header for csv file
rows=np.array(rowhead)                                                                  # Object to store sensor data recived
img_path = ""                                                                           # Path to where image is saved

if img_path == "":
    img_path = os.getcwd().replace('\\','/')+'/'                                        # Ensures there is a valid directory to save to if not given one

#Establish Conncection with AR
try:
    AR = serial.Serial(port='COM3', baudrate=9600, timeout=5)
    AR.close()
    AR.open()
    time.sleep(5)
    printSerial(AR)
except:
    print("Arduino Connection Failed")
    exit(0)

def kPs():
    """Function that decodes input from user and returns appropriate String"""
    CM = input("Enter Command: ")
    if CM.startswith("0,") or CM.startswith("1,") or CM == "MANUAL" or CM == "AUTO" or CM == "SD" or CM == "IMG":
        return CM
    else:
        print("CMD not recognized")
        return("0")

def logdata():
    """Function that logs data from sensors and stores it to csv"""
    x = AR.inWaiting()
    if x>0:
        data = str(datetime.now())+","+readSerial(AR).decode('utf-8')
        data = data.split(',')
        rows = np.append(rows,data)
        size = (int(np.size(rows)/np.size(np.array(rowhead))),int(np.size(np.array(rowhead))))
        rows = rows.reshape(size)
    with open('Sensors.csv', 'w') as f:
        write = csv.writer(f)
        write.writerows(rows)
        print(rows)

def logimg():
    """Function that acquires img from the remote device"""
    print("Receiving Image")
    # Variables to identify image shape
    imsize = (102240,230400,409920,921600,2073600,307200,480000,691200,786432,1228800,1470000,1555200,24300,76800)
    imshape = ((240,426),(360,640),(480,854),(720,1280),(1080,1920),(480,640),(800,600),(720,960),(768,1024),(960,1280),(1050,1400),(1440,1080),(180,135),(240,320))
    # Img placeholder
    img = []
    while True:
        x=AR.inWaiting()
        if x>0:
            data = readSerial(AR).decode('utf-8').split(',')
            print(data)
            if data[0][0:-2] == "e":
                break
            else:
                for x in data:
                    img.append(int(x))
    img = np.array(img)
    i = imsize.index(img.size)
    img = img.reshape(imshape[i])
    # img = img.astype(np.uint8) #Uncomment if using HSV or other float based image datasets
    time_utc = datetime.now()
    time_utc = str(time_utc)[0:-7].replace(':',"").replace('-',"").replace(' ',"")
    cv2.imwrite(img_path+time_utc+'.png', img)
    print("Image saved")

while True:
    """Loopqcalled during exectuion"""
    if not M_auto:
        if keyboard.is_pressed("i"):
            data = kPs()
            print(str(data))
            writeSerial(AR,data) if data!="0" else "Do Nothing"
            if data == "AUTO":
                M_auto = True
        if keyboard.is_pressed("q"):
            AR.close()
            exit(0)
        x = AR.inWaiting()
        if x>0:
            data = readSerial(AR).decode('utf-8')
            if data[0:-2] == "SD":
                logdata()
            elif data[0:-2] == "IMG":
                logimg()
            else:
                print(data)
    else:
        if keyboard.is_pressed("i"):
            data = kPs()
            writeSerial(AR,data) if data!="0" else "Do Nothing"
            if data == "MANUAL":
                M_auto = False
        if keyboard.is_pressed("q"):
            AR.close()
            exit(0)
        x = AR.inWaiting()
        if x>0:
            data = readSerial(AR).decode('utf-8')
            if data == "SD":
                rows=logdata(rows)
            elif data == "IMG":
                logimg()
            else:
                print(data)