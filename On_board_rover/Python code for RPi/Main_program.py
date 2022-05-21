# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Vistasp Edulji
# Created Date: 11/05/22
# version ='1.0'
# ---------------------------------------------------------------------------
"""Pyhton script for autonoumous rover using computer vision and inertial methods. The following is the main script that invokes different functions to execute rover operation"""
# ---------------------------------------------------------------------------

# Requried Libraries
from functions import *
from stereo_functions import MainSteroOut, cv,Imcap
# import os

# Program Variables

M_auto = False                                                  # Boolean value to check mode
T_data_avail = False                                            # Boolean value to check if target data available
dist = 0.0; dist_req = 0.0; hdg = 0.0; pitch = 0.0; roll=0.0    # Variables to pull data from IMU
treshold_dist = 0.1                                             # Treshold distance to stop execution
t=0.0                                                           # Float to count time

# Vector Variables for Navigation
current_vec = np.array([[0],[0]])                               # Vector to acquire data from IMU
tot_vec = current_vec                                           # Vector to keep track of total movement
req_vec = current_vec                                           # Vector used for calculating required speed and steering angle
target_vec = current_vec                                        # Target Vector used to define final position

# Variables for image capture devices
cap1 = 0                                                        # Cam_port for left camera
cap2 = 2                                                        # Cam_port for right camera.


# Setup connections with Arduino's and begin serial communication

#Establish Conncection with AR_1
try:
    AR_1 = serial.Serial(port='COM5', baudrate=9600, timeout=5)     # Arduino 1 - Motor and servo control board
    AR_1.close()
    AR_1.open()
    time.sleep(5)
    printSerial(AR_1)

except:
    print("Serial communication with Arduino 1 failed")
    AR_1.close()
    # os.system("sudo reboot") # WARNING: Uncomment line only on live systems or it may lead to crashes during testing

# Establish Connection with AR_2
try:
    AR_2 = serial.Serial(port='COM7', baudrate=9600, timeout=5)     # Arduino 2 - Sensor and CMD control board
    AR_2.close()
    AR_2.open()
    time.sleep(5)
    printSerial(AR_2)
except:
    print("Serial communication with Arduino 2 failed")
    AR_1.close()
    AR_2.close()
    # os.system("sudo reboot") # WARNING: Uncomment line only on live systems or it may lead to crashes during testing

while True:
    """Loop constantly running on RPI"""
    if M_auto:
        if not T_data_avail:
            writeSerial(AR_2,4)                                                         # Request data from remote in auto mode
            time.sleep(1)
            x=AR_2.inWaiting()
            if x>0:
                CMDs = readSerial(AR_2).decode('utf-8').split(',')
                print(CMDs)
            if CMDs[0][0:-2] == "MANUAL":                                               # Change mode to automatic
                M_auto = True
                writeSerial(AR_2,"M1")
                print("Mode -> MANUAL")
            elif CMDs[0][0:-2] == "SD":                                                 # Send Sensor data via RF to remote
                writeSerial(AR_2,3)
            elif CMDs[0][0:-2] == "IMG":
                img = Imcap(cam_port=cap1)                                              # Get img from camera
                img = cv.cvtColor(img,cv.COLOR_BGR2GRAY)                                # Convert img to BW
                sendIMGdata(img,AR_2)                                                   # Send Image data via RF to remote                                 # Send Image data via RF to remote
            elif CMDs[0] == "0":
                dist,hdg,pitch,roll = getIMU(AR_2,0.0)                                  # Get current hdg, pitch, roll
                dist_req,c_hdg = getDistfromVec(np.array([[float(CMDs[1])],[float(CMDs[2])]]))
                target_vec = getVecfromDist(dist_req,hdg+c_hdg)                         # Generate Target vector
                tot_vec = np.array([[0],[0]])                                           # Initialize total distance travelled to zero
                print("Target acquired, Target Vector is\n",target_vec)
                T_data_avail = True                                                     # Set status to target data acquired
                t=time.perf_counter()                                                   # Initialize timer
            elif CMDs[0] == "1":
                dist,hdg,pitch,roll= getIMU(AR_2,0.0)                                   # Get current hdg
                dist_req = float(CMDs[1]);hdg += float(CMDs[2])                         # Calculate req dist and heading for target vector
                target_vec = getVecfromDist(dist_req,hdg)                               # Generate Target vector
                tot_vec = np.array([[0],[0]])                                           # Initialize total distance travelled to zero
                print("Target acquired, Target Vector is\n",target_vec)
                T_data_avail = True                                                     # Set status to target data acquired
                t=time.perf_counter()                                                   # Initialize timer
        elif T_data_avail and (dist_req > treshold_dist) :
            dist,hdg,pitch,roll = getIMU(AR_2,time.perf_counter()-t)                    # Get current hdg, pitch, roll;
            t=time.perf_counter()                                                       # Reset timer for period
            current_vec = getVecfromDist(dist,hdg)                                      # Calulate distance travelled in current timestep
            tot_vec = np.add(tot_vec,current_vec)                                       # Calculate total distance travelled
            req_vec = np.subtract(target_vec,tot_vec)                                   # Calculate distance required to be travelled
            dist_req , angle = getDistfromVec(req_vec)                                  # Get values used for code exectuion
            status,DR = MainSteroOut(cap1,cap2)
            if status and DR != 0:                                                      # If object detected turn towards given direction
                st_angle = DR*23
            elif status and DR == 0:                                                    # If direction cannot be found, ask for help (No shame rover, you tried!)
                giveDriveCMD(0,0,0,AR_1)
                writeSerial(AR_2,"Failed to CV resolve")
                writeSerial(AR_2,"Mode Manual, Sending Img")
                imgL = cap1.read()[1]
                sendIMGdata(imgL,AR_2)
                writeSerial(AR_2,"M0")
                break
            else:
                st_angle = angle - hdg                                                  # Convert angle from vector into steering angle
            if pitch > 30 or roll > 30 or pitch < -30 or roll < -30:
                giveDriveCMD(0,0,0,AR_1)                                                # Stop the bot if it is dangerously tipping
            else:
                giveDriveCMD(dist_req,st_angle,6,AR_1)                                  # Send data to CMD Handler if required distance > treshold
        elif T_data_avail and (dist_req <= treshold_dist) :
            giveDriveCMD(0,0,0,AR_1)                                                    # Tell CMD Handler to stop if distance < treshold
            writeSerial(AR_2,3)                                                         # Send Sensor data via RF to remote
            img = Imcap(cam_port=cap1,size=(240,320))                                   # Get img from camera, size used for demonstration purposes
            img = cv.cvtColor(img,cv.COLOR_BGR2GRAY)                                    # Convert img to BW
            sendIMGdata(img,AR_2)                                                       # Send Image data via RF to remote
            T_data_avail = False                                                        # Set status to target data required
        else:
            print("Waiting for target data")
    else:
        x=AR_2.inWaiting()
        if x > 0:
            CMDs = readSerial(AR_2).decode('utf-8').split(',')                          # Handle serial data from Arduino
            if CMDs[0][0:-2] == "AUTO":                                                 # Change mode to automatic
                M_auto = True
                writeSerial(AR_2,"M0")
                print("Mode -> Auto")
            elif CMDs[0][0:-2] == "SD":                                                 # Send Sensor data via RF to remote
                writeSerial(AR_2,3)
            elif CMDs[0][0:-2] == "IMG":
                img = Imcap(cam_port=cap1,size=(240,320))                               # Get img from camera
                img = cv.cvtColor(img,cv.COLOR_BGR2GRAY)                                # Convert img to BW
                sendIMGdata(img,AR_2)                                                   # Send Image data via RF to remote

            else:                                                                       # Write Driving Command as recived from RF
                for x in CMDs:
                    writeSerial(AR_1,x)
                    print(x)
                    time.sleep(0.5)