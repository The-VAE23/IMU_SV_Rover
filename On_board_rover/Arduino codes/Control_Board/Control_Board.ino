//============================================================================================================
//Code for Arduino Board 2 / 3 required for Autonoumous Rover using Inertial Navigation and Computer Vission
//This code is responsible for processing commands and relaying essential data between remote and RPi
//Author: Vistasp Edulji, 11-05-2022, V1.3
//============================================================================================================

// Required Libraries 
#include<Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <MechaQMC5883.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <dht.h>
#define DHT11_PIN 5
#include <SoftwareSerial.h>
#include <TinyGPS.h>

// Variables for board
bool M_auto = false;                    // Boolean to check running mode
String CMD = "0";                       // String value to control board parameters
int x,y,z,stat=0;                       // Integers for remote control and compass readings in auto mode
RF24 radio(7,8);                        // CE, CSN Pins
SoftwareSerial gpsSerial(3,4);          //rx,tx
const byte reciv_address[6] = "10010";  //Setup Radio to recive
const byte trans_address[6] = "10101";  //Setup Radio to send commands
MPU6050 mpu;                            // Create IMU object (Accel+Gyro)
MechaQMC5883 qmc;                       // Create qmc object (compass)
dht DHT;                                // Create DHT object (Temp+Humidity)
TinyGPS gps;                            // Create GPS object 
String sensdata;                        // String to send via RF containing sensor data
bool control=false;                     // Extra boolean to use during board operations

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void setup() {
  //Serial and Wire Setup
  Serial.begin(9600);
  Serial.println("Arduino 2 Online. Serial established at 9600 bps");
  gpsSerial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  //RF Setup
  radio.begin();
  radio.openReadingPipe(1, reciv_address);
  radio.openWritingPipe(trans_address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("RF Setup Complete");

  //MPU6050 Setup
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // Accelerometer and Gyroscope offsets calibrated using IMU_Zero.ino
  mpu.setXGyroOffset(226);
  mpu.setYGyroOffset(-36);
  mpu.setZGyroOffset(116);
  mpu.setXAccelOffset(-2790); 
  mpu.setYAccelOffset(1477); 
  mpu.setZAccelOffset(1085); 

  if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);


        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  Serial.println("AccelGyro Setup Complete");

  // HMC Setup
  qmc.init();
  Serial.println("Compass Setup Complete");

  Serial.println("Control Board Initialized. Handover to RPI. AUTO MODE ="+String(M_auto));  
}

void loop() {

  if (Serial.available() > 0 && stat <1) {
    CMD = readSerialString();
    CMDHandler(CMD);
  }
  
  if (M_auto){
    switch (stat){
      case 0:
        break;
      case 1:
          GetDOF6();
          Serial.print(float(aaReal.x)/8192*9.81);
          Serial.print(',');
//          Serial.print(float(aaReal.y)/8192*9.81);
//          Serial.print(',');
//          Serial.print(float(aaReal.z)/8192*9.81);
//          Serial.print(',');
          Serial.print(int(ypr[0]*180/PI<0?ypr[0]*180/PI+360:ypr[0]*180/PI));
          Serial.print(',');
          Serial.print(ypr[1]*180/PI);
          Serial.print(',');
          Serial.println(ypr[2]*180/PI);
          stat=0;
          break;
        case 2:
          if (!control){
            sendRF("IMG");
            control = true;
            Serial.println(1);
          }
          else {
            if (Serial.available()){
              CMD=readSerialString();
              if (CMD !="e"){
                sendRF(CMD);
                Serial.println(1);
              }
              else {
                stat=0;
                sendRF("e");
              }
            }          
          }
          break;
        case 3:
          sensdata="";
          DHT.read11(DHT11_PIN);
          sensdata += String(DHT.temperature)+','+String(DHT.humidity)+',';
          float flat,flon;
          while(gpsSerial.available()){
            if(gps.encode(gpsSerial.read())){ 
              gps.f_get_position(&flat,&flon);
              break;
            }
          }
          flat = flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat;
          flon = flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon;
          sensdata += String(flat)+','+String(flon)+',';
          GetDOF6();
          sensdata += String(int(ypr[0]*180/PI<0?ypr[0]*180/PI+360:ypr[0]*180/PI));
          sendRF("SD");
          delay(1000); // Small delay to ensure data doesn't end up on same line
          sendRF(sensdata);
          stat=0;
          break;
        case 4:
          if (radio.available()>0){
            getRF();
            stat=0;
          }
          break;
        default:
          stat=0;
    }
    
  }
  else {
    if (radio.available()) {
      getRF();
    }
    switch (stat){
      case 0:
        break;
      case 1:
          GetDOF6();
          Serial.print(float(aaReal.x)/8192*9.81);
          Serial.print(',');
//          Serial.print(float(aaReal.y)/8192*9.81);
//          Serial.print(',');
//          Serial.print(float(aaReal.z)/8192*9.81);
//          Serial.print(',');
          Serial.print(int(ypr[0]*180/PI<0?ypr[0]*180/PI+360:ypr[0]*180/PI));
          Serial.print(',');
          Serial.print(ypr[1]*180/PI);
          Serial.print(',');
          Serial.println(ypr[2]*180/PI);
          stat=0;
          break;
        case 2:
          if (!control){
            sendRF("IMG");
            control = true;
            Serial.println(1);
          }
          else {
            if (Serial.available()){
              CMD=readSerialString();
              if (CMD !="e"){
                sendRF(CMD);
                Serial.println(1);
              }
              else {
                stat=0;
                sendRF("e");
              }
            }          
          }
          break;
        case 3:
        sensdata="";
          DHT.read11(DHT11_PIN);
          sensdata += String(int(DHT.temperature))+','+String(int(DHT.humidity))+',';
          float flat,flon;
          while(gpsSerial.available()){
            if(gps.encode(gpsSerial.read())){ 
              gps.f_get_position(&flat,&flon);
              break;
            }
          }
          flat = flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat;
          flon = flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon;
          sensdata += String(flat)+','+String(flon)+',';
          GetDOF6();
          sensdata += String(int(ypr[0]*180/PI<0?ypr[0]*180/PI+360:ypr[0]*180/PI));
          sendRF("SD");
          delay(1000); // Small delay to ensure data doesn't end up on same line
          sendRF(sensdata);
          stat=0;
          break;
        default:
          stat=0;
    }
  }

}

String readSerialString() {                        // Function that returns SerialData as String

  String output;                                // String to output

  while (Serial.available()) {
    delay(2);                                   // Delay to allow buffer to fill
    char c = char(Serial.read());               // Reading one byte from serial as char
    output += c;                                // Appending char to string
  }
  return output;
}

void GetDOF6(){
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

  qmc.read(&x,&y,&z);
  float cx = float(x)*cos(ypr[1]) + float(z)*sin(ypr[1]);
  float cy = float(x)*sin(ypr[2])*sin(ypr[1])+float(y)*cos(ypr[2]) - float(z)*sin(ypr[2])*cos(ypr[1]);
  ypr[0] = atan2(cy,cx);
}

void CMDHandler(String CMD){

  if (CMD == "M0"){
    M_auto = true;
//    Serial.println("Mode - Automatic");
  }
  else if (CMD == "M1") {
    M_auto = false;
//    Serial.println("Mode - Manual");
  }
  else{
    stat = CMD.toInt();
  }
}

void getRF(){
  char recvData[32] = "";
  radio.read(&recvData, sizeof(recvData));
  Serial.println(recvData);
}

void sendRF(String str){
  radio.stopListening();
  char buf[32];
  str.toCharArray(buf,32);
  radio.write(&buf, sizeof(buf));
//  Serial.println("RF sent as "+String(buf));
  radio.startListening();
}
