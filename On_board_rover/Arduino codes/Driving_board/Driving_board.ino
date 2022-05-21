//============================================================================================================
//Code for Arduino Board 1 / 3 required for Autonoumous Rover using Inertial Navigation and Computer Vission
//This code is responsible for controllong the 3 motor drivers and PWM board required for driving and steering
//Author: Vistasp Edulji, 21-05-2022, V1.3
//============================================================================================================

// Initalize Libraries required
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// Variables storing data87
String CMD; // Command Id to control bot actions
int Speed; // Value for speed control

const int FLF_Pin =  2;   // Forward Left Motor Forward Pin
const int FLR_Pin =  4;   // Forward Left Motor Reverse Pin
const int FRF_Pin = 15;   // Forward Right Motor Forward Pin
const int FRR_Pin = 16;   // Forward Right Motor Reverse Pin
const int MLF_Pin =  5;   // Middle Left Motor Forward Pin
const int MLR_Pin =  6;   // Middle Left Motor Reverse Pin
const int MRF_Pin = 14;   // Middle Right Motor Forward Pin
const int MRR_Pin = 13;   // Middle Right Motor Reverse Pin
const int RLF_Pin =  7;   // Rear Left Motor Forward Pin
const int RLR_Pin =  8;   // Rear Left Motor Reverse Pin
const int RRF_Pin = 11;   // Rear Right Motor Forward Pin
const int RRR_Pin = 12;   // Rear Right Motor Reverse Pin
const int EN_Pin = 3;     // Enable Pin. Common to all IC's for uniform speed control.
const int EN_Pin2 = 9;    // Enable Pin. Common to all IC's for uniform speed control.
const int EN_Pin3 = 10;   // Enable Pin. Common to all IC's for uniform speed control.

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Initialise PWM board variable for servo control
#define SERVOMIN  205 // This is the 'minimum' pulse length
#define SERVOMAX  410 // This is the 'maximum' pulse length

void setup() {
  Serial.begin(9600);
  Serial.println("Arduino 1 Online, Serial Established at 9600 bps");               // Serial Status for connection check
  // Set Pinmode to output for motor controller pins
  pinMode(FLF_Pin, OUTPUT);
  pinMode(FLR_Pin, OUTPUT);
  pinMode(FRF_Pin, OUTPUT);
  pinMode(FRF_Pin, OUTPUT);
  pinMode(MLF_Pin, OUTPUT);
  pinMode(MLR_Pin, OUTPUT);
  pinMode(MRF_Pin, OUTPUT);
  pinMode(MRR_Pin, OUTPUT);
  pinMode(RLF_Pin, OUTPUT);
  pinMode(RLR_Pin, OUTPUT);
  pinMode(RRF_Pin, OUTPUT);
  pinMode(RRR_Pin, OUTPUT);
  CMDHandler("0"); // Set all pins to low
  
  pwm.begin(); // Begin pwm board operations
  Serial.println("PWM check");
  pwm.setOscillatorFrequency(24400000); // Oscillator Frequency has to be adjusted on a per board basis.
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  CMDHandler("30"); // Set all servo angles to zero
  
  Serial.println("Driving board setup complete, swithcing to command based control");
}

void loop() {

  if (Serial.available() > 0) {
    CMD = readSerialString();
    Serial.println("Command recived as: " + String(CMD));                       // Debug Print of String
    CMDHandler(CMD);
  }


}

String readSerialString() {                     // Function that returns SerialData as String

  String output;                                // String to output

  while (Serial.available()) {
    delay(2);                                   // Delay to allow buffer to fill
    char c = char(Serial.read());                     //Reading one byte from serial as char
    output += c;                                //Appending char to string
  }

  return output;
}

void CMDHandler(String CMD) {
  // Command Handler function to convert command string to action
  // Function uses slower if else ladder as switch statement is incompatible with String datatype

  switch ((CMD.substring(0,1)).toInt()) {
    case 1:
      Speed = map(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt()),1,10,85,255); // Map recievd value to PWM compatible range.
      Serial.println(Speed);
      analogWrite(EN_Pin, Speed); analogWrite(EN_Pin2, Speed); analogWrite(EN_Pin3, Speed); // Set Speed on Enable Pin
      switch ((CMD.substring(1,2)).toInt()){
        case 0: // Forward motors only
          CMDHandler("0");
          digitalWrite(FLF_Pin, HIGH);
          digitalWrite(FRF_Pin, HIGH);
          Serial.println("Moving ahead, Forward motors active at speed setting "+ String(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt())));
          break;
        case 1: // Middle motors only
          CMDHandler("0");
          digitalWrite(MLF_Pin, HIGH);
          digitalWrite(MRF_Pin, HIGH);
          Serial.println("Moving ahead, Middle motors active at speed setting "+ String(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt())));
          break;
        case 2: // Rear motors only
          CMDHandler("0");
          digitalWrite(RLF_Pin, HIGH);
          digitalWrite(RRF_Pin, HIGH);
          Serial.println("Moving ahead, Rear motors active at speed setting "+ String(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt())));
          break;
        case 3: // Forward & Middle motors
          CMDHandler("0");
          digitalWrite(FLF_Pin, HIGH);
          digitalWrite(FRF_Pin, HIGH);
          digitalWrite(MLF_Pin, HIGH);
          digitalWrite(MRF_Pin, HIGH);
          Serial.println("Moving ahead, Forward & Middle motors active at speed setting "+ String(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt())));
          break;
        case 4: // Forward & Rear motors
          CMDHandler("0");
          digitalWrite(FLF_Pin, HIGH);
          digitalWrite(FRF_Pin, HIGH);
          digitalWrite(RLF_Pin, HIGH);
          digitalWrite(RRF_Pin, HIGH);
          Serial.println("Moving ahead, Forward & Rear motors active at speed setting "+ String(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt())));
          break;
        case 5: // Middle & Rear motors
          CMDHandler("0");
          digitalWrite(MLF_Pin, HIGH);
          digitalWrite(MRF_Pin, HIGH);
          digitalWrite(RLF_Pin, HIGH);
          digitalWrite(RRF_Pin, HIGH);
          Serial.println("Moving ahead, Middle & Rear motors active at speed setting "+ String(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt())));
          break;
        case 6: // All motors
          CMDHandler("0");
          digitalWrite(FLF_Pin, HIGH);
          digitalWrite(FRF_Pin, HIGH);
          digitalWrite(MLF_Pin, HIGH);
          digitalWrite(MRF_Pin, HIGH);
          digitalWrite(RLF_Pin, HIGH);
          digitalWrite(RRF_Pin, HIGH);
          Serial.println("Moving ahead, All motors active at speed setting "+ String(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt())));
          break;
        default:
          Serial.println("Unown CMD sent, possible alteration/corruption of program memory");
          break;
      }
      break;
    case 2:
      Speed = map(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt()),1,10,85,255); // Map recievd value to PWM compatible range.
      Serial.println(Speed);
      analogWrite(EN_Pin, Speed); analogWrite(EN_Pin2, Speed); analogWrite(EN_Pin3, Speed); // Set Speed on Enable Pin
      switch ((CMD.substring(1,2)).toInt()){
        case 0: // Forward motors only
          CMDHandler("0");
          digitalWrite(FLR_Pin, HIGH);
          digitalWrite(FRR_Pin, HIGH);
          Serial.println("Moving reverse, Forward motors active at speed setting "+ String(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt())));
          break;
        case 1: // Middle motors only
          CMDHandler("0");
          digitalWrite(MLR_Pin, HIGH);
          digitalWrite(MRR_Pin, HIGH);
          Serial.println("Moving reverse, Middle motors active at speed setting "+ String(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt())));
          break;
        case 2: // Rear motors only
          CMDHandler("0");
          digitalWrite(RLR_Pin, HIGH);
          digitalWrite(RRR_Pin, HIGH);
          Serial.println("Moving reverse, Rear motors active at speed setting "+ String(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt())));
          break;
        case 3: // Forward & Middle motors
          CMDHandler("0");
          digitalWrite(FLR_Pin, HIGH);
          digitalWrite(FRR_Pin, HIGH);
          digitalWrite(MLR_Pin, HIGH);
          digitalWrite(MRR_Pin, HIGH);
          Serial.println("Moving reverse, Forward & Middle motors active at speed setting "+ String(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt())));
          break;
        case 4: // Forward & Rear motors
          CMDHandler("0");
          digitalWrite(FLR_Pin, HIGH);
          digitalWrite(FRR_Pin, HIGH);
          digitalWrite(RLR_Pin, HIGH);
          digitalWrite(RRR_Pin, HIGH);
          Serial.println("Moving reverse, Forward & Rear motors active at speed setting "+ String(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt())));
          break;
        case 5: // Middle & Rear motors
          CMDHandler("0");
          digitalWrite(MLR_Pin, HIGH);
          digitalWrite(MRR_Pin, HIGH);
          digitalWrite(RLR_Pin, HIGH);
          digitalWrite(RRR_Pin, HIGH);
          Serial.println("Moving reverse, Middle & Rear motors active at speed setting "+ String(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt())));
          break;
        case 6: // All motors
          CMDHandler("0");
          digitalWrite(FLR_Pin, HIGH);
          digitalWrite(FRR_Pin, HIGH);
          digitalWrite(MLR_Pin, HIGH);
          digitalWrite(MRR_Pin, HIGH);
          digitalWrite(RLR_Pin, HIGH);
          digitalWrite(RRR_Pin, HIGH);
          Serial.println("Moving reverse, All motors active at speed setting "+ String(CMD.substring(2,CMD.length()).toInt()>10?10:(CMD.substring(2,CMD.length()).toInt()<1?1:CMD.substring(2,CMD.length()).toInt())));
          break;
        default:
          Serial.println("Unown CMD sent, possible alteration/corruption of program memory");
          break;
      }
      break;
    case 3:
      sv_steer(CMD.substring(1,CMD.length()));
      break;
    case 4:
      CMDHandler("0");
      sv_steer("C");
      digitalWrite(FLR_Pin, HIGH);
      digitalWrite(FRF_Pin, HIGH);
      digitalWrite(MLR_Pin, HIGH);
      digitalWrite(MRF_Pin, HIGH);
      digitalWrite(RLR_Pin, HIGH);
      digitalWrite(RRF_Pin, HIGH);
      Serial.println("Crabbing left");
      break;
    case 5:
      CMDHandler("0");
      sv_steer("C");
      digitalWrite(FLF_Pin, HIGH);
      digitalWrite(FRR_Pin, HIGH);
      digitalWrite(MLF_Pin, HIGH);
      digitalWrite(MRR_Pin, HIGH);
      digitalWrite(RLF_Pin, HIGH);
      digitalWrite(RRR_Pin, HIGH);
      Serial.println("Crabbing Right");
      break;
    case 0:
      digitalWrite(FLF_Pin, LOW);
      digitalWrite(FLR_Pin, LOW);
      digitalWrite(FRF_Pin, LOW);
      digitalWrite(FRR_Pin, LOW);
      digitalWrite(MLF_Pin, LOW);
      digitalWrite(MLR_Pin, LOW);
      digitalWrite(MRF_Pin, LOW);
      digitalWrite(MRR_Pin, LOW);
      digitalWrite(RLF_Pin, LOW);
      digitalWrite(RLR_Pin, LOW);
      digitalWrite(RRF_Pin, LOW);
      digitalWrite(RRR_Pin, LOW);
      Serial.println("Complete Stop");
      break;
    default:
      Serial.println("Unown CMD sent, possible alteration/corruption of program memory");
      break;
  }
}

void sv_steer(String input ){
  if (input=="C"){
    int s_angle;  
    s_angle = 23;
    pwm.setPWM(0,0,map(90-15-( 0.0214*s_angle*s_angle+1.2115*s_angle),0,180,SERVOMIN,SERVOMAX)); //Servo FL
    pwm.setPWM(3,0,map(90+( 0.0214*s_angle*s_angle+1.2115*s_angle),0,180,SERVOMIN,SERVOMAX)); //Servo BL
    s_angle = -23;
    pwm.setPWM(13,0,map(90+1-(-0.0214*s_angle*s_angle+1.2115*s_angle),0,180,SERVOMIN,SERVOMAX)); //Servo FR
    pwm.setPWM(15,0,map(90-15+(-0.0214*s_angle*s_angle+1.2115*s_angle),0,180,SERVOMIN,SERVOMAX)); //Servo BR
    return;
  }

  int s_angle = input.toInt();  // Convert String to Int
  s_angle = s_angle > 23 ? 23 : s_angle < -23? -23 : s_angle; // Ensure steer angle is withing function bounds
  Serial.println("Steering by angle "+String(s_angle));

  pwm.setPWM(13,0,map(90+1-(-0.0214*s_angle*s_angle+1.2115*s_angle),0,180,SERVOMIN,SERVOMAX)); //Servo FR
  pwm.setPWM(0,0,map(90-15-( 0.0214*s_angle*s_angle+1.2115*s_angle),0,180,SERVOMIN,SERVOMAX)); //Servo FL
  pwm.setPWM(15,0,map(90-15+(-0.0214*s_angle*s_angle+1.2115*s_angle),0,180,SERVOMIN,SERVOMAX)); //Servo BR
  pwm.setPWM(3,0,map(90+( 0.0214*s_angle*s_angle+1.2115*s_angle),0,180,SERVOMIN,SERVOMAX)); //Servo BL
  
}
