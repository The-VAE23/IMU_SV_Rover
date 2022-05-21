//============================================================================================================
//Code for Arduino Board 3 / 3 required for Autonoumous Rover using Inertial Navigation and Computer Vission
//This code is responsible for ensuring remote communication with the rover using a remote.
//Author: Vistasp Edulji, 10-05-2022 V1.1
//============================================================================================================

// Include Libraries for Remote
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Initialize RF
RF24 radio(7,8); // CE, CSN Pins
const byte trans_address[6] = "10010";
const byte reciv_address[6] = "10101";

//Initialize variables
int x[20],y[20],i=6,j,xmean,ymean,xs,ys;
String sendData;
String lastData="";
bool M_auto=false;
bool tap =false;

//Setup Transmitter
void setup() {
  Serial.begin(9600);
  Serial.println("Remote Control for bot. Default Mode - "+String(M_auto));
  radio.begin();
  radio.openWritingPipe(trans_address);
  radio.openReadingPipe(1,reciv_address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("Calibrating mid point of remote");
  for(j=0;j<20;j++){
    x[j]=analogRead(A0);
    y[j]=analogRead(A1);
  }
  xmean = mean(x);
  ymean = mean(y);
  Serial.println("Remote Ready for use");
}

void loop() {

  // Always check if there is cmd to be sent to in serial buffer and send on priority
  if(Serial.available())
  {
    sendData = readSerialString();
    if (M_auto == false && sendData == "AUTO"){
      M_auto = true;
      sendRF(sendData);
    }
    else if (M_auto == true && sendData == "MANUAL"){
      M_auto = false;
      sendRF(sendData);
      lastData = "";
    }
    else {
      sendRF(sendData);
    }
  }

  // Check execution mode and run accordingly
  if (M_auto){
    if (radio.available()) {
      char recvData[32] = "";
      radio.read(&recvData, sizeof(recvData));
      Serial.println(recvData);
    }
  }
  else {
    if (radio.available()) {
      char recvData[32] = "";
      radio.read(&recvData, sizeof(recvData));
      Serial.println(recvData);
    }
    else {
      for(j=0;j<20;j++){
        x[j]=analogRead(A0);
        y[j]=analogRead(A1);
      }
      xs=mean(x);
      xs = (xs>=xmean-5 && xs<=xmean+5)?0:map(xs,50,1022,-26,26);
      ys = mean(y);
      ys = (ys>=ymean-5 && ys<=ymean+5)?0:map(ys,0,1023,-10,10);
  
      if (!tap && analogRead(A2) <= 18){
        i++;
        i = i>6?0:i;
        tap = true;
      }
      else if (tap && analogRead(A2) > 18){
        tap = false;
      }
      
      sendData =("3"+String(xs));
      sendData += ","; 
      
      if (ys>0){  
        sendData +=("1"+String(i)+String(ys));
      }
      else if (ys == 0){
        sendData +=("0");
      }
      else {
        sendData +=("2"+String(i)+String(abs(ys)));
      }
    
      if(lastData != sendData) {
        sendRF(sendData);
        lastData = sendData;
      }
      delay(1000);
    }
  }

}

String readSerialString() {                           // Function that returns SerialData as String

  String output;                                      // String to output

  while (Serial.available()) {
    delay(2);                                         // Delay to allow buffer to fill
    char c = char(Serial.read());                     //Reading one byte from serial as char
    output += c;                                      //Appending char to string
  }

  return output;
}

void sendRF(String str){
  radio.stopListening();
  char buf[32];
  str.toCharArray(buf,32);
  radio.write(&buf, sizeof(buf));
  Serial.println("RF sent as "+String(buf));
  radio.startListening();
}

int mean(int x[20]){
  int i,sum=0;
  for(i=0;i<20;i++){
    sum += x[i];
  }
  return int(float(sum)/20);
}
