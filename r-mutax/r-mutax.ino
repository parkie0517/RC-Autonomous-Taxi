//Include Header Files
#include <Servo.h>
#include <SoftwareSerial.h>

// Bluetooth Setting
SoftwareSerial BTSerial(12,13); // BTSerial(Tx, Rx)

// Servo Motor Setting
Servo servo;
int servoPin = 8; // Pin Used by Servo Motor
int initAngle = 90; // 80 is the Initial Angle
int currentAngle;
int Rmax = initAngle+40, Lmin = initAngle-40;

// DC Motor Speed Setting
int defaultSpeed = 255;
int speedLevel = 1; // From 1~10
int Speed = defaultSpeed / speedLevel;

// DC Motor Setting
int motorL[2] = {3, 11};  // Pin Used by DC Motor
int motorR[2] = {5, 6};

String cmd="";

// Initialize R-MUTAX
void setup()
{
  // Initialize Serial Connection
  Serial.begin(9600); // Set Connection Speed
  BTSerial.begin(9600); // Begin Bluetooth Connection
  // Initialize DC Motor Control Pins to OUTPUT
  for(int i = 0; i < 2; i++){
    pinMode(motorL[i], OUTPUT);
    pinMode(motorR[i], OUTPUT);
  }
  // Initialize Servro Motor
  servo.attach(servoPin); // Attach Servo Motor Control Pin
  servo.write(initAngle); // Initialize Servro Motor Angle
}


void loop() {
  //delay(10);
  currentAngle = servo.read();
//  Serial.println(currentAngle);
  
  if(BTSerial.available()) {
    char bt;
    bt = BTSerial.read();
    Serial.println(bt);
    if(bt == 'f') {
      analogWrite(motorL[0], 100);
      analogWrite(motorL[1], 0);
      analogWrite(motorR[0], 100);
      analogWrite(motorR[1], 0);
    }
    else if(bt == 's') {
      analogWrite(motorL[0], 0);
      analogWrite(motorL[1], 0);
      analogWrite(motorR[0], 0);
      analogWrite(motorR[1], 0);
    }
    else if(bt == 'b') {
      analogWrite(motorL[0], 0);
      analogWrite(motorL[1], 100);
      analogWrite(motorR[0], 0);
      analogWrite(motorR[1], 100);
    }
    else if(bt == 'm') {
      
    }
    
    if(bt == 'r') {
      if (currentAngle < Rmax){
        currentAngle+=5;
      }
    }
    else if(bt == 'l') {
      if (currentAngle > Lmin){
         currentAngle-=5;
      }   
    }

  }
  servo.write(currentAngle);
}