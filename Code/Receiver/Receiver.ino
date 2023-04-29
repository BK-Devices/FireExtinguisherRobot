// Fire Extinguisher Robot Receiver
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <NewPing.h>
#include <Arduino.h>
#include <nRF24L01.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Battery Monitoring pin
#define BAT A0      // Battery Pin

// Servo Motor Pins
#define SR1 A1      // Water Ejector Servo
#define SR2 A2      // Camera Servo

// Fire Sensor Pins
#define F1 A3       // Fire Sensor 1
#define F2 A4       // Fire Sensor 2
#define F3 A5       // Fire Sensor 3

// // Smoke Sensor Pins
// #define SM_A A6     // Smoke Sensor Analog Pin
// #define SM_D A7     // Smoke Sensor Digital Pin

// Ultrasonic Sensor Pins
#define TRIG 23     // Trig Pin
#define ECHO 22     // Echo Pin
#define MAX_DIST 200

// Water Level Detector Pins
#define T1 21       // 100 %
#define T2 20       // 80 %
#define T3 19       // 60 %
#define T4 18       // 40 %
#define T5 17       // 20 %

// // ESP-32 Reset Pin
// #define RST 17      // ESP-32 Reset Pin

// Pump & Buzzer Pins
#define BUZ 16      // Buzzer
#define PMP 15      // Water Pump

// Motor driver Pins
#define ENA 3       // Enable A
#define IN1 10      // Input 1
#define IN2 11      // Input 2
#define IN3 14      // Input 3
#define IN4 13      // Input 4
#define ENB 12      // Enable B

// GPS Module Pins
#define RX 1        // GPS Receiver
#define TX 0        // GPS Transmitter


//Declare two servos
Servo Ser1;
Servo Ser2;

TinyGPSPlus gps;
RF24 radio(4, 2); // nRF24L01 (CE, CSN) // Declaring RF Module
SoftwareSerial GPS(RX, TX);             // Declaring GPS Module
NewPing sonar(TRIG, ECHO, MAX_DIST); 

// Variables
int Dist = 0;
int LDist = 0;
int RDist = 0;
int Fr1 = 450;
int Fr2 = 450;
int Fr3 = 450;
bool pump = false;
unsigned long start = 0;
unsigned long servo1 = 0;
unsigned long servo2 = 0;
unsigned long currentTime = 0;
unsigned long lastReceiveTime = 0;
const byte address[][6] = {"00001", "00002"};

// Receiving Structure
struct Data_Package_1
{
  byte j1X;
  byte j1Y;
  byte j2X;
  byte j2Y;
  byte TS1;
  byte TS2;
};
Data_Package_1 data1;

// Sending Structure
struct Data_Package_2
{
  String lat;
  String lon;
  byte fire;
  byte bat;
  byte tank;
};
Data_Package_2 data2;

// Reset Data
void resetData() 
{
  data1.j1X = 127;
  data1.j1Y = 131;
  data1.j2X = 135;
  data1.j2Y = 129;
  data1.TS1 = 1;
  data1.TS2 = 1;
}

void setup() 
{
  Serial.begin(9600);                      // Serial Begin

  // Calibratig Sensors
  Serial.println("Calibrating sensors");
  delay(1000);
  for(int i = 0 ; i < 10; i++){
    Serial.print(".");
    delay(1000);
  }

  GPS.begin(9600);                         // GPS Begin
  radio.begin();                           // Redio Begin
  radio.openWritingPipe(address[1]);       // Sending Address 00002
  radio.openReadingPipe(1, address[0]);    // Receiving Address 00001
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  // radio.startListening(); //  Set the module as receiver
  resetData();

  // Setting Other Pins as INP/OUT
  pinMode(T1, INPUT_PULLUP);
  pinMode(T2, INPUT_PULLUP);
  pinMode(T3, INPUT_PULLUP);
  pinMode(T4, INPUT_PULLUP);
  pinMode(T5, INPUT_PULLUP);
  pinMode(BUZ, OUTPUT);
  pinMode(PMP, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  
  // Setting Motor Driver 1 Pins as INP/OUT
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT); 

  // Attaching Servo
  Ser1.attach(SR1);  // Servo motor up-down movement for watar ejector
  Ser2.attach(SR2);  // Servo motor right-left movement for camera
  Ser1.write(135);   // Set position at centre 90 to 180
  Ser2.write(90);    // Set position at centre 0 to 180

  // Assigning the values
  data2.lat = "";
  data2.lon = "";
  data2.fire = 0;
  data2.bat = 0;
  data2.tank = 0;

  digitalWrite(BUZ, LOW);
  digitalWrite(PMP, LOW);
  digitalWrite(T1, HIGH);
  digitalWrite(T2, HIGH);
  digitalWrite(T3, HIGH);
  digitalWrite(T4, HIGH);
  digitalWrite(T5, HIGH);
}

void loop() 
{
  // Receiving data
  delay(5);
  radio.startListening();
  if(radio.available()){
    radio.read(&data1, sizeof(Data_Package_1));
    lastReceiveTime = millis();
  }
  currentTime = millis();
  if(currentTime - lastReceiveTime > 1000) resetData();

  // Controlling Robot
  if(data1.TS2 == 0) autoMode();
  else manualMode();
  
  // Sending Data
  delay(5);
  readGPS();
  readTank();
  readBattery();
  readFireSensors();
  radio.stopListening();
  radio.write(&data2, sizeof(Data_Package_2));
  delay(5);
}

// reading Battery Level
void readBattery(){
  float R1 = 2200.00;         // resistance of R1 (2.2K) in Ohm
  float R2 = 1000.00;         // resistance of R2 (1K) in Ohm
  float MaxVolt = 3.85;       // Maximum Voltage measured after resistor
  int MaxVal = 772;           // Maximum Analog values read by analog pin
  
  int val = analogRead(BAT);               // Reads the analog input
  float Vout = (val * MaxVolt) / MaxVal;   // formula for calculating voltage out 
  float Vin = Vout * (R2 + R1) / R2;       // formula for calculating voltage in   

  int Bat = map(Vin, 11.1, 12.8, 0, 100);
  if(Bat >= 100) Bat = 100;
  if(Bat <= 0) Bat = 0;
  // Serial.println(String("Bat = " + String(Bat) + "      Val = " + String(val) + "      Vin = " + String(Vin) + "      Vout = " + String(Vout)));

  data2.bat = Bat;
}

// Reading GPS Location  
void readGPS(){
  while(GPS.available() > 0){
    if(gps.encode(GPS.read())){
      if(gps.location.isValid()){
        data2.lat = gps.location.lat();
        data2.lon = gps.location.lng();
      }
      else{
        data2.lat = "20.888420";
        data2.lon = "77.731250";
      }
    }
  }
  data2.lat = "20.888420";
  data2.lon = "77.731250";
}

// Reding Tank Water Level
void readTank(){
  int tank = 0;
  if(digitalRead(T1) == 0) tank = 100;
  if(digitalRead(T2) == 0) tank = 80;
  if(digitalRead(T3) == 0) tank = 60;
  if(digitalRead(T4) == 0) tank = 40;
  if(digitalRead(T5) == 0) tank = 20;
  else tank = 0;

  data2.tank = tank;
}

// Reading Fire Sensors
void readFireSensors(){
  Fr1 = analogRead(F1);
  Fr2 = analogRead(F2);
  Fr3 = analogRead(F3);
  if(Fr1 <= 100 || Fr2 <= 100 || Fr3 <= 100) data2.fire = 1;
  else data2.fire = 0;
}

// Calculating Centre Distance
void centreDist(){
  delay(10);
  Ser2.write(90);
  delay(300);
  Dist = sonar.convert_cm(sonar.ping_median(10));
  delay(50);
}

// Calculating Left Distance
void leftDist(){
  delay(10);
  Ser2.write(160);
  delay(300);
  LDist = sonar.convert_cm(sonar.ping_median(10));
  delay(50);
  Ser2.write(90);
  delay(300);
}

// Calculating Right Distance
void rightDist(){
  delay(10);
  Ser2.write(0);
  delay(300);
  RDist = sonar.convert_cm(sonar.ping_median(10));
  delay(50);
  Ser2.write(90);
  delay(300);
}

// Automatic Mode
void manualMode(){
  setCamera(data1.j2X);
  setWaterEjector(data1.j2Y);
  controlRobot(data1.j1X, data1.j1Y);
  if(data1.TS1 == 0){
    start = millis();
    pump = true;
  }
  if(millis() - start >= 3000 && data1.TS1 == 1) pump = false;
  if(pump == true) digitalWrite(PMP, HIGH);
  else digitalWrite(PMP, LOW);
}

// Automatic Mode
void autoMode(){
  readFireSensors();
  if(pump == true){
    stop();
    digitalWrite(PMP, HIGH);
    int ang = Ser1.read();
    if(ang <= 130) Ser1.write(170);
    else if(ang > 130) Ser1.write(90);
    delay(300);
  }
  else digitalWrite(PMP, LOW);
  if(Fr2 <= 100){
    stop();
    if(pump == false){
      pump = true;
      start = millis();
    }
  }
  else if(Fr3 <= 100){
    if((millis() - start) >= 5000) pump = false;
    rightDist();
    if(RDist >= 30) right();
    else detectObject();
  }
  else if(Fr1 <= 100){
    if((millis() - start) >= 5000) pump = false;
    leftDist();
    if(LDist >= 30) left();
    else detectObject();
  }
  else{
    if((millis() - start) >= 5000) pump = false;
    detectObject();
  }
}

void detectObject(){
  centreDist();
  if(Dist >= 45) forward();
  else{
    stop();
    leftDist();
    delay(10);
    rightDist();
    delay(10);

    if(RDist > LDist && RDist >= 30) right();
    else if(LDist > RDist && LDist >= 30) left();
    else backward();
  }
}

void stop(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forward(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

void backward(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

void right(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

// Turning Left
void left(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

// Controlling Water Ejector servo Motor
void setWaterEjector(int val){
  if(millis() - servo1 > 40){
    if(0 <= val && val <= 112){
      servo1 = millis();
      int ang = Ser1.read() + map(val, 120, 0, 0, 3);
      if(ang >= 180) ang = 180;
      Ser1.write(ang);
    }
    else if(143 <= val && val <= 255){
      servo1 = millis();
      int ang = Ser1.read() - map(val, 150, 255, 0, 3);
      if(ang <= 90) ang = 90;
      Ser1.write(ang);
    }
  }
}

// Controlling camera servo Motor
void setCamera(int val){
  if(millis() - servo2 > 40){
    if(0 <= val && val <= 112){
      servo2 = millis();
      int ang = Ser2.read() + map(val, 114, 0, 0, 3);
      if(ang >= 180) ang = 180;
      Ser2.write(ang);
    }
    else if(143 <= val && val <= 255){
      servo2 = millis();
      int ang = Ser2.read() - map(val, 144, 255, 0, 3);
      if(ang <= 0) ang = 0;
      Ser2.write(ang);
    }
  }
}

// Controlling Robot Movement
void controlRobot(int x, int y){
  int MS_A, MS_B;
  // Forward Moving
  if(146 <= y){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    MS_A = map(y, 142, 255, 0, 255);
    MS_B = map(y, 142, 255, 0, 255);
  }

  // Backward Moving
  else if(y <= 116){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    MS_A = map(y, 112, 0, 0, 255);
    MS_B = map(y, 112, 0, 0, 255);
  }
  
  else{
    MS_A = 0;
    MS_B = 0;
  }

  // Right Moving
  if(142 <= x){
    int val = map(x, 142, 200, 0, 255);
    MS_A = MS_A - val;
    MS_B = MS_B + val * 2;

    if(MS_A < 0) MS_A = 0;
    if(MS_B < 0) MS_B = 0;
    if(MS_A > 255) MS_A = 255;
    if(MS_B > 255) MS_B = 255;
  }

  // Left Moving
  else if(x <= 112){
    int val = map(x, 112, 0, 0, 255);
    MS_A = MS_A + val * 2;
    MS_B = MS_B - val;

    if(MS_A < 0) MS_A = 0;
    if(MS_B < 0) MS_B = 0;
    if(MS_A > 255) MS_A = 255;
    if(MS_B > 255) MS_B = 255;
  }

  if(MS_A < 30) MS_A = 0;
  if(MS_B < 30) MS_B = 0;

  analogWrite(ENA, MS_A);
  analogWrite(ENB, MS_B);
}

