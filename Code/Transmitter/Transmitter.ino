// Fire Extinguisher Robot Transmitter
// Header files
#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <nRF24L01.h>
#include <LiquidCrystal.h>

// Pins for input of Toggle & Push Switches
#define ts1 3
#define ts2 A4
#define ps1 2
#define ps2 A1

// Pin of Buzzer
#define buz A5

// Variables
const byte address[][6] = {"00001", "00002"};
RF24 radio(A0, 10); // nRF24L01 (CE, CSN)
LiquidCrystal lcd(4, 5, 6, 7, 8, 9); //(rs, en, d4, d5, d6, d7)

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

int Mode = 1;
int Sw1, Sw2;
long Time = 0;
int Lst1 = HIGH;
int Lst2 = HIGH;
bool lost = false;
String Lat = "";
String Lon = "";
const int Stime = 250;
const int Ltime = 750;
unsigned long Start, Stop;

// Structure data Packet for sending
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

// Structure data Packet for Receiving
struct Data_Package_2
{
  String lat;
  String lon;
  byte fire;
  byte bat;
  byte tank;
};
Data_Package_2 data2;

// Function for Saving current location on EEPROM
void E_Write(int Address, const String &str)
{
  // Writing EEPROM  
  byte Len = str.length();
  EEPROM.write(Address, Len);
  for(int i = 0; i < Len; i++) EEPROM.write(Address + 1 + i, str[i]);
}

// Function for getting location from EEPROM
String E_Read(int Address)
{
  // Reading EEPROM
  int Len = EEPROM.read(Address);
  char data[Len + 1];
  for(int i = 0; i < Len; i++) data[i] = EEPROM.read(Address + 1 + i);
  data[Len] = '\0';
  return String(data);
}

// Reset Data
void resetData() 
{
  data2.lat = "";
  data2.lon = "";
  data2.fire = 0;
  data2.bat = 0;
  data2.tank = 0;

  Mode = 0;
  lost = true;

  // String Data = E_Read(0);
  // Lat = Data.substring(0, Data.indexOf("&"));
  // Lon = Data.substring(Data.indexOf("&"), Data.length());

  lcd.setCursor(0, 0);
  lcd.print(String("Lat = " + Lat + "          "));
  lcd.setCursor(0, 1);
  lcd.print(String("Lon = " + Lon + "          "));
}

void setup() 
{
  Serial.begin(9600);                               // Serial Begin
  lcd.begin(16,2);                                  // LCD Begin
  
  // Display Project Name on LCD Display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" Fire Fighting ");
  lcd.setCursor(0, 1);
  lcd.print("     Robot     ");   
  delay(2500);

  // Display Makers and Guide Name on LCD Display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("    Made By    ");
  lcd.setCursor(0, 1);
  lcd.print("Bhavesh G. Kale ");   
  delay(2500);
  lcd.setCursor(0, 1);
  lcd.print("Sarthak Tabhane ");   
  delay(2500);
  lcd.setCursor(0, 1);
  lcd.print("Vaishnavi V. D. ");   
  delay(2500);

  lcd.setCursor(0, 0);
  lcd.print("   Guided By   ");
  lcd.setCursor(0, 1);
  lcd.print("Prof. K. Belsare");   
  delay(2500);

  // Redio Begin
  radio.begin();
  radio.openWritingPipe(address[0]); // 00001
  radio.openReadingPipe(1, address[1]); // 00002
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  resetData();

  // Setting Pins as input or output
  pinMode(ts1, INPUT_PULLUP);
  pinMode(ts2, INPUT_PULLUP);
  pinMode(ps1, INPUT_PULLUP);
  pinMode(ps2, INPUT_PULLUP);
  pinMode(buz, OUTPUT);

  // Assigning the values
  data1.j1X = 127;
  data1.j1Y = 131;
  data1.j2X = 135;
  data1.j2Y = 122;
  data1.TS1 = 1;
  data1.TS2 = 1;
  resetData();

  digitalWrite(buz, LOW);
  digitalWrite(ts1, HIGH);
  digitalWrite(ts2, HIGH);
  digitalWrite(ps1, HIGH);
  digitalWrite(ps2, HIGH);
}

void loop() 
{
 delay(5);
  radio.stopListening();

  // Reading all Switches 7 Joysticks  
  data1.j1X = map(analogRead(A3), 1023, 0, 0, 255);
  data1.j1Y = map(analogRead(A2), 0, 1023, 0, 255);
  data1.j2X = map(analogRead(A7), 1023, 0, 0, 255);
  data1.j2Y = map(analogRead(A6), 0, 1023, 0, 255);
  data1.TS1 = digitalRead(ts1);
  data1.TS2 = digitalRead(ts2);
  int p1 = digitalRead(ps1);
  int p2 = digitalRead(ps2);

  // // Print data on serial monitor
  // Serial.print(String("J1X = " + String(data1.j1X)));
  // Serial.print(String("   J1Y = " + String(data1.j1Y)));
  // Serial.print(String("   J2X = " + String(data1.j2X)));
  // Serial.print(String("   J2Y = " + String(data1.j2Y)));
  // Serial.print(String("   TS1 = " + String(data1.TS1)));
  // Serial.print(String("   TS2 = " + String(data1.TS2)));
  // Serial.print(String("   PS1 = " + String(p1)));
  // Serial.println(String("   PS2 = " + String(p2)));
  
  radio.write(&data1, sizeof(Data_Package_1));

  delay(5);
  radio.startListening();

  if(radio.available())
  {
    if(lost == true){
      Mode = 1;
      lost = false;
    }
    radio.read(&data2, sizeof(Data_Package_2));
    lastReceiveTime = millis();
  }
  currentTime = millis();
  if(currentTime - lastReceiveTime > 5000) resetData();
    
  // Print data on serial monitor
  // Serial.print(String("Lat = " + String(data2.lat)));
  // Serial.print(String("   Lon = " + String(data2.lon)));
  // Serial.print(String("   Fire = " + String(data2.fire)));
  // Serial.print(String("   Bat = " + String(data2.bat)));
  // Serial.println(String("   Tank = " + String(data2.tank)));
  displayData();
}

void displayData(){
  // Read Switch 1
  Sw1 = digitalRead(ps1);
  if(Lst1 == HIGH && Sw1 == LOW) Start = millis();
  else if(Lst1 == LOW && Sw1 == HIGH){
    Stop = millis();
    Time = Stop - Start;

    if(Time <= Stime){
      // String Data = E_Read(0);
      // String Lat = Data.substring(0, Data.indexOf("&"));
      // String Lon = Data.substring(Data.indexOf("&"), Data.length());

      lcd.setCursor(0, 0);
      lcd.print(String("Lat = " + Lat + "          "));
      lcd.setCursor(0, 1);
      lcd.print(String("Lon = " + Lon + "          "));
    }
    if(Time >= Ltime){
      Mode = 0;
      // E_Write(0, data2.lat + "&" + data2.lon);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Current Location");
      lcd.setCursor(0, 1);
      lcd.print("    is Saved    ");
    }
  }
  Lst1 = Sw1;

  // Read Switch 2
  Sw2 = digitalRead(ps2);
  if(Sw2 == 0 && Lst2 == 1) Lst2 = Sw2;
  else if(Sw2 == 1 && Lst2 == 0){
    Mode++;
    Lst2 = Sw2;
  }
  if(Mode == 1){
    lcd.setCursor(0, 0);
    lcd.print(String("Battery = " + String(data2.bat) + "%         "));
    lcd.setCursor(0, 1);
    lcd.print(String("Tank = " + String(data2.tank) + "%           "));
  }
  else if(Mode == 2){
    lcd.setCursor(0, 0);
    lcd.print(String("Lat = " + data2.lat + "            "));
    lcd.setCursor(0, 1);
    lcd.print(String("Lon = " + data2.lon + "            "));
  }
  else if(Mode == 3){
    lcd.setCursor(0, 0);
    lcd.print(" Fire Detected  ");
    lcd.setCursor(0, 1);
    if(data2.fire == 0) lcd.print("       NO       ");
    else lcd.print("      YES       ");
  }
  if(Mode > 3) Mode = 1;
}
