#include <DS1307.h>
#include <mcp_can.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include "EasyNextionLibrary.h"
EasyNex myNex(Serial);
#include "Nextion.h"

MCP_CAN CAN0(10);                               // Set CS to pin 10
SoftwareSerial mySerial(2, 3);
DS1307 rtc(20, 21);

#define ss 6
#define rst 4
#define dio0 5


String LoRaMessage = "";
String HM = "";
int counter = 0;
String sData;
String arrData[6];
bool parsing = false;
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
double visc = 0.015625;
float viscosity = 0.0;
float density = 0.0;
float dielectric = 0.0;
float temperature = 0.0;
int i;
char *endptr;
String myStrHM;
String data;
char c;

int addr_detik = 0;
int addr_menit = 1;
int addr_jam1 = 2;
int addr_jam2 = 3;
int addr_jam3 = 4;
int addr_jam4 = 5;
int addr_jam5 = 6;
int addr_jam6 = 7;

int detik = 0;
int menit = 0;
int jam1 = 0;
int jam2 = 0;
int jam3 = 0;
int jam4 = 0;
int jam5 = 0;
int jam6 = 0;

int value_detik;
int value_menit;
int value_jam1;
int value_jam2;
int value_jam3;
int value_jam4;
int value_jam5;
int value_jam6;

int print_display;

String kirim = "";
String hmx = "";
String hm_string = "";
int kondisi = 2;

int statusPS;
long x = 0;
bool isReminder250 = false;
bool isReminder500 = false;
bool isReminder1000 = false;
bool isReminder2000 = false;
bool isReminder4000 = false;

int updateInitial = 9;
int buttonCancel = 7;
int motor = 3;
int alternator = 8;

bool engineMati = false;
bool engineRunning = true;
bool isUpdate = false;

uint16_t TempEngCoolPin = A0;
uint16_t TempPowPin = A1;
uint16_t FueLevPin = A2;
uint16_t EngAirCleanrPin = A3;


uint16_t sensor_value = 0;  // variable to store the value coming from the sensor
//uint16_t temperature = 0;
uint16_t temp = 0;

uint16_t VoltEngCoolVal = 0;
uint16_t VoltEngCool = 0;

uint16_t VoltPowTempVal = 0;
uint16_t VoltPowTemp = 0;
uint16_t TempPowTrain = 0;

uint16_t VoltFuelLevelVal = 0;
uint16_t VoltFuelLevel = 0;

float Temp_EngCoolant = 0.0;     // Store the value 
float lastSentTemp_EngCoolant = 0.0; // Store the last value that we have sent on Nextion

float AirCleaner_Value = 0.0;     // Store the value 
float lastSent_AirCleaner_Value = 0.0; // Store the last value that we have sent on Nextion

float viscosity_Value = 0.0;     // Store the value 
float lastSent_viscosity_Value = 0.0; // Store the last value that we have sent on Nextion

float density_Value = 0.0;     // Store the value 
float lastSent_density_Value = 0.0; // Store the last value that we have sent on Nextion

float dielectric_Value = 0.0;     // Store the value 
float lastSent_dielectric_Value = 0.0; // Store the last value that we have sent on Nextion

float temperature_Value = 0.0;     // Store the value 
float lastSent_temperature_Value = 0.0; // Store the last value that we have sent on Nextion

String hm_Value = "";     // Store the value 
String lastSent_hm_Value = ""; // Store the last value that we have sent on Nextion

unsigned long getDataTimer = millis(); // Timer for GET_DATA_EVERY
//----------------------------------
// Change Page Initialization
//----------------------------------
#define DATA_REFRESH_RATE 1000 // The time between each Data refresh of the page
// Depending on the needs of the project, the DATA_REFRESH_RATE can be set
// to 50ms or 100ms without a problem. In this example, we use 1000ms,
#define GET_DATA_EVERY 2000
unsigned long pageRefreshTimer = millis(); // Timer for DATA_REFRESH_RATE
bool newPageLoaded = false; // true when the page is first loaded ( lastCurrentPageId != currentPageId )


void setup() {
//  Serial.begin(115200);
//  //Serial.begin(9600);
//  while (!Serial);
//  Serial.println("LoRa Receiver");
//  LoRa.setPins(ss, rst, dio0);
//
//  if (!LoRa.begin(433E6)) {
//    Serial.println(".");
//    while (1);
//  }
//  LoRa.setSyncWord(0xF3);
//  Serial.println("LoRa Initializing OK!");

  rtc.begin();
//  rtc.setDOW(TUESDAY);           // Set Hari
//  rtc.setTime(13, 46, 00);      // Set waktu JJ:MM:DD (24hr format)
//  rtc.setDate(15, 6, 2021);     // Set tanggal 
 
  value_detik = EEPROM.read(addr_detik);
  value_menit = EEPROM.read(addr_menit);
  value_jam1 = EEPROM.read(addr_jam1);
  value_jam2 = EEPROM.read(addr_jam2);
  value_jam3 = EEPROM.read(addr_jam3);
  value_jam4 = EEPROM.read(addr_jam4);
  value_jam5 = EEPROM.read(addr_jam5);
  value_jam6 = EEPROM.read(addr_jam6);

  //  pinMode(buttonCancel, INPUT);
  pinMode(alternator, INPUT);
  digitalWrite(alternator, LOW);

  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  Serial.begin(115200);

  mySerial.begin(115200);
  myNex.begin(9600);
  delay(500);               // give Nextion some time to finish initialize
  myNex.writeStr("page 0"); // For synchronizing Nextion page in case of reset to Arduino
  delay(50);
  myNex.lastCurrentPageId = 2; // At the first run of the loop, the currentPageId and the lastCurrentPageId
  // must have different values, due to run the function firstRefresh()
  analogWrite(A0, LOW);
  analogWrite(A1, LOW);
  analogWrite(A2, LOW);
  digitalWrite(A3, HIGH);
  CAN0.begin(CAN_500KBPS);                       // init can bus : baudrate = 500k
  //pinMode(2, INPUT);                            // Setting pin 2 for /INT input
  Serial.print("   Viscosity       |    Density       |     Dielectric      |    Oil Temperature      ");

}

void loop() {

//  myNex.writeStr("tdate.txt", rtc.getDateStr());
//  myNex.writeStr("ttime.txt", rtc.getTimeStr());

  myNex.NextionListen(); // This function must be called repeatedly to response touch events
  readSensorValues();
  refereshCurrentPage();
  firstRefresh();

//  myNex.writeStr("hm.txt", hm_string);
//  String sviscosity = String (viscosity,2);
//  myNex.writeStr("tviscosity.txt", sviscosity);
//  String sdensity = String (density,2);
//  myNex.writeStr("tdensity.txt", sdensity);
//  String sdielectric = String (dielectric,2);
//  myNex.writeStr("tdielectric.txt", sdielectric);
//  String stemperature = String (temperature,2);
//  myNex.writeStr("ttemperature.txt", stemperature);

  

  // isUpdate = digitalRead(updateInitial) == HIGH;

  // if (isUpdate) {
  //   updateInitialHM();
  // }


 // logic update hm
  if (digitalRead(updateInitial) == HIGH) {
    updateInitialHM();
  }
  
// logic mematikan motor
  engineMati = digitalRead(motor) == LOW;
  if (engineMati) {
    digitalWrite(motor, HIGH);
  } else {
    digitalWrite(motor, LOW);
  }
  
// logic engine running
  engineRunning = digitalRead(alternator);
  if (engineRunning == HIGH) {
    hmCounter();
    //reminderService();
  }

//logic engine mati
  else if (!engineRunning) {
    Serial.println("Engine Stop");
    //delay(1000);
  }

//logic cancel notifisaksi PS
  if (digitalRead(buttonCancel) == HIGH) {
    statusPS = 0;
    kondisi = 1;
  }

 // LoRaMessage = String(kirim) + "/" + String(viscosity) + "&" + String(density)
                + "#" + String(dielectric) + "@" + String(temperature) + "%" + String(statusPS);
  // send packet
//  LoRa.beginPacket();
//  LoRa.print(LoRaMessage);
//  LoRa.endPacket();
  delay(1000);
}

void firstRefresh() { // This function's purpose is to update the values of a new page when is first loaded,
  if (myNex.currentPageId != myNex.lastCurrentPageId) { // If the two variables are different, means a new page is loaded.
    newPageLoaded = true;    // A new page is loaded
    switch (myNex.currentPageId) {
      case 0:
        refreshPage0();
        break;
      case 1:
        refreshPage1();
        break;
      case 2:
        refreshPage2();
        break;
    }

    newPageLoaded = false;  // After we have updated the new page for the first time, we update the variable to false.
    // Now the values updated ONLY if the new value is different from the last Sent value.
    // See void refreshPage0()

    myNex.lastCurrentPageId = myNex.currentPageId; // Afer the refresh of the new page We make them equal,
    // in order to identify the next page change.
  }
}

void readSensorValues() {

  if ((millis() - getDataTimer) > GET_DATA_EVERY) {
    FPS();
    Temp_EngCool ();
    Temp_PowTrain();
    Fuel_level ();
    EngAirCleaner();

    getDataTimer = millis();

  }
}

void refereshCurrentPage() {
  // In this function we refresh the page currently loaded every DATA_REFRESH_RATE
  if ((millis() - pageRefreshTimer) > DATA_REFRESH_RATE) {
    switch (myNex.currentPageId) {
      case 0:
        refreshPage0();
        break;
      case 1:
        refreshPage1();
        break;
      case 2:
        refreshPage2();
        break;
    }
    pageRefreshTimer = millis();
  }
}

void refreshPage0() {
  if (Temp_EngCoolant != lastSentTemp_EngCoolant|| newPageLoaded == true) {
    lastSentTemp_EngCoolant = Temp_EngCoolant;
  }

  if (hm_Value != kirim ||newPageLoaded == true) {
    lastSent_hm_Value = kirim;
    
    
}
}

void refreshPage1() {
  if (AirCleaner_Value != lastSent_AirCleaner_Value ||newPageLoaded == true) {
    lastSent_AirCleaner_Value = AirCleaner_Value;
  }
  
  if (viscosity != lastSent_viscosity_Value ||newPageLoaded == true) {
    lastSent_viscosity_Value = viscosity;
  }

   if (density_Value != lastSent_density_Value ||newPageLoaded == true) {
    lastSent_density_Value = density_Value;
  }

   if (dielectric_Value != lastSent_dielectric_Value ||newPageLoaded == true) {
    lastSent_dielectric_Value = dielectric_Value;
  }

   if (temperature_Value != lastSent_temperature_Value ||newPageLoaded == true) {
    lastSent_temperature_Value = temperature_Value;
  }
 
}

void refreshPage2() {
  if (newPageLoaded == true) {
  }
}

void Temp_EngCool ()
{
  VoltEngCoolVal = analogRead(TempEngCoolPin);    
 // int VoltEngCool = map(VoltEngCoolVal, 0, 1024, 0, 5000); // same like: voltage = analogRead(A0)*5000/1024
//  String coolantt = String (VoltEngCool);
float VoltEngCool = VoltEngCoolVal * 5000.0/1023;
  Serial.println(VoltEngCool);
  //Serial.println(VoltEngCool);
  //myNex.writeStr("tcoolanttemp.txt", coolantt);

}

void Temp_PowTrain()
{
  VoltPowTempVal = analogRead(TempPowPin);
  TempPowTrain = map(VoltPowTempVal, 0, 1024, 0, 5000);
  //  VoltPowTemp = VoltPowTempVal - 185;
  //  TempPowTrain = 100 - VoltPowTemp / 5.4;
  //myNex.writeNum("PowTempV.val", TempPowTrain);
}

void Fuel_level ()
{
  VoltFuelLevelVal = analogRead(FueLevPin);
  VoltFuelLevel = map(VoltFuelLevelVal, 0, 1024, 0, 5000);
  //myNex.writeNum("FuelLevVal.val", VoltFuelLevel);
}

void EngAirCleaner()
{
  int EngAirCleanrVal = digitalRead(EngAirCleanrPin);
  if (EngAirCleanrVal == LOW) {
    //myNex.writeNum("taircleaner.bco", 63488); // Set button b0 background color to RED (color code: 63488)
    //myNex.writeStr("taircleaner.txt", "CLOGGED");
  }
   if (EngAirCleanrVal == HIGH) {
    //myNex.writeNum("taircleaner.bco", 31);
    //myNex.writeStr("taircleaner.txt", "NORMAL");
   }
}


//void reminderService() {
//  long  hm = hmx.toInt();
//  Serial.println(hm);
//
//  if ((hm + 50) % 250  == 0 && (hm + 50) % 500 != 0) {
//    statusPS = 250;
//
//  } else if ((hm + 50) % 500 == 0 && (hm + 50) % 1000 != 0) {
//    statusPS = 500;
//  }
//  else if ((hm + 50) % 1000 == 0 && (hm + 50) % 2000 != 0) {
//    statusPS = 1000;
//  }
//  else if ((hm + 50) % 2000 == 0 && (hm + 50) % 4000 != 0) {
//    statusPS = 2000;
//  }
//  else if ( (hm +  50) % 4000 == 0) {
//    statusPS = 4000;
//  }
//  Serial.println(statusPS);
//
//  if (kondisi == 0) {
//    engineMati = true; //3 pin engine starter
//    Serial.println("Engine pun mati");
//  } else if (kondisi == 1 ) {
//    engineMati = false;
//    Serial.println("Engine hidup");
//  }
//  Serial.println(kondisi);
//
//  //Cut-off engine
//  if ((hm - 50) % 250  == 0 && (hm - 50) % 500 != 0) {
//
//    if (kondisi == 2) {
//      kondisi = 0;
//    } else if (kondisi == 1) {
//      kondisi = 1;
//    }
//
//  } else if ((hm - 50) % 500 == 0 && (hm - 50) % 1000 != 0) {
//    if (kondisi == 2) {
//      kondisi = 0;
//    } else {
//      kondisi = 1;
//    }
//
//  }
//  else if ((hm -  50) % 1000 == 0 && (hm - 50) % 2000 != 0) {
//    if (kondisi == 2) {
//      kondisi = 0;
//    } else if (kondisi == 1) {
//      kondisi = 1;
//    }
//  }
//  else if ((hm - 50) % 2000 == 0 && (hm - 50) % 4000 != 0) {
//    if (kondisi == 2) {
//      kondisi = 0;
//    } else if (kondisi == 1) {
//      kondisi = 1;
//    }
//  }
//  else if ((hm - 50) % 4000 == 0) {
//    if (kondisi == 2) {
//      kondisi = 0;
//    } else if (kondisi == 1) {
//      kondisi = 1;
//    }
//  } else {
//    kondisi = 2;
//  }
//
//  delay(1000);
//}


void FPS() {
  if (!digitalRead(7))                         // If pin 7 is low, read receive buffer
  {
    CAN0.readMsgBuf(&len, rxBuf);              // Read data: len = data length, buf = data byte(s)
    rxId = CAN0.getCanId();                    // Get message ID
    Serial.println();
    if (rxId == 0x1CFD083F) {
      String vis1 = String(rxBuf[1], HEX);
      String vis2 = String(rxBuf[0], HEX);
      String vis = vis1 + vis2;
      float v = hexToDec(vis);
      viscosity = v * 0.015625;
      
    }

    if (rxId == 0x1CFD083F) {
      String den1 = String(rxBuf[3], HEX);
      String den2 = String(rxBuf[2], HEX);
      String den = den1 + den2;
      float d = hexToDec(den);
      density = d * 0.00003052;
     
    }
    if (rxId == 0x1CFD083F) {
      String die1 = String(rxBuf[7], HEX);
      String die2 = String(rxBuf[6], HEX);
      String die = die1 + die2;
      float diel = hexToDec(die);
      dielectric = diel * 0.00012207;
      
    }

    if (rxId == 0x18FEEE3F) {
      String temp1 = String(rxBuf[3], HEX);
      String temp2 = String(rxBuf[2], HEX);
      String temp = temp1 + temp2;
      float a = hexToDec(temp);
      temperature = (a * 0.03125) - 273;
      
    }
    if (rxId == 0x18FF313F) {
      String diag = String(rxBuf[0], HEX);
      int a = hexToDec(diag);
    }
  }

//  Serial.print("Viscosity = ");
//  Serial.print(viscosity);
//  Serial.print("  |  ");
//  Serial.print("Density = ");
//  Serial.print(density);
//  Serial.print("  |  ");
//  Serial.print("Dielectric = ");
//  Serial.print(dielectric);
//  Serial.print("  |  ");
//  Serial.print("Oil Temperature = ");
//  Serial.print(temperature);
//  Serial.println("  |  ");
  delay(30);
  
 
  
}




void updateInitialHM() {
  EEPROM.update(addr_menit, 0);
  EEPROM.update(addr_jam1, 0);
  EEPROM.update(addr_jam2, 0);
  EEPROM.update(addr_jam3, 3);
  EEPROM.update(addr_jam4, 0);
  EEPROM.update(addr_jam5, 0);
  EEPROM.update(addr_jam6, 0);
}

void hmCounter() {
  // put your main code here, to run repeatedly:
  value_detik = value_detik + 1;
  EEPROM.update(addr_detik, value_detik);

  if (value_detik == 60) {
    detik = 0;
    EEPROM.update(addr_detik, detik);

    value_menit = value_menit + 1;
    EEPROM.update(addr_menit, value_menit);
  }

  value_detik = EEPROM.read(addr_detik);

  if (value_menit == 60) {
    menit = 0;
    EEPROM.update(addr_menit, menit);

    value_jam1 = value_jam1 + 1;
    EEPROM.update(addr_jam1, value_jam1);
  }
  value_menit = EEPROM.read(addr_menit);


  if (value_jam1 == 10) {
    jam1 = 0;
    EEPROM.update(addr_jam1, jam1);

    value_jam2 = value_jam2 + 1;
    EEPROM.update(addr_jam2, value_jam2);
  }
  value_jam1 = EEPROM.read(addr_jam1);

  if (value_jam2 == 10) {
    jam2 = 0;
    EEPROM.update(addr_jam2, jam2);

    value_jam3 = value_jam3 + 1;
    EEPROM.update(addr_jam3, value_jam3);
  }
  value_jam2 = EEPROM.read(addr_jam2);

  if (value_jam3 == 10) {
    jam3 = 0;
    EEPROM.update(addr_jam3, jam3);

    value_jam4 = value_jam4 + 1;
    EEPROM.update(addr_jam4, value_jam4);
  }
  value_jam3 = EEPROM.read(addr_jam3);

  if (value_jam4 == 10) {
    jam4 = 0;
    EEPROM.update(addr_jam4, jam4);

    value_jam5 = value_jam5 + 1;
    EEPROM.update(addr_jam5, value_jam5);
  }
  value_jam4 = EEPROM.read(addr_jam4);


  if (value_jam5 == 10) {
    jam5 = 0;
    EEPROM.update(addr_jam5, jam5);

    value_jam6 = value_jam6 + 1;
    EEPROM.update(addr_jam6, value_jam6);
  }
  value_jam5 = EEPROM.read(addr_jam5);

  if (value_jam6 == 255) {
    jam6 = 0;
    EEPROM.update(addr_jam6, jam6);
  }
  value_jam6 = EEPROM.read(addr_jam6);

  print_display = value_jam6, value_jam5, value_jam4, value_jam3, value_jam2, value_jam1, value_menit, value_detik;

  Serial.print("HM :");
  Serial.print(value_jam6);
  Serial.print(value_jam5);
  Serial.print(value_jam4);
  Serial.print(value_jam3);
  Serial.print(value_jam2);
  Serial.print(value_jam1);
  Serial.print(",");
  Serial.print(value_menit);
  Serial.print(":");
  Serial.print(value_detik);
  Serial.print(" | ");

  String myStr1;
  String myStr2;
  String myStr3;
  String myStr4;
  String myStr5;
  String myStr6;
  String myStr7;
  String myStr8;
  String myStrHM;
  myStr1 = String(value_jam6);
  myStr2 = String(value_jam5);
  myStr3 = String(value_jam4);
  myStr4 = String(value_jam3);
  myStr5 = String(value_jam2);
  myStr6 = String(value_jam1);
  myStr7 = String(value_menit);
  myStr8 = String(value_detik);

  hmx = myStr1 + myStr2 + myStr3 + myStr4 + myStr5 + myStr6;
  hm_string= myStr1 + myStr2 + myStr3 + myStr4 + myStr5 + myStr6 + " : " + myStr7 + " : " + myStr8;

  kirim += value_jam6;
  kirim += value_jam5;
  kirim += value_jam4;
  kirim += value_jam3;
  kirim += value_jam2;
  kirim += value_jam1;
  kirim += ",";
  kirim += value_menit;
  kirim += ":";
  kirim += value_detik;
  
  
}

unsigned int hexToDec(String hexString) {

  unsigned int decValue = 0;
  int nextInt;

  for (int i = 0; i < hexString.length(); i++) {

    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);

    decValue = (decValue * 16) + nextInt;
  }
  return decValue;
}


float mapping_temp(int data)
{float derajat;
       if(data>=592){derajat=(627-(float)data)/11.67;derajat+=31;}
  else if(data<=592&&data>=560){derajat=(592-(float)data)/8;derajat+=34;}
  else if(data<=560&&data>=490){derajat=(560-(float)data)/17.5;derajat+=38;}
  else if(data<=490&&data>=400){derajat=(490-(float)data)/11.25;derajat+=42;}
  else if(data<=400&&data>=304){derajat=(400-(float)data)/9.6;derajat+=50;}
  else if(data<=304){derajat=(304-(float)data)/8.4;derajat+=60;}
  return derajat;
}