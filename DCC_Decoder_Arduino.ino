//******************************************************************************************************
//
// DCC Decoder by Elvis Müller
//
// A little arduino (nano every) based tounout servo controller. Controls up to 4 turnouts with frog polarisation.
// Features:
// - controlled via DCC
// - frog polarisation (one relay)
// - saves the last position and reinit servos accordingly
// - configuration via serial interface
// - supports some I2C Display
//
// uses the DCC library from Aiko Pras (AP_DCC_library) and the display library from Oliver Kraus (U8g2lib).
//
// Copyright 1992-2022 The FreeBSD Project. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
//   in the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
// OF SUCH DAMAGE.
//
// history:
//   2022_07_02 V1.0 EMM: get the thing running on ardunio nano every
//     - runs with Fleischmann Profi-Boss (others not tested now)
//     - dccPin = 2, no PoM, no CV (no AccPin) (beeing both Acc and Loco at the same time lead to problems with resetCmd)
//     - correct initialisation, delay before move
//     - include ssd1306 display support
//     - basic prog button interaction
//   2022_07_12 V1.1 EMM: Prog button and display
//     - implement programming via prog button
//     - implement external buttons via I2C bua
//     - show status in display
//
//******************************************************************************************************
#define VERSION "v1.1"
#define DESCSTR1 "DCC tournout decoder"
#define DESCSTR2 "(c)EMM"

#include <Arduino.h>
#include <AP_DCC_library.h>
#include <Servo.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <Wire.h>

// configure the display (AZDelivery 0,96 Zoll OLED Display I2C SSD1306 Chip 128 x 64)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
unsigned long lastDisplayTime = 0;

const uint8_t dccPin = 2;
const uint8_t progBtnPin = 3;
const uint8_t servo1Pin = 14;
const uint8_t servo2Pin = 15;
const uint8_t servo3Pin = 16;
const uint8_t servo4Pin = 17;
const uint8_t servGnd1Pin = 4;
const uint8_t relay1Pin = 5;
const uint8_t servGnd2Pin = 6;
const uint8_t relay2Pin = 7;
const uint8_t servGnd3Pin = 9;
const uint8_t relay3Pin = 10;
const uint8_t servGnd4Pin = 20;
const uint8_t relay4Pin = 21;

extern Dcc dcc;                  // This object is instantiated in DCC_Library.cpp
extern Accessory accCmd;         // To retrieve data from accessory commands

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

unsigned long lastServoTime = 0;
uint16_t ledPattern = 0x0101;
int ledCount = 0;
enum decoderStates
{ stateIdle
  , stateSelection
};
decoderStates decoderState = stateIdle;
unsigned long lastLedTime = 0;
int eepromResetValue = 0xAA;
int locoAdress = 9999;
int selectedServo = 0;
bool servoTaskActive = true;

unsigned long lastSelftestTime = 0;
int selftestIteration = 0;
bool selftestRunning = false;

int resetAddress = 0;
int dataAddress = 10;

String serialBuffer = "";
String serialToken1 = "";
String serialToken2 = "";
String serialToken3 = "";
String serialToken4 = "";
String serialToken5 = "";
String *serialTokens[] = { &serialToken1, &serialToken2, &serialToken3, &serialToken4, &serialToken5 };

struct servoDataStruct
{
  int adress = 24; // Decoder 24 is switch 97..100
  int deltaMove = 20;

  bool pos1 = true;
  bool moving1 = false;
  int min1 = 1200;
  int max1 = 1800;
  int val1 = 1200;

  bool pos2 = true;
  bool moving2 = false;
  int min2 = 1200;
  int max2 = 1800;
  int val2 = 1200;

  bool pos3 = true;
  bool moving3 = false;
  int min3 = 1200;
  int max3 = 1800;
  int val3 = 1200;

  bool pos4 = true;
  bool moving4 = false;
  int min4 = 1200;
  int max4 = 1800;
  int val4 = 1200;

  int initDelay = 1000;
};
servoDataStruct servoData;

int posCounter1 = 0;
int posCounter2 = 0;
int posCounter3 = 0;
int posCounter4 = 0;

int progBtnPinCount = 5;
bool progBtnPinActive = false;
bool progBtnPinActiveOld = false;
#define DEBOUNCE_LIMIT 22
#define DEBOUNCE_LEVEL DEBOUNCE_LIMIT/2
unsigned long lastProgBtnTaskTime = 0;

#define ADD_BUTTON(name)            \
  bool btnActive##name = false;     \
  bool btnActiveOld##name = false;  \
  int btnCount##name = false;
// use macro
ADD_BUTTON(S1);
ADD_BUTTON(S2);
ADD_BUTTON(S3);
ADD_BUTTON(S4);
ADD_BUTTON(S5);
ADD_BUTTON(S6);
ADD_BUTTON(S7);
ADD_BUTTON(S8);
unsigned long lastKeybTaskTime = 0;

bool frogRelay1On = false;
bool frogRelay2On = false;
bool frogRelay3On = false;
bool frogRelay4On = false;

void saveToEEprom(void)
{
  EEPROM.put(dataAddress, servoData);
  EEPROM.put(resetAddress, 0xAA);
  Serial.print(sizeof(servoData));
  Serial.println(" bytes saved to EEProm!");
}

void resetServoData(void)
{
  servoData.adress = 24;
  servoData.deltaMove = 20;

  servoData.pos1 = true;
  servoData.moving1 = false;
  servoData.min1 = 1200;
  servoData.max1 = 1800;
  servoData.val1 = 1200;

  servoData.pos2 = true;
  servoData.moving2 = false;
  servoData.min2 = 1200;
  servoData.max2 = 1800;
  servoData.val2 = 1200;

  servoData.pos3 = true;
  servoData.moving3 = false;
  servoData.min3 = 1200;
  servoData.max3 = 1800;
  servoData.val3 = 1200;

  servoData.pos4 = true;
  servoData.moving4 = false;
  servoData.min4 = 1200;
  servoData.max4 = 1800;
  servoData.val4 = 1200;

  servoData.initDelay = 1000;

  saveToEEprom();
}

void printServoData(int servo)
{
  #define printServoDataPerServo(number)                                  \
    Serial.print("Servo #");                                              \
    Serial.print(number);                                                 \
    Serial.print("; pos = ");                                             \
    Serial.print(servoData.pos##number ? "true" : "false");               \
    Serial.print("; min = ");                                             \
    Serial.print(servoData.min##number);                                  \
    Serial.print("; max = ");                                             \
    Serial.print(servoData.max##number);                                  \
    Serial.print("; val = ");                                             \
    Serial.println(servoData.val##number);
    // use macro
    switch (servo)
    {
      case 1: printServoDataPerServo(1); break;
      case 2: printServoDataPerServo(2); break;
      case 3: printServoDataPerServo(3); break;
      case 4: printServoDataPerServo(4); break;
      default: break;
    }
  #undef printServoDataPerServo
}

void validateSettings(void)
{
  #define rangeCheckMaxMin(nameOfValue, minValue, maxValue)                      \
    if (minValue > nameOfValue) nameOfValue = minValue;                          \
    if (maxValue < nameOfValue) nameOfValue = maxValue;
  #define rangeCheckMaxMinForSetOfSettings(nameOfValue, minValue, maxValue)      \
    rangeCheckMaxMin(nameOfValue##1, minValue, maxValue);                        \
    rangeCheckMaxMin(nameOfValue##2, minValue, maxValue);                        \
    rangeCheckMaxMin(nameOfValue##3, minValue, maxValue);                        \
    rangeCheckMaxMin(nameOfValue##4, minValue, maxValue);
    //use macro(s)
    rangeCheckMaxMinForSetOfSettings(servoData.min, 1000, 2000);
    rangeCheckMaxMinForSetOfSettings(servoData.max, 1000, 2000);
    rangeCheckMaxMinForSetOfSettings(servoData.val, 1000, 2000);
    rangeCheckMaxMin(servoData.adress, 0, 50);
    rangeCheckMaxMin(servoData.deltaMove, 5, 50);
    rangeCheckMaxMin(servoData.initDelay, 1, 10000);
  #undef rangeCheckMaxMin
  #undef rangeCheckMaxMinForSetOfSettings
  // ... and show what we got
  printServoData(1);
  printServoData(2);
  printServoData(3);
  printServoData(4);
  Serial.print("1st acc (module) adress = ");
  Serial.print(servoData.adress);
  Serial.print("; deltaMove = ");
  Serial.print(servoData.deltaMove);
  Serial.print("us/50ms; deltaMove = ");
  Serial.print(servoData.initDelay);
  Serial.println("ms");
}

void readFromEEprom(void)
{
  EEPROM.get(resetAddress, eepromResetValue);
#ifdef DEBUG
  Serial.print("eepromResetValue = ");
  Serial.println(eepromResetValue);
#endif
  if (eepromResetValue == 0xAA)
  {
    EEPROM.get(dataAddress, servoData);
    Serial.print(sizeof(servoData));
    Serial.println(" bytes read from EEProm!");
  }
  else
  {
    resetServoData();
    Serial.println("No valid data stored in EEProm. Reset decoder data!");
  }
  //range check read values
  validateSettings();
}

void selftestTask(void)
{
  if ((selftestRunning) && (millis() - lastSelftestTime ) > 500)
  { // iterate
    switch (selftestIteration)
    { //indicate
      case 0:
        servoTaskActive = false;
        digitalWrite(LED_BUILTIN, HIGH);
        break;
      case 1: digitalWrite(LED_BUILTIN, LOW); break;
      case 2: digitalWrite(LED_BUILTIN, HIGH); break;
      case 3: digitalWrite(LED_BUILTIN, LOW); break;
      //switch relays
      case 4: digitalWrite(relay1Pin, HIGH); break;
      case 5: digitalWrite(relay2Pin, HIGH); break;
      case 6: digitalWrite(relay3Pin, HIGH); break;
      case 7: digitalWrite(relay4Pin, HIGH); break;
      case 8: digitalWrite(relay1Pin, LOW); break;
      case 9: digitalWrite(relay2Pin, LOW); break;
      case 10: digitalWrite(relay3Pin, LOW); break;
      case 11: digitalWrite(relay4Pin, LOW); break;
      //all to the left
      case 12:
        servoTaskActive = true;
        setSelectedServoPos(1, false);
        break;
      case 13: setSelectedServoPos(2, false); break;
      case 14: setSelectedServoPos(3, false); break;
      case 15: setSelectedServoPos(4, false); break;
      //all to the right
      case 16: setSelectedServoPos(1, true); break;
      case 17: setSelectedServoPos(2, true); break;
      case 18: setSelectedServoPos(3, true); break;
      case 19: setSelectedServoPos(4, true); break;
      // end this sequence
      case 20:
        selftestRunning = false;
        readFromEEprom();
        fixServoPosition();
        Serial.println("... finished!");
        break;
      default:
        break;
    }
    selftestIteration++;
    lastSelftestTime = millis();
  }
}

void serialInterfaceTask(void)
{
  if (Serial.available())
  {
    char incomingByte = Serial.read();
    Serial.print(incomingByte);
    serialBuffer += incomingByte;
    if (('\n' == incomingByte) || ('\r' == incomingByte))
    {
      analyseSerialCommand();
      serialBuffer = "";
    }
  }
}

void ledTask(void)
{
  if ((millis() - lastLedTime ) > 200)
  {
    switch (decoderState)
    {
      case stateIdle:
        if (0 < selectedServo)
        {
          ledCount = 0;
          Serial.println("LED: switch state = stateSelection");
        }
        break;
      case stateSelection:
        switch (selectedServo)
        {
          case 1: ledPattern = 0x0001; break;
          case 2: ledPattern = 0x0005; break;
          case 3: ledPattern = 0x0015; break;
          case 4: ledPattern = 0x0055; break;
        }
        digitalWrite(LED_BUILTIN, (ledPattern >> (ledCount % 16) & 0x01) ? HIGH : LOW);
        ledCount++;
        break;
      default: break;
    }
    lastLedTime = millis();
  }
}

void fixServoPosition(void)
{
  // fix position
  if (servoData.pos1) servoData.val1 = servoData.min1; else servoData.val1 = servoData.max1;
  if (servoData.pos2) servoData.val2 = servoData.min2; else servoData.val2 = servoData.max2;
  if (servoData.pos3) servoData.val3 = servoData.min3; else servoData.val3 = servoData.max3;
  if (servoData.pos4) servoData.val4 = servoData.min4; else servoData.val4 = servoData.max4;
}

void initialiseOutputsAndPins()
{
  // care about output defaults
  pinMode(servGnd1Pin, OUTPUT);
  pinMode(servGnd2Pin, OUTPUT);
  pinMode(servGnd3Pin, OUTPUT);
  pinMode(servGnd4Pin, OUTPUT);
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  pinMode(relay3Pin, OUTPUT);
  pinMode(relay4Pin, OUTPUT);
  digitalWrite(servGnd1Pin, LOW);
  digitalWrite(servGnd2Pin, LOW);
  digitalWrite(servGnd3Pin, LOW);
  digitalWrite(servGnd4Pin, LOW);
  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay2Pin, LOW);
  digitalWrite(relay3Pin, LOW);
  digitalWrite(relay4Pin, LOW);

  dcc.attach(dccPin);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(progBtnPin, INPUT_PULLUP);
}

void initialiseServos(void)
{
  //prepare servos
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  servo4.attach(servo4Pin);
  //activate servos
  digitalWrite(servGnd1Pin, HIGH);
  digitalWrite(servGnd2Pin, HIGH);
  digitalWrite(servGnd3Pin, HIGH);
  digitalWrite(servGnd4Pin, HIGH);
  delay(50);
  //write given values
  servo1.writeMicroseconds(servoData.val1);
  servo2.writeMicroseconds(servoData.val2);
  servo3.writeMicroseconds(servoData.val3);
  servo4.writeMicroseconds(servoData.val4);
  delay(50);
  //deactivate servos again
  digitalWrite(servGnd1Pin, LOW);
  digitalWrite(servGnd2Pin, LOW);
  digitalWrite(servGnd3Pin, LOW);
  digitalWrite(servGnd4Pin, LOW);
}

void printHelp(void)
{
  Serial.println("available commands (followed by enter):");
  Serial.println("| set;<type>;<servo_nr>;<value> ... set some servo values");
  Serial.println("|   type=min: minimum servo position");
  Serial.println("|   type=max: maximum servo position");
  Serial.println("|   type=val: actual servo position");
  Serial.println("| set;<type>;<value> .............. set other values");
  Serial.println("|   type=adr: first adress of servo");
  Serial.println("|   type=dlt: microsecs per step (50ms)");
  Serial.println("|   type=dly: initial delay [ms]");
  Serial.println("| slv ............................. start selftest");
  Serial.println("| rst ............................. reset servo data");
}

void getSerialTokens(void)
{
  int posSeparator = 0;
  int posSeparatorNext = 0;
  int startPoint = 0;
  int count = 0;
  while (0 <= posSeparator)
  {
    posSeparator = serialBuffer.indexOf(';', startPoint);
    if (0 <= posSeparator)
    {
      startPoint = posSeparator + 1;
      posSeparatorNext = serialBuffer.indexOf(';', startPoint);
      if ((0 <= posSeparator) && (4 > count))
      {
        *serialTokens[count] = serialBuffer.substring(startPoint, posSeparatorNext);
      }
      else
      {
        *serialTokens[count] = serialBuffer.substring(startPoint);
        break; //musst be the last, end anyway
      }
      #ifdef DEBUG
        Serial.print("token #");
        Serial.print(count);
        Serial.print("=");
        Serial.print(*serialTokens[count]);
        Serial.print("; ");
      #endif
      count++;
    }
  }
  #ifdef DEBUG
    Serial.println("");
  #endif
}

void setSelectedServoPos(int selServo, bool newValue)
{
  int counterValue = 20;
  switch (selServo)
  {
    case 1:
      servoData.pos1 = newValue;
      posCounter1 = counterValue;
      servoData.moving1 = true;
      break;
    case 2:
      servoData.pos2 = newValue;
      posCounter2 = counterValue;
      servoData.moving2 = true;
      break;
    case 3:
      servoData.pos3 = newValue;
      posCounter3 = counterValue;
      servoData.moving3 = true;
      break;
    case 4:
      servoData.pos4 = newValue;
      posCounter4 = counterValue;
      servoData.moving4 = true;
      break;
    default:
      break;
  }
  Serial.print("Set Servo #");
  Serial.print(selServo);
  Serial.print(" to new pos = ");
  Serial.println(newValue ? "true" : "false");
}

void setSelectedServoMax(int selServo, int newValue)
{
  switch (selServo)
  {
    case 1: servoData.max1 = newValue; break;
    case 2: servoData.max2 = newValue; break;
    case 3: servoData.max3 = newValue; break;
    case 4: servoData.max4 = newValue; break;
    default: break;
  }
  Serial.print("Set Servo #");
  Serial.print(selServo);
  Serial.print(" to new max = ");
  Serial.println(newValue);
}

void setSelectedServoMin(int selServo, int newValue)
{
  switch (selServo)
  {
    case 1: servoData.min1 = newValue; break;
    case 2: servoData.min2 = newValue; break;
    case 3: servoData.min3 = newValue; break;
    case 4: servoData.min4 = newValue; break;
    default: break;
  }
  Serial.print("Set Servo #");
  Serial.print(selServo);
  Serial.print(" to new min = ");
  Serial.println(newValue);
}

void setSelectedServoVal(int selServo, int newValue)
{
  switch (selServo)
  {
    case 1: servoData.val1 = newValue; break;
    case 2: servoData.val2 = newValue; break;
    case 3: servoData.val3 = newValue; break;
    case 4: servoData.val4 = newValue; break;
    default: break;
  }
  Serial.print("Set Servo #");
  Serial.print(selServo);
  Serial.print(" to new val = ");
  Serial.println(newValue);
}

void analyseSerialCommand(void)
{
  if (serialBuffer.startsWith("?"))
  {
    printHelp();
  }
  else if (serialBuffer.startsWith("set"))
  {
    getSerialTokens();
    int value1 = serialTokens[1]->toInt();
    bool save = true;
    if (serialTokens[0]->equals("max"))
    {
      if ((1 <= value1) && (5 >= value1))
      {
        setSelectedServoMax(value1, serialTokens[2]->toInt());
        setSelectedServoVal(value1, serialTokens[2]->toInt());
        setSelectedServoPos(value1, false);
      }
    }
    else if (serialTokens[0]->equals("min"))
    {
      if ((1 <= value1) && (5 >= value1))
      {
        setSelectedServoMin(value1, serialTokens[2]->toInt());
        setSelectedServoVal(value1, serialTokens[2]->toInt());
        setSelectedServoPos(value1, true);
      }
    }
    else if (serialTokens[0]->equals("val"))
    {
      if ((1 <= value1) && (5 >= value1))
      {
        setSelectedServoVal(value1, serialTokens[2]->toInt());
      }
    }
    else if (serialTokens[0]->equals("pos"))
    {
      if ((1 <= value1) && (5 >= value1))
      {
        setSelectedServoPos(value1, serialTokens[2]->toInt());
      }
    }
    else if (serialTokens[0]->equals("adr"))
    {
      servoData.adress = value1;
    }
    else if (serialTokens[0]->equals("dlt"))
    {
      servoData.deltaMove = value1;
    }
    else if (serialTokens[0]->equals("dly"))
    {
      servoData.initDelay = value1;
    }
    else
    {
      Serial.print("Unknown Command: ");
      Serial.println(serialBuffer);
    }
    validateSettings();
    if (save)
    {
      saveToEEprom();
    }
  }
  else if (serialBuffer.startsWith("slv"))
  {
    Serial.println("Start selftest ...");
    selftestIteration = 0;
    selftestRunning = true;
  }
  else if (serialBuffer.startsWith("rst"))
  {
    resetServoData();
  }
  else
  {
    Serial.println("Unknown Command!");
  }
}

void helloUart()
{
  Serial.println(DESCSTR1);
  Serial.print(VERSION);
  Serial.print(" ");
  Serial.println(DESCSTR2);
  Serial.println("type '?'+<CR> for help!");
}

void helloDisplay(void)
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 10, DESCSTR1);
  u8g2.drawStr(0, 22, DESCSTR2);
  u8g2.drawStr(60, 22, VERSION);
  u8g2.drawStr(30, 44, "initial delay");
  u8g2.drawStr(30, 56, "waiting...");
  u8g2.sendBuffer();
}

void I2CWrite(int adress, byte data)
{
  Wire.beginTransmission(adress);
  Wire.write(data);
  Wire.endTransmission();
}

byte I2CRead(int adress, bool& success)
{
  byte data = 0xff;
  Wire.requestFrom(adress, 1);
  if (Wire.available())
  {
    data = Wire.read();
    success = true;
  }
  else
  {
    success = false;
  }
  return data;
}

void scanI2CBus(void)
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  I2CWrite(0x20, 0xff);
}

void dccInterfaceTask(void)
{
  if (dcc.input())
  {
    switch (dcc.cmdType)
    {
      case Dcc::MyAccessoryCmd :
        accCmd.command == Accessory::basic ? Serial.print("B. Acc, Address=") : Serial.print("E. Acc, Adress=");
        Serial.print(accCmd.decoderAddress);
        Serial.print("; Turnout=");
        Serial.print(accCmd.turnout);
        Serial.print("; Switch number=");
        Serial.print(accCmd.outputAddress);
        accCmd.position == 1 ? Serial.print(" +") : Serial.print(" -");
        if (accCmd.activate) Serial.print("; Activate!");
        Serial.println("");
        setSelectedServoPos(accCmd.turnout, accCmd.position == 1);
        saveToEEprom();
        break;
      case Dcc::AnyAccessoryCmd :
        Serial.print(accCmd.decoderAddress);
        Serial.println(" (any)");
        if (0 != selectedServo)
        {
          servoData.adress = accCmd.decoderAddress;
        }
        break;
      default:
        break;
    }
  }
}

void servoTask(void)
{
  if (((millis() - lastServoTime ) > 50) && servoTaskActive)
  {
    // define macro
    #define careServo(number, direction, position)                                                                                        \
      if (servoData.pos##number == position)                                                                                              \
      {                                                                                                                                   \
        if (0 < posCounter##number)                                                                                                       \
        {                                                                                                                                 \
          posCounter##number--;                                                                                                           \
        }                                                                                                                                 \
        else                                                                                                                              \
        {                                                                                                                                 \
          if (abs(servoData.val##number - servoData.direction##number) < servoData.deltaMove)                                             \
          {                                                                                                                               \
            servoData.moving##number = false;                                                                                             \
            servoData.val##number = servoData.direction##number;                                                                          \
          }                                                                                                                               \
          else if (servoData.val##number < servoData.direction##number)                                                                   \
          {                                                                                                                               \
            servoData.val##number += servoData.deltaMove;                                                                                 \
            servoData.moving##number = true;                                                                                              \
          }                                                                                                                               \
          else                                                                                                                            \
          {                                                                                                                               \
            servoData.val##number -= servoData.deltaMove;                                                                                 \
            servoData.moving##number = true;                                                                                              \
          }                                                                                                                               \
          if ((MIN_PULSE_WIDTH <= servoData.val##number) && (MAX_PULSE_WIDTH > servoData.val##number))                                    \
          {                                                                                                                               \
            servo##number.writeMicroseconds(servoData.val##number);                                                                       \
          }                                                                                                                               \
          frogRelay##number##On = (servoData.val##number > ((servoData.max##number + servoData.min##number)/2));                          \
          digitalWrite(relay##number##Pin, frogRelay##number##On ? LOW : HIGH);                                                           \
          digitalWrite(servGnd##number##Pin, servoData.moving##number ? HIGH : LOW);                                                      \
        }                                                                                                                                 \
      }
      // use macro, one for each direction
      careServo(1, min, true);
      careServo(1, max, false);
      careServo(2, min, true);
      careServo(2, max, false);
      careServo(3, min, true);
      careServo(3, max, false);
      careServo(4, min, true);
      careServo(4, max, false);
      // get rid of macro
    #undef careServo
    lastServoTime = millis();
  }
}

void displayTask(void)
{
  static unsigned int aLiveClockCounter = 0;
  if ((millis() - lastDisplayTime) > 500)
  {
    u8g2.clearBuffer();
    if (selftestRunning)
    {
      u8g2.drawStr(0,  8, "Selftest running...");
      if (4 > selftestIteration) {
        u8g2.drawStr(0,  30, "> Intro");
      }
      else if (8 > selftestIteration) {
        u8g2.drawStr(0,  30, "> all relay left");
      }
      else if (12 > selftestIteration) {
        u8g2.drawStr(0,  30, "> all relay right");
      }
      else if (16 > selftestIteration) {
        u8g2.drawStr(0,  30, "> all pos false");
      }
      else {
        u8g2.drawStr(0,  30, "> all pos true");
      }
      char iterationStr[20]; snprintf(iterationStr, 20, "%d", selftestIteration); u8g2.drawStr(0,  45, iterationStr);
    }
    else
    {
      // header
      if ((0==selectedServo)||((aLiveClockCounter%2)||!(1==selectedServo))) u8g2.drawStr(0,  8, "S1__");
      if ((0==selectedServo)||((aLiveClockCounter%2)||!(2==selectedServo))) u8g2.drawStr(32, 8, "S2__");
      if ((0==selectedServo)||((aLiveClockCounter%2)||!(3==selectedServo))) u8g2.drawStr(64, 8, "S3__");
      if ((0==selectedServo)||((aLiveClockCounter%2)||!(4==selectedServo))) u8g2.drawStr(96, 8, "S4__");
      if (0 == selectedServo)
      {
        // show position
        char s1PosStr[20]; snprintf(s1PosStr, 20, "%d", servoData.val1); u8g2.drawStr(0,  20, s1PosStr);
        char s2PosStr[20]; snprintf(s2PosStr, 20, "%d", servoData.val2); u8g2.drawStr(32, 20, s2PosStr);
        char s3PosStr[20]; snprintf(s3PosStr, 20, "%d", servoData.val3); u8g2.drawStr(64, 20, s3PosStr);
        char s4PosStr[20]; snprintf(s4PosStr, 20, "%d", servoData.val4); u8g2.drawStr(96, 20, s4PosStr);
        // show status
        u8g2.drawStr(0,  30, servoData.moving1 ? "mov" : "idle");
        u8g2.drawStr(32, 30, servoData.moving2 ? "mov" : "idle");
        u8g2.drawStr(64, 30, servoData.moving3 ? "mov" : "idle");
        u8g2.drawStr(96, 30, servoData.moving4 ? "mov" : "idle");
        // show target
        u8g2.drawStr(0,  40, servoData.pos1 ? "|" : "/");
        u8g2.drawStr(32, 40, servoData.pos2 ? "|" : "/");
        u8g2.drawStr(64, 40, servoData.pos3 ? "|" : "/");
        u8g2.drawStr(96, 40, servoData.pos4 ? "|" : "/");
        // show relay
        u8g2.drawStr(0 + 15, 40, frogRelay1On ? "on" : "--");
        u8g2.drawStr(32 + 15, 40, frogRelay2On ? "on" : "--");
        u8g2.drawStr(64 + 15, 40, frogRelay3On ? "on" : "--");
        u8g2.drawStr(96 + 15, 40, frogRelay4On ? "on" : "--");
      }
      else
      {
        // show min
        char s1MaxStr[20]; snprintf(s1MaxStr, 20, "%d", servoData.min1); u8g2.drawStr(0,  20, s1MaxStr);
        char s2MaxStr[20]; snprintf(s2MaxStr, 20, "%d", servoData.min2); u8g2.drawStr(32, 20, s2MaxStr);
        char s3MaxStr[20]; snprintf(s3MaxStr, 20, "%d", servoData.min3); u8g2.drawStr(64, 20, s3MaxStr);
        char s4MaxStr[20]; snprintf(s4MaxStr, 20, "%d", servoData.min4); u8g2.drawStr(96, 20, s4MaxStr);
        // show target
        u8g2.drawStr(0,  30, servoData.pos1 ? "^^^" : "vvv");
        u8g2.drawStr(32, 30, servoData.pos2 ? "^^^" : "vvv");
        u8g2.drawStr(64, 30, servoData.pos3 ? "^^^" : "vvv");
        u8g2.drawStr(96, 30, servoData.pos4 ? "^^^" : "vvv");
        // show max
        char s1MinStr[20]; snprintf(s1MinStr, 20, "%d", servoData.max1); u8g2.drawStr(0,  40, s1MinStr);
        char s2MinStr[20]; snprintf(s2MinStr, 20, "%d", servoData.max2); u8g2.drawStr(32, 40, s2MinStr);
        char s3MinStr[20]; snprintf(s3MinStr, 20, "%d", servoData.max3); u8g2.drawStr(64, 40, s3MinStr);
        char s4MinStr[20]; snprintf(s4MinStr, 20, "%d", servoData.max4); u8g2.drawStr(96, 40, s4MinStr);
      }
      // adress, speed, initial delay
      char adressStr[20]; snprintf(adressStr, 20, "a=%d", servoData.adress); u8g2.drawStr(0,  50, adressStr);
      char deltaStr[20]; snprintf(deltaStr, 20, "dt=%d", servoData.deltaMove); u8g2.drawStr(32,  50, deltaStr);
      char initDelayStr[20]; snprintf(initDelayStr, 20, "id=%d", servoData.initDelay); u8g2.drawStr(64,  50, initDelayStr);
      char selectedStr[2] = "-";
      if (0 == selectedServo)
      {
        sprintf(selectedStr, "*");
      }
      else
      {
        sprintf(selectedStr, "%d", selectedServo);
      }
      u8g2.drawStr(120, 50, selectedStr);
    }
    // version
    u8g2.drawStr(0, 64, "DCC");
    u8g2.drawStr(32, 64, VERSION);
    // turn around the clock
    switch (aLiveClockCounter % 4)
    {
      case 0: u8g2.drawStr(64, 64, "|"); break;
      case 1: u8g2.drawStr(64, 64, "/"); break;
      case 2: u8g2.drawStr(64, 64, "-"); break;
      case 3: u8g2.drawStr(64, 64, "\\"); break;
    }
    aLiveClockCounter++;
    // me
    u8g2.drawStr(80, 64, "(c)EMM");
    // update buffer
    u8g2.sendBuffer();
    lastDisplayTime = millis();
  }
}

void progButtonTask(void)
{
  if ((millis() - lastProgBtnTaskTime) > 5)
  {
    if (digitalRead(progBtnPin))
    {
      if (0 < progBtnPinCount)
      {
        progBtnPinCount--;
      }
      else
      {
        progBtnPinCount = 0;
      }
    }
    else
    {
      if (DEBOUNCE_LIMIT > progBtnPinCount)
      {
        progBtnPinCount++;
      }
      else
      {
        progBtnPinCount = DEBOUNCE_LIMIT;
      }
    }
    progBtnPinActive = progBtnPinCount > DEBOUNCE_LEVEL;
    if (progBtnPinActive && (progBtnPinActive != progBtnPinActiveOld))
    {
      decoderState = stateSelection;
      selectedServo++;
      if (selectedServo > 4)
      {
        selectedServo = 0;
        decoderState = stateIdle;
      }
      Serial.print("Prog. Button edge detected. SelectedServo = ");
      Serial.println(selectedServo);
    }
    progBtnPinActiveOld = progBtnPinActive;
    lastProgBtnTaskTime = millis();
  }
}

void modifySelServo(int servo, bool upperLimit, bool up)
{
  int delta = 10;
  if (up) delta = -delta;
  if (upperLimit)
  {
    switch (servo)
    {
      case 1: servoData.max1 += delta; setSelectedServoPos(1, false); break;
      case 2: servoData.max2 += delta; setSelectedServoPos(2, false); break;
      case 3: servoData.max3 += delta; setSelectedServoPos(3, false); break;
      case 4: servoData.max4 += delta; setSelectedServoPos(4, false); break;
      default: break;
    }
  }
  else
  {
    switch (servo)
    {
      case 1: servoData.min1 += delta; setSelectedServoPos(1, true); break;
      case 2: servoData.min2 += delta; setSelectedServoPos(2, true); break;
      case 3: servoData.min3 += delta; setSelectedServoPos(3, true); break;
      case 4: servoData.min4 += delta; setSelectedServoPos(4, true); break;
      default: break;
    }
  }
}

void keyboardTask()
{
  if ((millis() - lastKeybTaskTime) > 5)
  {
    bool success = false;
    byte buttons = I2CRead(0x20, success);

    #define DEBOUNCE_BUTTON(state, name, action)                                                         \
      bool btnActive##name = false;                                                                      \
      if (state)                                                                                         \
      {                                                                                                  \
        if (0 < btnCount##name) btnCount##name--; else btnCount##name = 0;                               \
      }                                                                                                  \
      else                                                                                               \
      {                                                                                                  \
        if (DEBOUNCE_LIMIT > btnCount##name) btnCount##name++; else btnCount##name = DEBOUNCE_LIMIT;     \
      }                                                                                                  \
      btnActive##name = btnCount##name > DEBOUNCE_LEVEL;                                                 \
      if (btnActive##name && (btnActive##name != btnActiveOld##name))                                    \
      {                                                                                                  \
        action;                                                                                          \
      }                                                                                                  \
      btnActiveOld##name = btnActive##name;
      // use macro
      if (success)
      {
        DEBOUNCE_BUTTON(bitRead(buttons, 0), S1, Serial.println("S1 pressed"); (0 == selectedServo) ? setSelectedServoPos(1, false) : modifySelServo(selectedServo, false, true))
        DEBOUNCE_BUTTON(bitRead(buttons, 1), S2, Serial.println("S2 pressed"); (0 == selectedServo) ? setSelectedServoPos(1, true) : modifySelServo(selectedServo, false, false))
        DEBOUNCE_BUTTON(bitRead(buttons, 2), S3, Serial.println("S3 pressed"); (0 == selectedServo) ? setSelectedServoPos(2, false) : modifySelServo(selectedServo, true, true))
        DEBOUNCE_BUTTON(bitRead(buttons, 3), S4, Serial.println("S4 pressed"); (0 == selectedServo) ? setSelectedServoPos(2, true) : modifySelServo(selectedServo, true, false))
        DEBOUNCE_BUTTON(bitRead(buttons, 4), S5, Serial.println("S5 pressed"); (0 == selectedServo) ? setSelectedServoPos(3, false) : modifySelServo(selectedServo, false, true))
        DEBOUNCE_BUTTON(bitRead(buttons, 5), S6, Serial.println("S6 pressed"); (0 == selectedServo) ? setSelectedServoPos(3, true) : modifySelServo(selectedServo, false, false))
        DEBOUNCE_BUTTON(bitRead(buttons, 6), S7, Serial.println("S7 pressed"); (0 == selectedServo) ? setSelectedServoPos(4, false) : modifySelServo(selectedServo, true, true))
        DEBOUNCE_BUTTON(bitRead(buttons, 7), S8, Serial.println("S8 pressed"); (0 == selectedServo) ? setSelectedServoPos(4, true) : modifySelServo(selectedServo, true, false))
      }
    #undef DEBOUNCE_BUTTON
    lastKeybTaskTime = millis();
  }
}

void setup()
{
  initialiseOutputsAndPins();
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  // say hello on UART
  helloUart();
  // init the Display
  u8g2.begin();
  helloDisplay();
  // try to read data from eeprom
  readFromEEprom();
  fixServoPosition();
  // some decoder data
  accCmd.myMaster = Roco;
  accCmd.setMyAddress(servoData.adress);
  //wait some defined time before servos are activated
  delay(servoData.initDelay);
  initialiseServos();
  scanI2CBus();
}

void loop()
{
  dccInterfaceTask();
  servoTask();
  ledTask();
  serialInterfaceTask();
  selftestTask();
  displayTask();
  progButtonTask();
  keyboardTask();
}
