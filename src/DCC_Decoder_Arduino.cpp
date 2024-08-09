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
// uses:
// - the DCC library from Aiko Pras (AP_DCC_library)
// - the display library from Oliver Kraus (U8g2lib).
// - the statistic library from Rob Tillaart
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
//     - debug mode and statistic, add middle pos in program mode, fix workflow, fix display
//   2024_06_28 V1.2 EMM: Servo options, display improvement
//     - add error handling, improve validation
//     - add turnout type and care display
//     - add servo mode, new menu structure
//     - reduce code size
//   2024_07_21 V1.3 EMM: some bugfixes
//     - fix errorhandling, proceed with prog button in case
//
//******************************************************************************************************
#define VERSION "v1.3"
#define DESCSTR1 "DCC tournout decoder"
#define DESCSTR2 "(c)EMM"

#include <Arduino.h>
#include <AP_DCC_library.h>
#include <Servo.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "Statistic.h"

#define MIN_SERVO_VALUE 700
#define MAX_SERVO_VALUE 2300

// configure the display (AZDelivery 0,96 Zoll OLED Display I2C SSD1306 Chip 128 x 64)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

enum displayStatusEnum
  { displayStatus_clear
  , displayStatus_fill
  , displayStatus_show
  };
displayStatusEnum displayStatus = displayStatus_clear;

enum errorEnum
  { error_none
  , error_readEepromFailed
  , error_validationFailed
  , error_validationFixed
  };
errorEnum error = error_none;

const uint8_t dccPin = 2;
const uint8_t progBtnPin = 3;
const uint8_t servoPin[4] = {14, 15, 16, 17};
const uint8_t servGndPin[4] = {4, 6, 9, 20};
const uint8_t relayPin[4] = {5, 7, 10, 21};

extern Dcc dcc;                  // This object is instantiated in DCC_Library.cpp
extern Accessory accCmd;         // To retrieve data from accessory commands

Servo servo[4];

uint16_t ledPattern = 0x0101;
int ledCount = 0;
enum decoderStates
  { stateIdle
  , stateSelection
  };
decoderStates decoderState = stateIdle;
int eepromResetValue = 0xAA;
bool saveToEepromWheneverThereIsTime = false;
int locoAdress = 9999;
int selectedServo = 0;
enum progPage
  { progPageServoLimits
  , progPageAdrDelta
  , progPageTypeMode
  };
progPage selectedPage = progPageServoLimits;
bool servoTaskActive = true;
int initDelay = 0;

int selftestIteration = 0;
bool selftestRunning = false;

bool debugMode = false;

int resetAddress = 0;
int dataAddress = 10;

String serialBuffer = "";
String serialToken1 = "";
String serialToken2 = "";
String serialToken3 = "";
String serialToken4 = "";
String serialToken5 = "";
String *serialTokens[] = { &serialToken1, &serialToken2, &serialToken3, &serialToken4, &serialToken5 };

enum servoPosition
  { servoPositionStraight
  , servoPositionMiddle
  , servoPositionRound
  };
servoPosition lastSelectedServoPos[4] = {servoPositionStraight, servoPositionStraight, servoPositionStraight, servoPositionStraight};  
#define IS_WHERE(where, nr) servoPosition##where == servoData.servos[nr].pos

enum servoModeEnum
  { servoMode_AlwaysSignalSwitchGnd
  , servoMode_AlwaysSignalAlwaysGnd
  , servoMode_SwitchSignalSwitchGnd
  , servoMode_SwitchSignalAlwaysGnd
  };

enum turnoutTypeEnum
  { turnoutType_TurnoutRight
  , turnoutType_TurnoutLeft
  , turnoutType_DoubleCrossing
  , turnoutType_SingleCrossingRight
  , turnoutType_SingleCrossingLeft
  };

struct servoDataStructElement
{
  servoPosition pos = servoPositionStraight;
  bool moving = false;
  int min = 1200;
  int max = 1800;
  int val = 1200;
  servoModeEnum servoMode = servoMode_AlwaysSignalSwitchGnd;
  turnoutTypeEnum turnoutType = turnoutType_TurnoutRight;
};

struct servoDataStruct
{
  int adress = 24; // Decoder 24 is switch 97..100
  int deltaMove = 20;
  servoDataStructElement servos[4];
  int initDelay = 1000;
  int checkByte = 0xAA;
};
servoDataStruct servoData;

int posCounter[4] = {0, 0, 0, 0};

int progBtnPinCount = 5;
bool progBtnPinActive = false;
bool progBtnPinActiveOld = false;
#define DEBOUNCE_LIMIT 10
#define DEBOUNCE_LEVEL DEBOUNCE_LIMIT/2

#define ADD_BUTTON(name)            \
  bool btnActive##name = false;     \
  bool btnActiveOld##name = false;  \
  int btnCount##name = 0;
// use macro
ADD_BUTTON(S1);
ADD_BUTTON(S2);
ADD_BUTTON(S3);
ADD_BUTTON(S4);
ADD_BUTTON(S5);
ADD_BUTTON(S6);
ADD_BUTTON(S7);
ADD_BUTTON(S8);

bool frogRelayOn[4] = { false, false, false, false };

typedef statistic::Statistic<float, uint32_t, false> typeStats;
// some statistics
typeStats loopOverallStats;
unsigned long loopOverallLastCallTime = 0;
// ... for tasks
typeStats dccInterfaceTaskStats;
unsigned long dccInterfaceTaskLastCallTime = 0;
typeStats servoTaskStats;
unsigned long servoTaskLastCallTime = 0;
unsigned long lastServoTime = 0;
typeStats ledTaskStats;
unsigned long ledTaskLastCallTime = 0;
unsigned long lastLedTime = 0;
typeStats serialInterfaceTaskStats;
unsigned long serialInterfaceTaskLastCallTime = 0;
typeStats selftestTaskStats;
unsigned long selftestTaskLastCallTime = 0;
unsigned long lastSelftestTime = 0;
typeStats displayTaskStats;
unsigned long displayTaskLastCallTime = 0;
unsigned long lastDisplayTime = 0;
typeStats progButtonTaskStats;
unsigned long progButtonTaskLastCallTime = 0;
unsigned long lastProgBtnTaskTime = 0;
typeStats keyboardTaskStats;
unsigned long keyboardTaskLastCallTime = 0;
unsigned long lastKeybTaskTime = 0;
typeStats asyncMemoryTaskStats;
unsigned long asyncMemoryTaskLastCallTime = 0;
unsigned long lastAsyncMemoryTaskTime = 0;

// definitions
void saveToEEprom();
void resetServoData();
char* describeServoMode(servoModeEnum mode);
void serialPrintServoMode(servoModeEnum mode);
void serialPrintTurnoutType(turnoutTypeEnum turnout);
void printServoData(int servo);
void validateSettings();
void readFromEEprom();
void selftestTask();
void serialInterfaceTask();
void ledTask();
void fixServoPosition();
void initialiseOutputsAndPins();
void initialiseServos();
void printHelp();
void getSerialTokens();
void setSelectedServoPos(int selServo, servoPosition newValue, bool saveState);
void careSelectedServo();
void setSelectedServoMax(int selServo, int newValue);
void setSelectedServoMin(int selServo, int newValue);
void setSelectedServoVal(int selServo, int newValue);
void analyseSerialCommand();
void helloUart();
void helloDisplay();
void showStatisticsOnSerial();
void I2CWrite(int adress, byte data);
byte I2CRead(int adress, bool& success);
void scanI2CBus();
void dccInterfaceTask();
void careServo(int number, int target, servoPosition position);
void servoTask();
void displayTask();
void displayError();
void showTurnoutPosition(int number, u8g2_uint_t xPos, u8g2_uint_t yPos, turnoutTypeEnum type);
void showServoMode(u8g2_uint_t xPos, u8g2_uint_t yPos, servoModeEnum mode);
void displayFillContent();
void progButtonTask();
void modifySelServo(servoPosition pos, bool directionUp);
void modifyDeltaMove(int val);
void modifyInitDelay(int val);
void selectPrevPage();
void selectNextPage();
void modifyAdress(int delta);
void selectPrevType();
void selectNextType();
void selectPrevMode();
void selectNextMode();
void pressedButton1();
void pressedButton2();
void pressedButton3();
void pressedButton4();
void pressedButton5();
void pressedButton6();
void pressedButton7();
void pressedButton8();
void keyboardTask();
void asyncMemoryTask();
void setup();
void loop();

template <typename T>
bool rangeCheck(T value, T min, T max)
{
  return ((value >= min)&&(value <= max));
}

void saveToEEprom()
{
  unsigned long saveTimeMeas = micros();
  EEPROM.put(dataAddress, servoData);
  EEPROM.put(resetAddress, 0xAA);
  saveTimeMeas = micros() - saveTimeMeas;             \
  Serial.print(sizeof(servoData));
  Serial.print(" bytes -> EEProm! t=");
  Serial.print(saveTimeMeas);
  Serial.println("us");
}

void resetServoData()
{
  servoData.adress = 24;
  servoData.deltaMove = 20;

  for (int servoNumber = 0; servoNumber < 4; servoNumber++)
  {
    servoData.servos[servoNumber].pos = servoPositionStraight;
    servoData.servos[servoNumber].moving = false;
    servoData.servos[servoNumber].min = 1200;
    servoData.servos[servoNumber].max = 1800;
    servoData.servos[servoNumber].val = 1200;
    servoData.servos[servoNumber].servoMode = servoMode_AlwaysSignalSwitchGnd;
    servoData.servos[servoNumber].turnoutType = turnoutType_TurnoutRight;
  }

  servoData.initDelay = 1000;
  servoData.checkByte = 0xAA;

  saveToEEprom();
}

char* describeServoMode(servoModeEnum mode)
{
  switch (mode)
  {
    case servoMode_AlwaysSignalSwitchGnd: return "AlSSwG"; break;
    case servoMode_AlwaysSignalAlwaysGnd: return "AlSAlG"; break;
    case servoMode_SwitchSignalSwitchGnd: return "SwSSwG"; break;
    case servoMode_SwitchSignalAlwaysGnd: return "SwSAlG"; break;
  }
}

void serialPrintServoMode(servoModeEnum mode)
{
  Serial.print(describeServoMode(mode));
}
  
void serialPrintTurnoutType(turnoutTypeEnum turnout)
{
  switch (turnout)
  { 
    case turnoutType_TurnoutRight: Serial.print("TrnR"); break;
    case turnoutType_TurnoutLeft: Serial.print("TrnL"); break;
    case turnoutType_DoubleCrossing: Serial.print("DblC"); break;
    case turnoutType_SingleCrossingRight: Serial.print("CrsR"); break;
    case turnoutType_SingleCrossingLeft: Serial.print("CrsL"); break;
  }
}

void printServoData()
{
  for (int servoNumber = 0; servoNumber < 4; servoNumber++)
  {
    Serial.print("Servo #");
    Serial.print(servoNumber);
    Serial.print("; pos=");
    Serial.print(servoData.servos[servoNumber].pos ? "true " : "false");
    Serial.print("; min=");
    Serial.print(servoData.servos[servoNumber].min);
    Serial.print("; max=");
    Serial.print(servoData.servos[servoNumber].max);
    Serial.print("; val=");
    Serial.print(servoData.servos[servoNumber].val);
    Serial.print("; SM=");
    serialPrintServoMode(servoData.servos[servoNumber].servoMode);
    Serial.print("; TT=");
    serialPrintTurnoutType(servoData.servos[servoNumber].turnoutType);
    Serial.println(";");
  }
}

bool rangeCheckMaxMin(int& nameOfValue, int minValue, int maxValue)
{
  bool modified = false; 
  if (minValue > nameOfValue) { nameOfValue = minValue; modified = true; }
  if (maxValue < nameOfValue) { nameOfValue = maxValue; modified = true; }
  return modified;
}

void validateSettings()
{
  if (0xAA == servoData.checkByte)
  {
    bool modified = false;
    for (int servoNumber = 0; servoNumber < 4; servoNumber++)
    {
      modified = modified || rangeCheckMaxMin(servoData.servos[servoNumber].min, MIN_SERVO_VALUE, MAX_SERVO_VALUE);
      modified = modified || rangeCheckMaxMin(servoData.servos[servoNumber].max, MIN_SERVO_VALUE, MAX_SERVO_VALUE);
      rangeCheckMaxMin(servoData.servos[servoNumber].val, MIN_SERVO_VALUE, MAX_SERVO_VALUE);
    }
    modified = modified || rangeCheckMaxMin(servoData.adress, 0, 50);
    modified = modified || rangeCheckMaxMin(servoData.deltaMove, 5, 50);
    modified = modified || rangeCheckMaxMin(servoData.initDelay, 1, 10000);
    initDelay = servoData.initDelay;
    if (modified)
    {
      Serial.println("Modified!");
      error = error_validationFixed;
    }
  }
  else
  {
    Serial.print("checkByte=");
    Serial.println(servoData.checkByte);
    error = error_validationFailed;
    for (int servoNumber = 0; servoNumber < 4; servoNumber++) { servoData.servos[servoNumber].moving = false; }
  }
  // ... and show what we got
  printServoData();
  Serial.print("1st acc (module) adress=");
  Serial.print(servoData.adress);
  Serial.print("; deltaMove=");
  Serial.print(servoData.deltaMove);
  Serial.print("us/50ms; initDelay=");
  Serial.print(servoData.initDelay);
  Serial.println("ms");
}

void readFromEEprom()
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
    error = error_readEepromFailed;
    Serial.println("EEProm invalid. Reset!");
  }
  //range check read values
  validateSettings();
}

void selftestTask()
{
  if ((selftestRunning) && (millis() - lastSelftestTime ) > 500)
  { // iterate
    switch (selftestIteration)
    { //indicate
      case 0:
        servoTaskActive = false;
        digitalWrite(LED_BUILTIN, HIGH);
        break;
      case 1:  digitalWrite(LED_BUILTIN, LOW);  break;
      case 2:  digitalWrite(LED_BUILTIN, HIGH); break;
      case 3:  digitalWrite(LED_BUILTIN, LOW);  break;
      //switch relays
      case 4:  digitalWrite(relayPin[0], HIGH); break;
      case 5:  digitalWrite(relayPin[1], HIGH); break;
      case 6:  digitalWrite(relayPin[2], HIGH); break;
      case 7:  digitalWrite(relayPin[3], HIGH); break;
      case 8:  digitalWrite(relayPin[0], LOW);  break;
      case 9:  digitalWrite(relayPin[1], LOW);  break;
      case 10: digitalWrite(relayPin[2], LOW);  break;
      case 11: digitalWrite(relayPin[3], LOW);  break;
      //all to the left
      case 12:
        servoTaskActive = true;
        setSelectedServoPos(1, servoPositionRound, false);
        break;
      case 13: setSelectedServoPos(2, servoPositionRound, false); break;
      case 14: setSelectedServoPos(3, servoPositionRound, false); break;
      case 15: setSelectedServoPos(4, servoPositionRound, false); break;
      //all to the right
      case 16: setSelectedServoPos(1, servoPositionStraight, false); break;
      case 17: setSelectedServoPos(2, servoPositionStraight, false); break;
      case 18: setSelectedServoPos(3, servoPositionStraight, false); break;
      case 19: setSelectedServoPos(4, servoPositionStraight, false); break;
      // end this sequence
      case 20:
        selftestRunning = false;
        readFromEEprom();
        fixServoPosition();
        Serial.println("... done!");
        break;
      default:
        break;
    }
    selftestIteration++;
    lastSelftestTime = millis();
  }
}

void serialInterfaceTask()
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

void ledTask()
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

void fixServoPosition()
{
  // fix position
  for (int servoNumber = 0; servoNumber < 4; servoNumber++)
  {
    if (servoPositionStraight == servoData.servos[servoNumber].pos)
    {
      servoData.servos[servoNumber].val = servoData.servos[servoNumber].min;
    } else {
      servoData.servos[servoNumber].val = servoData.servos[servoNumber].max;
    }
  }
}

void initialiseOutputsAndPins()
{
  // care about output defaults
  for (int servoNumber = 0; servoNumber < 4; servoNumber++)
  {
    pinMode(servGndPin[servoNumber], OUTPUT);
    pinMode(relayPin[servoNumber], OUTPUT);
    digitalWrite(servGndPin[servoNumber], LOW);
    digitalWrite(relayPin[servoNumber], LOW);
  }
  dcc.attach(dccPin);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(progBtnPin, INPUT_PULLUP);
}

void initialiseServos()
{
  for (int servoNumber = 0; servoNumber < 4; servoNumber++)
  { //prepare servos
    servo[servoNumber].attach(servoPin[servoNumber]);
    //write given values
    servo[servoNumber].writeMicroseconds(servoData.servos[servoNumber].val);
  }
}

void printHelp()
{
  Serial.println("available commands (followed by enter):");
  Serial.println("set;<type>;<servo_nr>;<value> ... set some servo values");
  Serial.println("  type=min: min pos [1000..2000]");
  Serial.println("  type=max: max pos [1000..2000]");
  Serial.println("  type=val: actual pos [1000..2000]");
  Serial.println("  type=smo: servo mode [0..3]");
  Serial.println("  type=tty: turnout mode [0..4]");
  Serial.println("set;<type>;<value> set other values");
  Serial.println("  type=adr: first adress of servo");
  Serial.println("  type=dlt: microsecs per step (50ms)");
  Serial.println("  type=dly: initial delay [ms]");
  Serial.println("slv: start selftest");
  Serial.println("rst: reset servo data");
  Serial.println("dbg: debug mode");
  Serial.println("dbr: reset statistic");
  Serial.println("sta: show statistic");
}

void getSerialTokens()
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

void setSelectedServoPos(int selServo, servoPosition newValue, bool saveState)
{
  int counterValue = 5;
  if (!rangeCheck<int>(selServo, 1, 4)) return;
  servoData.servos[selServo-1].pos = newValue;
  posCounter[selServo-1] = counterValue;
  servoData.servos[selServo-1].moving = true;
  Serial.print("Set Servo #");
  Serial.print(selServo);
  Serial.print(" pos=");
  Serial.println(servoPositionMiddle == newValue ? "mid" : (servoPositionStraight == newValue ? "straight" : "round"));
  if (saveState)
  {
    saveToEepromWheneverThereIsTime = true;
  }
}

void setSelectedServoSmo(int selServo, int newValue)
{
  servoData.servos[selServo-1].servoMode = static_cast<servoModeEnum>(newValue);
}

void setSelectedServoTty(int selServo, int newValue)
{
  servoData.servos[selServo-1].turnoutType = static_cast<turnoutTypeEnum>(newValue);
}

void setSelectedServoMax(int selServo, int newValue)
{
  servoData.servos[selServo-1].max = newValue;
  Serial.print("Set Servo #");
  Serial.print(selServo);
  Serial.print(" max=");
  Serial.println(newValue);
}

void setSelectedServoMin(int selServo, int newValue)
{
  servoData.servos[selServo-1].min = newValue;
  Serial.print("Set Servo #");
  Serial.print(selServo);
  Serial.print(" min=");
  Serial.println(newValue);
}

void setSelectedServoVal(int selServo, int newValue)
{
  servoData.servos[selServo-1].val = newValue;
  Serial.print("Set Servo #");
  Serial.print(selServo);
  Serial.print(" val=");
  Serial.println(newValue);
}

servoPosition convertIntToServoPosition(int pos)
{
  switch (pos)
  {
    case 0: return servoPositionRound; break;
    case 1: return servoPositionStraight; break;
    default:
    case 2: return servoPositionMiddle; break;
  }
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
        setSelectedServoPos(value1, servoPositionRound, false);
      }
    }
    else if (serialTokens[0]->equals("min"))
    {
      if ((1 <= value1) && (5 >= value1))
      {
        setSelectedServoMin(value1, serialTokens[2]->toInt());
        setSelectedServoVal(value1, serialTokens[2]->toInt());
        setSelectedServoPos(value1, servoPositionStraight, false);
      }
    }
    else if (serialTokens[0]->equals("val"))
    {
      if ((1 <= value1) && (5 >= value1))
      {
        setSelectedServoVal(value1, serialTokens[2]->toInt());
      }
    }
    else if (serialTokens[0]->equals("smo"))
    {
      if ((0 <= value1) && (3 >= value1))
      {
        setSelectedServoSmo(value1, serialTokens[2]->toInt());
      }
    }
    else if (serialTokens[0]->equals("tty"))
    {
      if ((1 <= value1) && (5 >= value1))
      {
        setSelectedServoTty(value1, serialTokens[2]->toInt());
      }
    }
    else if (serialTokens[0]->equals("pos"))
    {
      if ((1 <= value1) && (5 >= value1))
      {
        setSelectedServoPos(value1, convertIntToServoPosition(serialTokens[2]->toInt()), false);
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
      Serial.print("Unk. Command: ");
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
  else if (serialBuffer.startsWith("dbg"))
  {
    debugMode ? debugMode = false : debugMode = true;
    Serial.print("debug mode = ");
    Serial.println(debugMode ? "ON" : "OFF");
  }
  else if (serialBuffer.startsWith("dbr"))
  { // reset statistics
    loopOverallStats.clear();
    dccInterfaceTaskStats.clear();
    servoTaskStats.clear();
    ledTaskStats.clear();
    serialInterfaceTaskStats.clear();
    selftestTaskStats.clear();
    displayTaskStats.clear();
    progButtonTaskStats.clear();
    keyboardTaskStats.clear();
    asyncMemoryTaskStats.clear();
  }
  else if (serialBuffer.startsWith("sta"))
  {
    showStatisticsOnSerial();
  }
  else if (serialBuffer.startsWith("rst"))
  {
    resetServoData();
  }
  else
  {
    if (serialBuffer.startsWith("\n") || serialBuffer.startsWith("\r"))
    {
      //nothing to do
    }
    else
    {
      Serial.println("Unk. Command!");
    }
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

void helloDisplay()
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

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__
int freeMemory()
{
  char top;
  #ifdef __arm__
    return &top - reinterpret_cast<char*>(sbrk(0));
  #elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
    return &top - __brkval;
  #else  // __arm__
    return __brkval ? &top - __brkval : &top - __malloc_heap_start;
  #endif  // __arm__
}

void printAStat(const char * name, const typeStats stat)
{
  Serial.print( name );
  Serial.print(": AVG=");
  Serial.print(stat.average());
  Serial.print("; MIN=");
  Serial.print(stat.minimum());
  Serial.print("; MAX=");
  Serial.println(stat.maximum());
}

void showStatisticsOnSerial()
{
  //use macro
  printAStat("loopOvr", loopOverallStats);
  printAStat("dccIntT", dccInterfaceTaskStats);
  printAStat("servoT_", servoTaskStats);
  printAStat("ledTask", ledTaskStats);
  printAStat("serialT", serialInterfaceTaskStats);
  printAStat("selftes", selftestTaskStats);
  printAStat("display", displayTaskStats);
  printAStat("progBut", progButtonTaskStats);
  printAStat("keybTas", keyboardTaskStats);
  printAStat("asyncMe", asyncMemoryTaskStats);
  Serial.print("free mem: ");
  Serial.print(freeMemory());
  Serial.println("bytes");
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

void scanI2CBus()
{
  byte error, address;
  int nDevices;
  Serial.println("Scan I2C...");
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
      Serial.print("I2C dev. at 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unk. err at 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C dev found\n");
  else
    Serial.println("done.\n");

  I2CWrite(0x20, 0xff);
}

void dccInterfaceTask()
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
        setSelectedServoPos(accCmd.turnout, accCmd.position == 1 ? servoPositionStraight : servoPositionRound, true);
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

void careServo(int number, int target, servoPosition position)
{
  if (servoData.servos[number].pos == position)
  {
    if (abs(servoData.servos[number].val - target) < servoData.deltaMove)
    {
      servoData.servos[number].moving = false;
      servoData.servos[number].val = target;
    }
    else if (servoData.servos[number].val < target)
    {
      if (0 < posCounter[number]) { posCounter[number]--; } else { servoData.servos[number].val += servoData.deltaMove; }
      servoData.servos[number].moving = true;
    }
    else
    {
      if (0 < posCounter[number]) { posCounter[number]--; } else { servoData.servos[number].val -= servoData.deltaMove; }
      servoData.servos[number].moving = true;
    }
    bool alwaysSignal = (servoData.servos[number].servoMode == servoMode_AlwaysSignalAlwaysGnd) || (servoData.servos[number].servoMode == servoMode_AlwaysSignalSwitchGnd);
    if ((MIN_PULSE_WIDTH <= servoData.servos[number].val) && (MAX_PULSE_WIDTH > servoData.servos[number].val))
    {
      int value = servoData.servos[number].val;
      if (servoData.servos[number].moving) { if (!servo[number].attached()) { servo[number].attach(servoPin[number]); }} else { if (!alwaysSignal) servo[number].detach(); };
      if (servo[number].attached()) servo[number].writeMicroseconds(value);
    }
    frogRelayOn[number] = (servoData.servos[number].val > ((servoData.servos[number].max + servoData.servos[number].min)/2));
    bool alwaysGnd = (servoData.servos[number].servoMode == servoMode_AlwaysSignalAlwaysGnd) || (servoData.servos[number].servoMode == servoMode_SwitchSignalAlwaysGnd);
    digitalWrite(relayPin[number], frogRelayOn[number] ? HIGH : LOW);
    digitalWrite(servGndPin[number], servoData.servos[number].moving || (number+1 == selectedServo) || alwaysGnd ? HIGH : LOW);
  }
}

void servoTask()
{
  if (((millis() - lastServoTime ) > 50) && servoTaskActive)
  {
      for (int servoNumber = 0; servoNumber < 4; servoNumber++)
      { //call for all three positions for every servo
        careServo(servoNumber, servoData.servos[servoNumber].min, servoPositionStraight);
        careServo(servoNumber, servoData.servos[servoNumber].max, servoPositionRound);
        careServo(servoNumber, ((servoData.servos[servoNumber].max + servoData.servos[servoNumber].min)/2), servoPositionMiddle);
      }
      // get rid of macro
    #undef careServo
    lastServoTime = millis();
  }
}

void displayTask()
{
  if ((millis() - lastDisplayTime) > 300)
  {
    if (error != error_none)
    { // in case of error just show that
      displayError();
      return;
    }
    if (servoData.servos[0].moving || servoData.servos[1].moving || servoData.servos[2].moving || servoData.servos[3].moving)
    { // no show when moving
      return;
    }
    // divide into slots to keep load low
    switch (displayStatus)
    {
      case displayStatus_clear:
        u8g2.clearBuffer();
        displayStatus = displayStatus_fill;
        break;
      case displayStatus_fill:
        displayFillContent();
        displayStatus = displayStatus_show;
        break;
      case displayStatus_show:
        u8g2.sendBuffer();
        displayStatus = displayStatus_clear;
        break;
    }
    lastDisplayTime = millis();
  }
}

void displayError()
{
  u8g2.clearBuffer();
  u8g2.drawStr(0,  8, "ERROR:");
  const char * string1 = "Read EEProm failed!";
  const char * string2 = "Validation failed!";
  const char * string3 = "Validation fixed!";
  switch (error)
  {
    default:
    case error_none:
      break;
    case error_readEepromFailed:
      u8g2.drawStr(0,  25, string1);
      Serial.println(string1);
      break;
    case error_validationFailed:
      u8g2.drawStr(0,  25, string2);
      Serial.println(string2);
      break;
    case error_validationFixed:
      u8g2.drawStr(0,  25, string3);
      Serial.println(string3);
      break;
  }
  u8g2.drawStr(0,  40, "Prog to proceed!");
  u8g2.sendBuffer();
}

void showTurnoutPosition(int number, u8g2_uint_t xPos, u8g2_uint_t yPos, turnoutTypeEnum type)
{
  bool isRound = IS_WHERE(Round, number-1);
  bool isStraight = IS_WHERE(Straight, number-1);
  bool isMiddle = IS_WHERE(Middle, number-1);
  switch(type)
  {
    case turnoutType_TurnoutLeft: 
      u8g2.drawStr(isRound ? xPos + 4 : xPos , yPos, isStraight ? "||" : "//");
      u8g2.drawStr(xPos,  yPos+8, isMiddle ? "><" : isStraight ? "||" : "//");
      break;
    case turnoutType_TurnoutRight: 
      u8g2.drawStr(xPos,  yPos, isMiddle ? "><" : isStraight ? "||" : "\\\\");
      u8g2.drawStr(isRound ? xPos + 4 : xPos , yPos+8, isStraight ? "||" : "\\\\");
      break;
    case turnoutType_DoubleCrossing: 
      u8g2.drawStr(xPos,  yPos, isStraight ? "\\\\//" : "//\\\\");
      u8g2.drawStr(xPos,  yPos+8, isStraight ? "//\\\\" : "\\\\//");
      break;
    case turnoutType_SingleCrossingLeft: 
      u8g2.drawStr(xPos,  yPos, isStraight ? "\\\\||" : "//||");
      u8g2.drawStr(xPos,  yPos+8, isStraight ? "//||" : "\\\\||");
      break;
    case turnoutType_SingleCrossingRight: 
      u8g2.drawStr(xPos,  yPos, isStraight ? "||\\\\" : "||//");
      u8g2.drawStr(xPos,  yPos+8, isStraight ? "||//" : "||\\\\");
      break;
  }
}

void showServoMode(const u8g2_uint_t xPos, const u8g2_uint_t yPos, const servoModeEnum mode)
{
  u8g2.drawStr(xPos,  yPos, "Signal GND");
  switch (mode)
  {
    case servoMode_AlwaysSignalSwitchGnd: u8g2.drawStr(xPos, yPos, "Alws   Sw"  ); break;
    case servoMode_AlwaysSignalAlwaysGnd: u8g2.drawStr(xPos, yPos, "Alws   Alws"); break;
    case servoMode_SwitchSignalSwitchGnd: u8g2.drawStr(xPos, yPos, "Turn   Sw"  ); break;
    case servoMode_SwitchSignalAlwaysGnd: u8g2.drawStr(xPos, yPos, "Turn   Alws"); break;
  }
}

void displayFillContent()
{
  static unsigned int aLiveClockCounter = 0;
  static u8g2_uint_t dw = u8g2.getWidth();
  static u8g2_uint_t dw2 = u8g2.getWidth() / 2;

  if (debugMode)
  { // show debug screen
    #define SHOW_STATS_FOR(name, title, x_value, y_value, width)                                         \
      u8g2.drawStr(x_value, y_value, title);                                                             \
      char name##StatsStr[20];                                                                           \
      if      (0 == aLiveClockCounter % 3) { dtostrf(name##Stats.average(),     8, 0, name##StatsStr); } \
      else if (1 == aLiveClockCounter % 3) { dtostrf(name##Stats.maximum(),     8, 0, name##StatsStr); } \
      else if (2 == aLiveClockCounter % 3) { dtostrf(name##Stats.minimum(),     8, 0, name##StatsStr); } \
      u8g2.drawStr(width - u8g2.getStrWidth(name##StatsStr), y_value, name##StatsStr);
    // use macro
    SHOW_STATS_FOR(loopOverall,         "OV",   0,   12, dw2-5);
    SHOW_STATS_FOR(dccInterfaceTask,    "DCC",  0,   22, dw2-5);
    SHOW_STATS_FOR(servoTask,           "SERV", 0,   32, dw2-5);
    SHOW_STATS_FOR(ledTask,             "LED",  0,   42, dw2-5);
    SHOW_STATS_FOR(serialInterfaceTask, "COM",  0,   52, dw2-5);
    SHOW_STATS_FOR(selftestTask,        "SELF", dw2, 22, dw   );
    SHOW_STATS_FOR(displayTask,         "DIS",  dw2, 32, dw   );
    SHOW_STATS_FOR(progButtonTask,      "PRG",  dw2, 42, dw   );
    SHOW_STATS_FOR(keyboardTask,        "KEY",  dw2, 52, dw   );
    // show the type
    char typeStr[10];
    if      (0 == aLiveClockCounter % 3) { snprintf(typeStr, 10, "%s", "AVG [us]"); }
    else if (1 == aLiveClockCounter % 3) { snprintf(typeStr, 10, "%s", "MAX [us]"); }
    else if (2 == aLiveClockCounter % 3) { snprintf(typeStr, 10, "%s", "MIN [us]"); }
    u8g2.drawStr(dw - u8g2.getStrWidth(typeStr), 10, typeStr);
  }
  else if (selftestRunning)
  { // show selftest screen
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
    if (0 == selectedServo)
    { // show the standard screen
      for (int servoNumber = 0; servoNumber < 4; servoNumber++)
      {
        u8g2_uint_t pos = servoNumber * 32;
        char servoPosStr[5]; snprintf(servoPosStr, 5, "S%d__", servoNumber); u8g2.drawStr(pos,  8, servoPosStr);
        char s1PosStr[20]; snprintf(s1PosStr, 20, "%d", servoData.servos[servoNumber].val); u8g2.drawStr(pos,  20, s1PosStr);
        showTurnoutPosition(servoNumber+1, pos, 33, servoData.servos[servoNumber].turnoutType);
      }
      char adressStr[20]; snprintf(adressStr, 20, "a=%d", servoData.adress); u8g2.drawStr(0,  53, adressStr);
      char deltaStr[20]; snprintf(deltaStr, 20, "dt=%d", servoData.deltaMove); u8g2.drawStr(32,  53, deltaStr);
      char initDelayStr[20]; snprintf(initDelayStr, 20, "id=%d", servoData.initDelay); u8g2.drawStr(64,  53, initDelayStr);
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
      // me
      u8g2.drawStr(80, 64, "(c)EMM");
    }
    else
    { // show programming screens
      auto showServo = (0 == aLiveClockCounter%2);
      if (showServo)
      {
        char servoPosStr[5]; snprintf(servoPosStr, 5, "S%d Selected", selectedServo); u8g2.drawStr(0,  8, servoPosStr);
      }
      if (progPageServoLimits == selectedPage)
      { // Servo limits
        for (int servoNumber = 0; servoNumber < 4; servoNumber++)
        {
          u8g2_uint_t pos = servoNumber * 32;
          char s1MaxStr[20]; snprintf(s1MaxStr, 20, "%d", servoData.servos[servoNumber].min); u8g2.drawStr(pos,  28, s1MaxStr);
          u8g2.drawStr(pos,  39, IS_WHERE(Middle, servoNumber) ? "--M--" : (IS_WHERE(Round, servoNumber) ? " \\/\\/" : " /\\/\\"));
          char s1MinStr[20]; snprintf(s1MinStr, 20, "%d", servoData.servos[servoNumber].max); u8g2.drawStr(pos,  50, s1MinStr);
        }
        if (!showServo) u8g2.drawStr(0,  8, "POSITION");
        u8g2.drawStr(0,  62, "Left");
        u8g2.drawStr(32, 62, "Rght");
      }
      if (progPageAdrDelta == selectedPage)
      { // Adress and delta
        if (!showServo) u8g2.drawStr(0,  8, "DCC | DELTA");
        u8g2.drawStr(0,  62, "Adr");
        u8g2.drawStr(32, 62, "Dlt");
        char adressStr[20]; snprintf(adressStr, 20, "a=%d",  servoData.adress);    u8g2.drawStr(0,  41, adressStr);
        char deltaStr[20];  snprintf(deltaStr,  20, "dt=%d", servoData.deltaMove); u8g2.drawStr(32, 41, deltaStr);
      }
      if (progPageTypeMode == selectedPage)
      { // Type und Mode
        if (!showServo) u8g2.drawStr(0,  8, "TYPE | MODE");
        u8g2.drawStr(0,  62, "Type");      
        u8g2.drawStr(32, 62, "Mode");
        if (0 < selectedServo)
        {
        showTurnoutPosition(selectedServo, 0, 41, servoData.servos[selectedServo-1].turnoutType);
        showServoMode(32, 41, servoData.servos[selectedServo-1].servoMode);
        }
      }
      u8g2.drawStr(93, 8, "Cancel");
      u8g2.drawStr(64, 62, "Page");
      u8g2.drawStr(96, 62, "Save");
    }
  }
  aLiveClockCounter++;
}

void careSelectedServo()
{
  // start programming if no servo is selected
  if (0 == selectedServo) 
  {
    decoderState = stateSelection;
  }
  // restore last selectet Servo position
  if (0 < selectedServo) 
  {
    servoData.servos[selectedServo-1].pos = lastSelectedServoPos[selectedServo-1];
  }
  // select next Servo
  selectedServo++;
  // if all servos are done leave prog mode
  if (selectedServo > 4)
  {
    selectedServo = 0;
    decoderState = stateIdle;
    saveToEepromWheneverThereIsTime = true;
  }
  // remember old servo position and set selected to middle position
  if (0 < selectedServo) 
  {
    lastSelectedServoPos[selectedServo-1] = servoData.servos[selectedServo-1].pos;
    setSelectedServoPos(selectedServo, servoPositionMiddle, false);
  }
}

void progButtonTask()
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
      if (error_none == error)
      {
        careSelectedServo();
        Serial.print("Prog -> SelServ=");
        Serial.println(selectedServo);
      }
      else if (error_validationFailed == error)
      {
        resetServoData();
      }
      else if (error_validationFixed == error)
      {
        saveToEEprom();
      }
    }
    progBtnPinActiveOld = progBtnPinActive;
    lastProgBtnTaskTime = millis();
  }
}

void modifySelServo(servoPosition pos, bool directionUp)
{
  static servoPosition lastPos = servoPositionRound;
  int delta = 10;
  if (!directionUp) delta = -delta;
  if (lastPos == pos)
  {
    if (servoPositionRound == pos)
    {
      servoData.servos[selectedServo-1].max += delta;
    }
    if (servoPositionStraight == pos)
    {
      servoData.servos[selectedServo-1].min += delta;
    }
    rangeCheckMaxMin(servoData.servos[selectedServo-1].min, MIN_SERVO_VALUE, MAX_SERVO_VALUE);
    rangeCheckMaxMin(servoData.servos[selectedServo-1].max, MIN_SERVO_VALUE, MAX_SERVO_VALUE);
  }
  setSelectedServoPos(selectedServo, pos, false);
  lastPos = pos;
}

void modifyDeltaMove(int val)
{
  servoData.deltaMove += val;
}

void modifyInitDelay(int val)
{
  servoData.initDelay += val;
}

void selectPrevPage()
{
  if (progPageServoLimits == selectedPage)
  { // rotate
    selectedPage = progPageTypeMode;
  }
  else
  { // fair enough
    selectedPage = (progPage)((int)selectedPage - 1);
  }
}

void selectNextPage()
{
  if (progPageTypeMode == selectedPage)
  { // rotate
    selectedPage = progPageServoLimits;
  }
  else
  { // fair enough
    selectedPage = (progPage)((int)selectedPage + 1);
  }
}

void modifyAdress(int delta)
{
  servoData.adress += delta;
  if (servoData.adress < 0) servoData.adress = 0;
  if (servoData.adress > 80) servoData.adress = 80;
}

void selectPrevType()
{
  if (0 != selectedServo)
  { 
    auto type = servoData.servos[selectedServo-1].turnoutType;
    if (0 < type)
    {
      servoData.servos[selectedServo-1].turnoutType = (turnoutTypeEnum)((int)type - 1);
    }
  }
}

void selectNextType()
{
  if (0 != selectedServo)
  { 
    auto type = servoData.servos[selectedServo-1].turnoutType;
    if (4 > type)
    {
      servoData.servos[selectedServo-1].turnoutType = (turnoutTypeEnum)((int)type + 1);
    }
  }
}

void selectPrevMode()
{
  if (0 != selectedServo)
  {
    auto mode = servoData.servos[selectedServo-1].servoMode;
    if (0 < mode)
    { // fair enough
      servoData.servos[selectedServo-1].servoMode = (servoModeEnum)((int)mode - 1);
    }
  }
}

void selectNextMode()
{
  if (0 != selectedServo)
  {
    auto mode = servoData.servos[selectedServo-1].servoMode;
    if (3 > mode)
    { // fair enough
      servoData.servos[selectedServo-1].servoMode = (servoModeEnum)((int)mode + 1);
    }
  }
}

void pressedButton1()
{
  Serial.println("S1 pressed");
  if (0 == selectedServo)
  {
    setSelectedServoPos(1, servoPositionStraight, true);
  }
  else
  {
    switch (selectedPage)
    {
      case progPageServoLimits: modifySelServo(servoPositionStraight, true); break;
      case progPageAdrDelta: modifyAdress(1); break;
      case progPageTypeMode: selectPrevType(); break;
    }
  }
}

void pressedButton2()
{
  Serial.println("S2 pressed");
  if (0 == selectedServo)
  {
    setSelectedServoPos(1, servoPositionRound, true);
  }
  else
  {
    switch (selectedPage)
    {
      case progPageServoLimits: modifySelServo(servoPositionStraight, false); break;
      case progPageAdrDelta: modifyAdress(-1); break;
      case progPageTypeMode: selectNextType(); break;
    }
  }
}

void pressedButton3()
{
  Serial.println("S3 pressed");
  if (0 == selectedServo)
  {
    setSelectedServoPos(2, servoPositionStraight, true);
  }
  else
  {
    switch (selectedPage)
    {
      case progPageServoLimits: modifySelServo(servoPositionRound, true); break;
      case progPageAdrDelta: modifyDeltaMove(1); break;
      case progPageTypeMode: selectPrevMode(); break;
    }
  }
}

void pressedButton4()
{
  Serial.println("S4 pressed");
  if (0 == selectedServo)
  {
    setSelectedServoPos(2, servoPositionRound, true);
  }
  else
  {
    switch (selectedPage)
    {
      case progPageServoLimits: modifySelServo(servoPositionRound, false); break;
      case progPageAdrDelta: modifyDeltaMove(1); break;
      case progPageTypeMode: selectNextMode(); break;
    }
  }
}

void pressedButton5()
{
  Serial.println("S5 pressed");
  if (0 == selectedServo)
  {
    setSelectedServoPos(3, servoPositionStraight, true);
  }
  else
  {
    selectPrevPage();
  }
}

void pressedButton6()
{
  Serial.println("S6 pressed");
  if (0 == selectedServo)
  {
    setSelectedServoPos(3, servoPositionRound, true);
  }
  else
  {
    selectNextPage();
  }
}

void pressedButton7()
{
  Serial.println("S7 pressed");
  if (0 == selectedServo)
  {
    setSelectedServoPos(4, servoPositionStraight, true);
    
   }
   else
   { // reset
     selectedServo = 0;
     decoderState = stateIdle;
     readFromEEprom();
     fixServoPosition();
   }
}

void pressedButton8()
{
  Serial.println("S8 pressed");
  if (0 == selectedServo)
  {
    setSelectedServoPos(4, servoPositionRound, true);
  }
  else
  {
    // save
    selectedServo = 0;
    decoderState = stateIdle;
    saveToEepromWheneverThereIsTime = true;
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
        DEBOUNCE_BUTTON( bitRead(buttons, 0), S1, pressedButton1(); )
        DEBOUNCE_BUTTON( bitRead(buttons, 1), S2, pressedButton2(); )
        DEBOUNCE_BUTTON( bitRead(buttons, 2), S3, pressedButton3(); )
        DEBOUNCE_BUTTON( bitRead(buttons, 3), S4, pressedButton4(); )
        DEBOUNCE_BUTTON( bitRead(buttons, 4), S5, pressedButton5(); )
        DEBOUNCE_BUTTON( bitRead(buttons, 5), S6, pressedButton6(); )
        DEBOUNCE_BUTTON( bitRead(buttons, 6), S7, pressedButton7(); )
        DEBOUNCE_BUTTON( bitRead(buttons, 7), S8, pressedButton8(); )
      }
    #undef DEBOUNCE_BUTTON
    lastKeybTaskTime = millis();
  }
}

void asyncMemoryTask()
{
  if (servoData.servos[0].moving || servoData.servos[1].moving || servoData.servos[2].moving || servoData.servos[3].moving)
  {
    return;
  }
  if ((millis() - lastAsyncMemoryTaskTime) > 1000)
  {
    if (saveToEepromWheneverThereIsTime)
    {
      saveToEEprom();
      saveToEepromWheneverThereIsTime = false;
    }
  }
}

void setup()
{
  initialiseOutputsAndPins();
  servoTaskStats.clear();
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
  // init I2C
  scanI2CBus();
  //wait some defined time before servos are activated
  delay(initDelay);
  initialiseServos();
}

#define CALL_TASK_WITH_STATS(name)                        \
  unsigned long name##TimeMeas = micros();                \
  name();                                                 \
  name##TimeMeas = micros() - name##TimeMeas;             \
  name##Stats.add(static_cast<float>(name##TimeMeas));    \
  name##LastCallTime = name##TimeMeas;

void loop()
{
  // loop statistics
  unsigned long loopOverallTimeMeas = micros();

  // all tasks
  CALL_TASK_WITH_STATS(dccInterfaceTask);
  CALL_TASK_WITH_STATS(servoTask);
  CALL_TASK_WITH_STATS(ledTask);
  CALL_TASK_WITH_STATS(serialInterfaceTask);
  CALL_TASK_WITH_STATS(selftestTask);
  CALL_TASK_WITH_STATS(displayTask);
  CALL_TASK_WITH_STATS(progButtonTask);
  CALL_TASK_WITH_STATS(keyboardTask);
  CALL_TASK_WITH_STATS(asyncMemoryTask);

  // loop statistics
  loopOverallTimeMeas = micros() - loopOverallTimeMeas;
  loopOverallStats.add(static_cast<float>(loopOverallTimeMeas));
  loopOverallLastCallTime = loopOverallTimeMeas;
}

