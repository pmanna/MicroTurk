#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "SquarePositions.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define SERVICE_UUID "4FAFC201-1FB5-459E-8FCC-C5C9C331914B"
#define CHARACTERISTIC_UUID "BEB5483E-36E1-4688-B7F5-EA07361B26A8"

#define DEBUG
//#define TEST_MOVES

#define MAX_STEPPER_SPEED 600
#define MAX_STEPPER_ACCEL 1000
#define MAGNET_PIN 15

const long stepsPerRevolution = 2038;
const float mmPerRevolution = 75.40; // To calibrate
const float stepsPerMm = (float)stepsPerRevolution / mmPerRevolution;

AccelStepper motor1(AccelStepper::DRIVER, 13, 12);
AccelStepper motor2(AccelStepper::DRIVER, 27, 26);
MultiStepper table;

char inData[80];
char *outData[4] = {NULL};
byte idx = 0;
bool motorsEnabled = true;

float movePositions[4][2] = {0};
float currentPosition[2] = {0.0, 0.0};

bool magnetStates[4] = {false, true, true, false};

int progress = -1;

void enableMotors()
{
  motor1.enableOutputs();
  motor2.enableOutputs();

  motorsEnabled = true;
}

void disableMotors()
{
  motor1.disableOutputs();
  motor2.disableOutputs();

  motorsEnabled = false;
}

void resetMotors()
{
  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);

  currentPosition[0] = 0.0;
  currentPosition[1] = 0.0;

  disableMotors();

  delay(1000);
}

void moveHeadTo(float positions[2])
{
  float dX = positions[0] - currentPosition[0];
  float dY = positions[1] - currentPosition[1];
  long steps[2];

  if (!motorsEnabled)
    enableMotors();

  steps[0] = (long)((dX + dY) * stepsPerMm);
  steps[1] = (long)((dX - dY) * stepsPerMm);

#ifdef DEBUG
  Serial.printf("Moving by %ld %ld\n", steps[0], steps[1]);
#endif

  table.move(steps);

  currentPosition[0] = positions[0];
  currentPosition[1] = positions[1];
}

void parseSquarePos(char *moveStr)
{
  if (strlen(moveStr) < 4)
    return;

  float x1, y1;
  float x2, y2;

  if (getSquarePos(moveStr[0], moveStr[1], x1, y1) &&
      getSquarePos(moveStr[2], moveStr[3], x2, y2))
  {
    progress = 0;
    movePositions[0][0] = x1;
    movePositions[0][1] = y1;
    movePositions[1][0] = x1;
    movePositions[1][1] = y1;
    movePositions[2][0] = x2;
    movePositions[2][1] = y2;
    movePositions[3][0] = x2;
    movePositions[3][1] = y2;
  }
}

void parseCommand(const char *cmdStr)
{
  float newPos[2] = {0.0, 0.0};
  bool magnetOn;
  char *p = (char *)cmdStr; //point to *p to the string in inData
  char *str;        //declaring *str
  int counter = 0;  //initialise the counter

  while ((str = strtok(p, " ")) != NULL) // delimiter is the space
  {
    outData[counter] = str; //use the counter as an index to add each value to the array
    counter++;              //increment the counter

    p = NULL;

    if (counter > 3)
      break;
  }

#ifdef DEBUG
  Serial.print(outData[0]);
  Serial.print("\t");
  Serial.print(outData[1]);
  Serial.print("\t");
  Serial.print(outData[2]);
  Serial.print("\t");
  Serial.print(outData[3]);
  Serial.print("\t");
  Serial.println(strlen(outData[0]));

  if (strlen(outData[0]) < 3)
  {
    switch (toupper(outData[0][0]))
    {
    case 'R':
      Serial.println("Resetting");
      break;
    case 'D':
      Serial.println("Disabling motors");
      break;
    case 'E':
      Serial.println("Enabling motors");
      break;
    case 'I':
      Serial.println("Moving Up");
      break;
    case 'J':
      Serial.println("Moving Left");
      break;
    case 'K':
      Serial.println("Moving Down");
      break;
    case 'L':
      Serial.println("Moving Right");
      break;
    case 'H':
      Serial.println("Homing");
      break;
    case 'M':
      Serial.print("X: ");
      Serial.println(atof(outData[1]));
      Serial.print("Y: ");
      Serial.println(atof(outData[2]));
      Serial.print("M: ");
      Serial.println(atoi(outData[3]));
      break;
    }
#endif

    switch (toupper(outData[0][0]))
    {
    case 'R':
      resetMotors();
      break;

    case 'D':
      disableMotors();
      break;

    case 'E':
      enableMotors();
      break;

    case 'I':
      newPos[0] = currentPosition[0];
      newPos[1] = currentPosition[1] + 10.0;

      moveHeadTo(newPos);
      break;

    case 'J':
      newPos[0] = currentPosition[0] - 10.0;
      newPos[1] = currentPosition[1];

      moveHeadTo(newPos);
      break;
    case 'K':
      newPos[0] = currentPosition[0];
      newPos[1] = currentPosition[1] - 10.0;

      moveHeadTo(newPos);
      break;
    case 'L':
      newPos[0] = currentPosition[0] + 10.0;
      newPos[1] = currentPosition[1];

      moveHeadTo(newPos);
      break;
    case 'H':
      newPos[0] = 0.0;
      newPos[1] = 0.0;

      moveHeadTo(newPos);
      break;
    case 'M':
      newPos[0] = atof(outData[1]);
      newPos[1] = atof(outData[2]);

      moveHeadTo(newPos);

      magnetOn = atoi(outData[3]) != 0;

      if (magnetOn)
      {
        digitalWrite(MAGNET_PIN, HIGH);
      }
      else
      {
        digitalWrite(MAGNET_PIN, LOW);
      }

      break;
    default:
      break;
    }
  }
  else
  {
    parseSquarePos(outData[0]);
  }
  delay(500);

  for (int ii = 0; ii < 4; ii++)
  {
    outData[ii] = NULL;
  }
}

void parseInput()
{
  while (Serial.available() > 0)
  {
    char aChar = Serial.read();

    if (aChar == '\n')
    {
      parseCommand(inData);

      idx = 0;
      inData[idx] = '\0';
    }
    else if (idx < 80)
    {
      inData[idx] = aChar;
      idx++;
      inData[idx] = '\0'; // Keep the string NULL terminated
    }
  }
}

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0)
    {
      char  cmdStr[80];

      strcpy(cmdStr,value.c_str());
      cmdStr[strlen(cmdStr) - 1]  = '\0';
      
      parseCommand(cmdStr);

      pCharacteristic->setValue("OK");
    }
  }
};

void setup()
{
  Serial.begin(115200);

  BLEDevice::init("MicroTurk");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("Ready");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  pinMode(MAGNET_PIN, OUTPUT);

  motor1.setEnablePin(14);
  motor1.setPinsInverted(false, false, true);
  motor1.setMinPulseWidth(20);
  motor1.setMaxSpeed(MAX_STEPPER_SPEED);
  motor1.setAcceleration(MAX_STEPPER_ACCEL);
  table.addStepper(motor1);

  motor2.setEnablePin(25);
  motor2.setPinsInverted(false, false, true);
  motor2.setMinPulseWidth(20);
  motor2.setMaxSpeed(MAX_STEPPER_SPEED);
  motor2.setAcceleration(MAX_STEPPER_ACCEL);
  table.addStepper(motor2);

  resetMotors();

#ifdef DEBUG
  Serial.println("Ready");
#endif
}

void loop()
{
  if (!table.run())
  {
    if (progress < 0)
    {
      parseInput();
    }
    else
    {
      if (progress >= 4)
      {
        progress = -1;
        disableMotors();
        return;
      }

      moveHeadTo(movePositions[progress]);

      if (magnetStates[progress])
      {
        digitalWrite(MAGNET_PIN, HIGH);
      }
      else
      {
        digitalWrite(MAGNET_PIN, LOW);
      }

      progress++;
      delay(500);
    }
  }
  else
  {
    // Serial.print(".");
  }
}