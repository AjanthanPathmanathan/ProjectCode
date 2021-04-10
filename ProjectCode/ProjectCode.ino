
//MSE 2202
//Western Engineering base code
//2020 05 13 E J Porter


/*
  esp32                                           MSE-DuinoV2
  pins         description                        Brd Jumpers /Labels                                                                  User (Fill in chart with user PIN usage)
  1             3v3                               PWR 3V3                                                                              3V3
  2             gnd                               GND                                                                                  GND
  3             GPIO15/AD2_3/T3/SD_CMD/           D15 (has connections in both 5V and 3V areas)
  4             GPIO2/AD2_2/T2/SD_D0              D2(has connections in both 5V and 3V areas)  /INDICATORLED ( On ESP32 board )        Heartbeat LED
  5             GPIO4/AD2_0/T0/SD_D1              D4(has connections in both 5V and 3V areas)                                          Left Motor, Channel A
  6             GPIO16/RX2                        Slide Switch S1b                                                                     IR Receiver
  7             GPIO17/TX2                        Slide Switch S2b                                                                     Left Encoder, Channel A
  8             GPIO5                             D5 (has connections in both 5V and 3V areas)                                         Left Encoder, Channel B
  9             GPIO18                            D18 (has connections in both 5V and 3V areas)                                        Left Motor, Channel B
  10            GPIO19/CTS0                       D19 (has connections in both 5V and 3V areas)                                        Right Motor, Channel A
  11            GPIO21                            D21/I2C_DA
  12            GPIO3/RX0                         RX0
  13            GPIO1//TX0                        TX0
  14            GPIO22/RTS1                       D22/I2C_CLK
  15            GPIO23                            D23 (has connections in both 5V and 3V areas)
  16            EN                                JP4 (Labeled - RST) for reseting ESP32
  17            GPI36/VP/AD1_0                    AD0
  18            GPI39/VN/AD1_3/                   AD3
  19            GPI34/AD1_6/                      AD6
  20            GPI35/AD1_7                       Potentiometer R2 / AD7
  21            GPIO32/AD1_4/T9                   Potentiometer R1 / AD4                                                               Pot 1 (R1)
  22            GPIO33/AD1_5/T8                   IMon/D33  monitor board current
  23            GPIO25/AD2_8/DAC1                 SK6812 Smart LEDs / D25                                                              Smart LEDs
  24            GPIO26/A2_9/DAC2                  Push Button PB2                                                                      Limit switch
  25            GPIO27/AD2_7/T7                   Push Button PB1                                                                      PB1
  26            GPOP14/AD2_6/T6/SD_CLK            Slide Switch S2a                                                                     Right Encoder, Channel A
  27            GPIO12/AD2_5/T5/SD_D2/            D12(has connections in both 5V and 3V areas)                                         Right Motor, Channel B
  28            GPIO13/AD2_4/T4/SD_D3/            Slide Switch S1a                                                                     Right Encoder, Channel B
  29            GND                               GND                                                                                  GND
  30            VIN                               PWR 5V t 7V                                                                          PWR 5V to 7V
*/


//Pin assignments
const int ciHeartbeatLED = 2;
const int ciPB1 = 27;
const int ciPB2 = 26;
const int ciPot1 = A4;    //GPIO 32  - when JP2 has jumper installed Analog pin AD4 is connected to Poteniometer R1
const int ciLimitSwitch = 26;
const int ciIRDetector = 16;
const int ciMotorLeftA = 4;
const int ciMotorLeftB = 18;
const int ciMotorRightA = 19;
const int ciMotorRightB = 12;
const int ciEncoderLeftA = 17;
const int ciEncoderLeftB = 5;
const int ciEncoderRightA = 14;
const int ciEncoderRightB = 13;
const int ciSmartLED = 25;
const int ciStepperMotorDir = 22;
const int ciStepperMotorStep = 21;

const int Front_Switch = 15;
const int Top_Switch = 26;

volatile uint32_t vui32test1;
volatile uint32_t vui32test2;

#include "0_Core_Zero.h"

#include "Wire.h"
#include <MPU6050_light.h>

#include <esp_task_wdt.h>

#include <ESP32Servo.h>
Servo myservo;

#include <Adafruit_NeoPixel.h>
#include <Math.h>
#include "Motion.h";
#include "MyWEBserver.h"
#include "BreakPoint.h"
#include "WDT.h";

void loopWEBServerButtonresponce(void);

const int CR1_ciMainTimer =  1000;
const int CR1_ciHeartbeatInterval = 500;
const int CR1_ciMotorRunTime = 1000;
const long CR1_clDebounceDelay = 50;
const long CR1_clReadTimeout = 220;

const uint8_t ci8RightTurn = 18;
const uint8_t ci8LeftTurn = 17;

unsigned char CR1_ucMainTimerCaseCore1;
uint8_t CR1_ui8LimitSwitch;

uint8_t CR1_ui8IRDatum;
uint8_t CR1_ui8WheelSpeed;
uint8_t CR1_ui8LeftWheelSpeed;
uint8_t CR1_ui8RightWheelSpeed;

uint32_t CR1_u32Now;
uint32_t CR1_u32Last;
uint32_t CR1_u32Temp;
uint32_t CR1_u32Avg;

unsigned long CR1_ulLastDebounceTime;
unsigned long CR1_ulLastByteTime;

unsigned long CR1_ulMainTimerPrevious;
unsigned long CR1_ulMainTimerNow;

unsigned long CR1_ulMotorTimerPrevious;
unsigned long CR1_ulMotorTimerNow;
unsigned char ucMotorStateIndex = 0;

unsigned long CR1_ulHeartbeatTimerPrevious;
unsigned long CR1_ulHeartbeatTimerNow;

boolean btHeartbeat = true;
boolean btRun = false;
boolean btToggle = true;
int iButtonState;
int iLastButtonState = HIGH;

int rightSpeed = 180;
int leftSpeed = 180;

MPU6050 mpu(Wire);
unsigned long timer = 0;
unsigned long timer2 = 0;
float angleZ = 0;

int target = 0;
float absolute;

int servoPin = 2;

// Declare our SK6812 SMART LED object:
Adafruit_NeoPixel SmartLEDs(2, 25, NEO_GRB + NEO_KHZ400);
// Argument 1 = Number of LEDs (pixels) in use
// Argument 2 = ESP32 pin number
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

void setup() {
  Serial.begin(115200);
  Serial2.begin(2400, SERIAL_8N1, ciIRDetector);  // IRDetector on RX2 receiving 8-bit words at 2400 baud

  Core_ZEROInit();

  Wire.begin();
  mpu.begin();
  Serial.println(F("Calculating gyro offset, do not move MPU6050"));
  delay(1000);
  mpu.calcGyroOffsets();
  Serial.println("Done!\n");

  WDT_EnableFastWatchDogCore1();
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[0] = 0;
  WDT_vfFastWDTWarningCore1[1] = 0;
  WDT_vfFastWDTWarningCore1[2] = 0;
  WDT_vfFastWDTWarningCore1[3] = 0;
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[4] = 0;
  WDT_vfFastWDTWarningCore1[5] = 0;
  WDT_vfFastWDTWarningCore1[6] = 0;
  WDT_vfFastWDTWarningCore1[7] = 0;
  WDT_ResetCore1();
  WDT_vfFastWDTWarningCore1[8] = 0;
  WDT_vfFastWDTWarningCore1[9] = 0;
  WDT_ResetCore1();

  setupMotion();
  pinMode(ciHeartbeatLED, OUTPUT);
  pinMode(ciPB1, INPUT_PULLUP);
  pinMode(ciLimitSwitch, INPUT_PULLUP);

  ledcAttachPin(2, 5);
  ledcAttachPin(5, 6);

  ledcSetup(5, 20000, 8);
  ledcSetup(6, 20000, 8);

  ledcWrite(5, 0);
  ledcWrite(6, 0);

    attachInterrupt(Front_Switch, driveUp, CHANGE);
  attachInterrupt(Top_Switch, driveDown, CHANGE);
}

void driveUp()
{

      ledcWrite(5, 250);
      ledcWrite(6, 0);

}

void driveDown()
{

  ledcWrite(5, 0);
  ledcWrite(6, 0);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////

/*void driveStraight(int duration)
  {
  unsigned long tim = millis();
  while (millis() < (duration+tim)) {
    if((millis()-timer)>10){ // print data every 10ms
    mpu.update();
    //int rightSpeed = 160 - (mpu.getAngleZ());
    int rightSpeed = 180 - (mpu.getAngleZ());
    int leftSpeed = 180 + (mpu.getAngleZ());
    MoveTo(4, leftSpeed, rightSpeed);

    timer = millis();
  }
  }

  }


  void driveLeft(int duration)
  {
  unsigned long tim = millis();
  while (millis() < (duration + tim)) {
    //if (mpu.getAngleZ() < 90 && mpu.getAngleZ() > 1) { // print data every 10ms
    //ledcWrite(2, 170);
    //ledcWrite(1, 0);
    //ledcWrite(4, 0);
    //ledcWrite(3, 170);
    MoveTo(2, 140, 140);
    //tim = millis();
    //}
  }
  }

  void driveRight(int duration)
  {
  unsigned long tim = millis();
  while (millis() < (duration + tim)) {
    //if ((millis() - timer) > 10) { // print data every 10ms
    ledcWrite(2, 0);
    ledcWrite(1, 170);
    ledcWrite(4, 170);
    ledcWrite(3, 0);

    //tim = millis();
    //}
  }
  }

  void turnRight(int duration)
  {
  // 500 for a 90deg turn
  unsigned long tim = millis();
  while (millis() < (duration+tim)) {
    //if((millis()-timer)>10){ // print data every 10ms
    //mpu.update();
    //int rightSpeed = 170 + (mpu.getAngleZ());
    //int leftSpeed = 130;

    MoveTo(1, 130, 0);

    timer = millis();
  }
  }

  void driveBackward(int duration)
  {
  unsigned long tim = millis();
  while (millis() < (duration + tim)) {
    if ((millis() - timer) > 10) { // print data every 10ms
      mpu.update();
      int rightSpeed = 180 + (mpu.getAngleZ());
      int leftSpeed = 180 - (mpu.getAngleZ());
      /*Serial.print("R: ");
        Serial.print(rightSpeed);
        Serial.print("\tL: ");
        Serial.println(leftSpeed);
      MoveTo(1, leftSpeed, rightSpeed);

      timer = millis();
    }
  }

  }*/


//////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////
void driveLeft(int duration)
{
  unsigned long tim = millis();
  while (millis() < (duration + tim)) {

    ledcWrite(2, 0);
    ledcWrite(1, 170);
    ledcWrite(4, 170);
    ledcWrite(3, 0);

  }
}

void driveRight(int duration)
{
  unsigned long tim = millis();
  while (millis() < (duration + tim)) {


      ledcWrite(2, 170);
        ledcWrite(1, 0);
        ledcWrite(4, 0);
        ledcWrite(3, 170);

  }
}

void driveStraight(int duration)
{
  unsigned long tim = millis();
  while (millis() < (duration + tim)) {
    if ((millis() - timer) > 10) { // print data every 10ms
      mpu.update();
      //int rightSpeed = 160 - (mpu.getAngleZ());
      int rightSpeed = 180 - (mpu.getAngleZ());
      int leftSpeed = 180 + (mpu.getAngleZ());

      Serial.print("R: ");
      Serial.print(rightSpeed);
      Serial.print("\tL: ");
      Serial.println(leftSpeed);

      MoveTo(1, leftSpeed, rightSpeed);

      timer = millis();
    }
  }

}

void driveBackward(int duration)
{
  unsigned long tim = millis();
  while (millis() < (duration + tim)) {
    if ((millis() - timer) > 10) { // print data every 10ms
      mpu.update();
      //int rightSpeed = 160 - (mpu.getAngleZ());
      int rightSpeed = 200 + (mpu.getAngleZ());
      int leftSpeed = 20,0 - (mpu.getAngleZ());
.
      Serial.print("R: ");
      Serial.print(rightSpeed);
      Serial.print("\tL: ");
      Serial.println(leftSpeed);

      MoveTo(1, leftSpeed, rightSpeed);

      timer = millis();
    }
  }

}

void loop()
{
  //WSVR_BreakPoint(1);

  //average the encoder tick times
  ENC_Averaging();

  int iButtonValue = digitalRead(ciPB1);       // read value of push button 1
  if (iButtonValue != iLastButtonState) {      // if value has changed
    CR1_ulLastDebounceTime = millis();        // reset the debouncing timer
  }

  if ((millis() - CR1_ulLastDebounceTime) > CR1_clDebounceDelay) {
    if (iButtonValue != iButtonState) {        // if the button state has changed
      iButtonState = iButtonValue;               // update current button state

      // only toggle the run condition if the new button state is LOW
      if (iButtonState == LOW)
      {
        ENC_ClearLeftOdometer();
        ENC_ClearRightOdometer();
        btRun = !btRun;
        Serial.println(btRun);
        // if stopping, reset motor states and stop motors
        if (!btRun)
        {
          ucMotorStateIndex = 0;
          ucMotorState = 0;
          move(0);
        }

      }
    }
  }
  iLastButtonState = iButtonValue;             // store button state

  if (!digitalRead(ciLimitSwitch))
  {
    //btRun = 0; //if limit switch is pressed stop bot
    ucMotorStateIndex = 10;
    ucMotorState = 0;
    move(0);
  }

  if (Serial2.available() > 0) {               // check for incoming data
    CR1_ui8IRDatum = Serial2.read();          // read the incoming byte
    // Serial.println(iIncomingByte, HEX);        // uncomment to output received character
    CR1_ulLastByteTime = millis();            // capture time last byte was received
  }
  else
  {
    // check to see if elapsed time exceeds allowable timeout
    if (millis() - CR1_ulLastByteTime > CR1_clReadTimeout) {
      CR1_ui8IRDatum = 0;                     // if so, clear incoming byte
    }
  }
  CR1_ulMainTimerNow = micros();
  if (CR1_ulMainTimerNow - CR1_ulMainTimerPrevious >= CR1_ciMainTimer)
  {
    WDT_ResetCore1();
    WDT_ucCaseIndexCore0 = CR0_ucMainTimerCaseCore0;

    CR1_ulMainTimerPrevious = CR1_ulMainTimerNow;

    switch (CR1_ucMainTimerCaseCore1) //full switch run through is 1mS
    {
      //###############################################################################
      case 0:
        {

          if (btRun)
          {

            CR1_ulMotorTimerNow = millis();
            if (CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime)
            {
              CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
              switch (ucMotorStateIndex)
              {
                case 0:
                  {
                    delay(1000);
                    ucMotorStateIndex = 1;
                    driveStraight(1300);
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 1:
                  {
                    ucMotorStateIndex = 2;
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 2:
                  {
                    ucMotorStateIndex = 3;
                    driveLeft(185);
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 3:
                  {
                    ucMotorStateIndex = 4;
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 4:
                  {
                    ucMotorStateIndex = 5;
                    driveStraight(1600);
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 5:
                  {
                    ucMotorStateIndex =  6;
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 6:
                  {
                    ucMotorStateIndex = 7;
                    driveLeft(185);
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 7:
                  {
                    ucMotorStateIndex = 8;
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 8:
                  {
                    ucMotorStateIndex = 9;
                    driveStraight(2000);
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 9:
                  {
                    //ucMotorStateIndex = 10;
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 10:
                  {
                    ucMotorStateIndex = 11;
                    driveUp();
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 11:
                  {
                    ucMotorStateIndex = 12;
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 12:
                  {
                    ucMotorStateIndex = 13;
                    driveStraight(200);
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 13:
                  {
                    ucMotorStateIndex = 14;
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 14:
                  {
                    driveLeft(165);
                    ucMotorStateIndex = 15;
                    break;
                  }
                case 15:
                  {
                    ucMotorStateIndex = 16;
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 16:
                  {
                      driveStraight(1100);
                      ucMotorStateIndex = 17;
                    break;
                  }
                case 17:
                  {
                    ucMotorStateIndex = 18;
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 18:
                  {
                    
                    break;
                  }
                case 19:
                  {
                    ucMotorStateIndex = 20;
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
                case 20:
                  {
                    ledcSetup(5, 50 , 16);
                    ledcAttachPin(15, 5);
                    ledcWrite(5, 7000);
                    driveRight(600);
                    ucMotorState = 0;
                    move(0);
                    break;
                  }
              }
            }
            CR1_ucMainTimerCaseCore1 = 1;

            break;
          }
        //###############################################################################
        case 1:
          {
            //read pot 1 for motor speeds
            //CR1_ui8WheelSpeed = map(analogRead(ciPot1), 0, 4096, 130, 255);  // adjust to range that will produce motion

            CR1_ucMainTimerCaseCore1 = 2;
            break;
          }
        //###############################################################################
        case 2:
          {
            // asm volatile("esync; rsr %0,ccount":"=a" (vui32test1)); // @ 240mHz clock each tick is ~4nS

            //   asm volatile("esync; rsr %0,ccount":"=a" (vui32test2)); // @ 240mHz clock each tick is ~4nS

            CR1_ucMainTimerCaseCore1 = 3;
            break;
          }
        //###############################################################################
        case 3:
          {
            //move bot X number of odometer ticks
            if (ENC_ISMotorRunning())
            {
              //MoveTo(ucMotorState, CR1_ui8LeftWheelSpeed,CR1_ui8LeftWheelSpeed);
            }

            CR1_ucMainTimerCaseCore1 = 4;
            break;
          }
        //###############################################################################
        case 4:
          {

            CR1_ucMainTimerCaseCore1 = 5;
            break;
          }
        //###############################################################################
        case 5:
          {


            CR1_ucMainTimerCaseCore1 = 6;
            break;
          }
        //###############################################################################
        case 6:
          {


            CR1_ucMainTimerCaseCore1 = 7;
            break;
          }
        //###############################################################################
        case 7:
          {
            if (CR1_ui8IRDatum == 0x55) {                // if proper character is seen
              SmartLEDs.setPixelColor(0, 0, 25, 0);      // make LED1 green with 10% intensity
            }
            else if (CR1_ui8IRDatum == 0x41) {           // if "hit" character is seen
              SmartLEDs.setPixelColor(0, 25, 0, 25);     // make LED1 purple with 10% intensity
            }
            else {                                       // otherwise
              SmartLEDs.setPixelColor(0, 25, 0, 0);      // make LED1 red with 10% intensity
            }
            SmartLEDs.show();                            // send updated colour to LEDs

            CR1_ucMainTimerCaseCore1 = 8;
            break;
          }
        //###############################################################################
        case 8:
          {

            CR1_ucMainTimerCaseCore1 = 9;
            break;
          }
        //###############################################################################
        case 9:
          {

            CR1_ucMainTimerCaseCore1 = 0;
            break;
          }

        }
    }

    // Heartbeat LED
    CR1_ulHeartbeatTimerNow = millis();
    if (CR1_ulHeartbeatTimerNow - CR1_ulHeartbeatTimerPrevious >= CR1_ciHeartbeatInterval)
    {
      CR1_ulHeartbeatTimerPrevious = CR1_ulHeartbeatTimerNow;
      btHeartbeat = !btHeartbeat;
      digitalWrite(ciHeartbeatLED, btHeartbeat);
      // Serial.println((vui32test2 - vui32test1)* 3 );
    }
  }
}
