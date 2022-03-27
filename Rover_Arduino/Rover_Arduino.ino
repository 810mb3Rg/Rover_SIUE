#include    <SoftwareSerial.h>
#include    "RoboClaw.h"
///////////////////////////////// RC Receiver Pins ////////////////////////////////////////////////
const int   xChnlPin   = 2;
const int   yChnlPin   = 3;
const int   sChnlPin   = 4;
////////////////////////////////// RoboClaw Pins //////////////////////////////////////////////////
const int   RelayPin   = 12;                             // 5V SSR to enable main battery
const int   RoboClawS2 = 5;
const int   RoboClawS1 = 6;
/////////////////////////////////// Sensor Pins ///////////////////////////////////////////////////
const int   voltage_sensor_pin = A1;
const int   current_sensor_pin = A2;
//////////////////////////////////// LED Pins /////////////////////////////////////////////////////
const int   LED_R = 9;
const int   LED_G = 10;
const int   LED_B = 11;
///////////////////////////////// RoboClaw Library ////////////////////////////////////////////////
const int   address = 0x80;
const float Kp      = 0.1;                          // P = 0.18766       (0.1 ~ 0.3)
const float Ki      = 0.00;                          // I = 0.00695     (0.004 ~ 0.01)
const int   Kd      = 0.00000;                          // D = 0.00000
const int   qpps    = 44000;                           // qpps = 650000  (640000 ~ 700000)
///////////////////////////////////////////////////////////////////////////////////////////////////
/******************************** Sensor Parameters **********************************************/
const int   avgSamples = 100;
const int   Vref = 0;                                   // Output voltage with no current: 0 mV
const float sensitivity = 1;                            // 5000mA per 5000mV = 1 (5 Amps = 5 Volts)
float       vOUT, voltage_sen;
int         vol_sensorValue, cur_sensorValue, current_sen;

/*********************************  RC Parameters ************************************************/
int Ystick, Xstick, SwitchA, RCoff;

/************************************ RPM ********************************************************/
double rpm_setpoint[2];
char rcv_buffer[64];
/*************************************************************************************************/
SoftwareSerial serial(RoboClawS2, RoboClawS1);
RoboClaw roboclaw(&serial, 10000);

void setup() {
  Serial.begin(9600);

  pinMode(RelayPin, OUTPUT);
  digitalWrite(RelayPin, HIGH);                         // Turn Relay ON
  delay(200);                                           // Wait for RoboClaw

  roboclaw.begin(38400);
  roboclaw.SetM1VelocityPID(address, Kd, Kp, Ki, qpps);
  roboclaw.SetM2VelocityPID(address, Kd, Kp, Ki, qpps);
}
/*************************************************************************************************/
void loop() {
  RCin();                                               // Check for RC input
  receiveBytes();                                       // Do serial command
  if (RCoff > 20) {                                     // No RC signal for awhile
    setRGB(0, 255, 0);                                  // green
    /*****************  Left Motor  ********************/
    roboclaw.SpeedM1M2(address, rpm_setpoint[0], rpm_setpoint[1]);
//    if (rpm_setpoint[0] > 0) {                          // Left Forward
//      roboclaw.ForwardM1(address, constrain(rpm_setpoint[0], 0, 127));
//    }
//    else {                                              // Left Backward
//      roboclaw.BackwardM1(address, constrain(-1 * rpm_setpoint[0], 0, 127));
//    }
//    /****************** Right Motor  *******************/
//    if (rpm_setpoint[1] > 0) {                          // Right Forward
//      roboclaw.ForwardM2(address, constrain(rpm_setpoint[1], 0, 127));
//    }
//    else {                                              // Right Backward
//      roboclaw.BackwardM2(address, constrain(-1 * rpm_setpoint[1], 0, 127));
//    }
  }
}
