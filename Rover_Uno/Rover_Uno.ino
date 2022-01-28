#include    <SoftwareSerial.h>
#include    "RoboClaw.h"
///////////////////////////////// RC Receiver Pins ////////////////////////////////////////////////
const int   xChnlPin   = 2;
const int   yChnlPin   = 3;
const int   sChnlPin   = 4;
////////////////////////////////// RoboClaw Pins //////////////////////////////////////////////////
const int   RelayPin   = 9;                             // 5V SSR on D9
const int   RoboClawS2 = 10;
const int   RoboClawS1 = 11;
/////////////////////////////////// Sensor Pins ///////////////////////////////////////////////////
const int   voltage_sensor_pin = A1;
const int   current_sensor_pin = A2;
//////////////////////////////////// LED Pins /////////////////////////////////////////////////////
const int   LED_R = 5;
const int   LED_G = 6;
const int   LED_B = 7;
///////////////////////////////// RoboClaw Library ////////////////////////////////////////////////
const int   address = 0x80;                             // try const uint8_t
const float Kp      = 0.18766;                          // P = 0.18766       (0.1 ~ 0.3)
const float Ki      = 0.00695;                          // I = 0.00695     (0.004 ~ 0.01)
const int   Kd      = 0.00000;                          // D = 0.00000
const int   qpps    = 650000;                           // qpps = 650000  (640000 ~ 700000)
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
void setup() {
  SoftwareSerial serial(RoboClawS2, RoboClawS1);
  Serial.begin(115200);

  pinMode(RelayPin, OUTPUT);
  digitalWrite(RelayPin, HIGH);                         // Turn Relay ON
  delay(200);                                           // Wait for RoboClaw
  
  RoboClaw roboclaw(&serial, 10000);
  roboclaw.begin(38400); 
  roboclaw.SetM1VelocityPID(address, Kd, Kp, Ki, qpps);
  roboclaw.SetM2VelocityPID(address, Kd, Kp, Ki, qpps);
}
/*************************************************************************************************/
void loop() {
  RCin();                                               // Check for RC input
  receiveBytes();                                       // Do serial command
  if (RCoff > 20) {                                     // No RC signal for awhile

    /*****************  Left Motor  ********************/
    if (rpm_setpoint[0] > 0) {                          // Left Forward
      roboclaw.ForwardM1(address, constrain(rpm_setpoint[0], 0, 127));
    }
    else {                                              // Left Backward
      roboclaw.BackwardM1(address, constrain(-1 * rpm_setpoint[0], 0, 127));
    }
    /****************** Right Motor  *******************/
    if (rpm_setpoint[1] > 0) {                          // Right Forward
      roboclaw.ForwardM2(address, constrain(rpm_setpoint[1], 0, 127));
    }
    else {                                              // Right Backward
      roboclaw.BackwardM2(address, constrain(-1 * rpm_setpoint[1], 0, 127));
    }
  }
}
