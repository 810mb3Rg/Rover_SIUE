/******************************** RoboClaw Library ************************************************/
#include <SoftwareSerial.h>
#include "RoboClaw.h"

SoftwareSerial serial(10, 11);    // RoboClaw S2 to D10, S1 to D11
RoboClaw roboclaw(&serial, 10000);

#define address 0x80
#define Kp 0.18766                // P = 0.18766       (0.1 ~ 0.3)
#define Ki 0.00695                // I = 0.00695     (0.004 ~ 0.01)
#define Kd 0.00000                // D = 0.00000
#define qpps 650000               // qpps = 650000  (640000 ~ 700000)
#define RelayPin 9                // 5V SSR on D9

/*******************************    RC parameters   **********************************************/
int Ystick;
int Xstick;
int SwitchA;
int RCoff = 0;

/************************** Voltage Current Sensor Library ***************************************/
const int voltage_sensor_pin = A1;
const int current_sensor_pin = A2;
const int avgSamples = 100;
float vOUT;
float voltage_sen = 0;
int vol_sensorValue;
int cur_sensorValue = 0;
int current_sen = 0;
float sensitivity = 1.0;          // 5000mA per 5000mV = 1
float Vref = 0;                   // Output voltage with no current: 0.0 mV

/************************************ RPM ********************************************************/
double rpm_setpoint[2]                     = {0, 0};
char rcv_buffer[64];

void setup() {
  roboclaw.begin(38400);
  Serial.begin(115200);
  // RC pin setup
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(RelayPin, OUTPUT);
  digitalWrite(RelayPin, HIGH);   // Turn Relay ON
  delay(200);                     // Wait for RoboClaw
  roboclaw.SetM1VelocityPID(address, Kd, Kp, Ki, qpps);
  roboclaw.SetM2VelocityPID(address, Kd, Kp, Ki, qpps);
}

void loop() {
  RCin();
  if (RCoff > 20) {
    receiveBytes();
    if (rpm_setpoint[0] > 0) {
      roboclaw.ForwardM1(address, constrain(rpm_setpoint[0], 0, 127));
    }
    else {
      roboclaw.BackwardM1(address, constrain(-1 * rpm_setpoint[0], 0, 127));
    }
    if (rpm_setpoint[1] > 0) {
      roboclaw.ForwardM2(address, constrain(rpm_setpoint[1], 0, 127));
    }
    else {
      roboclaw.BackwardM2(address, constrain(-1 * rpm_setpoint[1], 0, 127));
    }
  }
}
