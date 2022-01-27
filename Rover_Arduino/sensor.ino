/******************************** Current Sensor ************************************************/
void current_sensor()
{
  for (int i = 0; i < avgSamples; i++) {
    cur_sensorValue += analogRead(current_sensor_pin);
    delay(2);
  }
  cur_sensorValue = cur_sensorValue / avgSamples;
  current_sen = ((4.955 * cur_sensorValue) + 10.0) * sensitivity;
  Serial.println(current_sen);
}

/******************************** Voltage Sensor ************************************************/
void voltage_sensor()
{
  vol_sensorValue = analogRead(voltage_sensor_pin);
  vOUT = (vol_sensorValue * 5) / 1024.0; // map to ADC resolution
  voltage_sen = vOUT / (7500.0 / (30000.0 + 7500.0)); // voltage division
  Serial.println(voltage_sen);
}

/********************************  PID Settings  ************************************************/
void check_pid()
{
  float kp1,ki1,kd1,kp2,ki2,kd2;
  uint32_t qpps1; 
  if (roboclaw.ReadM1VelocityPID(address, kd1, kp1, ki1, qpps1)) {
    Serial.print("M1 PID: ");
    Serial.print(kp1,5);
    Serial.print(' ');
    Serial.print(ki1,5);
    Serial.print(' ');
    Serial.println(kd1,5);
  }
  if (roboclaw.ReadM2VelocityPID(address, kd2, kp2, ki2, qpps1)) {
    Serial.print("M2 PID: ");
    Serial.print(kp2,5);
    Serial.print(' ');
    Serial.print(ki2,5);
    Serial.print(' ');
    Serial.println(kd2,5);
  }
  else{
    Serial.println("Failed to read PID values");
  }
}
