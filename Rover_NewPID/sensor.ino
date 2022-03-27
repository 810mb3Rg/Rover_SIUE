/********************************* Current Sensor ************************************************/
void current_sensor() {
  for (int i = 0; i < avgSamples; i++) {
    cur_sensorValue += analogRead(current_sensor_pin);
    delay(2);
  }
  current_sen = ((4.955 * cur_sensorValue / avgSamples) + 10.0) * sensitivity;
  Serial.println(current_sen);
}

/********************************* Voltage Sensor ************************************************/
void voltage_sensor() {
  for (int i = 0; i < avgSamples; i++) {
    vol_sensorValue += analogRead(voltage_sensor_pin);
    delay(2);
  }
  vOUT = (vol_sensorValue * 5) / (1024.0 * avgSamples); // map to ADC resolution
  voltage_sen = vOUT / (7500.0 / (30000.0 + 7500.0));   // voltage division
  Serial.println(voltage_sen);
  vol_sensorValue = 0;
}

/*********************************  PID Settings  ************************************************/
void check_pid() {
  float kp1, ki1, kd1, kp2, ki2, kd2;
  uint32_t qpps1;
  if (roboclaw.ReadM1VelocityPID(address, kd1, kp1, ki1, qpps1)) {
    Serial.print("M1 PID: ");
    Serial.print(kp1, 5);
    Serial.print(' ');
    Serial.print(ki1, 5);
    Serial.print(' ');
    Serial.println(kd1, 5);
  }
  if (roboclaw.ReadM2VelocityPID(address, kd2, kp2, ki2, qpps1)) {
    Serial.print("M2 PID: ");
    Serial.print(kp2, 5);
    Serial.print(' ');
    Serial.print(ki2, 5);
    Serial.print(' ');
    Serial.println(kd2, 5);
  }
  else {
    Serial.println("Failed to read PID values");
  }
}

/***********************************  RGB LED  ***************************************************/
void setRGB(int R, int G, int B) {
  analogWrite(LED_R, R);
  analogWrite(LED_G, G);
  analogWrite(LED_B, B);
}
