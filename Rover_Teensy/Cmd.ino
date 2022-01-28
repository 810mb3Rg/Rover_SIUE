/********************************  GIVING COMMAND  ***********************************************/
void parseCommand(){
  char command = rcv_buffer[0];   // our first byte tells us the command char is equivalent to byte
  switch (command){
    ///////////////////////////// LOGIC CURRENT (i) ///////////////////////////////////////////////
    case 'i':case 'I':
      current_sensor();
      break;

    ///////////////////////////// LOGIC VOLTAGE (o) ///////////////////////////////////////////////
    case 'o':case 'O':
      voltage_sensor();
      break;

    ///////////////////////////// PRINT PID (p) ///////////////////////////////////////////////////
    case 'p':case 'P':
      check_pid();
      break;

    ///////////////////////////// MOTOR RPM  (v # #)  /////////////////////////////////////////////
    case 'v':case 'V':
      int rpm0;
      int rpm1;
      sscanf(&rcv_buffer[1], "%d %d \r", &rpm0, &rpm1);
      Serial.print(rpm0);
      Serial.print(' ');
      Serial.println(rpm1);
      rpm_setpoint[0] = rpm0;
      rpm_setpoint[1] = rpm1;
      break;

    ///////////////////////////// STOP MOTORS (s) /////////////////////////////////////////////////
    case 's':case 'S':
      rpm_setpoint[0] = 0;
      rpm_setpoint[1] = 0;
      break;

    ///////////////////////////// MAIN BATTERY VOLTAGE (b)  ///////////////////////////////////////
    case 'b':case 'B':
      double mbv = roboclaw.ReadMainBatteryVoltage(address) * 0.1;
      Serial.print("Main Battery: ");
      Serial.print(mbv);
      Serial.println(" V");
      break;
  }
}
