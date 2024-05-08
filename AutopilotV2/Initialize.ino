void autopilotInitialize()
{
  Serial.begin(115200);
  
  //setup time stuff
  lastTime = 0;

  //imu setup
  imuDevice.initialize();
  imuDevice.filterValue(0.98, 0.98, 0.98);
  imuDevice.setGyroError(196, -58, -8);
  imuDevice.setMagError(-145.5, -52.5, 120, 0.64516, 0.75188, 0.73529);

  //pid setup
  pid[0].kConstants(1.5, 0, 0);
  pid[1].kConstants(1, 0, 0);
  pid[2].kConstants(1, 0, 0);
  pid[3].kConstants(1, 0, 0);
  pid[4].kConstants(1, 0, 0);
  pid[5].kConstants(1, 0.001, 1000);
  pid[6].kConstants(15, 0.01, 200);
}

void servoInitialize(void)
{
  aileron.attach(AILERONPIN);
  elevator.attach(ELEVATORPIN);
  rudder.attach(RUDDERPIN);
  throttle.attach(THROTTLEPIN);
}

void wakeUp(void)
{
  aileron.write(invert_Pulse(pulse_Check(1800 + trimValues[0], 1500), aileron_Invert));
  elevator.write(invert_Pulse(pulse_Check(1800 + trimValues[1], 1500), elevator_Invert));
  rudder.write(invert_Pulse(pulse_Check(1800 + trimValues[2], 1500), rudder_Invert));

  long unsigned int tempTime = millis();
  while ((millis() - tempTime) < 1000);

  aileron.write(invert_Pulse(pulse_Check(1200 + trimValues[0], 1500), aileron_Invert));
  elevator.write(invert_Pulse(pulse_Check(1200 + trimValues[1], 1500), elevator_Invert));
  rudder.write(invert_Pulse(pulse_Check(1200 + trimValues[2], 1500), rudder_Invert));

  tempTime = millis();
  while ((millis() - tempTime) < 1000);

  aileron.write(invert_Pulse(pulse_Check(1500 + trimValues[0], 1500), aileron_Invert));
  elevator.write(invert_Pulse(pulse_Check(1500 + trimValues[1], 1500), elevator_Invert));
  rudder.write(invert_Pulse(pulse_Check(1500 + trimValues[2], 1500), rudder_Invert));
}
