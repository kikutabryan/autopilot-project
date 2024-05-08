void manual_Control(void)
{
  while(true)
  {
    //update all the measureable variables (prevents large errors when auto resumed)
    if (millis() % 50 == 0)
      updateMeasurements();
    
    //reads the values, checks if they are appropriate, writes to the aileron the position or esc etc.
    aileron.write(invert_Pulse(pulse_Check(receiverInput[0], 1500), aileron_Invert));
    elevator.write(invert_Pulse(pulse_Check(receiverInput[1], 1500), elevator_Invert));
    rudder.write(invert_Pulse(pulse_Check(receiverInput[2], 1500), rudder_Invert));
    throttle.write(invert_Pulse(pulse_Check(receiverInput[3], 1500), throttle_Invert));

    if (flight_Mode_Switch() == true)
    {
      for (int i = 0; i < (sizeof(pid)/sizeof(pid[0])); i++)
      {
        pid[i].resetPID();
      }
      lastTime = millis();
      break;
    }
  }
}

bool flight_Mode_Switch(void)
{
  if ((receiverInput[4] > 1500)  and ((millis() - receiverUpdateTime[4]) <= 100))
    return(false);
  else
    return(true);
}
