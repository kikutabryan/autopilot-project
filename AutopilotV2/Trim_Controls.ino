void trimControls()
{
  int trimAileronArray[11];
  int trimElevatorArray[11];
  int trimRudderArray[11];

  long unsigned int trimTime = millis();
  for (int i = 0; i < 11; i++)
  {
    while ((millis() - trimTime) < 100);
    
    trimAileronArray[i] = receiverInput[0];
    trimElevatorArray[i] = receiverInput[1];
    trimRudderArray[i] = receiverInput[2];

    trimTime = millis();
  }

  sortArray(trimAileronArray, 11);
  sortArray(trimElevatorArray, 11);
  sortArray(trimRudderArray, 11);

  trimValues[0] = (trimAileronArray[5] - 1500);
  trimValues[1] = (trimElevatorArray[5] - 1500);
  trimValues[2] = (trimRudderArray[5] - 1500);
}
