void receiverInitialize(void)
{
  pinMode(CHANNEL1, INPUT);
  pinMode(CHANNEL2, INPUT);
  pinMode(CHANNEL3, INPUT);
  pinMode(CHANNEL4, INPUT);
  pinMode(CHANNEL5, INPUT);
  pinMode(CHANNEL6, INPUT);

  attachPCINT(digitalPinToPCINT(CHANNEL1), receiverCh1, CHANGE);
  attachPCINT(digitalPinToPCINT(CHANNEL2), receiverCh2, CHANGE);
  attachPCINT(digitalPinToPCINT(CHANNEL3), receiverCh3, CHANGE);
  attachPCINT(digitalPinToPCINT(CHANNEL4), receiverCh4, CHANGE);
  attachPCINT(digitalPinToPCINT(CHANNEL5), receiverCh5, CHANGE);
  attachPCINT(digitalPinToPCINT(CHANNEL6), receiverCh6, CHANGE);
}

//channel 1
void receiverCh1(void)
{
  receiverNow[0] = micros();

  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(CHANNEL1));
  if(trigger == RISING)
    receiverPast[0] = receiverNow[0];
  else
    receiverInput[0] = (receiverNow[0] - receiverPast[0]);
    receiverUpdateTime[0] = millis();
}

//channel 2
void receiverCh2(void)
{
  receiverNow[1] = micros();

  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(CHANNEL2));
  if(trigger == RISING)
    receiverPast[1] = receiverNow[1];
  else
    receiverInput[1] = (receiverNow[1] - receiverPast[1]);
    receiverUpdateTime[1] = millis();
}

//channel 3
void receiverCh3(void)
{
  receiverNow[2] = micros();

  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(CHANNEL3));
  if(trigger == RISING)
    receiverPast[2] = receiverNow[2];
  else
    receiverInput[2] = (receiverNow[2] - receiverPast[2]);
    receiverUpdateTime[2] = millis();
}

//channel 4
void receiverCh4(void)
{
  receiverNow[3] = micros();

  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(CHANNEL4));
  if(trigger == RISING)
    receiverPast[3] = receiverNow[3];
  else
    receiverInput[3] = (receiverNow[3] - receiverPast[3]);
    receiverUpdateTime[3] = millis();
}

//channel 5
void receiverCh5(void)
{
  receiverNow[4] = micros();

  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(CHANNEL5));
  if(trigger == RISING)
    receiverPast[4] = receiverNow[4];
  else
    receiverInput[4] = (receiverNow[4] - receiverPast[4]);
    receiverUpdateTime[4] = millis();
}

//channel 6
void receiverCh6(void)
{
  receiverNow[5] = micros();

  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(CHANNEL6));
  if(trigger == RISING)
    receiverPast[5] = receiverNow[5];
  else
    receiverInput[5] = (receiverNow[5] - receiverPast[5]);
    receiverUpdateTime[5] = millis();
}
