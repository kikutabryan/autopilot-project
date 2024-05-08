//checks if the pulse is in range
int pulse_Check(int pulse_Value, int safe_Value)
{
  if ((pulse_Value > 2000) or (pulse_Value < 1000))
    return (safe_Value);
  else
    return (pulse_Value);
}

//converts degrees into a valid pulse length
float degree_To_Pulse(float degree)
{
  float pulse_Value = (((degree/180) * 1000) + 1000);
  return int(pulse_Value);
}

//inverts the axis
int invert_Pulse(int original, bool invert)
{
  if (invert == false)
  {
    return original;
  }
  else
  {
    int inverted = (3000 - original);
    return inverted;
  }
}
