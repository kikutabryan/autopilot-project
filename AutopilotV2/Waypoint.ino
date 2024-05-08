bool trackWaypoint(float waypointLat, float waypointLng, float waypointSpeed, float waypointAltitude)
{
  //updating the lastwaypoint to equal the previous waypoint and updating the current one with new coordinates
  lastWaypoint[0] = currentWaypoint[0];
  lastWaypoint[1] = currentWaypoint[1];

  currentWaypoint[0] = waypointLat;
  currentWaypoint[1] = waypointLng;

  //the course heading between the two waypoints (last waypoint and the current waypoint)
  float wayPointCourse = (TinyGPSPlus::courseTo(lastWaypoint[0], lastWaypoint[1], currentWaypoint[0], currentWaypoint[1]) - 180);

  while (true)
  {
    //check if manual flight mode on
    if (flight_Mode_Switch() == false)
      manual_Control();  //if switch set to maual, calls the manual flight function
     
    //update all the measureable variables
    updateMeasurements();

    //gets the distance off from path between waypoints
    float distanceOffPath = distanceOffPathFun();

    if ((millis() - lastTime) > 0)  //checks if the time passed is greater than 0 to prevent division by 0 in pid loops
    {
      //gets the roll angle required to maintain the path
      float targetHeading = pid[0].PID(0, distanceOffPath, 90, -90, 45, -45, false) + wayPointCourse;  //gets the target heading
      rangeCheck(targetHeading, 180, -180, 360);  //keeps the headings within the limits
      float targetRollAngle = pid[1].PID(targetHeading, gpsYaw, 45, -45, 45, -45, true);  //gets the roll angle
      float targetAileronAngle = -pid[2].PID(targetRollAngle, imuRoll, 45, -45, 45, -45, true) + 90;  //gets the aileron angle

      //gets the pitch angle required to maintain the altitude
      float targetPitchAngle = pid[3].PID(waypointAltitude, gpsAltitude, 30, -30, 30, -30, false);  //gets the target pitch
      float targetElevatorAngle = -pid[4].PID(targetPitchAngle, imuPitch, 45, -45, 45, -45, false) + 90;  //gets the elevator angle

      //rudder or steering for ground vehicle 
      float targetRudderAngle = -pid[5].PID(targetHeading, gpsYaw, 70, -70, 5, -5, true) + 90;

      //throttle
      float targetThrottlePulse = pid[6].PID(waypointSpeed, gpsSpeed, 200, 0, 200, 0, false) + 1500; 

      //Servo updates
      aileron.write(invert_Pulse(pulse_Check(degree_To_Pulse(targetAileronAngle), 1500) + trimValues[0], aileron_Invert));
      elevator.write(invert_Pulse(pulse_Check(degree_To_Pulse(targetElevatorAngle), 1500) + trimValues[1], elevator_Invert));
      rudder.write(invert_Pulse(pulse_Check(degree_To_Pulse(targetRudderAngle), 1500) + trimValues[2], rudder_Invert));
      throttle.write(invert_Pulse(pulse_Check(targetThrottlePulse, 1500), throttle_Invert));

      //updates the last time
      lastTime = millis();
    }

    //check if waypoint is complete or if the waypoint is missed
    if ((gpsTrgtDist < 4) or (missedWaypoint() == true))
      return false;
  }
}


float distanceOffPathFun(void)
{
  //gets coordinates
  float A[2], B[2];
  A[0] = currentWaypoint[0] - lastWaypoint[0];
  A[1] = currentWaypoint[1] - lastWaypoint[1];
  B[0] = gpsCoordinates[0] - lastWaypoint[0];
  B[1] = gpsCoordinates[1] - lastWaypoint[1];

  float AdotB = A[0] * B[0] + A[1] * B[1];
  float AdotA = A[0] * A[0] + A[1] * A[1];

  float lambda = AdotB / AdotA;

  float S[2];
  S[0] = lastWaypoint[0] + lambda * A[0];
  S[1] = lastWaypoint[1] + lambda * A[1];

  //checks which side of line coordinates are on
  float side = (gpsCoordinates[0] - lastWaypoint[0]) * (currentWaypoint[1] - lastWaypoint[1]) - (gpsCoordinates[1] - lastWaypoint[1]) * (currentWaypoint[0] - lastWaypoint[0]);
  float sign;
  //sets the multiplication sign to correspond to the side of the path
  if (side >= 0)
    sign = -1;
  else if (side < 0)
    sign = 1;

  //calculates the distance between the 2 gps points
  float distance = (TinyGPSPlus::distanceBetween(gpsCoordinates[0], gpsCoordinates[1], S[0], S[1]) * sign);

  //returns the final distance
  return distance;
}


bool missedWaypoint(void)
{
  //calculates a point perpendicular to the path of the two waypoints on the target waypoint
  float perpCurrentWaypoint[2] = {(currentWaypoint[0] + lastWaypoint[1] - currentWaypoint[1]), (currentWaypoint[1] + currentWaypoint[0] - lastWaypoint[0])};

  //calculates the side the vehicle is on
  float side = (gpsCoordinates[0] - currentWaypoint[0]) * (perpCurrentWaypoint[1] - currentWaypoint[1]) - (gpsCoordinates[1] - currentWaypoint[1]) * (perpCurrentWaypoint[0] - currentWaypoint[0]);

  if (side > 0)
    return true;
  else
    return false;
}


void rangeCheck(float& value, float max, float min, float change)
{
  if (value > max)
    value -= change;
  else if (value < min)
    value += change;
}


void setHome(float latHome, float lonHome)
{
  currentWaypoint[0] = latHome;
  currentWaypoint[1] = lonHome;
}
