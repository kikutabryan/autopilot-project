void updateMeasurements()
{
  //IMU device update and get roll pitch and yaw angles
  if (imuDevice.found())
  {
    imuDevice.updateOrientation();
    imuRoll = imuDevice.returnRoll(); imuPitch = imuDevice.returnPitch(); imuYaw = imuDevice.returnYaw();
  }

  //GPS device update
  while (Serial.available())
    gpsDevice.encode(Serial.read());
  if (gpsDevice.satellites.isUpdated() and gpsDevice.satellites.isValid())
  {
    gpsNumSats = gpsDevice.satellites.value();
    if (gpsNumSats < 2)
      return;
  }
  if (gpsDevice.location.isUpdated() and gpsDevice.location.isValid())
  {
    //gpsYaw = gpsDevice.course.deg() - 180;
    gpsYaw = filterData(gpsYaw, (gpsDevice.course.deg() - 180), 0.8);
    
    gpsTrgtDist = TinyGPSPlus::distanceBetween(gpsDevice.location.lat(), gpsDevice.location.lng(), currentWaypoint[0], currentWaypoint[1]);
    gpsTrgtCourse = TinyGPSPlus::courseTo(gpsDevice.location.lat(), gpsDevice.location.lng(), currentWaypoint[0], currentWaypoint[1]) - 180;

    //gpsCoordinates[0] = gpsDevice.location.lat();
    gpsCoordinates[0] = filterData(gpsCoordinates[0], gpsDevice.location.lat(), 0.8);
    //gpsCoordinates[1] = gpsDevice.location.lng();
    gpsCoordinates[1] = filterData(gpsCoordinates[1], gpsDevice.location.lng(), 0.8);
  }
  if (gpsDevice.speed.isUpdated() and gpsDevice.speed.isValid())
  {
    //gpsSpeed = gpsDevice.speed.knots();
    gpsSpeed = filterData(gpsSpeed, gpsDevice.speed.knots(), 0.8);
  }
  if (gpsDevice.altitude.isUpdated() and gpsDevice.altitude.isValid())
  {
    //gpsAltitude = gpsDevice.altitude.feet();
    gpsAltitude = filterData(gpsAltitude, gpsDevice.altitude.feet(), 0.8);
  }
}

float filterData(float& pastValue, float newValue, float filterFactor)
{
  pastValue = ((newValue * filterFactor) + (pastValue * (1 - filterFactor)));
  return pastValue;
}
