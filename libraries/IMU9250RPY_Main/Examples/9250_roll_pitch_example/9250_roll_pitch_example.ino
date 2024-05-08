#include <MPU9250RPY.h>
MPU9250RPY imuDevice;

long unsigned runtime;
long unsigned lasttime = 0;

void setup() {
  imuDevice.initialize();
  imuDevice.filterValue(0.98, 0.98, 0.98);
  imuDevice.setGyroError(196, -58, -8);
  imuDevice.setMagError(-26, -39.5, -6, 0.6849, 0.6060, 0.5952);  //first 3 arguments are offset, next 3 are the bias

  Serial.begin(115200); 

  delay(1000);
}

void loop() {
  runtime = millis();

  if ((runtime - lasttime) > 10)
  {
    imuDevice.updateOrientation();
    Serial.print(imuDevice.returnPitch());
    Serial.print(" ");
    Serial.print(imuDevice.returnRoll());
    Serial.print(" ");
    Serial.println(imuDevice.returnYaw());

    lasttime = runtime;
  }

}
