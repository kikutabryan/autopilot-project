//includes
#include <MPU9250RPY.h>
#include <PID.h>
#include <ServoTimer2.h>
#include <TinyGPS++.h>
#include <PinChangeInterrupt.h>
#include <ArduinoSort.h>

//receiver channels
#define CHANNEL1 2  //aileron
#define CHANNEL2 3  //elevator
#define CHANNEL3 4  //rudder
#define CHANNEL4 5  //throttle
#define CHANNEL5 6  //switch
#define CHANNEL6 7  //something
//servo/esc pins
#define AILERONPIN 8
#define ELEVATORPIN 9
#define RUDDERPIN 10
#define THROTTLEPIN 11

//object creations
MPU9250RPY imuDevice;
TinyGPSPlus gpsDevice;
PIDClass pid[7];
ServoTimer2 aileron, elevator, rudder, throttle;

//variables
float lastWaypoint[2], currentWaypoint[2];
float imuRoll , imuPitch, imuYaw;
float gpsYaw, gpsSpeed, gpsAltitude, gpsTrgtDist, gpsTrgtCourse, gpsCoordinates[2];
int gpsNumSats = 0;
unsigned long lastTime;  //time value to check if pids should b updated
//receiver arrays
volatile uint32_t receiverNow[6], receiverPast[6], receiverInput[6], receiverUpdateTime[6];
//inverts the outputs to servos/esc
bool aileron_Invert = false, elevator_Invert = false, rudder_Invert = true, throttle_Invert = false;
int trimValues[6] = {0};


void setup() {
  //initialize required components
  autopilotInitialize();
  receiverInitialize();
  servoInitialize();
  //prevents code from starting unless there are 3 or more satellites
  while (gpsNumSats < 5)
  {
    updateMeasurements();
  }
  //prevents code from running when autopilot mode is initially on
  while ((receiverInput[4] < 1500)  and ((millis() - receiverUpdateTime[4]) >= 100));
  //trims the controls for autopilot using the transmitter manual trim
  trimControls();
  //moves the controls to signify user that the autopilot is ready
  wakeUp();
}


void loop() {
  //main autopilot functions
  setHome(43.773275, -79.373073);  //sets start destination
  trackWaypoint(43.773175, -79.373635, 3, 10);  //tracks path to next waypoint
  trackWaypoint(43.773275, -79.373073, 3, 10);

}
