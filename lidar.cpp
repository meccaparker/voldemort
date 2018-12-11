/* 	Voldemort the Mobile Drawing Robot (Morti) v1.0
		A Capstone Project for the Electromechanical Systems Design course 
		Carnegie Mellon University
		December 2018

		Code written by Christopher Bright
		Team Members: Kenny Sladick, Mecca Parker, Rory Hubbard, Kam Undieh

		Voldemort is a mobile drawing robot. Other drawing mechanisms are limited 
		to a single, maximum canvas size. Morti is not. 

		Read more: (https://github.com/meccaparker/voldemort)

    Built for RPLidar A1 and an IMU sensor. Detects a reference object that is 
    placed at the edge of the canvas and calculates Morti's current position 
    determined by a trigonometric algorithm.
*/

#include "voldemort.h"

XYPlot MyPlot;

// You need to create an driver instance
RPLidar ld;

#define RPLIDAR_MOTOR 6 // The PWM pin for control the speed of RPLIDAR's motor.
// This pin should connected with the RPLIDAR's MOTOCTRL signal

/* Setup IMU Sensor */
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long startTime = 5000;

int IMUangle = 0;

const float pi = 3.14159;

int xTemp = 0;
int yTemp = 0;
int x = 0;
int y = 0;

// CANVAS SIZE
int canvasX = 500;
int canvasY = 500;
int cornerThreshold = 150;

int minDist[2];
int maxDist[2];

// DATA TRACKING
int data[90];
int count = 0;
int initCount = 0;

// POSITIONING STUFF
int cornerDist = 0;
int cornerAngle = 0;
int pointXPos = 0;
int pointYPos = 0;
int initXTotal = 0;
int initYTotal = 0;
int initX = 0;
int initY = 0;
int prevX = 0;
int prevY = 0;
boolean checkAngle = false;
boolean corner_initialized = false;
boolean foundCorner = false;

int get_Bounds() {
  minDist[0] = canvasX - x;
  minDist[1] = canvasY - y;
  maxDist[0] = canvasX + cornerThreshold - x;
  maxDist[1] = canvasY + cornerThreshold - y;
}

void set_canvas(int x, int y) {
  canvasX = x; canvasY = y;
}

void reset_lidar() {
  analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

  // try to detect RPLIDAR...
  rplidar_response_device_info_t info;
  if (IS_OK(ld.getDeviceInfo(info, 100))) {
    ld.startScan();
    analogWrite(RPLIDAR_MOTOR, 255); // start motor rotating at max allowed speed
    delay(1000);
  } else {
    Serial.println("[ERROR]: Lidar unresponsive.");
  }
}

boolean isinBounds(int pointX, int pointY) {
  if (pointXPos > minDist[0] && pointXPos < maxDist[0] && pointYPos > minDist[1] && pointYPos < maxDist[1]) {
    return true;
  } else return false;
}

void initialize_corner(int laserX, int laserY) {
  initXTotal = initXTotal + laserX;
  initYTotal = initYTotal + laserY;
  initCount++;
  if (initCount >= 5) {
    initX = initXTotal / initCount;
    initY = initYTotal / initCount;
    corner_initialized = true;
  }
}

int get_pos(int initX, int initY, int laserX, int laserY) {
  if (initX - laserX > 0) {
    xTemp = initX - laserX;
  }
  if (initY - laserY > 0) {
    yTemp = initY - laserY;
  }
}

int *update_position(int prevX, int prevY, int xTemp, int yTemp) {
  if (abs(prevX - xTemp) < 5) x = xTemp;
  if (abs(prevY - yTemp) < 5) y = yTemp;
  int res[2];
  res[0] = x; res[1] = y;
  Serial.print('x:\t'); Serial.println(x);
  Serial.print('y:\t'); Serial.println(y);
  return res;
}

void check_IMUsensor() {
  /* Get a new sensor event */
  sensors_event_t event;

  bno.getEvent(&event);

  IMUangle = event.orientation.x;
}


bool update_lidar() {

  if (IS_OK(ld.waitPoint())) {
    int angle = ld.getCurrentPoint().angle + IMUangle;
    int distance = ld.getCurrentPoint().distance;
    angle = round(angle);
    if (angle >= 360) {
      angle = angle - 360;
    }
    if (distance == 0) {
      distance = -1;
    }
      if (angle < 90) {
        byte index = angle;
        data[index] = distance;
        count++;
      }
  }

  else reset_lidar(); // lidar is not ok; repair it 

  if (count >= 90) {
    /* i represents angle */
    for (int i = 0; i < 90; i++) {
      int curr_dist = data[i];
      pointXPos = abs(curr_dist * sin(i * (pi / 180.0)));
      pointYPos = abs(curr_dist * cos(i * (pi / 180.0)));
      if ((foundCorner == false) && isinBounds(pointXPos, pointYPos)) {
        if (corner_initialized == false) {
          initialize_corner(pointXPos, pointYPos);
          foundCorner = true;
        }
        else {
          get_pos(initX, initY, pointXPos, pointYPos);
          update_position(prevX, prevY, xTemp, yTemp);
          foundCorner = true;
        }
      }
    }
    prevX = xTemp;
    prevY = yTemp;
    get_Bounds();
    count = 0;
    foundCorner = false;
    //    check_IMUsensor();
    return true;
  } else return false;
}

int *lidar_update_position() {
  int res[2];
  while (!update_lidar());
  res[0] = x; res[1] = y;
  return res;
}

void lidar_init(int x, int y) {
  /* bind the RPLIDAR driver to the arduino hardware serial */
  ld.begin(Serial3);

  set_canvas(x, y);

  /* set pin modes */
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  /* Initialise the sensor */
  sensor_t sensor;
  bno.getSensor(&sensor);
  //  if (!bno.begin())
  //  {
  //    /* There was a problem detecting the BNO055 ... check your connections */
  //    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //    while (1);
  //  }
  //  delay(1000);

  get_Bounds();

  reset_lidar();

  for (int i = 0; i < 5; i++) update_lidar();
  /* Use external crystal for better accuracy */
  //  bno.setExtCrystalUse(true);
}

