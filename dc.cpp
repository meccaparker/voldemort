/* 	Voldemort the Mobile Drawing Robot (Morti) v1.0
		A Capstone Project for the Electromechanical Systems Design course 
		Carnegie Mellon University
		December 2018

		Code written by Mecca Parker
		Team Members: Kenny Sladick, Christopher Bright, Rory Hubbard, Kam Undieh

		Voldemort is a mobile drawing robot. Other drawing mechanisms are limited 
		to a single, maximum canvas size. Morti is not. 

		Read more: (https://github.com/meccaparker/voldemort)

    Actuates 4 DC motors to move Morti to the setpoints defined in the GCode
    input using PID and minimum-jerk trajectory. Also actuates the drawing tool.
*/

#include "voldemort.h"

#define PRINT_OUTPUT 0      // flag to print PID data for debugging

#define DIAMETER 60.0       // diameter of one omni wheel [mm]  
#define TICS_PER_REV 3200.0 // encoder tics in one wheel revolution

#define MAX_PWM 125         // higher MAX_PWM may lead to less accurate travel
#define MIN_PWM 0           // could be used to bypass dc deadzone  

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); // motorshield object

// Assign motors to a port (M1, M2, M3, M4)
Adafruit_DCMotor *motorA = AFMS.getMotor(3);
Adafruit_DCMotor *motorB = AFMS.getMotor(1);
Adafruit_DCMotor *motorC = AFMS.getMotor(2);
Adafruit_DCMotor *motorD = AFMS.getMotor(4);

// Encoder pins
#define ENCODER_PIN_X1 2
#define ENCODER_PIN_X2 3
#define ENCODER_PIN_Y1 18
#define ENCODER_PIN_Y2 19

// Encoder objects 
Encoder encoder_x(ENCODER_PIN_X1, ENCODER_PIN_X2);
Encoder encoder_y(ENCODER_PIN_Y1, ENCODER_PIN_Y2);

// set X and Y motor variables so PID knows which motors we're using
#define MOTORS_X 0          
#define MOTORS_Y 1          

// Proportional, integral, and derivative gain constants. Tuned for 3 motors 
// independently. Our Motor C has a significantly large deadzone which is why
// it has its own set of gain constants.
float KX_A[3] = {4.5, 0.001, 0.5};
float KX_C[3] = {7, 0.001, 0.5};      
float KY[3] = {7.5, 0.0015, 0.5}; 

float setpoint_x = 0; float setpoint_y = 0;   // x and y setpoints
float x0; float y0;                           // initial positions

// PID errors: {proportional, integral, derivative, previous}
float err_x[4] = {0, 0, 0, 0}; 
float err_y[4] = {0, 0, 0, 0}; 

byte motor_dir_x; byte motor_dir_y; // determines motor direction

float pos_x = 0; float pos_y = 0;   // current position

float gain_x = 0; float gain_y = 0; // gains calculated in PID loop

float dx_per_tic = (PI * DIAMETER) / TICS_PER_REV; // circumference / tics per revolution

// Initialize DC motors
void dc_init() {
  AFMS.begin();
  delay(500);
  report_success(SUCCESS_DC_INIT);
}

// Rounding helper function to round to the nearest 25
int round_25(int i) {
  int n;
  float k = i/100.0;
  float j = floor(k);
  float r = 100*(k - j);
  if (r >= 25) return (int)100 * j + 25;
  else if (r == 25) return i;
  else return (int)100 *j;
}

// Update position (used in LiDAR implementation)
void update_position() {
  // int *res = lidar_update_position();
  // pos_x = res[0];
  // pos_y = res[1];
}

// @TODO
void dc_disable() { report_success(SUCCESS_DC_DISABLE); }

// @TODO
float dc_get_max_speed() { return 0; }

// Store setpoints for dc PID, reset errors, and move tool if necessary
void dc_set_target(float sp_x, float sp_y, float sp_z) {
  Serial.print(sp_x); Serial.print("\t"); Serial.print(sp_y); Serial.print("\t"); Serial.print(sp_z); Serial.println(); Serial.println();
  
  x0 = pos_x; y0 = pos_y;

  setpoint_x = sp_x; setpoint_y = sp_y;

  err_x[0] = 0; err_x[1] = 0; err_x[2] = 0; err_x[3] = 0;
  err_y[0] = 0; err_y[1] = 0; err_y[2] = 0; err_y[3] = 0;

  if (sp_z < 0) tool_engage();
  else if (sp_z > 0) tool_disengage();

  report_success(SUCCESS_DC_SET_TARGET);
}

// Set speed for motors
void dc_set_speed() {
  motorA->setSpeed(abs(gain_x)); motorC->setSpeed(abs(gain_x));
  motorB->setSpeed(abs(gain_y)); motorD->setSpeed(abs(gain_y));
}

// Change direction of motors depending on sign of gain
void dc_set_dir() {
  if (gain_x > 0) { 
    motorA->run(FORWARD); 
    motorC->run(BACKWARD); 
  } else {
    motorA->run(BACKWARD); 
    motorC->run(FORWARD); 
  }
  
  if (gain_y > 0) { 
    motorB->run(BACKWARD); 
    motorD->run(FORWARD); 
  } else {
    motorB->run(FORWARD); 
    motorD->run(BACKWARD);
  }
}

/*  A standard PID loop. Takes in:
      *gain_ptr - a pointer to the gain variable so we can update it
      k - array of gain constants
      err - array of errors 
      pos - current position of the current motor
      setpoint - setpoint of current motor
      *motor_dir_ptr - a pointer to the current motor direction so we can update
                       it in the case of negative gain
*/
float PID_control(float *gain_ptr, float *k, float *err, float pos, float setpoint, byte *motor_dir_ptr) {
  float gain = *gain_ptr;
  byte motor_dir = *motor_dir_ptr;

  // Proportional, integral, and derivative errors
  float err_p = err[0];
  float err_i = err[1];
  float err_d = err[2];

  float prev_err = err[3]; // previous error

  // Proportional, integral, and derivative gain constants
  float kp = k[0];
  float ki = k[1];
  float kd = k[2];

  err_p = setpoint - pos;
  err_d = err_p - prev_err; 
  err_i += err_p;

  gain = (kp * err_p) + (ki * err_i) + (kd * err_d); // compute total gain
  
  // Scale gain PWM value
  if (gain < 0) gain = constrain(gain, -MAX_PWM, -MIN_PWM); 
  else gain = constrain(gain, MIN_PWM, MAX_PWM); 

  // Change the motor_dir variable depending on the sign of the gain
  if (gain < 0) motor_dir = -1;
  else motor_dir = 1; 

  prev_err = err_p; // set the previous err to the current one for the next loop

  // Update error array with new error values
  err[0] = err_p;
  err[1] = err_i;
  err[2] = err_d;
  err[3] = prev_err;

  // Update motor and gain values
  *motor_dir_ptr = motor_dir;
  *gain_ptr = gain;
}

// Move dc motors using minimum-jerk trajectory and PID
void dc_move(float fr) {
  Serial.println("[IN PROGRESS]: Starting up the motors...");

  if (PRINT_OUTPUT) {
    Serial.print("SP_X\t|\tPOS_X\t|\tX(t)\t|\tGAIN_X\t\tSP_Y\t|\tPOS_Y\t|\tY(t)\t|\tGAIN_Y");
    Serial.println();
  }

  int step_count = 0; // number of steps taken in minmum-jerk algorithm
  float t_curr = 0; // current time
  float dist = pow(pow(setpoint_x - x0, 2) + pow(setpoint_y - y0, 2), 0.5); // resultant of setpoint componenets
  float dist_per_step = 1; // [mm]
  float time_per_step = 25; // [ms]
  float num_steps = ceil(dist/dist_per_step); 
  float t_final = round_25(200 * pow(dist, 0.40)); // total amount of time to complete steps. 100 with PRINT_OUTPUT, 200 for real use

  // Execute algorithm until t_curr/t_final = 1. Run only if we have a non-zero t_final.
  while (1 && t_final != 0) {
    // encoder count to position conversion (CHANGE WHEN IMPLEMENTING LIDAR)
    pos_x = encoder_x.read() * dx_per_tic;
    pos_y = encoder_y.read() * dx_per_tic;

    t_curr = step_count * time_per_step;
    float t = t_curr/t_final;

    // Minimum-jerk trajectory. Breaks setpoints into small segments to be 
    // completed in a certain amount of time. We basically set a final time
    // for both motors to complete their movements which is important for 
    // trying to execute vectors with uneven x and y components.
    float xt = x0 + (x0 - setpoint_x)*(15*pow(t, 4.0) - 6*pow(t, 5.0) - 10*pow(t, 3.0));
    float yt = y0 + (y0 - setpoint_y)*(15*pow(t, 4.0) - 6*pow(t, 5.0) - 10*pow(t, 3.0));

    // Find PID pwm value
    PID_control(&gain_x, KX_C, err_x, pos_x, xt, &motor_dir_x); 
    PID_control(&gain_y, KY, err_y, pos_y, yt, &motor_dir_y); 

    dc_set_speed(); // set speed
    dc_set_dir(); // set direction

    // For debugging
    if (PRINT_OUTPUT) {
      Serial.print(setpoint_x); Serial.print("\t\t"); Serial.print(pos_x); Serial.print("\t\t");
      Serial.print(xt); Serial.print("\t\t"); Serial.print(gain_x); Serial.print("\t\t");
      Serial.print(setpoint_y); Serial.print("\t\t"); Serial.print(pos_y); Serial.print("\t\t");
      Serial.print(yt); Serial.print("\t\t"); Serial.print(gain_y); 
      Serial.println(); 
    }

    step_count++;
    if (abs(1 - t) < 0.001) break; // break when t_curr/t_final = 1
    delay(time_per_step);
  }

  gain_x = 0; gain_y = 0; // turn off motors
  dc_set_speed(); 
  parser_set_current_units(setpoint_x, setpoint_y); // update parser with current position
  report_success(SUCCESS_DC_MOVE);
  return;
}
