// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS CONTROLLED WITH YOUR SMARTPHONE
// JJROBOTS BROBOT KIT: (Arduino Leonardo + BROBOT ELECTRONIC BRAIN SHIELD + STEPPER MOTOR drivers)
// This code is prepared for new BROBOT shield  with ESP8266 Wifi module
// Author: JJROBOTS.COM
// Date: 02/09/2014
// Updated: 25/06/2017
// Version: 2.82
// License: GPL v2
// Compiled and tested with Arduino 1.6.8. This new version of code does not need external libraries (only Arduino standard libraries)
// Project URL: http://jjrobots.com/b-robot-evo-2-much-more-than-a-self-balancing-robot (Features,documentation,build instructions,how it works, SHOP,...)
// New updates:
//   - New default parameters specially tuned for BROBOT EVO 2 version (More agile, more stable...)
//   - New Move mode with position control (for externally programming the robot with a Blockly or pyhton programming interfaces)
//   - New telemtry packets send to TELEMETRY IP for monitoring Battery, Angle, ... (old battery packets for touch osc not working now)
//   - Default telemetry server is 192.168.4.2 (first client connected to the robot)
//  Get the free android app (jjrobots) from google play. For IOS users you need to use TouchOSC App + special template (info on jjrobots page)
//  Thanks to our users on the forum for the new ideas. Specially sasa999, KomX, ...

// The board needs at least 10-15 seconds with no motion (robot steady) at beginning to give good values... Robot move slightly when it´s ready!
// MPU6050 IMU connected via I2C bus. Angle estimation using complementary filter (fusion between gyro and accel)
// Angle calculations and control part is running at 100Hz

// The robot is OFF when the angle is high (robot is horizontal). When you start raising the robot it
// automatically switch ON and start a RAISE UP procedure.
// You could RAISE UP the robot also with the robot arm servo (Servo button on the interface)
// To switch OFF the robot you could manually put the robot down on the floor (horizontal)

// We use a standard PID controllers (Proportional, Integral derivative controller) for robot stability
// More info on the project page: How it works page at jjrobots.com
// We have a PI controller for speed control and a PD controller for stability (robot angle)
// The output of the control (motors speed) is integrated so it´s really an acceleration not an speed.

// We control the robot from a WIFI module using OSC standard UDP messages
// You need an OSC app to control de robot (Custom free JJRobots APP for android, and TouchOSC APP for IOS)
// Join the module Wifi Access Point (by default: JJROBOTS_XX) with your Smartphone/Tablet...
//   Wifi password: 87654321
// For TouchOSC users (IOS): Install the BROBOT layout into the OSC app (Touch OSC) and start play! (read the project page)
// OSC controls:
//    fader1: Throttle (0.0-1.0) OSC message: /1/fader1
//    fader2: Steering (0.0-1.0) OSC message: /1/fader2
//    push1: Move servo arm (and robot raiseup) OSC message /1/push1 
//    if you enable the touchMessage on TouchOSC options, controls return to center automatically when you lift your fingers
//    PRO mode (PRO button). On PRO mode steering and throttle are more aggressive
//    PAGE2: PID adjustements [optional][dont touch if you dont know what you are doing...;-) ]

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#include <Wire.h>
// required for interrupts that drive stepper motor steps
#include <DueTimer.h>
// required for Playstation 2 controller

#include <PS2X_lib.h>  //for v1.6



// PS2X variables
/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT       27 //Blue 
#define PS2_CMD       26 //Orange
#define PS2_SEL       25 //Yellow
#define PS2_CLK       24 //White

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures   true
//#define pressures   false
#define rumble      true
//#define rumble      false


PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you conect the controller, 
//or call config_gamepad(pins) again after connecting the controller.
int ps2x_error = 0; 
byte ps2x_type = 0;
byte ps2x_vibrate = 0;

int rightStickY = 0;
int leftStickY = 0;


// Uncomment this lines to connect to an external Wifi router (join an existing Wifi network)
//#define EXTERNAL_WIFI
//#define WIFI_SSID "YOUR_WIFI"
//#define WIFI_PASSWORD "YOUR_PASSWORD"
//#define WIFI_IP "192.168.1.101"  // Force ROBOT IP
//#define TELEMETRY "192.168.1.38" // Tlemetry server port 2223

//PDB #define TELEMETRY "192.168.4.2" // Default telemetry server (first client) port 2223

// NORMAL MODE PARAMETERS (MAXIMUN SETTINGS)
#define MAX_THROTTLE 550
#define MAX_STEERING 140
#define MAX_TARGET_ANGLE 14

// PRO MODE = MORE AGGRESSIVE (MAXIMUN SETTINGS)
#define MAX_THROTTLE_PRO 780   // Max recommended value: 860
#define MAX_STEERING_PRO 260   // Max recommended value: 280
#define MAX_TARGET_ANGLE_PRO 26   // Max recommended value: 32

// Default control terms for EVO 2
// #define KP 0.32       
// #define KD 0.050
//PDB testing new values for KP and KD
#define KP 0.32       
#define KD 0.052

#define KP_THROTTLE 0.080
#define KI_THROTTLE 0.1 
#define KP_POSITION 0.06  
#define KD_POSITION 0.45  
//#define KI_POSITION 0.02

// Control gains for raiseup (the raiseup movement requiere special control parameters)
#define KP_RAISEUP 0.1   
#define KD_RAISEUP 0.16   
#define KP_THROTTLE_RAISEUP 0   // No speed control on raiseup
#define KI_THROTTLE_RAISEUP 0.0

//PDB MAX_CONTROL_OUTPUT constrains the max motor speed
#define MAX_CONTROL_OUTPUT 100
#define ITERM_MAX_ERROR 30   // Iterm windup constants for PI control 
#define ITERM_MAX 10000

#define ANGLE_OFFSET 6.4  // Offset angle for balance (to compensate robot own weight distribution)

// Servo definitions
#define SERVO_AUX_NEUTRO 1500  // Servo neutral position
#define SERVO_MIN_PULSEWIDTH 700
#define SERVO_MAX_PULSEWIDTH 2500

#define SERVO2_NEUTRO 1500
#define SERVO2_RANGE 1400

// Telemetry
  //PDB #define TELEMETRY_BATTERY 1
  //PDB #define TELEMETRY_ANGLE 1
//#define TELEMETRY_DEBUG 1  // Dont use TELEMETRY_ANGLE and TELEMETRY_DEBUG at the same time!
//PDB: changed ZERO_SPEED to units of sec/step
//PDB: rather than clock cycles
//PDB: = 65535 cycles /2,000,000 cycles/sec
//PDB: drawn from original numbers used in B-ROBOT
//PDB: Then multiply by 1,000,000 to get microseconds
#define ZERO_SPEED 32768
#define MAX_ACCEL 14      // Maximun motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)

#define MICROSTEPPING 8   // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)

#define DEBUG 3   // 0 = No debug info (default) DEBUG 1 for console output
#define TUNE 1   // 0 = No tuning mode (default) TUNE 1 to tune stability control PID

// AUX definitions
//#define CLR(x,y) (x&=(~(1<<y)))
//#define SET(x,y) (x|=(1<<y))
#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

  //PDB String MAC;  // MAC address of Wifi module

uint8_t cascade_control_loop_counter = 0;
uint8_t loop_counter;       // To generate a medium loop 40Hz
uint8_t slow_loop_counter;  // slow loop 2Hz
uint8_t sendBattery_counter; // To send battery status
int16_t BatteryValue;

long timer_old;
long timer_value;
//PDB
double timer6_period;
double timer7_period;

float debugVariable;
float dt;

// Angle of the robot (used for stability control)
float angle_adjusted;
float angle_adjusted_Old;
float angle_adjusted_filtered=0.0;

// Default control values from constant definitions
float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;
float Kp_user = KP;
float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;
float Kp_position = KP_POSITION;
float Kd_position = KD_POSITION;
bool newControlParameters = false;
bool modifing_control_parameters = false;
int16_t position_error_sum_M1;
int16_t position_error_sum_M2;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
int16_t throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;
float angle_offset = ANGLE_OFFSET;

boolean positionControlMode = false;
uint8_t mode;  // mode = 0 Normal mode, mode = 1 Pro mode (More agressive)

//input speed to motors
int16_t motor1;
int16_t motor2;

// position control
volatile int32_t steps1;
volatile int32_t steps2;
int32_t target_steps1;
int32_t target_steps2;
int16_t motor1_control;
int16_t motor2_control;

int16_t speed_M1, speed_M2;        // Actual speed of motors
int8_t  dir_M1, dir_M2;            // Actual direction of steppers motors
int16_t actual_robot_speed;        // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;
float estimated_speed_filtered;    // Estimated robot speed

// OSC output variables
//PDB Testing using some of these
uint8_t OSCpage;
uint8_t OSCnewMessage;
float OSCfader[4];
float OSCxy1_x;
float OSCxy1_y;
float OSCxy2_x;
float OSCxy2_y;
uint8_t OSCpush[4];
uint8_t OSCtoggle[4];
uint8_t OSCmove_mode;
int16_t OSCmove_speed;
int16_t OSCmove_steps1;
int16_t OSCmove_steps2;


// INITIALIZATION
void setup()
{
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  ps2x_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  ps2x_type = ps2x.readType();

  //PDB STEPPER PINS MODIFIED FROM JJROBOTS BROBOT BRAIN BOARD -->
  // UDOO QUAD (Arduino Due)
  pinMode(8, OUTPUT); // ENABLE MOTORS
  pinMode(2, OUTPUT); // STEP MOTOR 1 PORTB25
  pinMode(5, OUTPUT); // DIR MOTOR 1  PORTC25
  pinMode(3, OUTPUT); // STEP MOTOR 2 PORTC28
  pinMode(6, OUTPUT); // DIR MOTOR 2  PORTC24
  digitalWrite(8, HIGH);  // Disable motors

  //PDB Need to check if I need to change servo code or pins for Arduino Due
  //PDB pinMode(10, OUTPUT);  // Servo1 (arm)
  //PDB pinMode(13, OUTPUT);  // Servo2

  Serial.begin(250000); // Serial output to console
  //  Serial1.begin(115200);
  //PDB  OSC_init();

  // Initialize I2C bus (MPU6050 is connected via I2C)
  Wire.begin();

#if DEBUG > 0
  delay(9000);
#else
  delay(1000);
#endif
  Serial.println("JJROBOTS");
  delay(200);
  Serial.println("Don't move for 10 sec...");
  MPU6050_setup();  // setup MPU6050 IMU
  delay(500);

  // With the new ESP8266 WIFI MODULE WE NEED TO MAKE AN INITIALIZATION PROCESS
  //PDB Comment out all of this
/*   Serial.println("WIFI init"); */
/*   Serial1.flush(); */
/*   Serial1.print("+++");  // To ensure we exit the transparent transmision mode */
/*   delay(100); */
/*   ESPsendCommand("AT", "OK", 1); */
/*   ESPsendCommand("AT+RST", "OK", 2); // ESP Wifi module RESET */
/*   ESPwait("ready", 6); */
/*   ESPsendCommand("AT+GMR", "OK", 5); */

/* #ifdef EXTERNAL_WIFI */
/*   ESPsendCommand("AT+CWQAP", "OK", 3); */
/*   ESPsendCommand("AT+CWMODE=1", "OK", 3); */
/*   //String auxCommand = (String)"AT+CWJAP="+WIFI_SSID+","+WIFI_PASSWORD; */
/*   char auxCommand[90] = "AT+CWJAP=\""; */
/*   strcat(auxCommand, WIFI_SSID); */
/*   strcat(auxCommand, "\",\""); */
/*   strcat(auxCommand, WIFI_PASSWORD); */
/*   strcat(auxCommand, "\""); */
/*   ESPsendCommand(auxCommand, "OK", 14); */
/* #ifdef WIFI_IP */
/*   strcpy(auxCommand, "AT+CIPSTA=\""); */
/*   strcat(auxCommand, WIFI_IP); */
/*   strcat(auxCommand, "\""); */
/*   ESPsendCommand(auxCommand, "OK", 4); */
/* #endif */
/*   ESPsendCommand("AT+CIPSTA?", "OK", 4); */
/* #else  // Deafault : we generate a wifi network */
/*   Serial1.println("AT+CIPSTAMAC?"); */
/*   ESPgetMac(); */
/*   //Serial.print("MAC:"); */
/*   //Serial.println(MAC); */
/*   delay(200); */
/*   ESPsendCommand("AT+CWQAP", "OK", 3); */
/*   ESPsendCommand("AT+CWMODE=2", "OK", 3); // Soft AP mode */
/*   // Generate Soft AP. SSID=JJROBOTS, PASS=87654321 */
/*   char *cmd =  "AT+CWSAP=\"JJROBOTS_XX\",\"87654321\",5,3"; */
/*   // Update XX characters with MAC address (last 2 characters) */
/*   cmd[19] = MAC[10]; */
/*   cmd[20] = MAC[11]; */
/*   ESPsendCommand(cmd, "OK", 6); */
/* #endif */
/*   // Start UDP SERVER on port 2222, telemetry port 2223 */
/*   Serial.println("Start UDP server"); */
/*   ESPsendCommand("AT+CIPMUX=0", "OK", 3);  // Single connection mode */
/*   ESPsendCommand("AT+CIPMODE=1", "OK", 3); // Transparent mode */
/*   char Telemetry[80]; */
/*   strcpy(Telemetry,"AT+CIPSTART=\"UDP\",\""); */
/*   strcat(Telemetry,TELEMETRY); */
/*   strcat(Telemetry,"\",2223,2222,0"); */
/*   ESPsendCommand(Telemetry, "OK", 3);  */

  // Calibrate gyros
  MPU6050_calibrate();

  //PDB  ESPsendCommand("AT+CIPSEND", ">", 2); // Start transmission (transparent mode)

  // Init servos
  //PDB  Serial.println("Servo init");
  //PDB  BROBOT_initServo();
  //PDB  BROBOT_moveServo1(SERVO_AUX_NEUTRO);

  // STEPPER MOTORS INITIALIZATION
  Serial.println("Stepers init");
  //PDB: Change this to use DueTimer library
  // MOTOR1 => TIMER6
  timer6_period = ZERO_SPEED;

  dir_M1 = 0;

  // MOTOR2 => TIMER7
  timer7_period = ZERO_SPEED;
  dir_M2 = 0;

  delay(200);
  Serial.print("timer6_period: ");
  Serial.println(timer6_period);
  Serial.print("timer7_period: ");
  Serial.println(timer7_period);

  // Enable stepper drivers and TIMER interrupts
  digitalWrite(8, LOW);   // Enable stepper drivers

  Serial.println("Steppers enabled?");
  Serial.println("NUM_TIMERS: ");
  Serial.println(NUM_TIMERS);

  // Enable TIMERs interrupts
  Timer6.attachInterrupt(runMotor1).start(timer6_period);
  Timer7.attachInterrupt(runMotor2).start(timer7_period);

  Serial.println("timer6 and timer7 started");


  // Little motor vibration and servo move to indicate that robot is ready
  for (uint8_t k = 0; k < 5; k++)
  {
    setMotorSpeedM1(5);
    setMotorSpeedM2(5);
    Serial.println("Motors forward!");
    //PDB    BROBOT_moveServo1(SERVO_AUX_NEUTRO + 100);
    //PDB    BROBOT_moveServo2(SERVO2_NEUTRO + 100);
    delay(200);
    setMotorSpeedM1(-5);
    setMotorSpeedM2(-5);
    Serial.println("Motors backward!");
    //PDB    BROBOT_moveServo1(SERVO_AUX_NEUTRO - 100);
    //PDB    BROBOT_moveServo2(SERVO2_NEUTRO - 100);
    delay(200);
    
  }
  //PDB  BROBOT_moveServo1(SERVO_AUX_NEUTRO);
  //PDB  BROBOT_moveServo2(SERVO2_NEUTRO);

/*PDB  #if TELEMETRY_BATTERY==1 */
/*   BatteryValue = BROBOT_readBattery(true); */
/*   Serial.print("BATT:"); */
/*   Serial.println(BatteryValue); */
/*PDB #endif */
  Serial.println("BROBOT by JJROBOTS v2.82");
  Serial.println("Start...");
  //PDB Arduino Due micros() uses native clock ticks not timer
  timer_old = micros();
}


// MAIN LOOP
void loop()
{

  if(ps2x_error == 1) //skip loop if no PS2 controller found
    return; 
  
  //PDB: Here you will probably substitute logic that gets input from PS2 controller
/*PDB   OSC_MsgRead();  // Read UDP OSC messages */
  ps2x.read_gamepad(false, vibrate);          //read PS2 controller and set large motor to spin at 'vibrate' speed

  OSCtoggle[0] = 0;  // Normal mode */
  rightStickY = ps2x.Analog(PSS_RY);
  leftStickX = ps2x.Analog(PSS_LX);

  if (rightStickY <= 100) {
    OSCfader[0] = mapf(rightStickY,0,100,1.0,0.5);
    //    motorRight->setSpeed(motorRightSpeed);
    //    motorRight->run(FORWARD);
  }
  else if (rightStickY >= 155) {
    OSCfader[0] = map(rightStickY,155,255,0.5,0.0);
    //    motorRight->setSpeed(motorRightSpeed);
    //    motorRight->run(BACKWARD);
  }
  else {
    OSCfader[0] = 0.5; // default neutral value
  }

  if (leftStickX <= 100) {
    OSCfader[1] = mapf(leftStickX,0,100,1.0,0.5);
    //    motorLeft->setSpeed(motorLeftSpeed);
    //    motorLeft->run(FORWARD);
  }
  else if (leftStickX >= 155) {
    OSCfader[1] = mapf(leftStickX,155,255,0.5,0.0);
    //    motorLeft->setSpeed(motorLeftSpeed);
    //    motorLeft->run(BACKWARD);
  }
  else {
    OSCfader[1] = 0.5; // default neutral value
  }


/*   if (OSCnewMessage) */
/*   { */
/*     OSCnewMessage = 0; */
/*     if (OSCpage == 1)   // Get commands from user (PAGE1 are user commands: throttle, steering...) */
/*     { */
/*       if (modifing_control_parameters)  // We came from the settings screen */
/*       { */
//         OSCfader[0] = 0.5; // default neutral values
//         OSCfader[1] = 0.5;
//         OSCtoggle[0] = 0;  // Normal mode */
/*       mode = 0; */
/*         modifing_control_parameters = false; */
/*       } */

/*       if (OSCmove_mode) */
/*       { */
/*         //Serial.print("M "); */
/*         //Serial.print(OSCmove_speed); */
/*         //Serial.print(" "); */
/*         //Serial.print(OSCmove_steps1); */
/*         //Serial.print(","); */
/*         //Serial.println(OSCmove_steps2); */
/*         positionControlMode = true; */
/*         OSCmove_mode = false; */
/*         target_steps1 = steps1 + OSCmove_steps1; */
/*         target_steps2 = steps2 + OSCmove_steps2; */
/*       } */
/*       else */
/*       { */
/*         positionControlMode = false; */
        throttle = (OSCfader[0] - 0.5) * max_throttle;
        // We add some exponential on steering to smooth the center band
        steering = OSCfader[1] - 0.5;
        if (steering > 0)
          steering = (steering * steering + 0.5 * steering) * max_steering;
        else
          steering = (-steering * steering + 0.5 * steering) * max_steering;
/*       } */

/*       if ((mode == 0) && (OSCtoggle[0])) */
/*       { */
/*         // Change to PRO mode */
/*         max_throttle = MAX_THROTTLE_PRO; */
/*         max_steering = MAX_STEERING_PRO; */
/*         max_target_angle = MAX_TARGET_ANGLE_PRO; */
/*         mode = 1; */
/*       } */
/*       if ((mode == 1) && (OSCtoggle[0] == 0)) */
/*       { */
/*         // Change to NORMAL mode */
/*         max_throttle = MAX_THROTTLE; */
/*         max_steering = MAX_STEERING; */
/*         max_target_angle = MAX_TARGET_ANGLE; */
/*         mode = 0; */
/*       } */
/*     } */
/*     else if (OSCpage == 2) { // OSC page 2 */
/*       // Check for new user control parameters */
/*       readControlParameters(); */
/*     } */
 #if DEBUG > 0
  Serial.print("Throttle: ");
  Serial.print(throttle);
  Serial.print(" Steering: ");
  Serial.println(steering);
 #endif
/*PDB   } // End new OSC message */

  timer_value = micros();
  //Serial.print("timer_value: ");
  //Serial.println(timer_value);

  // New IMU data?
  if (MPU6050_newData())
  {
    MPU6050_read_3axis();
    loop_counter++;
    slow_loop_counter++;
    dt = (timer_value - timer_old) * 0.000001; // dt in seconds
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;

    // Get new orientation angle from IMU (MPU6050)
    float MPU_sensor_angle = MPU6050_getAngle(dt);
    angle_adjusted = MPU_sensor_angle + angle_offset;
    //Seems like this should use angle_adjusted throughout
    /* if ((MPU_sensor_angle>-15)&&(MPU_sensor_angle<15)) */
    /*   angle_adjusted_filtered = angle_adjusted_filtered*0.99 + MPU_sensor_angle*0.01; */
    if ((angle_adjusted > -15)&&(angle_adjusted < 15))
      angle_adjusted_filtered = angle_adjusted_filtered*0.99 + angle_adjusted*0.01;

#if DEBUG==1
    Serial.print(dt);
    Serial.print(" ");
    Serial.print(angle_offset);
    Serial.print(" ");
    Serial.print(angle_adjusted);
    Serial.print(",");
    Serial.println(angle_adjusted_filtered);
#endif
    //Serial.print("\t");

    // We calculate the estimated robot speed:
    // Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
    actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward  

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0; // 25 is an empirical extracted factor to adjust for real units
    int16_t estimated_speed = -actual_robot_speed + angular_velocity;
    estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float)estimated_speed * 0.1; // low pass filter on estimated speed

#if DEBUG==2
    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.println(estimated_speed_filtered);
#endif

    if (positionControlMode)
    {
      // POSITION CONTROL. INPUT: Target steps for each motor. Output: motors speed
      motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
      motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

      // Convert from motor position control to throttle / steering commands
      throttle = (motor1_control + motor2_control) / 2;
      throttle = constrain(throttle, -190, 190);
      steering = motor2_control - motor1_control;
      steering = constrain(steering, -50, 50);
    }

    // ROBOT SPEED CONTROL: This is a PI controller.
    //    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
    //PDB: disable speed control for tuning of stability PID control
#if TUNE==0 
    target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output
#else 
    target_angle = 0;
#endif

#if DEBUG==3
    Serial.print("dt: ");
    Serial.print(dt);
    Serial.print(", ");
    Serial.print("angular_velocity: ");
    Serial.println(angular_velocity);
    Serial.print("angle_adjusted: ");
    Serial.println(angle_adjusted);
    Serial.print(",");
    Serial.println(angle_adjusted_filtered);
    Serial.print("estimated_speed_filtered ");
    Serial.println(estimated_speed_filtered);
    Serial.print("target_angle: ");
    Serial.println(target_angle);
#endif

    // Stability control (100Hz loop): This is a PD controller.
    //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
    //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
    control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

    // The steering part from the user is injected directly to the output
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    // Limit max speed (control output)
    motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
    motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

    int angle_ready;
    //PDB    if (OSCpush[0])     // If we press the SERVO button we start to move
    //PDB      angle_ready = 82;
    //PDB    else
      angle_ready = 74;  // Default angle
    if ((angle_adjusted < angle_ready) && (angle_adjusted > -angle_ready)) // Is robot ready (upright?)
    {
      // NORMAL MODE
      digitalWrite(8, LOW);  // Motors enable
      // NOW we send the commands to the motors
      setMotorSpeedM1(motor1);
      setMotorSpeedM2(motor2);
    }
    else   // Robot not ready (flat), angle > angle_ready => ROBOT OFF
    {
      digitalWrite(8, HIGH);  // Disable motors
      setMotorSpeedM1(0);
      setMotorSpeedM2(0);
      PID_errorSum = 0;  // Reset PID I term
      Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
      // RESET steps
      steps1 = 0;
      steps2 = 0;
      positionControlMode = false;
      //PDB      OSCmove_mode = false;
      throttle = 0;
      steering = 0;
    }

    // Push1 Move servo arm
    /*PDB if (OSCpush[0])  // Move arm */
    /* { */
    /*   if (angle_adjusted > -40) */
    /*     BROBOT_moveServo1(SERVO_MIN_PULSEWIDTH); */
    /*   else */
    /*     BROBOT_moveServo1(SERVO_MAX_PULSEWIDTH); */
    /* } */
    /* else */
    /*   BROBOT_moveServo1(SERVO_AUX_NEUTRO); */

    /* // Servo2 */
    /*PDB BROBOT_moveServo2(SERVO2_NEUTRO + (OSCfader[2] - 0.5) * SERVO2_RANGE); */

    // Normal condition?
    if ((angle_adjusted < 56) && (angle_adjusted > -56))
    {
      Kp = Kp_user;            // Default user control gains
      Kd = Kd_user;
      Kp_thr = Kp_thr_user;
      Ki_thr = Ki_thr_user;
    }
    else    // We are in the raise up procedure => we use special control parameters
    {
      Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
    }

  } // End of new IMU data

  // Medium loop 7.5Hz
  if (loop_counter >= 15)
  {
    loop_counter = 0;
    // Telemetry here?
#if TELEMETRY_ANGLE==1
    char auxS[25];
    int ang_out = constrain(int(angle_adjusted * 10),-900,900);
    sprintf(auxS, "$tA,%+04d", ang_out);
    Serial1.println(auxS);
#endif
#if TELEMETRY_DEBUG==1
    char auxS[50];
    sprintf(auxS, "$tD,%d,%d,%ld", int(angle_adjusted * 10), int(estimated_speed_filtered), steps1);
    Serial1.println(auxS);
#endif

  } // End of medium loop
  else if (slow_loop_counter >= 100) // 1Hz
  {
    slow_loop_counter = 0;
    // Read  status
#if TELEMETRY_BATTERY==1
    BatteryValue = (BatteryValue + BROBOT_readBattery(false)) / 2;
    sendBattery_counter++;
    if (sendBattery_counter >= 3) { //Every 3 seconds we send a message
      sendBattery_counter = 0;
      Serial.print("B");
      Serial.println(BatteryValue);
      char auxS[25];
      sprintf(auxS, "$tB,%04d", BatteryValue);
      Serial1.println(auxS);
    }
#endif
  }  // End of slow loop
}

