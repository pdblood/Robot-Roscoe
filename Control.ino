// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// License: GPL v2
// Control functions (PID controls, Steppers control...)

// PD controller implementation(Proportional, derivative). DT in seconds
//PDB: Changing Timer logic to use DueTimer Library
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-1).
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8); // We limit the input part...
  output = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / DT;
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  //PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return (output);
}


// PI controller implementation (Proportional, integral). DT in seconds
float speedPIControl(float DT, int16_t input, int16_t setPoint,  float Kp, float Ki)
{
  int16_t error;
  float output;

  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  //Serial.println(PID_errorSum);

  output = Kp * error + Ki * PID_errorSum * DT; // DT is in miliseconds...
  return (output);
}


float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM)
{
  float output;
  float P;

  P = constrain(Kpp * float(setPointPos - actualPos), -115, 115); // Limit command
  output = P + Kdp * float(speedM);
  return (output);
}

void stepDelay() {
	// Code below should be about 2 µS on a 84 Mhz Arduino Due
	for (auto i = 0; i < 40; ++i) {
		asm("nop \n");
	}
}

//PDB: Need to change this to a Due Timer
//PDB Callback for TIMER 6 : STEPPER MOTOR1 SPEED CONTROL
void runMotor1()
{
  if (dir_M1 == 0) // If we are not moving we dont generate a pulse
    return;

  // following this method:
  // https://hackaday.io/project/16483-building-the-thor-robot/log/59588-motor-control-faster-and-faster

  //Digital Pin 2 (Controlling X Motor Step) set HIGH - Start Pulse
  REG_PIOB_SODR = PIO_PB25; 
  // 2 µS pulse on a 84 Mhz Arduino Due
  stepDelay();

  if (dir_M1 > 0)
    steps1--;
  else
    steps1++;

  //Digital Pin 2 Set Low - End Pulse
  REG_PIOB_CODR = PIO_PB25; 
}
//PDB: Change to a Due Timer
//PDB: Callback for TIMER 7 : STEPPER MOTOR2 SPEED CONTROL
void runMotor2()
{
  if (dir_M2 == 0) // If we are not moving we dont generate a pulse
    return;

  REG_PIOC_SODR = PIO_PC28; //Pin 3 (Controlling Y Motor Step) - start pulse
  // 2 µS pulse on a 84 Mhz Arduino Due
  stepDelay();

  if (dir_M2 > 0)
    steps2--;
  else
    steps2++;

  REG_PIOC_CODR = PIO_PC28; //Pin 3 Step end pulse
}


// Set speed of Stepper Motor1
// tspeed could be positive or negative (reverse)
void setMotorSpeedM1(int16_t tspeed)
{
  double timer_period;
  double speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M1 - tspeed) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;

#if DEBUG > 0
  Serial.print("tspeed1: ");
  Serial.println(tspeed);
#endif

#if MICROSTEPPING==32
  speed = speed_M1 * 100; // Adjust factor from control output speed to real motor speed in steps/second
#elif MICROSTEPPING==16
  speed = speed_M1 * 50; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M1 * 25; // 1/8 Microstepping
#endif

#if DEBUG > 0
  Serial.print("speed1: ");
  Serial.println(speed);
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M1 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 1000000.0l / speed; //PDB sec/step * 1,000,000 =  useconds/step
    dir_M1 = 1;
    REG_PIOC_SODR = PIO_PC25; //Pin 5 (X Motor Dir) Set High
  }
  else
  {
    timer_period = 1000000.0l / -speed; //PDB sec/step * 1,000,000 =  useconds/step
    dir_M1 = -1;
    REG_PIOC_CODR = PIO_PC25; //Pin 5 Set Low (opposite direction)
  }
  if (timer_period > ZERO_SPEED)  // Check that timer does not exceed max period
    timer_period = ZERO_SPEED;

#if DEBUG > 0
  Serial.print("timer6_period_control: ");
  Serial.println(timer_period);
#endif

  Timer6.setPeriod(timer_period).start();
}

// Set speed of Stepper Motor2
// tspeed could be positive or negative (reverse)
void setMotorSpeedM2(int16_t tspeed)
{
  double timer_period;
  double speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M2 - tspeed) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;

#if DEBUG > 0
  Serial.print("tspeed2: ");
  Serial.println(speed);
#endif

#if MICROSTEPPING==32
  speed = speed_M2 * 100; // Adjust factor from control output speed to real motor speed in steps/second
#elif MICROSTEPPING==16
  speed = speed_M2 * 50; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M2 * 25; // 1/8 Microstepping
#endif

#if DEBUG > 0
  Serial.print("speed2: ");
  Serial.println(speed);
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M2 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 1000000.0l / speed; //PDB sec/step * 1,000,000 =  useconds/step
    dir_M2 = 1;
    REG_PIOC_SODR = PIO_PC24; //Pin 6 Set High (Y Motor Dir)
  }
  else
  {
    timer_period = 1000000.0l / -speed; //PDB sec/step * 1,000,000 =  useconds/step
    dir_M2 = -1;
    REG_PIOC_CODR = PIO_PC24; //Pin 6 Set Low (Run Opposite Dir)
  }
  if (timer_period > ZERO_SPEED)   // Check that timer does not exceed max period
    timer_period = ZERO_SPEED;

#if DEBUG > 0
  Serial.print("timer7_period_control: ");
  Serial.println(timer_period);
#endif

  Timer7.setPeriod(timer_period).start();
}

