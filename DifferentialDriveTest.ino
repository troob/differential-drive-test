/*
 * Differential Drive Test
 * Command to Drive or Turn to given position (w/ speedLimit)
 * Command to Drive or Turn at given velocity.
 * run one wheel for one pulse at a time, 
 * observing changes in Drive and Turn pulses
 */

//======Advisor======
//===arbitration struct===
typedef struct layer LAYER; // C struct for subsumption task output

struct layer
{
  int cmd, // assertion command
    arg, // assertion argument
    flag; // subsumption flag (instead of layer state?)
};

LAYER rotate,
  moveStraight,
  halt; // default

const int job1Size = 3;

LAYER *job1[job1Size] = { &rotate, &moveStraight, &halt };

LAYER *thisLayer = &halt;

LAYER **job;

int jobSize, // number of tasks in priority list
  arbitrate; // global flag to enable subsumption

volatile byte rotateEnable,
  haltBot;

//======Interface======
String inputString = "";

boolean stringComplete = false;  // whether the string is complete

//======Encoders======
const byte esPins[] = 
{
  3, // encoder signal 1 pin
  7 // encoder signal 2 pin
};

const byte numEncoders = 2,
  pulsesPerRev = 20;

double minAngularRes; // [deg/pulse]

// values change in callback methods:
volatile int velPulseCounts[numEncoders],
  rotVels[numEncoders],
  prevRotVels[numEncoders],
  setVels[numEncoders];
  
volatile long pulseCounts[numEncoders];
  
//======Motor Driver======
const byte mSigPins[] = { 8, 9 },
  mEnablePins[] = { 5, 6 };
  
//======Mobile Platform (DFRobot Turtle)======
int wheelDiam = 64, // [mm]
  botDiam = 138; // [mm] (i.e. wheel base or distance between wheel centers
  
//======Circle======
float piApprox = 3.14159,
  degsPerRad = 57.2958; // radians to deg conversion

//======Controller======
const int numMtrs = 2;

int sensorsTmrCtr,
  maxOutVal,
  pubVelRate = 10, // [Hz]
  topRotVel, // [pulses/(1/pubVelRate)s]
  minLinearRes; // [mm/pulse]

volatile int measRelHeading,
  measAbsHeading,
  botVel; // global, current requested robot velocity

volatile double kp, ki, kd;

volatile byte signs[numMtrs];

volatile int pulses[numMtrs],
  pubMtrCmds[numMtrs],
  mtrOutAccums[numMtrs];
  
volatile long samples[numMtrs],
  prevSamples[numMtrs];

void setup() 
{
  initSystem();

  initBehaviors();

  initSensorsTimer(); 

  Serial.println("Heading (deg)");
  Serial.println();
}

int initSystem()
{
  initNode("DifferentialDriveTest");

  initSubscribers();

  initPublishers();
  
  return 0;
}

void initNode(String id)
{
  Serial.begin(9600);

  while(!Serial);
  
  Serial.print("Starting ");
  Serial.print(id);
  Serial.println(".ino\n");
}

void initSubscribers()
{
  // pulse count
  attachInterrupt(digitalPinToInterrupt(esPins[0]), encoder1Callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(esPins[1]), encoder2Callback, CHANGE);
}

void initPublishers()
{
  /* Start Motor Channels */
  for(int i=0; i < numMtrs; i++)
  {
    pinMode(mEnablePins[i], OUTPUT);
    pinMode(mSigPins[i], OUTPUT);
  }
}

void initBehaviors()
{
  initVars();

  setParams();
  
  initJob1(); // set job1 as active job at startup time
}

void initVars()
{
  for(int i=0; i < numMtrs; i++)
  {
    velPulseCounts[i] = 0;
  
    pulseCounts[i] = 0; // left and right distances
  
    pubMtrCmds[i] = 0;
  
    signs[i] = 1;
  
    mtrOutAccums[i] = 0;
  
    rotVels[i] = 0;
  
    prevRotVels[i] = 0;

    samples[i] = 0;

    prevSamples[i] = 0;

    pulses[i] = 0;
  }

  measRelHeading = 0; // [deg]
  measAbsHeading = 0; // [deg]
}

void setParams()
{
  setPIDGains(1.0, 0.0, 0.0);

  maxOutVal = 100 * 256; // max. output value in fixed point integer
  
  pubVelRate = 10; // [Hz]

  minAngularRes = 360.0 / pulsesPerRev;
  
  minLinearRes = (int) round( piApprox * wheelDiam / pulsesPerRev ); // r_{min} [mm/pulse]

  rotateEnable = 0;

  arbitrate = 1;

  haltBot = 1;

  topRotVel = 2; // [pulses/(1/pubVelRate)s]
}

void setPIDGains(double pg, double ig, double dg)
{
  if(pg < 0 || ig < 0 || dg < 0) return;
  
  kp = pg;

  ki = ig;

  kd = dg;

  Serial.print("Controller got kp:");
  Serial.print(kp, 3);
  Serial.print(", ki:");
  Serial.print(ki, 3);
  Serial.print(", kd:");
  Serial.println(kd, 3);
}

int initJob1() // make job1 the active job
{
  job = &job1[0]; // global job priority list pointer

  jobSize = job1Size; // no. tasks in job1 list

  return 0;
}

void initSensorsTimer()
{
  noInterrupts();           // disable all interrupts
  
  TCCR1A = 0;
  TCCR1B = 0;
  sensorsTmrCtr = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
  
  TCNT1 = sensorsTmrCtr;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  
  interrupts();             // enable all interrupts
}

void loop() {}

//======Interrupt Service Routines======
void encoder1Callback()
{
  velPulseCounts[0]++;

  determinePulseCounts(1);

  Serial.print("Vel. Pulse Count 1: ");
  Serial.print(velPulseCounts[0]);
  Serial.println("\n");
}

void encoder2Callback()
{
  velPulseCounts[1]++;

  determinePulseCounts(2);

  Serial.print("Vel. Pulse Count 2: ");
  Serial.print(velPulseCounts[1]);
  Serial.println("\n");
}

void determinePulseCounts(int id)
{
  if(signs[id - 1] == 1)
    pulseCounts[id - 1]++;
  else
    pulseCounts[id - 1]--;

  Serial.print("Pulse Count ");
  Serial.print(id);
  Serial.print(": ");
  Serial.print(pulseCounts[id-1]);
  Serial.println("\n");
}

ISR(TIMER1_OVF_vect)
{
  readUserInput();

  moveStraightTask();

  rotateTask();

  arbitrator();
}

void readUserInput()
{
  if(stringComplete)
  {
    Serial.print("inputString: ");

    // receive command from user
    if(inputString.substring(0,1) == "g")
    {
      Serial.println("go");

      rotateEnable = 1;

      haltBot = 0;
    }
    else if(inputString.substring(0,1) == "s")
    {
      Serial.println("stop");

      rotateEnable = 0;

      haltBot = 1;
    }
//    else if(inputString.substring(0,1) == "t") // given in m/s, convert to mm/s, so operations can be done with integer
//    { 
//      setTanVel = (int) round( inputString.substring(1, inputString.length()).toFloat() * 1000 ); // get string after 't'
//    
//      Serial.print("v_t = ");
//      Serial.print(setTanVel / 1000.0);
//      Serial.println(" m/s\n");
//
//      followEnable = 1;
//    }
//    else if(inputString.substring(0,1) == "r") // given in deg/s
//    {
//      setRotVel = inputString.substring(1, inputString.length()).toInt(); // get string after 'r'
//      
//      Serial.print("\\omega = ");
//      Serial.print(setRotVel);
//      Serial.println(" deg/s\n");
//
//      followEnable = 1;
//    }
//    else if(inputString.substring(0,1) == "x") // given in m
//    {
//      setX = (int) round( inputString.substring(1, inputString.length()).toFloat() * 100 ); // get string after 'x'
//    
//      Serial.print("x = ");
//      Serial.print(setX);
//      Serial.println(" cm\n");
//    }
//    else if(inputString.substring(0,1) == "y") // given in m
//    {
//      setY = (int) round( inputString.substring(1, inputString.length()).toFloat() * 100 ); // get string after 'y'; inputString.substring(1, inputString.length()).toInt();
//    
//      Serial.print("y = ");
//      Serial.print(setY);
//      Serial.println(" cm\n");
//    }
//    else if(inputString.substring(0,1) == "h") // given in deg
//    {
//      setHeading = inputString.substring(1, inputString.length()).toInt(); // get string after 'h'
//    
//      Serial.print("heading = ");
//      Serial.print(setHeading);
//      Serial.println(" deg\n");
//
//      rotateEnable = 1;
//
//      followEnable = 0;
//
//      searchEnable = 0;
//
//      haltBot = 0;
//    }
    else if(inputString.substring(0,2) == "kp")
      kp = inputString.substring(2, inputString.length()).toFloat(); // get string after 'kp'
    else if(inputString.substring(0,2) == "ki")
      ki = inputString.substring(2, inputString.length()).toFloat(); // get string after 'ki'
    else if(inputString.substring(0,2) == "kd")
      kd = inputString.substring(2, inputString.length()).toFloat(); // get string after 'kd'

    // clear string:
    inputString = ""; //note: in code below, inputString will not become blank, inputString is blank until '\n' is received

    stringComplete = false;
  }

  if(Serial.available())
    serialEvent();
}

void serialEvent()
{
  while (Serial.available()) 
  {
    // get the new byte:
    char inChar = (char) Serial.read();
    
    // add it to the inputString:
    inputString += inChar;
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n')
      stringComplete = true;
  }
}

/* Run arbitrarily long, continously printing heading. 
 * When stopped, check that theoretical matches actual heading.
 */
void rotateTask()
{
  extern LAYER rotate;

  readAngle();

  if(rotateEnable)
  {
    rotate.cmd = 0;

    rotate.arg = topRotVel; // [pulses/(1/pubVelRate)s]
    
    rotate.flag = true;
  }
  else
    rotate.flag = false;
}

void readAngle()
{
  int i;

  for(i=0; i < numMtrs; i++)
    samples[i] = pulseCounts[i];

  for(i=0; i < numMtrs; i++)
    pulses[i] = samples[i] - prevSamples[i]; // P pulses

  for(i=0; i < numMtrs; i++)
    prevSamples[i] = samples[i];

  measRelHeading = (int) round( 360 * minLinearRes * ( pulses[1] - pulses[0] ) / ( piApprox * botDiam ) ); //2 * minLinearRes * ( pulses[1] - pulses[0] ) / botDiam; // theta_j^{(rel)} [rad]; before simplified: 2 * piApprox * minLinearRes * ( pulses[1] - pulses[0] ) / ( piApprox * botDiam )

  measAbsHeading = ( measAbsHeading + measRelHeading ) % 360; // [deg]

  Serial.println(measAbsHeading);
  Serial.println();
}

/* Run arbitrarily long, continously printing displacement. 
 * When stopped, check that theoretical matches actual displacement.
 */
void moveStraightTask()
{
  extern LAYER moveStraight;

  readDisplacement();
}

void readDisplacement()
{
  
}

/* "winnerId" feedback line 
 * from Arbitrate back to tasks:
 * Essentially global var containing ID of task 
 * that "won" this round of arbitration.
 * It can be used by the individual tasks
 * to determine if they have been subsumed.
 */
void arbitrator()
{
  int i = 0;

  if(arbitrate)
  {
    for(i=0; i < jobSize - 1; i++) // step through tasks
    {
      if(job[i]->flag) break; // subsume
    }

    thisLayer = job[i]; // global output winner
  }

  mtrCmd(thisLayer); // send command to motors; execute, given pointer to winning layer
}

void mtrCmd(LAYER *l)
{
  botVel = l->cmd; // [pulses/(1/pubVelRate)s], current requested velocity

  // ADD: convert mm to pulses and deg to pulses before computing setVels!
  // Compute control signals:
  setVels[0] = botVel + l->arg; // // left motor = velocity + rotation, (int) round( 1000 * setRotVel / ( minAngularRes * pubVelRate ) ); // convert [deg/s] to [pulses/(1/pubVelRate)s]
  setVels[0] = clip(setVels[0], 100, -100); // don't overflow +/- 100% full speed
  
  setVels[1] = botVel - l->arg; // right motor = velocity - rotation
  setVels[1] = clip(setVels[1], 100, -100); // don't overflow +/- 100% full speed

//  Serial.print("Set Vels (pulses/(1/pubVelRate)s): ");
//  for(int i=0; i < numMtrs; i++)
//  {
//    Serial.print(setVels[i]);
//    Serial.print(" ");
//  }
//  Serial.println("\n");

  if(haltBot)
    stopMoving();
  else
    controlVel(); // PID
}

void stopMoving()
{
  for(int i=0; i < numMtrs; i++)
    digitalWrite(mEnablePins[i], LOW);
}

void controlVel()
{
  int aOutputs[numMtrs];

  // Read analog input (i.e. calc rot vel):
  speedometer();
  
  // Compute control signals:
  computeControlSignals(); 

  // Set analog outputs:
  for(int i=0; i < 2; i++)
    aOutputs[i] = (int) round( mtrOutAccums[i] / 256.0 );
  
  modulatePulseWidths(aOutputs); // Note: divide by 256 and earlier multiply by 256 b/c earlier operation in fixed point integer
}

/* Read and zero velPulseCounts.
 * Copy and accumulate counts from velPulseCounts
 * to the rot. vel. variable and
 * then reset velPulseCounts to zero.
 */
void speedometer()
{
  for(int i=0; i < numMtrs; i++)
  {
    rotVels[i] = velPulseCounts[i] * signs[i]; // copy and accumulate counts from velPulseCount to rotVel
    velPulseCounts[i] = 0; // reset velPulseCount to zero
  }
}

/* Basic behavior: generate error signal 
 * based on difference b/t measured rotVel and
 * requested rotVel for the wheel.
 * Use of error signal: Increase or decrease speed of motor
 * to force measured to equal requested rotVel
 * Input to PID controller: Requested rotVel "vel,"
 * which is input rotVel expressed as encoder pulses
 * per 1/pubVelRate second.
 */
void computeControlSignals()
{
  int errs[numMtrs],
    P[numMtrs],
    I[numMtrs],
    D[numMtrs];
  
  int b = 1; // set point weight
  
//  Serial.print("Controller got kp:");
//  Serial.print(kp);
//  Serial.print(", ki:");
//  Serial.print(ki);
//  Serial.print(", kd:");
//  Serial.println(kd, 3);

  for(int i=0; i < numMtrs; i++)
  {
    errs[i] = (int) round( ( b * setVels[i] - rotVels[i] ) * 256 ); // [pulses/(1/pubVelRate)s]*256, generate error signal based on difference b/t measured rotVel and requested rotVel for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
    
    P[i] = (int) round( errs[i] / kp ); // P(t_k) = K(by_{sp}(t_k) — y(t_k))

    D[i] = (int) round( ( ( rotVels[i] - prevRotVels[i] ) * 256 ) / kd ); // large when amount of change requested by PID controller is large, and small as signal approaches zero

    mtrOutAccums[i] += P[i] + D[i]; // Increase or decrease speed of motor to force measured to equal requested rotVel

    prevRotVels[i] = rotVels[i]; // maintain history of previous measured rotVel

    mtrOutAccums[i] = clip(mtrOutAccums[i], maxOutVal, -maxOutVal); // accumulator
  }
}

/* The accumulator motorOutAccum must be clipped 
 * at some positive and negative value
 * to keep from overflowing the fixed point arithmetic.
 */
int clip(int a, int maximum, int minimum)
{ 
  if(a > maximum) 
    a = maximum;
  else if(a < minimum) 
    a = minimum;

  return a;
}

/* The PWM code drives the hardware H-bridge, 
 * which actually control the motor.
 * This routine takes a signed value, 
 * -100 < signedVal < 100,
 * sets the sign variable used by the speedometer code,
 * sets the forward/backward (i.e. direct/reverse) bits 
 * on the H-bridge, and
 * uses abs(signedVal) as an index into a 100 entry table 
 * of linear PWM values.
 * This function uses a timer interrupt to generate 
 * a x Hz (maybe 120 Hz) variable pulse-width output.
 */
void modulatePulseWidths(int signedVals[]) // take signed value, b/t -100 and 100
{
  int i;
  
  for(i=0; i < numMtrs; i++)
  {
    setSpeedometerSign(i, signedVals[i]); // set sign variable used by speedometer code

    setHBridgeDirectionBit(i, signedVals[i]);
  
    pubMtrCmds[i] = getPWMValueFromEntryTable(i, abs(signedVals[i])); // use abs(signedVal) as an index into a 100 entry table of linear PWM values
  }
  
  for(i=0; i < numMtrs; i++)
    analogWrite(mEnablePins[i], pubMtrCmds[i]); // generate variable pulse-width output
}

/* The sign variable represents the direction of rotation
 * of the motor (1 for forward and -1 for backward).
 * With more expensive quadrature encoders this info is
 * read directly from the encoders.
 * In this implementation I have only simple encoders so
 * the direction of rotation is taken from the sign of the
 * most recent command issued by the PID to the PWM code.
 */
void setSpeedometerSign(int mid, int signedVal) // signedVal should be most recent cmd issued by PID to PWM code
{
  if(signedVal < 0) // {motor direction of rotation} = backward
    signs[mid] = -1;
  else if(signedVal >= 0)
    signs[mid] = 1; // {motor direction of rotation} = {forward | resting}
  else
    Serial.println("Invalid command issued by PID to PWM code!\n");

  Serial.print("M");
  Serial.print(mid + 1);
  Serial.print(" speedometer sign: ");
  Serial.println(signs[mid]);
}

void setHBridgeDirectionBit(int mid, int signedVal)
{
  if(signedVal < 0) // {motor direction of rotation} = backward
  { 
    if(mid == 0) digitalWrite(mSigPins[mid], HIGH);
    else if(mid == 1) digitalWrite(mSigPins[mid], LOW);
  }
  else if(signedVal >= 0) // {motor direction of rotation} = {forward | resting}
  { 
    if(mid == 0) digitalWrite(mSigPins[mid], LOW);
    else if(mid == 1) digitalWrite(mSigPins[mid], HIGH);
  }
  else
    Serial.println("Invalid command issued by PID to PWM code!\n");
}

// use magnitude as an index into a 100 entry table of linear PWM values
int getPWMValueFromEntryTable(int mid, int magnitude)
{
  return (int) round( magnitude * 255.0 / 100 ); // cruise outputs
}
