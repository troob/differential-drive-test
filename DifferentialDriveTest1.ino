/*
 * Differential Drive Test
 * Command to Drive or Turn to given position (w/ speedLimit)
 * Command to Drive or Turn at given velocity.
 * run one wheel for one pulse at a time, 
 * observing changes in Drive and Turn pulses
 */

#include <math.h>

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
  search,
  halt; // default

const int job1Size = 4;

LAYER *job1[job1Size] = { &rotate, &moveStraight, &search, &halt };

LAYER *thisLayer = &halt;

LAYER **job;

int jobSize, // number of tasks in priority list
  arbitrate; // global flag to enable subsumption

volatile byte rotateEnable,
  moveStraightEnable,
  searchEnable,
  setAngleActive,
  setDispActive,
  setPointActive,
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
  topTanVel, // [pulses/(1/pubVelRate)s]
  topRotVel, // [pulses/(1/pubVelRate)s]
  minLinearRes, // [mm/pulse]
  navDeadZone, // [deg]
  setRadius, // [mm]
  threshIntegral; // [pulses/(1/pubVelRate)s]

volatile int measRelHeading, // [deg]
  measAbsHeading, // [deg]
  measAbsDisp, // [mm]
  measRelDisp,
  headingError, // [deg]
  setAbsHeading,
  setUserAbsHeading,
  setWyPtAbsHeading,
  setAbsDisp, // [mm]
  setUserAbsDisp,
  setWyPtAbsDisp,
  setWyPtRelDisp,
  dispError, // [mm]
  botVel, // global, current requested robot velocity
  setTanVel, // [pulses/(1/pubVelRate)s]
  setRotVel, // [pulses/(1/pubVelRate)s]
  setX, // [mm]
  setY, // [mm]
  measX, // [mm]
  measY, // [mm]
  dx, dy;

volatile double kp, ki, kd;

volatile int pulses[numMtrs],
  pubMtrCmds[numMtrs],
  mtrOutAccums[numMtrs],
  signs[numMtrs];
  
volatile long samples[numMtrs],
  prevSamples[numMtrs];

void setup() 
{
  initSystem();

  initBehaviors();

  initSensorsTimer(); 
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

  displayCommands();
}

void displayCommands()
{
  Serial.println("======User Commands======");
  Serial.println("s: stop moving (i.e. halt robot)");
  Serial.println("ahx: set absolute heading to x deg");
  Serial.println("rhx: set relative heading to x deg");
  Serial.println("adx: set absolute displacement to x mm");
  Serial.println("rdx: set relative displacement to x mm");
  Serial.println("tvx: set tangential velocity to x mm/s");
  Serial.println("rvx: set rotational velocity to x deg/s");
  Serial.println("xx: set x coordinate to x mm");
  Serial.println("yx: set y coordinate to x mm");
  Serial.println();
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
  measAbsHeading = 90; // [deg]
  setAbsHeading = 90; // [deg]
  setUserAbsHeading = 90; // [deg]
  setWyPtAbsHeading = 90;
  headingError = 0; // [deg]

  measAbsDisp = 0; // [mm]
  measRelDisp = 0;
  setAbsDisp = 0; // [mm]
  setUserAbsDisp = 0;
  setWyPtAbsDisp = 0;
  setWyPtRelDisp = 0;

  topTanVel = 2; // [pulses/(1/pubVelRate)s]
  setTanVel = topTanVel;

  topRotVel = 2; // [pulses/(1/pubVelRate)s]
  setRotVel = topRotVel;

  setX = 0; // [mm]
  measX = 0; // [mm]
  
  setY = 0; // [mm]
  measY = 0; // [mm]
}

void setParams()
{
  setPIDGains(1.125, 0.1, 0.0); // kd may be good at 0.5; init vals tuned roughly

  maxOutVal = 100 * 256; // max. output value in fixed point integer
  
  pubVelRate = 10; // [Hz]

  minAngularRes = 360.0 / pulsesPerRev;
  
  minLinearRes = (int) round( piApprox * wheelDiam / pulsesPerRev ); // r_{min} [mm/pulse]

  rotateEnable = 0;

  moveStraightEnable = 0;

  searchEnable = 0;

  setAngleActive = 0;

  setDispActive = 0;

  setPointActive = 0;

  arbitrate = 1;

  haltBot = 1;

  navDeadZone = 18; // [deg], CHANGE: tune based on resolution

  setRadius = 18; // [mm]

  threshIntegral = 2;
}

void setPIDGains(float pg, float ig, float dg)
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

  observeSample();

  searchTask();

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
//    if(inputString.substring(0,1) == "g")
//    {
//      Serial.println("go");
//
//      rotateEnable = 1;
//
//      haltBot = 0;
//    }
    if(inputString.substring(0,1) == "s")
    {
      Serial.println("stop");

      rotateEnable = 0;

      moveStraightEnable = 0;

      searchEnable = 0;

      haltBot = 1;
    }
    else if(inputString.substring(0,2) == "tv") // given in mm/s, convert to pulses/(1/pubVelRate)s, so operations can be done with integer
    { 
      setTanVel = (int) round( convertMMToPulses(inputString.substring(2, inputString.length()).toInt()) / (float) pubVelRate ); // get string after 'tv'
    
      Serial.print("v_t = ");
      Serial.print(inputString.substring(2, inputString.length()).toInt());
      Serial.println(" mm/s\n");

      moveStraightEnable = 1;
      
      setDispActive = 0;

      haltBot = 0;
    }
    else if(inputString.substring(0,2) == "rv") // given in deg/s
    {
      setRotVel = (int) round( convertDegToPulses(inputString.substring(2, inputString.length()).toInt()) / (float) pubVelRate ); // get string after 'rv'
      
      Serial.print("\\omega = ");
      Serial.print(inputString.substring(2, inputString.length()).toInt());
      Serial.println(" deg/s\n");
    }
    else if(inputString.substring(0,1) == "x") // given in mm
    {
      setX = (int) round( inputString.substring(1, inputString.length()).toInt() ); // get string after 'x'
    
      Serial.print("x = ");
      Serial.print(setX);
      Serial.println(" mm\n");

      rotateEnable = 0;

      moveStraightEnable = 0;

      searchEnable = 1;

      setPointActive = 1;

      haltBot = 0; // consider waiting to set equal to 0 for inside search task, once you've used the knowledge that haltBot was previously = 1 to determine whether you're starting from rest
    }
    else if(inputString.substring(0,1) == "y") // given in m
    {
      setY = (int) round( inputString.substring(1, inputString.length()).toInt() ); // get string after 'y'; inputString.substring(1, inputString.length()).toInt();
    
      Serial.print("y = ");
      Serial.print(setY);
      Serial.println(" mm\n");

      rotateEnable = 0;

      moveStraightEnable = 0;

      searchEnable = 1;

      setPointActive = 1;

      haltBot = 0; // consider waiting to set equal to 0 for inside search task, once you've used the knowledge that haltBot was previously = 1 to determine whether you're starting from rest
    }
    else if(inputString.substring(0,2) == "ah") // absolute heading, given in deg
    {
      setAbsHeading = inputString.substring(2, inputString.length()).toInt(); // get string after 'ah'
    
      Serial.print("Set Abs Heading (deg): ");
      Serial.print(setAbsHeading);
      Serial.println("\n");

      rotateEnable = 1;

      setAngleActive = 1;

      moveStraightEnable = 0;

      searchEnable = 0;

      haltBot = 0;
    }
    else if(inputString.substring(0,2) == "rh") // relative heading, given in deg
    {
      setAbsHeading = measAbsHeading + inputString.substring(2, inputString.length()).toInt(); // get string after 'ah'
    
      Serial.print("Set Abs Heading (deg): ");
      Serial.print(setAbsHeading);
      Serial.println("\n");

      rotateEnable = 1;

      setAngleActive = 1;

      moveStraightEnable = 0;

      searchEnable = 0;

      haltBot = 0;
    }
    else if(inputString.substring(0,2) == "ad") // absolute displacement, given in mm
    {
      setAbsDisp = inputString.substring(2, inputString.length()).toInt(); // get string after 'ad'
    
      Serial.print("Set Disp (mm): ");
      Serial.print(setAbsDisp);
      Serial.println("\n");

      rotateEnable = 0;

      setAngleActive = 0;

      moveStraightEnable = 1;

      setDispActive = 1;

      searchEnable = 0;

      haltBot = 0;
    }
    else if(inputString.substring(0,2) == "rd") // relative displacement, given in mm
    {
      setAbsDisp = measAbsDisp + inputString.substring(2, inputString.length()).toInt(); // get string after 'rd'
    
      Serial.print("Set Disp (mm): ");
      Serial.print(setAbsDisp);
      Serial.println("\n");

      rotateEnable = 0;

      setAngleActive = 0;

      moveStraightEnable = 1;

      setDispActive = 1;

      searchEnable = 0; // may not be necessary if desired to keep searching after higher priority command done

      haltBot = 0;
    }
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

// get displacement of each wheel to get this instant's
// linear and angular displacements
void observeSample()
{
  int i;

  for(i=0; i < numMtrs; i++)
    samples[i] = pulseCounts[i];

//  Serial.print("samples[0]: ");
//  Serial.println(samples[0]);

  for(i=0; i < numMtrs; i++)
    pulses[i] = samples[i] - prevSamples[i]; // P pulses

//  Serial.print("pulses[0]: ");
//  Serial.println(pulses[0]);
  
  for(i=0; i < numMtrs; i++)
    prevSamples[i] = samples[i];
}

void searchTask()
{
  extern LAYER search;

  readLocation();

  if(searchEnable)
  {
    if(setPointActive == 0) // do halt behavior
    {
      search.cmd = 0; // yes, stop
  
      search.arg = 0; // stop turning
    }
    else
    {
      locateTarget(); // rotate will need to know setWyPtAbsHeading

      if(abs(headingError) > navDeadZone && abs(dispError) > setRadius)
      {
        Serial.print(abs(headingError));
        Serial.println(" deg > navDeadZone");
        Serial.print(abs(dispError));
        Serial.println(" mm > setRadius");
        Serial.print("===Pivot to ");
        Serial.print(setWyPtAbsHeading);
        Serial.println(" deg===");
        
        search.cmd = 0; // pivot

        if(headingError > 0)
          if(headingError > 180)
            search.arg = -setRotVel; // CCW
          else
            search.arg = setRotVel; // CW
        else
          if(headingError < -180)
            search.arg = setRotVel; // CW
          else
            search.arg = -setRotVel; // CCW
      }
      else if(abs(dispError) > setRadius) // do move straight behavior
      {
        Serial.print(abs(headingError));
        Serial.println(" deg <= navDeadZone");
        Serial.print(abs(dispError));
        Serial.println(" mm > setRadius");
        Serial.print("===Move Straight ");
        Serial.print(setWyPtRelDisp);
        Serial.println(" mm===");
        
        // if disp error negative, it means you're at a position less than set position, so you should go in positive direc (forward) to correct that
        if(dispError < 0)
          search.cmd = setTanVel; // forward
        else
          search.cmd = -setTanVel; // backward
          
        search.arg = 0; // straight ahead
      }
      else // found target
      {
        haltBot = 1; // needed b/c PID control shift down to 0 takes too long b/c not tuned

        setPointActive = 0; // signal that target is acquired
      }
    }
    
    search.flag = true;
  }
  else
    search.flag = false;
}

void readLocation()
{
  Serial.println("======Read Location======");

  readDisplacement();
  
  readAngle();

  readCoordinates();

  Serial.println();
}

void readDisplacement()
{
  Serial.println("===Read Displacement===");

  measRelDisp = (int) round( minLinearRes * ( pulses[0] + pulses[1] ) / 2.0 ); // [mm]

  Serial.print("measRelDisp (mm): ");
  Serial.println(measRelDisp);
  
  //measAbsDisp += measRelDisp; // [mm]

  measAbsDisp = (int) round( minLinearRes * ( samples[0] + samples[1] ) / 2.0 ); // [mm]

  Serial.print("measAbsDisp = minLinearRes * ( samples[0] + samples[1] ) / 2.0 ) = ");
  Serial.print(minLinearRes);
  Serial.print(" * ( ");
  Serial.print(samples[0]);
  Serial.print(" + ");
  Serial.print(samples[1]);
  Serial.print(" ) / 2.0 = ");
  Serial.print(measAbsDisp);
  Serial.println(" mm");
}

void readAngle()
{
  Serial.println("===Read Angle===");
  
  measRelHeading = (int) round( 360 * minLinearRes * ( pulses[1] - pulses[0] ) / ( piApprox * botDiam ) ); //2 * minLinearRes * ( pulses[1] - pulses[0] ) / botDiam; // theta_j^{(rel)} [rad]; before simplified: 2 * piApprox * minLinearRes * ( pulses[1] - pulses[0] ) / ( piApprox * botDiam )

  Serial.print("measRelHeading (deg): ");
  Serial.println(measRelHeading);
  
  measAbsHeading = ( measAbsHeading + measRelHeading ) % 360; // [deg]

  if(measAbsHeading < 0)
    measAbsHeading += 360;
    
  Serial.print("measAbsHeading (deg): ");
  Serial.println(measAbsHeading);
}

void readCoordinates()
{
  Serial.println("===Read Coordinates===");

  // measRelDisp is like position vector magnitude
  measX += (int) round ( measRelDisp * sin( -measRelHeading / degsPerRad ) ); // [mm]

  measY += (int) round ( measRelDisp * cos( measRelHeading / degsPerRad ) ); // [mm]

  Serial.print("measX (mm): ");
  Serial.println(measX);
  Serial.print("measY (mm): ");
  Serial.println(measY);
}

// NEXT: fix heading err calculation
// probably only needed when converting from cartesian to cylindrical coordinates
void locateTarget()
{   
  int he1, he2; // heading errors
  
  Serial.print("======Locate Target (");
  Serial.print(setX);
  Serial.print(", ");
  Serial.print(setY);
  Serial.println(")======");
 
  dx = setX - measX;
  dy = setY - measY;

  Serial.print("dx = setX - measX = ");
  Serial.print(setX);
  Serial.print(" mm - ");
  Serial.print(measX);
  Serial.print(" mm = ");
  Serial.print(dx);
  Serial.println(" mm");
  Serial.print("dy = setY - measY = ");
  Serial.print(setY);
  Serial.print(" mm - ");
  Serial.print(measY);
  Serial.print(" mm = ");
  Serial.print(dy);
  Serial.println(" mm");

  Serial.print("setWyPtAbsHeading = ");
  if(abs(dx) > 0.00001)
  {
    setWyPtAbsHeading = atan2(dy, dx) * degsPerRad;

    Serial.print("atan2(dy, dx) = atan2(");
    Serial.print(dy);
    Serial.print(", ");
    Serial.print(dx);
    Serial.print(") = ");
    
  }
  else if(dx == 0 && dy == 0) // note: dx == 0 is redundant b/c it must be 0 to reach this point
  {
    setWyPtAbsHeading = measAbsHeading;

    Serial.print("measAbsHeading = ");
  }
  else if(dx == 0)
  {
    if(dy > 0)
      setWyPtAbsHeading = 90;
    else if(dy < 0)
      setWyPtAbsHeading = -90; // or 270 deg
  }
  else if(dy == 0)
  {
    if(dx > 0)
      setWyPtAbsHeading = 0;
    else if(dx < 0)
      setWyPtAbsHeading = 180;
  }
  
  if(setWyPtAbsHeading < 0)
    setWyPtAbsHeading += 360; 
    
  Serial.print(setWyPtAbsHeading);
  Serial.println(" deg");

  he1 = measAbsHeading - setWyPtAbsHeading; //( measAbsHeading - setWyPtAbsHeading ) % 360; // [deg]
  Serial.print("he1 = measAbsHeading - setWyPtAbsHeading = ");
  Serial.print(measAbsHeading);
  Serial.print(" deg - ");
  Serial.print(setWyPtAbsHeading);
  Serial.print(" deg = ");
  Serial.print(he1);
  Serial.println(" deg");

  Serial.print("he2 = he1 ");
  if(he1 > 0)
  {
    he2 = he1 - 360; // [deg]
    Serial.print("- ");
  }
  else
  {
    he2 = he1 + 360;
    Serial.print("+ ");
  }

  Serial.print("360 deg = ");
  Serial.print(he2);
  Serial.println(" deg");

  if(abs(he1) > abs(he2))
    headingError = he2;
  else
    headingError = he1;

  Serial.print("headingError = ");
  Serial.print(headingError);
  Serial.println(" deg");

  setWyPtRelDisp = (int) round( sqrt( pow(dx, 2) + pow(dy, 2) ) );

  Serial.print("setWyPtRelDisp = ");
  Serial.print(setWyPtRelDisp);
  Serial.println(" mm");
  
  dispError = -setWyPtRelDisp; // [mm], can assume negative disp err b/c bot always first faces target before driving to it
  Serial.print("dispError = ");
  Serial.print(dispError);
  Serial.println(" mm");
  Serial.println();
  
  if(abs(dispError) > 0)
    setDispActive = 1;
}

/* Run arbitrarily long, continously printing displacement. 
 * When stopped, check that theoretical matches actual displacement.
 */
void moveStraightTask()
{
  extern LAYER moveStraight;

  //readDisplacement(); already happens for the cycle in read location

  if(moveStraightEnable)
  {
//    if(moveStraight.flag == false)
//    {
//      for(int i=0; i < numMtrs; i++)
//        mtrOutAccums[i] = 6000;
//    }
    
    moveStraight.arg = 0; // straight
    
    if(setDispActive == 0)
      moveStraight.cmd = setTanVel; // [pulses/(1/pubVelRate)s], turn CCW
    else // set displacement active
    {
      // determine whether rotate is being used directly by user or indirectly to find way point
      if(setPointActive == 0)
        setAbsDisp = setUserAbsDisp;
      else
        setAbsDisp = setWyPtAbsDisp;
        
      dispError = measAbsDisp - setAbsDisp; // [deg]
      Serial.print("Drive Disp Error (mm): ");
      Serial.println(dispError);
      Serial.println();
      
      if(abs(dispError) <= setRadius) // arrived at set angle?
      {
        moveStraight.cmd = 0; // stop driving

        setDispActive = 0; // signal that target is acquired
      
        if(setPointActive)
          searchEnable = 1;
        else
          haltBot = 1; // needed b/c PID control shift down to 0 takes too long b/c not tuned
      }
      else // still looking for position
      {
        // if disp error negative, it means you're at a position less than set position, so you should go in positive direc (forward) to correct that
        if(dispError < 0)
          moveStraight.cmd = setTanVel; // forward
        else
          moveStraight.cmd = -setTanVel; // backward
      }
    }

    moveStraight.flag = true;
  }
  else
    moveStraight.flag = false;
}

/* Run arbitrarily long, continously printing heading. 
 * When stopped, check that theoretical matches actual heading.
 */
void rotateTask()
{
  extern LAYER rotate;

  //readAngle(); already happens for the cycle in read location

  if(rotateEnable)
  { 
    rotate.cmd = 0; // pivot
    
    if(setAngleActive == 0)
      rotate.arg = -setRotVel; // [pulses/(1/pubVelRate)s], turn CCW
    else // set angle active
    {
      // determine whether rotate is being used directly by user or indirectly to find way point
      if(setPointActive == 0)
        setAbsHeading = setUserAbsHeading;
      else
        setAbsHeading = setWyPtAbsHeading;

      headingError = ( measAbsHeading - setAbsHeading ) % 360; // [deg]
      Serial.print("Rotate Heading Error (deg): ");
      Serial.println(headingError);
      Serial.println();
      
      if(abs(headingError) <= navDeadZone) // arrived at set angle?
      {
        rotate.arg = 0; // stop turning

        setAngleActive = 0; // signal that target is acquired
      
        if(setPointActive)
          searchEnable = 1;
        else
          haltBot = 1; // needed b/c PID control shift down to 0 takes too long b/c not tuned
      
        rotateEnable = 0;
      }
      else // still looking for angle
      {
        // if heading error negative, it means you're at an angle less than set angle, so you should go in positive direc (CCW) to correct that, unless it's shorter to go the other way
        if(headingError > 0)
          if(headingError > 180)
            rotate.arg = -setRotVel; // CCW
          else
            rotate.arg = setRotVel; // CW
        else
          if(headingError < -180)
            rotate.arg = setRotVel; // CW
          else
            rotate.arg = -setRotVel; // CCW

        if(rotate.flag == false)
        {
          if(rotate.arg > 0) // if CW
          {
            mtrOutAccums[0] = 6000;
            mtrOutAccums[1] = -6000;
          }
          else
          {
            mtrOutAccums[0] = -6000;
            mtrOutAccums[1] = 6000;
          }
        }
      }
    }

    rotate.flag = true;
  }
  else
    rotate.flag = false;
}

int convertMMToPulses(int mm) 
{
  return (int) round( mm / minLinearRes );
}

int convertDegToPulses(int deg) 
{
  return (int) round( deg / minAngularRes );
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
  //Serial.print("Meas Vels (pulses/(1/pubVelRate)s): ");
  for(int i=0; i < numMtrs; i++)
  {
    rotVels[i] = velPulseCounts[i] * signs[i]; // copy and accumulate counts from velPulseCount to rotVel
    velPulseCounts[i] = 0; // reset velPulseCount to zero

    //Serial.print(rotVels[i]);
    //Serial.print(" ");
  }
  //Serial.println("\n");
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
  
  Serial.print("Controller got kp:");
  Serial.print(kp);
  Serial.print(", ki:");
  Serial.print(ki);
  Serial.print(", kd:");
  Serial.println(kd, 3);

  for(int i=0; i < numMtrs; i++)
  {
    Serial.print("setVels[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(setVels[i]);
    Serial.print(", rotVels[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(rotVels[i]);
    
    errs[i] = (int) round( ( b * setVels[i] - rotVels[i] ) * 256 ); // [pulses/(1/pubVelRate)s]*256, generate error signal based on difference b/t measured rotVel and requested rotVel for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
    
    P[i] = (int) round( kp * errs[i] ); // P(t_k) = K(by_{sp}(t_k) — y(t_k))

    if(abs(errs[i]) < threshIntegral)
    {
      I[i] += (int) round( ki * errs[i] );

      I[i] = clip(I[i], maxOutVal, -maxOutVal);
    }
    else
      I[i] = 0;

    D[i] = (int) round( kd * ( ( rotVels[i] - prevRotVels[i] ) * 256 ) ); // large when amount of change requested by PID controller is large, and small as signal approaches zero

    mtrOutAccums[i] += P[i] + I[i] + D[i]; // Increase or decrease speed of motor to force measured to equal requested rotVel

    Serial.print("mtrOutAccums[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(mtrOutAccums[i]);
    
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

//  Serial.print("M");
//  Serial.print(mid + 1);
//  Serial.print(" speedometer sign: ");
//  Serial.println(signs[mid]);
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
  Serial.print("magnitude: ");
  Serial.println(magnitude);
  
  return map(magnitude, 0, 100, 60, 255); // cruise outputs
}
