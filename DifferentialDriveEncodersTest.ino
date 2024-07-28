/*
 * Differential Drive Encoders Test
 * Given independent encoder readings, 
 * convert to Drive and Turn pulses.
 */

volatile long pulseCount[] = {0,0}; 
volatile long totPulses[] = {0,0};  
//volatile long totRevs[] = {0,0}; 

double dLastPosition,
  tLastPosition,
  dPulses,
  tPulses,
  dSamplePosition,
  tSamplePosition;

long timeDiff;

long totRevs[] = {0,0};

unsigned long currentTime;

const byte es1Pin = 2, 
  es2Pin = 3; // pin connected to encoder signal

int lastEncoderVel[] = {0,0}; 

int ppr = 20,
  samplePeriod = 100;
  
static unsigned long lastTime = 0; // print manager timer

void setup() 
{
  Serial.begin(9600);
  
  while(!Serial);

  /* Initialize */

  attachInterrupt(digitalPinToInterrupt(es1Pin), updateEs1Vel, CHANGE);
  attachInterrupt(digitalPinToInterrupt(es2Pin), updateEs2Vel, CHANGE);
}

void loop() {
  // run one or both wheels manually for one pulse at a time, 
  // observing changes in Drive and Turn pulses
  currentTime = millis();
  
  timeDiff = currentTime - lastTime;
  
  if(timeDiff > samplePeriod)
  {
    dSamplePosition = getDrivePosition();
    tSamplePosition = getTurnPosition();

    // determine how many pulses since our last sampling
    dPulses = dSamplePosition - dLastPosition;
    tPulses = tSamplePosition - tLastPosition;
  
    // and update the last sampling for next time
    dLastPosition = dSamplePosition;
    tLastPosition = tSamplePosition;
    
    totRevs[0] = floor(totPulses[0] / ppr);
    totRevs[1] = floor(totPulses[1] / ppr);
    
    displayStatus();

    lastEncoderVel[0] = pulseCount[0] * ( 1000 / timeDiff ); // record latest speed value
    lastEncoderVel[1] = pulseCount[1] * ( 1000 / timeDiff ); // record latest speed value

    Serial.print("Last Encoder 1 Velocity (pulses/s): ");
    Serial.println(lastEncoderVel[0]);
    Serial.print("Last Encoder 2 Velocity (pulses/s): ");
    Serial.println(lastEncoderVel[1]);
    Serial.println();
    
    pulseCount[0] = 0; // clear data buffer
    pulseCount[1] = 0;

    lastTime = millis();
  }
}

double getDrivePosition()
{
  return ( totPulses[0] + totPulses[1] ) / 2.0;
}

double getTurnPosition()
{
  return (double) totPulses[0] - totPulses[1];
}

void displayStatus()
{
  Serial.print("Total Elapsed Time (ms): ");
  Serial.println(currentTime);

  Serial.print("Last Recorded Time (ms): ");
  Serial.println(lastTime);
  
  Serial.print("Time Diff (ms): ");
  Serial.println(timeDiff);

  Serial.println("======Pulses======");
  Serial.println("M1\tM2\tD\tT");
  Serial.print(pulseCount[0]);
  Serial.print("\t");
  Serial.print(pulseCount[1]);
  Serial.print("\t");
  Serial.print(dPulses);
  Serial.print("\t");
  Serial.println(tPulses);
  Serial.println();
  
//  Serial.print("Pulse Counts: [ ");
//  Serial.print(pulseCount[0]);
//  Serial.print(", ");
//  Serial.print(pulseCount[1]);
//  Serial.println(" ]");
  
  Serial.print("Total Pulses: [ ");
  Serial.print(totPulses[0]);
  Serial.print(", ");
  Serial.print(totPulses[1]);
  Serial.println(" ]");
  
  Serial.print("Total Wheel Revs: [ ");
  Serial.print(totRevs[0]);
  Serial.print(", ");
  Serial.print(totRevs[1]);
  Serial.println(" ]");

  Serial.println();
}

//======Interrupt Service Routines======
void updateEs1Vel()
{
  pulseCount[0]++; // count wheel encoder interrupts

  totPulses[0]++;

  //totRevs[0] = floor(totPulses[0] / ppr);
}

void updateEs2Vel()
{
  pulseCount[1]++; // count wheel encoder interrupts

  totPulses[1]++;

  //totRevs[1] = floor(totPulses[1] / ppr);
}
