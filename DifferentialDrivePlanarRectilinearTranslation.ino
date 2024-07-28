/* Differential Drive Planar, Rectilinear Translation
 * All particles in rigid body move along
 * paths that are equidistant from
 * a fixed plane,
 * line in body remains || to
 * its original orientation throughout 
 * motion, and
 * paths of motion for any 2 points on
 * body are parallel lines
 * Create a robot that drives forward and backward to given displacement at controlled velocity
 */

void setup() 
{
  // init base point 
  x_A = 0;
  y_A = 0; 
}

void loop() 
{
  if(twoPtsParallel)
    state = rectTranslation;
  else if(parallelMotion)
    state = curvTranslation;
  else if(allBodyParticlesHaveCircularPaths)
    state = rotationAbtFixedAxis;
  else if(comboOfTransAndRot 
          && transOccursWinRefPane 
          && rtnOccursAbtAxisPerpToRefPane)
    state = generalPlaneMotion;
    
  switch(state)
  {
    case rectTranslation:
       // compute position vector of base point
       relativePositionVector = getRelDisp();
       
       rOfBasePointDirec = getDispDirec();

       break;
       
    case curvTranslation:
       break;
       
    case rotationAbtFixedAxis:
      break;

    case generalPlaneMotion:
      break;
  }
}

int getRelDisp(int)
{
  measRelDisp = (int) round( minLinearRes * ( pulses[0] + pulses[1] ) / 2.0 ); // [mm]

  Serial.print("measRelDisp = minLinearRes * ( pulses[0] + pulses[1] ) / 2.0 ) = ");
  Serial.print(minLinearRes);
  Serial.print(" * ( ");
  Serial.print(pulses[0]);
  Serial.print(" + ");
  Serial.print(pulses[1]);
  Serial.print(" ) / 2.0 = ");
  Serial.print(measAbsDisp);
  Serial.println(" mm");

  return measRelDisp;
}

