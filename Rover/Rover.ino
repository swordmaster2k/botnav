/**
 * File: Robot.h
 * 
 * 
 *  
 * @author Joshua Michael Daly
 * @version 14/04/2014
 */

#include "Rover.h"

void setup()
{
  Serial.begin(9600);                            
  attachInterrupt(LEFT_INTERRUPT, leftEncoderInterrupt, CHANGE);    // Init the interrupt mode for the digital pin 2.
  attachInterrupt(RIGHT_INTERRUPT, rightEncoderInterrupt, CHANGE);  // Init the interrupt mode for the digital pin 3.

  heading = 1.57;
}

void loop()
{
  if (millis() - timer > 100)
  {
    updateOdometry(leftTicks - lastLeftTicks, rightTicks - lastRightTicks);
    lastLeftTicks = leftTicks;
    lastRightTicks = rightTicks;
    
    
    timer = millis();
  }
}

void updateOdometry(signed long deltaLeft, signed long deltaRight)
{
  double deltaDistance = (double) ((deltaLeft + deltaRight) * DISTANCE_PER_TICK) / 2;
  double deltaX = deltaDistance * cos(heading);
  double deltaY = deltaDistance * sin(heading);
  double deltaHeading = (double)(deltaRight - deltaLeft) * RADIANS_PER_TICK;

  x += deltaX;
  y += deltaY;
  heading += deltaHeading;

  // Limit heading to 0 < - > 6.28
  if (heading < 0)
  {
    heading += PI * 2;
  }
  else if (heading >= PI * 2)
  {
    heading -= PI * 2;
  }  
}

void leftEncoderInterrupt()
{
  leftTicks++;
}

void rightEncoderInterrupt()
{
  rightTicks++;
}
