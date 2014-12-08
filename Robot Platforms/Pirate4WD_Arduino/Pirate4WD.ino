/**
 * File: Pirate4WD.ino
 * 
 * 
 * 
 * @author Joshua Michael Daly
 * @version 23/09/2014
 */

#include <ps2.h>
#include <Wire.h>
#include <Servo.h>
#include <AFMotor.h>
#include <HMC5883L.h>

#include "Pirate4WD_Arduino.h"

/************************************************************
 * Arduino Functions
 ************************************************************/

void setup()
{
  Serial.setTimeout(500);
  Serial.begin(115200);

  sonarServo.attach(SERVO_PIN);

  mouseInit();
  compassInit();

  motor1.setSpeed(MOTOR_SPEED);
  motor2.setSpeed(MOTOR_SPEED);
  motor3.setSpeed(MOTOR_SPEED);
  motor4.setSpeed(MOTOR_SPEED);
}

void loop()
{   
  // Check to see if at least one character is available.
  if (Serial.available()) 
  {
    processSerial();
  }

  updateOdometry();

  // To save bombarding the serial line.
  if (millis() - timer >= messageRate)
  {
    sendOdometry();
    timer = millis();
  }
}

/************************************************************
 * Robot Functions
 ************************************************************/

void processSerial()
{
  // Redeclare this every time to clear the buffer.
  char buffer[MAX_CHARACTERS]; 

  bytes = Serial.readBytesUntil(terminator, buffer, MAX_CHARACTERS);

  if (bytes > 0)
  {
    processCommand(buffer);
  }
}

void processCommand(char command[])
{
  switch (tolower(command[0]))
  {
  case 'w':
    goForward();
    break;
  case 's':
    goBackward();
    break;
  case 'a':
    turnLeft();
    break;
  case 'd':
    turnRight();
    break;  
  case 'q':
    halt();
    break;
  case 'r':
    parseRotation(command);
    break;
  case 'e':
    halt();
    scan();
    break;
  case 't':
    parseDistance(command);
    break;
  case 'p': // Take a single "ping" from the sonar.
    halt();
    ping();
    break;
  case 'c': // Set our x, y, and theta.
    changeOdometry(command);
    break;
  default: 
    Serial.print("Unknown command \"");
    Serial.print(command[0]);
    Serial.println("\"");
  }
}

void mouseInit()
{
  mouse.write(0xff);  // Reset.
  mouse.read();       // Ack byte.
  mouse.read();       // Blank. 
  mouse.read();       // Blank. 
  mouse.write(0xf0);  // Remote mode.
  mouse.read();       // Ack byte.
  delayMicroseconds(100);
}

void compassInit()
{
  // Ignore errors as they just fire anyway, problem with the library.
  compass = HMC5883L(); // Construct a new HMC5883L compass.
  compass.SetScale(0.88); // Set the scale of the compass.
  compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
}

void sendOdometry()
{
  // Send odometry data to host with,
  // a resolution of 2 decimal places.
  Serial.print(odometryHeader);
  Serial.print(separator);
  Serial.print(x);
  Serial.print(separator);
  Serial.print(y);
  Serial.print(separator);
  Serial.print(theta);
  Serial.println(); // Message terminated by \n.
}

void updateOdometry()
{
  // Read mouse registers.
  mouse.write(0xeb);  // Give me data!
  mouse.read();       // Ignore ack.
  mouse.read();       // Ignore stat.
  
  x += ((int8_t)mouse.read() / MOUSE_CPI) * INCH_TO_METER;
  y += ((int8_t)mouse.read() / MOUSE_CPI) * INCH_TO_METER;
 
  getHeading();
}

void getHeading()
{
  // Read compass rotation.
  MagnetometerScaled scaled = compass.ReadScaledAxis();

  int MilliGauss_OnThe_XAxis = scaled.XAxis; // (or YAxis, or ZAxis)

  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  theta = atan2(scaled.YAxis, scaled.XAxis);
  theta += offset;

  // Correct for when signs are reversed.
  if(theta < 0)
  {
    theta += 2 * PI;
  }

  // Check for wrap due to addition of declination.
  if(theta > 2 * PI)
  {
    theta -= 2 * PI;
  }
}

void scan()
{
  Serial.println("Scanning");
  
  for (unsigned char pos = 0; pos < 180; pos++) // Goes from 0 degrees to 180 degrees 
  {                                             // in steps of 1 degree.            
    sonarServo.write(pos);   
    delay(25);                                  // Waits 25ms for the servo to reach the position.
    distances[pos] = takeReading();             // Store sonar reading at current position.
  } 

  sonarServo.write(90);

  // Send readings back to the host.
  Serial.print(scanReadingsHeader);

  for (unsigned char i = 179; i >= 0; i++)
  {
    Serial.print(",");
    Serial.print(distances[i]); 
  }

  Serial.println(); // Message terminated by \n.
}

double takeReading()
{
  double distance; // Distance to object.
  double duration; // Duration of pulse from PING))).

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse.
  pinMode(SIG_PIN, OUTPUT);
  digitalWrite(SIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SIG_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SIG_PIN, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(SIG_PIN, INPUT);
  duration = pulseIn(SIG_PIN, HIGH);

  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  distance = ((duration / 2) / 29) / 100;

  return distance;
}

void ping()
{
  Serial.println("Scanning");

  Serial.print(scanReadingsHeader);
  Serial.print(',');
  Serial.println(takeReading());
}

void parseRotation(char command[MAX_CHARACTERS])
{  
  char data[4];
  data[0] = command[1];
  data[1] = command[2];
  data[2] = command[3];
  data[3] = command[4];

  double heading = atof(data);

  if (heading >= 0.0 && heading <= 6.27)
  {
    rotateTo(heading); 
  }
}

void rotateTo(double heading)
{
  getHeading();
  double angle = heading - theta;

#if DEBUG
  Serial.print("Current Heading = ");
  Serial.println(theta);
#endif

  /*
   * Check to see if our angle has extended beyond the 0/360 degree 
   * line. Our maximum turning arcs from right or left is 180 degrees,
   * so if our angle has crossed the line we must "wrap" it around.
   */
  if (angle < -3.14)
  {
    angle += 6.28; 
  }
  else if (angle > 3.14)
  {
    angle -= 6.28;
  }

#if DEBUG
  Serial.print("Angle = ");
  Serial.println(angle);
#endif

  // Set up a buffer on either side of our requested angle of 1 degree.
  double leftBuffer = heading + 0.01;
  double rightBuffer = heading - 0.01;

  // Wrap around the buffer values if needed.
  if (leftBuffer > 6.28)
  {
    leftBuffer -= 6.28;
  }

  if (rightBuffer < -6.28)
  {
    rightBuffer += 6.28;
  }

  // Round them off to 2 decimal places.
  leftBuffer = roundDigit(leftBuffer, 16);
  rightBuffer = roundDigit(rightBuffer, 16);

#if DEBUG
  Serial.print("LeftBuffer = ");
  Serial.println(leftBuffer);
  Serial.print("RightBuffer = ");
  Serial.println(rightBuffer);
#endif

  // Our circle is based on a right hand axis where all negative values
  // indicated movement around the right of the circle.
  if (angle < 0)
  {
    turnRight();  
  }
  else if (angle > 0)
  {
    turnLeft();
  }

  boolean interrupted = false;
  char command[MAX_CHARACTERS];

  while (theta < rightBuffer || theta > leftBuffer)
  {
    // Check to see if at least one character is available.
    if (Serial.available()) 
    {
      // Redeclare this every time to clear the buffer.
      char buffer[MAX_CHARACTERS]; 

      bytes = Serial.readBytesUntil(terminator, buffer, MAX_CHARACTERS);

      if (bytes > 0)
      {
        strcpy(command, buffer);
        interrupted = true;
      }
    }

    if (interrupted)
    {
      Serial.println("Interrupted during rotatation!");
      break; 
    }

    updateOdometry();

    if (millis() - timer > 1000)
    {
      sendOdometry();
      timer = millis();
    }
  }

  halt();

  Serial.print("Current Heading = ");
  Serial.println(theta); // Output in degrees.

  if (interrupted)
  {
    processCommand(command); 
  }
}

void parseDistance(char command[MAX_CHARACTERS])
{  
  char data[4];
  data[0] = command[1];
  data[1] = command[2];
  data[2] = command[3];
  data[3] = command[4];

  double distance = atof(data);

  if (distance > 0)
  {
    travel(distance); 
  }
}

void travel(double distance)
{
  double startX = x;
  double startY = y;
  double travelled = 0;
  distance -= 0.09; // Stop 0.09m short so we halt in time.

  boolean interrupted = false;
  char command[MAX_CHARACTERS];

#if DEBUG
  Serial.print("travel: ");
  Serial.println(distance);  
#endif

  goForward();

  while (true)
  {
    // Check to see if at least one character is available.
    if (Serial.available()) 
    {
      // Redeclare this every time to clear the buffer.
      char buffer[MAX_CHARACTERS]; 

      bytes = Serial.readBytesUntil(terminator, buffer, MAX_CHARACTERS);

      if (bytes > 0)
      {
        strcpy(command, buffer);
        interrupted = true;
      }
    }

    if (interrupted)
    {
      Serial.println("Interrupted during travel!");
      break; 
    }

    updateOdometry();

    travelled = sqrt(pow((startX - x), 2) + pow((startY - y), 2));

    if (travelled >= distance)
      break;

    if (millis() - timer > 1000)
    {
      sendOdometry();    
      timer = millis();
    }
  }

  halt();

#if DEBUG
  Serial.print("travelled: ");
  Serial.println(travelled);
#endif

  Serial.println("Travelled");

  if (interrupted)
  {
    processCommand(command); 
  }
}

void changeOdometry(char command[MAX_CHARACTERS])
{
  String message = String(command);

  if (message.length() < 6) // Needs to be at least c,X,Y,0.
  {
    return; 
  }

  String xComponent = "";
  String yComponent = "";
  String thetaComponent = "";

  for (char i = 0; i < 3; i++)
  {
    char commaIndex = message.indexOf(',');

    if (commaIndex != -1)
    {
      message = message.substring(commaIndex + 1, message.length());

      if (i == 0)
      {
        xComponent = message;
      }
      else if (i == 1)
      {
        yComponent = message;
      }
      else
      {
        thetaComponent = message;
      }
    }
    else
    {
      break;
    } 
  }

  if (xComponent != "")
  {
    char xAxis[xComponent.length() + 1];
    xComponent.toCharArray(xAxis, xComponent.length() + 1);

    double newX = atof(xAxis);

    if (x != newX)
    {
      x = newX; 
    }
  }

  if (yComponent != "")
  {
    char yAxis[yComponent.length() + 1];
    yComponent.toCharArray(yAxis, yComponent.length() + 1);

    double newY = atof(yAxis);

    if (y != newY)
    {
      y = newY; 
    }
  }

  if (thetaComponent != "")
  {
    char thetaBuffer[thetaComponent.length() + 1];
    thetaComponent.toCharArray(thetaBuffer, thetaComponent.length() + 1);

    double newTheta = atof(thetaBuffer);

    if (theta != newTheta)
    {
       offset = newTheta - theta;
    }
  }
}

/************************************************************
 * Motor Functions
 ************************************************************/

void goForward()
{
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  
  state = GOING_FORWARD;

  Serial.println("Going Forward");
}

void goBackward()
{
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  
  state = GOING_BACKWARD;

  Serial.println("Going Backward");
}

void turnLeft()
{
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  
  state = TURNING_LEFT;

  Serial.println("Turning Left");
}

void turnRight()
{
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  
  state = TURNING_RIGHT;

  Serial.println("Turning Right");
}

void halt()
{
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  
  state = HALTED;

  Serial.println("Halted");
}

/************************************************************
 * Extra Functions
 ************************************************************/

double roundDigit(double number, int digits)
{
  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;

  for (uint8_t i=0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  return number;
}
