/**
 * File: Robot.ino
 * 
 * 
 * 
 * @author Joshua Michael Daly
 * @version 27/06/2014
 */

#include <Servo.h> 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "Rover.h"

/************************************************************
 * Arduino Functions
 ************************************************************/

void setup()
{
  Serial.setTimeout(500);
  Serial.begin(115200);

  Serial.println("------------------------------ Booting ------------------------------\n");

  // Join I2C bus (I2Cdev library doesn't do this automatically).
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  Serial.println("Using Arduino Wire library.\n");
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  Serial.println("Using Fastwire library.\n");
#endif

  // Setup the encoders.
  Serial.println("Attaching encoder interrupts...");
  attachInterrupt(LEFT_INTERRUPT, leftEncoderInterrupt, CHANGE);    // Init the interrupt mode for the left encoder.
  attachInterrupt(RIGHT_INTERRUPT, rightEncoderInterrupt, CHANGE);  // Init the interrupt mode for the right encoder. 
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_ENCODER, INPUT);

  // Setup the sonar.
  Serial.print("Attaching sonar pins echo: ");
  Serial.print(ECHO_PIN);
  Serial.print(" and trig: ");
  Serial.print(TRIG_PIN);
  Serial.println("...");
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  // Attach sonar servo.
  Serial.print("Attaching servo to pin ");
  Serial.print(SERVO_PIN);
  Serial.println("...");
  sonarServo.attach(SERVO_PIN);

  // Setup the motors.
  Serial.println("Attaching motors 1 to 4...");
  for (int i = 4; i <= 7; i++)
  {
    pinMode(i, OUTPUT);
  }  

  digitalWrite(M1_SPEED_CONTROL,LOW);   
  digitalWrite(M2_SPEED_CONTROL,LOW);

  /* MPU_6050 Configuration */

  // Initialize device.
  Serial.println(F("\nInitializing MPU6050..."));
  mpu.initialize();

  // Verify connection.
  Serial.println(F("Testing MPU6050 connection..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful\n") : F("MPU6050 connection failed\n"));

  // Load and configure the DMP.
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Supply your own gyro offsets here, scaled for min sensitivity.
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(1185);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1788 factory default for my test chip.

  // Make sure it worked (returns 0 if so).
  if (devStatus == 0) 
  {
    // Turn on the DMP, now that it's ready.
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // Enable Arduino interrupt detection.
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 4)..."));
    attachInterrupt(DMP_INTERRUPT, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // Set our DMP Ready flag so the main loop() function knows it's okay to use it.
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // Get expected DMP packet size for later comparison.
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else 
  {
    // ERROR!
    // 1 = initial memory load failed.
    // 2 = DMP configuration updates failed.
    // (if it's going to break, usually the code will be 1).
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    
    while (true) // Go it an infinite loop because we failed.
    {
      
    }
  }

  Serial.println("\n------------------------------ Ready ------------------------------\n");
}

void loop()
{
  // Wait for MPU interrupt or extra packet(s) available.
  while (!mpuInterrupt && fifoCount < packetSize) 
  {
    if (Serial.available())
    {
      processSerial(); 
    }

    if (millis() - timer > messageRate)
    {
      sendOdometry();     
      timer = millis();

#if DEBUG
//      Serial.print("left ticks: ");
//      Serial.println(leftTicks);
//      Serial.print("right ticks: ");
//      Serial.println(rightTicks);
#endif
    }
  }

  processGyro();
  updateOdometry(leftTicks - lastLeftTicks, rightTicks - lastRightTicks);
}

/************************************************************
 * Rover Functions
 ************************************************************/

void processSerial()
{
  // Check to see if at least one character is available.
  if (Serial.available()) 
  {
    // Redeclare this every time to clear the buffer.
    char buffer[MAX_CHARACTERS]; 

    bytes = Serial.readBytesUntil(terminator, buffer, MAX_CHARACTERS);

    if (bytes > 0)
    {
      processCommand(buffer);
    }
  }
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
  case 'c': // Set our x and y.
    parseCoordinates(command);
    break;
  default: 
    Serial.print("Unknown command \"");
    Serial.print(command[0]);
    Serial.println("\"");
  }
}

void processGyro()
{
  // Reset interrupt flag and get INT_STATUS byte.
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // Get current FIFO count.
  fifoCount = mpu.getFIFOCount();

  // Check for overflow (this should never happen unless our code is too inefficient).
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
  {
    // Reset so we can continue cleanly.
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } 
  else if (mpuIntStatus & 0x02) // Otherwise, check for DMP data ready interrupt (this should happen frequently).
  {
    // Wait for correct available data length, should be a VERY short wait.
    while (fifoCount < packetSize) 
      fifoCount = mpu.getFIFOCount();

    // Read a packet from FIFO.
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // Track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt).
    fifoCount -= packetSize;

    // Display Euler angles in degrees.
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Negatate here because on my Cherokey rig the x-axis is on the wrong side.
    theta = -ypr[0];
    theta -= 3.14; // Due to the negatation.

    // Limit heading to 0 < heading < 6.28.
    if (theta < 0)
    {
      theta += 6.28;
    }
    else if (theta >= 6.28)
    {
      theta -= 6.28;
    }
  }
}

void updateOdometry(signed long deltaLeft, signed long deltaRight)
{
  // Assume a pure rotation around our point until we fix the encoders...
  if (state == TURNING_LEFT || state == TURNING_RIGHT)
  {
    lastLeftTicks = leftTicks;
    lastRightTicks = rightTicks;
    return; 
  }

  double deltaDistance = (double) ((deltaLeft + deltaRight) * DISTANCE_PER_TICK) / 2;
  double deltaX;
  double deltaY;
  
  if (deltaDistance != 0.0)
  {
    if (state == GOING_BACKWARD)
    {
      deltaX = (deltaDistance) * cos(-theta);
      deltaY = (deltaDistance) * sin(-theta);
    }
    else
    {
      deltaX = (deltaDistance) * cos(theta);
      deltaY = (deltaDistance) * sin(theta);
    }
    
    deltaX += X_DRIFT_CARPET * (deltaX / 0.01); // Calculate x drift over every centimeter and add it to our change in x.
    deltaY += Y_DRIFT_CARPET * (deltaY / 0.01); // Calculate y drift over every centimeter and add it to our change in y.
  
    x += deltaX;
    y += deltaY;
  }

  lastLeftTicks = leftTicks;
  lastRightTicks = rightTicks;  
}

void ping()
{
  Serial.println("Scanning");
  
  mpu.setDMPEnabled(false); // Disabled DMP to avoid any over flows!
  
  Serial.print(scanReadingsHeader);
  Serial.print(',');
  Serial.println(takeReading());
  
  mpu.setDMPEnabled(true);
}

void scan()
{
  Serial.println("Scanning");
  
  mpu.setDMPEnabled(false);
  
  state = SCANNING;

  unsigned char degree;

  for (degree = 0; degree < 170; degree++)        // Goes from 11 degrees to 180 degrees 
  {                                               // in steps of 1 degree.  
    sonarServo.write(degree + 11);                // Account for the fact that we start at 0 instead of 11.
    delay(25);                                    // Waits 25ms for the servo to reach the degree.
    distances[degree] = takeReading();            // Store sonar reading at current degree.
  } 

  // Return to center.
  sonarServo.write(90);

  // Send readings back to the host.
  Serial.print(scanReadingsHeader);

  for (int i = 0; i < 170; i++)
  {
    Serial.print(",");
    Serial.print(distances[i]); 
  }

  Serial.println(); // Message terminated by CR/LF.
  
  mpu.setDMPEnabled(true);
}

double takeReading()
{
  double duration; // Duration used to calculate distance.
  double distance; 

  /* The following trigPin/echoPin cycle is used to determine the
   distance of the nearest object by bouncing soundwaves off of it. */
  digitalWrite(TRIG_PIN, LOW); 
  delayMicroseconds(2); 

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10); 

  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance (in m) based on the speed of sound.
  distance = (duration / 58.2) / 100;

  return distance;
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
  processGyro();
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

  while (!(theta >= rightBuffer && theta <= leftBuffer))
  {
    while (!mpuInterrupt && fifoCount < packetSize)
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
    }

    if (interrupted)
    {
      Serial.println("Interrupted during rotatation!");
      break; 
    }

    processGyro();
    updateOdometry(leftTicks - lastLeftTicks, rightTicks - lastRightTicks);

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
    while (!mpuInterrupt && fifoCount < packetSize)
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
    }

    if (interrupted)
    {
      Serial.println("Interrupted during travel!");
      break; 
    }

    processGyro();
    updateOdometry(leftTicks - lastLeftTicks, rightTicks - lastRightTicks);

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

  if (interrupted)
  {
    processCommand(command); 
  }
}

void parseCoordinates(char command[MAX_CHARACTERS])
{
  String message = String(command);
  
  if (message.length() < 4) // Needs to be at least cX,Y.
  {
    return; 
  }
  
  char index = message.indexOf(',');
  
  // Make sure we are properly delimited and that the ',' is not the last character.
  if (index != -1 && index != message.length() - 1) 
  {
    // Extract the new x and y coordinates.
    String xComponent = message.substring(1, index);
    String yComponent = message.substring(index + 1);

    char xAxis[xComponent.length() + 1];
    char yAxis[yComponent.length() + 1];
    
    xComponent.toCharArray(xAxis, xComponent.length() + 1);
    yComponent.toCharArray(yAxis, yComponent.length() + 1);
    
    double newX = atof(xAxis);
    double newY = atof(yAxis);
  
    if (x != newX)
    {
      x = newX; 
    }
    
    if (y != newY)
    {
      y = newY; 
    }
  }
}

/************************************************************
 * Interrupt Handlers
 ************************************************************/

void leftEncoderInterrupt()
{
  leftTicks++;
}

void rightEncoderInterrupt()
{
  rightTicks++;
}

void dmpDataReady() 
{
  mpuInterrupt = true;
}

/************************************************************
 * Motor Functions
 ************************************************************/

void goForward()
{
  analogWrite (M1_SPEED_CONTROL, MOTOR_SPEED);
  digitalWrite(M1, LOW);    
  analogWrite (M2_SPEED_CONTROL, MOTOR_SPEED);    
  digitalWrite(M2, LOW);

  state = GOING_FORWARD;

  Serial.println("Going Forward");
}

void goBackward()
{
  analogWrite (M1_SPEED_CONTROL, MOTOR_SPEED);
  digitalWrite(M1, HIGH);    
  analogWrite (M2_SPEED_CONTROL, MOTOR_SPEED);    
  digitalWrite(M2, HIGH);

  state = GOING_BACKWARD;

  Serial.println("Going Backward");
}

void turnLeft()
{
  analogWrite (M1_SPEED_CONTROL, MOTOR_SPEED);
  digitalWrite(M1, LOW);    
  analogWrite (M2_SPEED_CONTROL, MOTOR_SPEED);    
  digitalWrite(M2, HIGH);

  state = TURNING_LEFT;

  Serial.println("Turning Left");
}

void turnRight()
{
  analogWrite (M1_SPEED_CONTROL, MOTOR_SPEED);
  digitalWrite(M1, HIGH);    
  analogWrite (M2_SPEED_CONTROL, MOTOR_SPEED);    
  digitalWrite(M2, LOW);

  state = TURNING_RIGHT;

  Serial.println("Turning Right");
}

void halt()
{
  digitalWrite(M1_SPEED_CONTROL,0); 
  digitalWrite(M1, LOW);    
  digitalWrite(M2_SPEED_CONTROL,0);   
  digitalWrite(M2, LOW);

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





















