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
  mpu.setYGyroOffset(1200);
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
  }

  Serial.println("\n------------------------------ Ready ------------------------------\n");
}

void loop()
{
  // If programming failed, don't try to do anything.
  if (!dmpReady) 
    return;

  // Wait for MPU interrupt or extra packet(s) available.
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
        processCommand(buffer);
      }
    }

    if (millis() - timer > messageRate)
    {
      updateOdometry(leftTicks - lastLeftTicks, rightTicks - lastRightTicks);

      // Send odometry data to host.
      Serial.print(odometryHeader);
      Serial.print(separator);
      Serial.print(x, DEC);
      Serial.print(separator);
      Serial.print(y, DEC);
      Serial.print(separator);
      Serial.print(theta, DEC);
      Serial.println(); // Message terminated by \n.

      timer = millis();
    }
  }

  processGyro();
}

/************************************************************
 * Rover Functions
 ************************************************************/

void processCommand(char command[])
{
  switch (tolower(command[0]))
  {
  case 'w':
#if DEBUG
    Serial.println("Forward");
#endif
    goForward();
    break;
  case 's':
#if DEBUG
    Serial.println("Backward");
#endif
    goBackward();
    break;
  case 'a':
#if DEBUG
    Serial.println("Left");
#endif
    turnLeft();
    break;
  case 'd':
#if DEBUG
    Serial.println("Right");
#endif
    turnRight();
    break;  
  case 'q':
#if DEBUG
    Serial.println("Halt");
#endif
    halt();
    break;
  case 'r':
    //    // Second byte is the angle to rotate too,
    //    // divide by 100 to get radians.
    //    double angle = command[1] / 100;
    //    
    //    if (command[1] >= 0 && command[1] <= 6.28)
    //    {
    //      // Rotate the robot to this degreeition.
    //    }
    //    else
    //    {
    //      Serial.print("Rotation outside of bounds: ");
    //      Serial.println((short)command[1]); 
    //    }
    rotateTo(3.14);
    break;
  case 'e':
#if DEBUG
    Serial.println("Scan");
#endif
    halt();
    scan();
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

    theta = ypr[0];

    // Limit heading to 0 < heading < 6.28.
    if (theta < 0)
    {
      theta += PI * 2;
    }
    else if (theta >= PI * 2)
    {
      theta -= PI * 2;
    }
  }
}

void updateOdometry(signed long deltaLeft, signed long deltaRight)
{
  double deltaDistance = (double) ((deltaLeft + deltaRight) * DISTANCE_PER_TICK) / 2;
  double deltaX = deltaDistance * cos(theta);
  double deltaY = deltaDistance * sin(theta);

  x += deltaX;
  y += deltaY;

  // Limit heading to 0 < heading < 6.28.
  if (theta < 0)
  {
    theta += PI * 2;
  }
  else if (theta >= PI * 2)
  {
    theta -= PI * 2;
  }

  lastLeftTicks = leftTicks;
  lastRightTicks = rightTicks;  
}

void scan()
{
  unsigned char degree;

  for (degree = 0; degree < 170; degree++)        // Goes from 11 degrees to 180 degrees 
  {                                               // in steps of 1 degree.  
    sonarServo.write(degree + 11);                // Account for the fact that we start at 0 instead of 11.
    delay(25);                                    // Waits 25ms for the servo to reach the degreeition.
    distances[degree] = takeReading();            // Store sonar reading at current degreeition.
  } 

  // Return to center degreeition.
  sonarServo.write(90);

  // Send readings back to the host.
  Serial.print(scanReadingsHeader);

  for (int i = 0; i < 170; i++)
  {
    Serial.print(",");
    Serial.print(distances[i], DEC); 
  }

  Serial.println(); // Message terminated by CR/LF.
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
  if (angle < -M_PI)
  {
     angle += M_PI * 2; 
  }
  else if (angle > M_PI)
  {
    angle -= M_PI * 2;
  }
  
  #if DEBUG
    Serial.print("Angle = ");
    Serial.println(angle);
  #endif
  
  // Set up a buffer on either side of our requested angle of 1 degree.
  double leftBuffer = heading + 0.02;
  double rightBuffer = heading - 0.02;
  
  // Wrap around the buffer values if needed.
  if (leftBuffer > M_PI * 2)
  {
    leftBuffer -= M_PI * 2;
  }
  
  if (rightBuffer < -(M_PI * 2))
  {
    rightBuffer += M_PI * 2;
  }
  
  #if DEBUG
    Serial.print("LeftBuffer = ");
    Serial.println(leftBuffer);
    Serial.print("RightBuffer = ");
    Serial.println(rightBuffer);
  #endif
  
  // Our circle is based on a left hand axis where all negative values
  // indicated movement around the left of the circle.
  if (angle < 0)
  {
    Serial.println("Turning Left");
    turnLeft();  
  }
  else if (angle > 0)
  {
    Serial.println("Turning Right");
    turnRight();
  }
  
  while (!(theta >= rightBuffer && theta <= leftBuffer))
  {
    while (!mpuInterrupt && fifoCount < packetSize)
      ;
    
    processGyro();
    updateOdometry(leftTicks - lastLeftTicks, rightTicks - lastRightTicks);
    
    if (millis() - timer > 1000)
    {
      updateOdometry(leftTicks - lastLeftTicks, rightTicks - lastRightTicks);

      // Send odometry data to host.
      Serial.print(odometryHeader);
      Serial.print(separator);
      Serial.print(x, DEC);
      Serial.print(separator);
      Serial.print(y, DEC);
      Serial.print(separator);
      Serial.print(theta, DEC);
      Serial.println(); // Message terminated by \n.

      timer = millis();
    }
  }

  halt();

  Serial.print("Current Heading = ");
  Serial.println(theta); // Output in degrees.
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
}

void goBackward()
{
  analogWrite (M1_SPEED_CONTROL, MOTOR_SPEED);
  digitalWrite(M1, HIGH);    
  analogWrite (M2_SPEED_CONTROL, MOTOR_SPEED);    
  digitalWrite(M2, HIGH);
}

void turnLeft()
{
  analogWrite (M1_SPEED_CONTROL, MOTOR_SPEED);
  digitalWrite(M1, LOW);    
  analogWrite (M2_SPEED_CONTROL, MOTOR_SPEED);    
  digitalWrite(M2, HIGH);
}

void turnRight()
{
  analogWrite (M1_SPEED_CONTROL, MOTOR_SPEED);
  digitalWrite(M1, HIGH);    
  analogWrite (M2_SPEED_CONTROL, MOTOR_SPEED);    
  digitalWrite(M2, LOW);
}

void halt()
{
  digitalWrite(M1_SPEED_CONTROL,0); 
  digitalWrite(M1, LOW);    
  digitalWrite(M2_SPEED_CONTROL,0);   
  digitalWrite(M2, LOW);
}













