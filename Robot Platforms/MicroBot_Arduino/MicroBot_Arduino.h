/**
 * File: MicroBot_Arduino.h
 * 
 * 
 *  
 * @author Joshua Michael Daly
 * @version 07/12/2014
 */
 
#include "I2Cdev.h"
#include <AFMotor.h>
#include "MPU6050_6Axis_MotionApps20.h"

#define DEBUG 1

// MPU6050 connections.
#define DMP_INTERRUPT 0

// Sonar pins.
#define TRIG_PIN 22
#define ECHO_PIN 24

// Speed to run motors at.
#define MOTOR_SPEED 255

enum states { 
              GOING_FORWARD = 0, GOING_BACKWARD = 1, 
              TURNING_LEFT = 2, TURNING_RIGHT = 3, 
              HALTED = 4, SCANNING = 5 
            };

/************************************************************
 * Serial Communication
 ************************************************************/

// Max characters on allowed message line.
#define MAX_CHARACTERS 19

// Number of bytes read from the serial line.
short bytes;

// Message constants.
const char separator = ',';
const char terminator = '\n';
const char odometryHeader = 'o';
const char scanReadingsHeader = 's';

// Message timing.
static unsigned long timer = 0;
const unsigned long messageRate = 500.0; // Milliseconds.

/************************************************************
 * Odometry and Scan Data
 ************************************************************/

// Average drift in x and y on a carpet surface.
const double X_DRIFT_CARPET = 0.000045;
const double Y_DRIFT_CARPET = 0.000261;

double x;
double y;
double theta;
double offset = 0;

enum states state;

/************************************************************
 * Hardware Connections
 ************************************************************/
 
// MPU6050.
MPU6050 mpu;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
 
// Motors attached to Adafruit motor controller.
AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor3(3);
