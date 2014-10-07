/**
 * File: Pirate4WD.h
 * 
 * TODO: Calculate average drift for the mouse sensor.
 *  
 * @author Joshua Michael Daly
 * @version 23/09/2014
 */

#include <ps2.h>
#include <Wire.h>
#include <Servo.h>
#include <AFMotor.h>
#include <HMC5883L.h>

#define DEBUG 1

// Speed to run motors at.
#define MOTOR_SPEED 255

// IC2 Uno pins.
#define IC2_CLOCK A4
#define IC2_DATA A5

// PS/2 mouse pins.
#define DATA_PIN A2
#define CLOCK_PIN A3

// Sonar pin.
#define SIG_PIN A1

// Servo pin.
#define SERVO_PIN 10

// CPI of optical mouse.
#define MOUSE_CPI 800.0

// Value of 1 inch in meters.
#define INCH_TO_METER 0.0254

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
unsigned long timer = 0.0; // Time since last message was sent.
const unsigned long messageRate = 500.0; // Milliseconds.

/************************************************************
 * Odometry and Scan Data
 ************************************************************/

// Odometry data.
double x;          // X displacement.
double y;          // Y displacement.
double theta;   // Current rotation.
double offset = 0;

double distances[180]; // Stores range finder distances during scans.

enum states state;

/************************************************************
 * Hardware Connections
 ************************************************************/

PS2 mouse(CLOCK_PIN, DATA_PIN);
HMC5883L compass;

Servo sonarServo;

// Motors attached to Adafruit motor controller.
AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);
