import time
import RPi.GPIO as io

'''

'''
class MotorController():
	'''
	
	'''
	def __init__(self):
		# Motor 1 pins.
		self.M1_EN = 2
		self.M1_PWM = 3 

		# Motor 2 pins.
		self.M2_EN = 4
		self.M2_PWM = 17 

		# Motor 3 pins.
		self.M3_EN = 27
		self.M3_PWM = 22 

		# Motor 4 pins.
		self.M4_EN = 23
		self.M4_PWM = 24 
		
		self.MOTOR_SPEED = 50
		
		self.setup_motors()

	'''
	Sets all the motor pins to output mode and configures PWM.
	'''
	def setup_motors(self):
		io.setup(self.M1_EN, io.OUT)
		io.setup(self.M1_PWM, io.OUT)

		io.setup(self.M2_EN, io.OUT)
		io.setup(self.M2_PWM, io.OUT)

		io.setup(self.M3_EN, io.OUT)
		io.setup(self.M3_PWM, io.OUT)

		io.setup(self.M4_EN, io.OUT)
		io.setup(self.M4_PWM, io.OUT)
		
		# Setup PWM outputs for controlling speed.
		self.motor_1 = io.PWM(self.M1_PWM, self.MOTOR_SPEED)
		self.motor_2 = io.PWM(self.M2_PWM, self.MOTOR_SPEED)
		self.motor_3 = io.PWM(self.M3_PWM, self.MOTOR_SPEED)
		self.motor_4 = io.PWM(self.M4_PWM, self.MOTOR_SPEED)
		
	'''
	Clean up the motor controller and stops the motors from running 
	after GPIO.cleanup() by setting the pins to GPIO.IN.
	'''
	def clean_up(self):
		self.halt()
		
		io.setup(self.M1_EN, io.IN)
		io.setup(self.M1_PWM, io.IN)

		io.setup(self.M2_EN, io.IN)
		io.setup(self.M2_PWM, io.IN)

		io.setup(self.M3_EN, io.IN)
		io.setup(self.M3_PWM, io.IN)

		io.setup(self.M4_EN, io.IN)
		io.setup(self.M4_PWM, io.IN)

	'''
	Drives all motors forward.
	'''
	def go_forward(self):
		io.output(self.M1_EN, False)
		self.motor_1.start(self.MOTOR_SPEED)
		
		io.output(self.M2_EN, False)
		self.motor_2.start(self.MOTOR_SPEED)
		
		io.output(self.M3_EN, False)
		self.motor_3.start(self.MOTOR_SPEED)
		
		io.output(self.M3_EN, False)
		self.motor_3.start(self.MOTOR_SPEED)

	'''
	Drives all motors backward.
	'''	
	def go_backward(self):
		io.output(self.M1_EN, True)
		self.motor_1.start(self.MOTOR_SPEED)
		
		io.output(self.M2_EN, True)
		self.motor_2.start(self.MOTOR_SPEED)
		
		io.output(self.M3_EN, True)
		self.motor_3.start(self.MOTOR_SPEED)
		
		io.output(self.M4_EN, True)
		self.motor_4.start(self.MOTOR_SPEED)

	'''
	Rotates the robot right by driving M1 and M4 forward, M2 and M3 backward.
	'''	
	def rotate_right(self):	
		io.output(self.M1_EN, True)
		self.motor_1.start(self.MOTOR_SPEED)
		
		io.output(self.M2_EN, False)
		self.motor_2.start(self.MOTOR_SPEED)
		
		io.output(self.M3_EN, True)
		self.motor_3.start(self.MOTOR_SPEED)
		
		io.output(self.M4_EN, False)
		self.motor_4.start(self.MOTOR_SPEED)

	'''
	Rotates the robot left by driving M2 and M3 forward, M1 and M4 backward.
	'''	
	def rotate_left(self):
		io.output(self.M1_EN, False)
		self.motor_1.start(self.MOTOR_SPEED)
		
		io.output(self.M2_EN, True)
		self.motor_2.start(self.MOTOR_SPEED)
		
		io.output(self.M3_EN, False)
		self.motor_3.start(self.MOTOR_SPEED)
		
		io.output(self.M4_EN, True)
		self.motor_4.start(self.MOTOR_SPEED)

	'''
	Halts all motors.
	'''	
	def halt(self):
		io.output(self.M1_EN, False)
		self.motor_1.stop()
		
		io.output(self.M2_EN, False)
		self.motor_2.stop()
		
		io.output(self.M3_EN, False)
		self.motor_3.stop()
		
		io.output(self.M4_EN, False)
		self.motor_4.stop()
