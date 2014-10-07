import time
import RPi.GPIO as io

class MotorController():
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
		
		self.setup_motors()

	'''
	Sets all the motor pins to output mode.
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

	'''
	Drives all motors forward.
	'''
	def go_forward(self):
		io.output(self.M1_EN, True)
		io.output(self.M1_PWM, False)
		
		io.output(self.M2_EN, True)
		io.output(self.M2_PWM, False)
		
		io.output(self.M3_EN, True)
		io.output(self.M3_PWM, False)
		
		io.output(self.M4_EN, True)
		io.output(self.M4_PWM, False)

	'''
	Drives all motors backward.
	'''	
	def go_backward(self):
		io.output(self.M1_EN, True)
		io.output(self.M1_PWM, True)
		
		io.output(self.M2_EN, True)
		io.output(self.M2_PWM, True)
		
		io.output(self.M3_EN, True)
		io.output(self.M3_PWM, True)
		
		io.output(self.M4_EN, True)
		io.output(self.M4_PWM, True)

	'''
	Rotates the robot right by driving M1 and M4 forward, M2 and M3 backward.
	'''	
	def rotate_right(self):
		io.output(self.M1_EN, True) # Drive M1 forward.
		io.output(self.M1_PWM, False)
		
		io.output(self.M2_EN, True) # Drive M2 backward.
		io.output(self.M2_PWM, True)
		
		io.output(self.M3_EN, True) # Drive M3 backward.
		io.output(self.M3_PWM, True)
		
		io.output(self.M4_EN, True) # Drive M4 forward.
		io.output(self.M4_PWM, True)

	'''
	Rotates the robot left by driving M2 and M3 forward, M1 and M4 backward.
	'''	
	def rotate_left(self):
		io.output(self.M1_EN, True) # Drive M1 backward.
		io.output(self.M1_PWM, True)
		
		io.output(self.M2_EN, True) # Drive M2 forward.
		io.output(self.M2_PWM, False)
		
		io.output(self.M3_EN, True) # Drive M3 forward.
		io.output(self.M3_PWM, False)
		
		io.output(self.M4_EN, True) # Drive M4 backward.
		io.output(self.M4_PWM, True)

	'''
	Halts all motors.
	'''	
	def halt(self):
		io.output(self.M1_EN, False)
		io.output(self.M1_PWM, False)
		
		io.output(self.M2_EN, False)
		io.output(self.M2_PWM, False)
		
		io.output(self.M3_EN, False)
		io.output(self.M3_PWM, False)
		
		io.output(self.M4_EN, False)
		io.output(self.M4_PWM, False)
