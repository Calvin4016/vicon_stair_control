#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.PWM as PWM
from time import sleep

STAIR_TOP = 0.28 # VICON unit
STAIR_BOTTOM = 1.07 # VICON unit
STAIR_LENGTH = STAIR_BOTTOM - STAIR_TOP
DESIRED_LOCATION = 0.2 # Percentage of stair length

P_GAIN = 0.5
I_GAIN = 0.2
D_GAIN = 0

# PWM Pin P9_14
speedPin = "P9_14"
PWM.start(speedPin, 0)

class PIDController:
    def __init__(self,p,i,d):
        self.kP = p
	self.kI = i
	self.kD = d
	self.target = 0

	self.lastError = 0
	self.integrator = 0

    def setTarget(self, newTarget):
	self.target = newTarget
	self.integrator = 0

    def step(self, currentValue):
	error = currentValue - self.target
	output = self.kP*error + self.kI*self.integrator + self.kD*(error-self.lastError)
	self.lastError = error
	self.integrator += error

	return output

    def callback(self, msg):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        z = msg.transform.translation.z

	x = (x - STAIR_TOP) / STAIR_LENGTH

	if x < DESIRED_LOCATION:
		x = 0

	dutycycle = self.step(x)

	if dutycycle < 0:
		dutycycle = 0

	# Change dutycycle from percentage to 0 ~ 100
	dutycycle = dutycycle * 100

        # Display position info on log screen
	rospy.loginfo('x: {},    PWM: {}'.format(x, dutycycle))

        PWM.set_duty_cycle(speedPin, dutycycle)


def main():
    # PID values
    p = P_GAIN
    i = I_GAIN
    d = D_GAIN
    target = DESIRED_LOCATION
    pidControl = PIDController(p, i, d)
    pidControl.setTarget(target);

    rospy.init_node('controller_node')
    # Temporary name used here, modify according to VICON software in gaits lab
    rospy.Subscriber("/vicon/stairBody/threeMarker", TransformStamped, pidControl.callback)
    rospy.spin()

if __name__ == '__main__':
    main()
