"""task_A_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#A handle for each wheel encoder
encoder_left = robot.getDevice("left wheel sensor")
encoder_right = robot.getDevice("right wheel sensor")

encoder_left.enable(timestep)
encoder_right.enable(timestep)

# A handle for each motor
motor_left = robot.getDevice("left wheel motor")
motor_right = robot.getDevice("right wheel motor")

#Enable Velocity Control 
motor_left.setPosition(float('+inf')) #This is required for velocity control
motor_right.setPosition(float('+inf'))

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

# Start your code here!

# Set parameters
l = 0.053  # distance between two wheels in m
diam = 0.04  # diameter of wheels in m
dt = 10  # fix the time to integrate and change velocity
START = [0.21, 0.09, 90]  # red, blue (allocentric in m), angle in deg with 0 deg as in ex sheet
END = [0.07, 0.34]

x_diff = START[0]-END[0]
y_diff = END[1]-START[1]

# 1) Rotate the robot
theta_tri = math.atan2(y_diff, x_diff)  # angle of triangle in rad to positive zero x-axis
# total rotation in rad
dtheta = -math.radians(START[2]  # go to 0 deg
                       ) + theta_tri  # right rotation in allocentric frame

# # compute velocity
# v = (l * dtheta) / (2 * dt)
# print(theta_tri, dtheta)
# motor_right.setVelocity(v)
# motor_left.setVelocity(-v)
# [robot.step(timestep) for _ in range(dt)]

# Use encoder to see if wheel traveled the right amount
needed_dist = abs(l * dtheta*1000)  # 2pi (rad) : 2pi*l (circumference of the circle made by two wheels)
accumulated = 0  # distance covered until now

if dtheta >= 0:  # anti-clockwise
    motor_right.setVelocity(0.5)
    motor_left.setVelocity(-0.5)
else:  # clockwise
    motor_right.setVelocity(-0.5)
    motor_left.setVelocity(0.5)

while accumulated < needed_dist:
    accumulated += encoder_right.getValue()  # distance traveled by wheels
    robot.step(timestep)
    print(dtheta, needed_dist, accumulated)

# # 2) Move in a straight line
# hypotenuse = abs(x_diff * math.cos(theta_tri))
# v = (hypotenuse * 2*math.pi) / (diam * dt)
# motor_right.setVelocity(v)
# motor_left.setVelocity(v)
# print(v, hypotenuse)
# [robot.step(timestep) for _ in range(dt)]


# Stop the robot
motor_right.setVelocity(0.0)
motor_left.setVelocity(0.0)


