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
l = 53  # distance between two wheels in mm
diam = 40  # diameter of wheels in mm
dt = 10  # fix the time to integrate and change velocity
START = [210, 90,  # x,z in mm, angle in rad as given in the webot interface
         3.14 - math.pi/2]  # -pi/2 controls for the difference between webot and convention (0 deg at positive x-axis)
END = [50, 60]

x_diff = START[0]-END[0]  # x-coord is flipped
y_diff = END[1]-START[1]


# 1) Compute the angle
theta_tri = math.atan2(y_diff, x_diff)  # angle of triangle in rad w.r.t. positive zero x-axis
# total rotation in rad
dtheta = -START[2] + theta_tri  # right rotation in allocentric frame
# revert turning direction if inefficient
if dtheta > math.pi:
    dtheta -= 2*math.pi
if dtheta < -math.pi:
    dtheta += 2*math.pi


# 2) Rotate the robot
needed_dist = abs(l/2 * dtheta)  # 2pi (rad) : 2pi*(l/2) (circumference of the circle made by two wheels)
accumulated = 0  # distance covered until now

while accumulated < needed_dist:
    if dtheta >= 0:  # counter-clockwise
        motor_right.setVelocity(1)
        motor_left.setVelocity(-1)
    else:  # clockwise
        motor_right.setVelocity(-1)
        motor_left.setVelocity(1)
    robot.step(timestep)
    accumulated = diam * abs(encoder_right.getValue()) / 2  # distance traveled by wheels


# 3) Move in a straight line
hypotenuse = math.hypot(x_diff, y_diff)  # don't travel until the end but only to center of robot
accumulated = 0
old_encode = encoder_right.getValue()
motor_right.setVelocity(1)
motor_left.setVelocity(1)

while accumulated < hypotenuse:
    robot.step(timestep)
    accumulated = diam * (encoder_right.getValue() - old_encode) / 2

# Stop the robot
motor_right.setVelocity(0.0)
motor_left.setVelocity(0.0)
