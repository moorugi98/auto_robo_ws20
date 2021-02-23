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

# Enable Velocity Control
motor_left.setPosition(float('+inf'))  # This is required for velocity control
motor_right.setPosition(float('+inf'))

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

### Start your code here! ###

# Set parameters
l = 53  # distance between two wheels in mm
diam = 40  # diameter of wheels in mm
dt = 10  # fix the time to integrate and change velocity
START = [210, 90, 90]  # red, blue (allocentric in mm), angle in deg with 0 deg as in ex sheet
END = [70, 340]

x_diff = START[0]-END[0]  # x-axis is inverted to what we use
y_diff = END[1]-START[1]


# 1) Compute the angle
theta_tri = math.atan2(y_diff, x_diff)  # angle of triangle in rad w.r.t. positive zero x-axis (sign is took care of)
# total rotation in rad
dtheta = -math.radians(START[2]  # go to 0 deg
                       ) + theta_tri  # right rotation in an allocentric frame

# instead of turning 240 anti-clw, turn 120 clw
if dtheta > math.pi:
    dtheta -= 2*math.pi
if dtheta < -math.pi:
    dtheta += 2*math.pi


# 2) Rotate the robot
needed_dist = abs(l * dtheta / 2)  # full rotation = 2 * pi * (l/2), theta rotation = 2* pi * (l/2) * (theta/(2*pi))
accumulated = 0  # distance covered until now

# set arbitrary velocity and control turning direction
if dtheta >= 0:  # anti-clockwise
    motor_right.setVelocity(1)
    motor_left.setVelocity(-1)
else:  # clockwise
    motor_right.setVelocity(-1)
    motor_left.setVelocity(1)

# turn the robot until wheel traveled the needed distance
robot.step(timestep)
while accumulated < needed_dist:
    # 2pi(diam/2): circumference of the wheel, enc_val/2pi: num_rotation of wheel, both multiplied
    accumulated = diam * abs(encoder_right.getValue()) / 2
    robot.step(timestep)
# subtract to 'reset' the encoder later
old_encode = abs(encoder_right.getValue())


# 3) Move in a straight line
hypotenuse = math.hypot(x_diff, y_diff)  # length of the hypotenuse of a right triangle
accumulated = 0

motor_right.setVelocity(1)
motor_left.setVelocity(1)

robot.step(timestep)
while accumulated < hypotenuse:
    accumulated = diam * (abs(encoder_right.getValue()) - old_encode) / 2
    robot.step(timestep)


# Stop the robot
motor_right.setVelocity(0.0)
motor_left.setVelocity(0.0)
print('angle, rotat dist, line dist', dtheta, needed_dist, hypotenuse, accumulated)
