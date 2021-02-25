"""task_2_1_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
from odometry import robot_path

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
l = 53
diam = 40
pos = [210, 90, math.pi - (math.pi/2)]
target = [190, 250]
lamda = 0.1  # robot's turning speed
psi = math.atan2(target[1] - pos[1], pos[0] - target[0])  # angle to target w.r.t. 0
print(psi)
epsilon = 5  # error margin
old_right = 0
old_left = 0


path = robot_path(pos)

while math.hypot(target[1]-pos[1], pos[0]-target[0]) > epsilon:
    robot.step(timestep)
    d_pi = - lamda * math.sin(pos[2] - psi)  # how much the robot should turn in a single timestep
    v = l * d_pi / 2  # motor velocity at current time
    motor_left.setVelocity(1-v)
    motor_right.setVelocity(1+v)
    pos = path.step(encoder_left, encoder_right)
    

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)
