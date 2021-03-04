"""task_2_1_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
from odometry import robot_path
import pickle

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
target = [70, 340]
lamda = 10  # robot's turning speed
psi = math.atan2(target[1] - pos[1], pos[0] - target[0])  # angle to target w.r.t. 0
epsilon = 10  # error margin

path = robot_path(pos)
counter = 0
while math.hypot(target[1]-pos[1], pos[0]-target[0]) > epsilon and counter < 1e3:
    counter += 1
    print(counter)
    robot.step(timestep)
    d_pi = - lamda * math.sin(pos[2] - psi)  # how much the robot should turn in a single timestep  (eq 17)
    v = l * d_pi / 2  # motor velocity at current time  (eq 3,5)
    motor_left.setVelocity(1-v)  # move forward and turn at the same time
    motor_right.setVelocity(1+v)
    pos = path.step(encoder_left, encoder_right)  # update the robot position

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

trace = path.trace
with open('trace_lamda={}.pickle'.format(lamda), 'wb') as f:
    pickle.dump(trace, f)
