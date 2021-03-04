"""task_1_2_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import pickle
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#A list containing sensor handles
prox_sensors = []
for i in range(8):
    p_sensor = robot.getDevice("ps"+str(i))
    p_sensor.enable(timestep)
    prox_sensors.append(p_sensor)

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

##### Start your code here!

# # To measure
# for _ in range(100):  # make sure the robot doesn't move
#     robot.step(timestep)
# for _ in range(10):  # take 10 measurements
#     robot.step(timestep)
#     print(prox_sensors[0].getValue())

# Demonstration
crit_val = 10  # mm, stop when robot get this much close to the obstacle
motor_left.setVelocity(1.0)
motor_right.setVelocity(1.0)
obstacle = False
while not obstacle:
    robot.step(timestep)
    v = prox_sensors[0].getValue()
    v = (v - 272.1955449905647) / (1391.974949347419 - 272.1955449905647)  # normalize
    if v > 0:  # otherwise log won't be defined
        d = -3.561773132741485 * math.log(v) + 4.2668543946692274  # convert sensor to distance
        if d <= crit_val:
            obstacle = True
motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

