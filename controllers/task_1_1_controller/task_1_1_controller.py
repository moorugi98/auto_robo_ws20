"""task_1_1_controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import random_movement_generator

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#A handle for each wheel encoder
encoder_left = robot.getDevice("left wheel sensor")
encoder_right = robot.getDevice("right wheel sensor")

encoder_left.enable(timestep)
encoder_right.enable(timestep)

#### Write your code here



####

#Initialize the Random Movement Generator
is_still_moving = True
rmg = random_movement_generator.random_move_generator(robot)
#Do not change anything about this!

#Random Movmement Loop
while robot.step(timestep) != -1  and is_still_moving:
    is_still_moving = rmg.move() #Update Speed Commands
    
    #### Write your Code here
    
    
    ####


### Plot your position here!