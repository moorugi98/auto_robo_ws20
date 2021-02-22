"""task_A_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# A handle for each motor
motor_left = robot.getDevice("left wheel motor")
motor_right = robot.getDevice("right wheel motor")

#Enable Velocity Control 
motor_left.setPosition(float('+inf')) # This is required for velocity control
motor_right.setPosition(float('+inf'))

# initial state
motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

# Start your code here!

# rotate
motor_left.setVelocity(4.0)
for _ in range(30):
    robot.step(timestep)

# straight line
motor_right.setVelocity(4.0)
[robot.step(timestep) for _ in range(30)]

# rotate around obstacle
motor_left.setVelocity(0.0)
[robot.step(timestep) for _ in range(40)]

# diagonal line
motor_left.setVelocity(4.0)
[robot.step(timestep) for _ in range(70)]

# rotate to target
motor_right.setVelocity(0.0)
[robot.step(timestep) for _ in range(40)]

# move to target
motor_right.setVelocity(4.0)
[robot.step(timestep) for _ in range(45)]

# stop the robot
motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)
robot.step(timestep)

