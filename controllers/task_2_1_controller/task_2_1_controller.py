"""task_2_1_controller controller."""

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
l = 53
diam = 40
pos = [210, 90, 2 - (math.pi/2)]
target = [190, 250]
lamda = 0.1  # robot's turning speed
psi = math.atan2(target[1] - pos[1], pos[0] - target[0])  # angle to target w.r.t. 0
epsilon = 30  # error margin
old_right = 0
old_left = 0


# d_pi = -lamda * math.sin((pos[2] - psi))  # how much the robot should turn
# if d_pi >= 0:
#     motor_right.setVelocity(1.0)
#     motor_left.setVelocity(-1.0)
# else:
#     motor_right.setVelocity(-1.0)
#     motor_left.setVelocity(1.0)
#
# desired_wheel_travel = abs(d_pi) * l / 2  # how much the wheel should travel so that robot turns d_pi
# accum = 0
# while accum < desired_wheel_travel:
#     robot.step(timestep)
#     accum += abs(encoder_right.getValue() - old_right)
#     print(accum)
#     old_right = encoder_right.getValue()
#
# motor_left.setVelocity(0.0)
# motor_right.setVelocity(0.0)


# robot.step(timestep)
while math.hypot(pos[0]-target[0], pos[1]-target[1]) > epsilon:  # until convergence
    d_pi = -lamda * math.sin((pos[2] - psi))  # how much the robot should turn
    if d_pi >= 0:
        motor_right.setVelocity(2.0)
        motor_left.setVelocity(-1.0)
    else:
        motor_right.setVelocity(-1.0)
        motor_left.setVelocity(2.0)

    desired_wheel_travel = abs(d_pi) * l / 2  # how much the wheel should travel so that robot turns d_pi
    accum = 0
    while accum < desired_wheel_travel:
        robot.step(timestep)

        dleft = encoder_left.getValue() - old_left
        dright = encoder_right.getValue() - old_right
        old_left = encoder_left.getValue()
        old_right = encoder_right.getValue()
        # print(dleft, dright)

        accum += abs(dright)

        # update position
        r = ((dright + dleft) * l) / ((dright - dleft) * 2)  # eq 6
        dtheta = (dright - dleft) * diam / (2 * l)  # eq 7
        dx_ego = r * math.sin(dtheta)  # eq 9
        dy_ego = r * (1 - math.cos(dtheta))  # eq 10
        pos[0] -= dx_ego * math.cos(pos[2]) - dy_ego * math.sin(pos[2])  # eq 12, 13, 11
        pos[1] += dx_ego * math.sin(pos[2]) + dy_ego * math.cos(pos[2])
        pos[2] += dtheta


motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

# l = 53
# pos = [210, 90, math.pi - (math.pi/2)]
# target = [50, 60]
# lamda = 0.1  # robot's turning speed
# psi = math.atan2(target[1] - pos[1], pos[0] - target[0])  # angle to target w.r.t. 0
# epsilon = 0.01  # error margin
# dt = timestep * 0.001
# print('psi in deg', math.degrees(psi))
#
# while abs(pos[2] - psi) > epsilon:
#     # Compute the necessary wheel velocity
#     v = (l * lamda * (psi - pos[2])) / (2 * dt)
#     print(psi - pos[2])
#     # Turn the robot
#     motor_left.setVelocity(-v)
#     motor_right.setVelocity(v)
#
#     # Keep track of position, eq 17
#     pos[2] += (-lamda * (pos[2] - psi)) * dt
#     print(pos[2] + (math.pi/2))
#
#     robot.step(timestep)

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)
