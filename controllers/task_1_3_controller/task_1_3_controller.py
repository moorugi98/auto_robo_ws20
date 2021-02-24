"""task_1_3_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# A list containing sensor handles
prox_sensors = []
for i in range(0,8):
    p_sensor = robot.getDevice("ps"+str(i))
    p_sensor.enable(timestep)
    prox_sensors.append(p_sensor)

# A handle for each wheel encoder
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

# Start your code here!
# While the robot is not in target
#  while the robot doesn't have obstacle
#   move towards target
#  avoid obstacle

# If obstacle, avoid
# Recalibrate and move to the target

# Params
goal_eps = 100  # error for reaching the target
T = 1e5  # limit on timestep resource
l = 53
diam = 40
START = [140, 50, math.pi - (math.pi/2)]
END = [50, 360]
pos = START  # current position of the robot
trace = [pos]
old_left = 0
old_right = 0
obstacle_found = False
val_to_dist = lambda v: 70 * math.log(v) + 10  # convert sensor value to distance
dist_eps = 10  # minimal distance allowed between robot and obstacle

robot.step(timestep)
# Until convergence or divergence
while math.hypot(pos[0] - END[0], END[1] - pos[1]) > goal_eps and len(trace) < T:
    # Turn and go straight towards target when no obstacle found
    while not obstacle_found:
        x_diff = pos[0] - END[0]
        y_diff = END[1] - pos[1]

        # 1) Compute the angle between ROBOT and END
        dtheta = -pos[2] + math.atan2(y_diff, x_diff)
        if dtheta > math.pi:
            dtheta -= 2*math.pi
        if dtheta < -math.pi:
            dtheta += 2*math.pi

        # 2) Rotate the robot in its place
        needed_dist = l/2 * abs(dtheta)
        accumulated = 0
        while (accumulated < needed_dist) and (not obstacle_found):
            dright = abs(encoder_right.getValue() - old_right)  # angular distance change in single timestep
            old_left = encoder_left.getValue()
            old_right = encoder_right.getValue()
            if dtheta >= 0:
                motor_right.setVelocity(1)
                motor_left.setVelocity(-1)
                pos[2] += dright * diam / l  # TODO: eventually might have to control for cyclicity
            else:
                motor_right.setVelocity(-1)
                motor_left.setVelocity(1)
                pos[2] -= dright * diam / l
            robot.step(timestep)
            accumulated += diam * dright / 2
            # Record pos
            trace.append(pos)
            # Detect obstacle
            distances = [sensor.getValue() for sensor in prox_sensors]  # sensor values
            distances = [val_to_dist(sval) for sval in distances]  # converted to distances
            if min(distances) < dist_eps:
                obstacle_found = True

        # 3) Move in a straight line
        hypotenuse = math.hypot(x_diff, y_diff)
        accumulated = 0
        motor_right.setVelocity(1)
        motor_left.setVelocity(1)
        while (accumulated < hypotenuse) and (not obstacle_found):
            dright = abs(encoder_right.getValue() - old_right)
            old_left = encoder_left.getValue()
            old_right = encoder_right.getValue()
            drobot = dright * diam / 2  # how much the robot has moved in a single timestep
            accumulated += drobot
            pos[0] -= drobot * math.cos(pos[2])
            pos[1] += drobot * math.sin(pos[2])
            robot.step(timestep)
            # Record pos
            trace.append(pos)
            # Detect obstacle
            distances = [sensor.getValue() for sensor in prox_sensors]  # sensor values
            distances = [val_to_dist(sval) for sval in distances]  # converted to distances
            if min(distances) < dist_eps:
                obstacle_found = True

        # TODO: avoid obstacle

# Stop the robot
motor_right.setVelocity(0.0)
motor_left.setVelocity(0.0)
