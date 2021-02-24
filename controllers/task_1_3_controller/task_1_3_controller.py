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
for i in [0,1,6,7]:  # TODO: only front 4 sensors should be enough?
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
T = 1e5  # limit on timestep resource
l = 53  # distance between two wheels
diam = 40  # diameter of the wheel
r = 71/2  # radius of the robot

START = [140, 50, math.pi - (math.pi/2)]
END = [50, 360]
target = END  # where the robot is currently trying to go, alternate between START and END
pos = START  # current position of the robot
trace = [pos]

old_left = 0
old_right = 0
obstacle_found = False
dist_eps = 20  # minimal distance allowed between robot and obstacle, error reaching the target
threshold_sensor = 100  # anything bigger than threshold means that sensor doesn't sense anything


def val_to_dist (v):
    v = (v - 272.1955449905647) / (1391.974949347419 - 272.1955449905647)
    if v > 0:
        d = -3.561773132741485 * math.log(v) + 4.2668543946692274
    else:  # value so small that distance is far away enough safely
        d = threshold_sensor+10  # make sure its bigger than the threshold
    return d


robot.step(timestep)
# TODO: 0) change Target when converged
# 1) Until convergence or divergence
while math.hypot(pos[0] - target[0], target[1] - pos[1]) > dist_eps and len(trace) < T:
    # 2) Turn towards the target
    # compute the angle needed in total
    x_diff = pos[0] - target[0]
    y_diff = target[1] - pos[1]
    dtheta = -pos[2] + math.atan2(y_diff, x_diff)
    if dtheta > math.pi:
        dtheta -= 2 * math.pi
    if dtheta < -math.pi:
        dtheta += 2 * math.pi
    # set velocity
    if dtheta >= 0:
        motor_right.setVelocity(1)
        motor_left.setVelocity(-1)
    else:
        motor_right.setVelocity(-1)
        motor_left.setVelocity(1)
    rotat_dist_needed = l / 2 * abs(dtheta)  # if robot turning accumulates until this, robot has fully turned to target
    accum_rotation = 0  # absolute amount that has turned
    while (accum_rotation < rotat_dist_needed) and (not obstacle_found):
        # compute the distance moved by wheels
        dright = encoder_right.getValue() - old_right  # angular distance change in single timestep
        old_left = encoder_left.getValue()
        old_right = encoder_right.getValue()
        accum_rotation += diam * abs(dright) / 2
        # update position
        pos[2] += dright * diam / l  # TODO: eventually might have to control for cyclicity (modulo)
        # make the movement
        robot.step(timestep)
        # Record pos
        trace.append(pos)
        # Detect obstacle
        distances = [sensor.getValue() for sensor in prox_sensors]  # sensor values
        distances = [val_to_dist(sval) for sval in distances]  # converted to distances
        if min(distances) < dist_eps:  # If any sensor detect something
            obstacle_found = True


    # 3) Move in a straight line
    strat_dist_needed = math.hypot(x_diff, y_diff)
    accum_straight = 0
    motor_right.setVelocity(1.0)
    motor_left.setVelocity(1.0)
    while (accum_straight < strat_dist_needed) and (not obstacle_found):
        # compute the distance moved by wheels
        drobot = abs(encoder_right.getValue() - old_right) * diam / 2  # how much the robot has moved in a single timestep
        old_left = encoder_left.getValue()
        old_right = encoder_right.getValue()
        accum_straight += drobot
        # update position
        pos[0] -= drobot * math.cos(pos[2])
        pos[1] += drobot * math.sin(pos[2])
        # make the movement
        robot.step(timestep)
        # Record pos
        trace.append(pos)
        # Detect obstacle
        distances = [sensor.getValue() for sensor in prox_sensors]  # sensor values
        distances = [val_to_dist(sval) for sval in distances]  # converted to distances
        if min(distances) < dist_eps:
            obstacle_found = True


    # 4) Avoid the obstacle
    while obstacle_found:
        motor_right.setVelocity(0.0)
        motor_left.setVelocity(0.0)
        robot.step(timestep)
        # Case 1: the wall is in front of the robot
        # print(distances)
        if all([d < threshold_sensor for d in distances]):
            # compute the angle
            istribase = 2*r*math.sin(math.radians(13))
            b1 = val_to_dist(prox_sensors[0].getValue())
            b2 = val_to_dist(prox_sensors[-1].getValue())
            phi = math.pi/2 #- math.asin((b1-b2) / istribase)  # TODO: b1-b2 or b2-b1 depends on the sign

            # turn that much

            # go straight until case 2

        # Case 2: the wall is to the side of the robot
        # else:



# Stop the robot
motor_right.setVelocity(0.0)
motor_left.setVelocity(0.0)
