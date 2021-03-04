"""task_1_3_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
from copy import deepcopy
import pickle


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# A list containing sensor handles
prox_sensors = []
for i in [0,1,2,5,6,7]:  # don't use sensors at the back
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


# Params
T = 1e5  # limit on timestep resource
l = 53  # distance between two wheels
diam = 40  # diameter of the wheel
r = 71/2  # radius of the robot

START = [50, 50, 2 - (math.pi/2)]
END = [230, 360]
target = deepcopy(END)  # where the robot is currently trying to go, alternate between START and END
pos = deepcopy(START)  # current position of the robot
trace = [deepcopy(pos)]

old_right = 0
obstacle_found = False
dist_eps = 10  # minimal distance allowed between robot and obstacle, error reaching the target
threshold_sensor = 100  # anything bigger than threshold means that sensor doesn't sense anything


def val_to_dist(v):
    v = (v - 272.1955449905647) / (1391.974949347419 - 272.1955449905647)
    if v > 0:
        d = -3.561773132741485 * math.log(v) + 4.2668543946692274
    else:  # value so small that distance is far away enough safely
        d = threshold_sensor+10  # make sure its bigger than the threshold
    return d


def get_distance():
    distances = [sensor.getValue() for sensor in prox_sensors]  # sensor values
    distances = [val_to_dist(sval) for sval in distances]  # converted to distances
    return distances


def move_turn():
    '''turn on the spot and update angle'''
    global old_right
    robot.step(timestep)
    dright = encoder_right.getValue() - old_right
    old_right = encoder_right.getValue()
    pos[2] += dright * diam / l
    pos[2] = pos[2] % (2*math.pi)  # stop angle accumulating over 2pi
    trace.append(deepcopy(pos))
    return dright


def move_straight():
    '''move straight and update x and y coord'''
    global old_right
    robot.step(timestep)
    drobot = abs(encoder_right.getValue() - old_right) * diam / 2
    old_right = encoder_right.getValue()
    pos[0] -= drobot * math.cos(pos[2])
    pos[1] += drobot * math.sin(pos[2])
    trace.append(deepcopy(pos))
    return drobot



obstacle_found = False
robot.step(timestep)

# 0) change Target when converged
counter = 0
while counter < 1:
    if counter % 2 == 0:
        target = deepcopy(END)
    else:
        target = deepcopy(START)
    counter += 1

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
            dright = move_turn()
            accum_rotation += diam * abs(dright) / 2
            # Detect obstacle
            distances = get_distance()
            if min(distances) < dist_eps:  # If any sensor detect something
                obstacle_found = True


        # 3) Move in a straight line
        strat_dist_needed = math.hypot(x_diff, y_diff)
        accum_straight = 0
        motor_right.setVelocity(1.0)
        motor_left.setVelocity(1.0)
        while (accum_straight < strat_dist_needed) and (not obstacle_found):
            accum_straight += move_straight()

            # Detect obstacle
            distances = get_distance()
            if min(distances) < dist_eps:  # If any sensor detect something
                obstacle_found = True
        while obstacle_found:
            
            # Set velocity depending on angle
            distances = get_distance()
            argmin_dist = distances.index(min(distances))
            if argmin_dist == 0:  # ps0, rapid clw
                motor_right.setVelocity(1)
                motor_left.setVelocity(-1)
                [move_turn() for _ in range(50)]
            elif argmin_dist == 1:  # ps1, mid clw
                motor_right.setVelocity(1)
                motor_left.setVelocity(-1)
                [move_turn() for _ in range(20)]
            elif argmin_dist == 2:  # ps2, slow clw
                motor_right.setVelocity(1)
                motor_left.setVelocity(-1)
                [move_turn() for _ in range(10)]
            elif argmin_dist == 3:  # ps5, slow aclw
                motor_right.setVelocity(-1)
                motor_left.setVelocity(1)
                [move_turn() for _ in range(10)]
            elif argmin_dist == 4:  # ps6, mid aclw
                motor_right.setVelocity(-1)
                motor_left.setVelocity(1)
                [move_turn() for _ in range(20)]
            else:  # ps7, rapid aclw
                motor_right.setVelocity(-1)
                motor_left.setVelocity(1)
                [move_turn() for _ in range(50)]
            
            # go straight for a bit
            motor_left.setVelocity(1.0)
            motor_right.setVelocity(1.0)
            [move_straight()for _ in range(10)]

            # detect obstacle
            distances = get_distance()
            if min(distances) < dist_eps:  # If any sensor detect something
                obstacle_found = True
            else:
                obstacle_found = False
                

        # Now move straight
        motor_left.setVelocity(1.0)
        motor_right.setVelocity(1.0)
        if not obstacle_found:
            move_straight()
            # detect obstacle
            distances = get_distance()
            if min(distances) < dist_eps:
                obstacle_found = True


# Stop the robot
motor_right.setVelocity(0.0)
motor_left.setVelocity(0.0)



# # Save the trace
with open('trace_8cm.pickle', 'wb') as handle:
    pickle.dump(trace, handle, protocol=pickle.HIGHEST_PROTOCOL)
