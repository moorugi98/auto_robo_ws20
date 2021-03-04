"""task_2_2_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from odometry import robot_path
import math
import pickle

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#A list containing sensor handles
prox_sensors = []
for i in [0, 1, 2, 5, 6, 7]:
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
motor_left.setPosition(float('+inf')) # This is required for velocity control
motor_right.setPosition(float('+inf'))

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)



# Start your code here!

def val_to_dist(v):
    '''
    translate the sensor value to the distance
    '''
    # v = (v - 272.1955449905647) / (1391.974949347419 - 272.1955449905647)
    maxi = 1391.974949347419
    mini = 272.1955449905647
    an = 10
    bn = 20
    v = (bn-an) * (v - mini) / (maxi - mini) + an
    if v > 0:
        # d = -3.561773132741485 * math.log(v) + 4.2668543946692274
        d = -19.928891496261045 * math.log(v) + 62.40491321790478
    else:  # value so small that distance is far away enough safely
        d = threshold_sensor+10  # make sure its bigger than the threshold
    return d


def get_distance():
    '''
    get distances for all sensors
    '''
    distances = [sensor.getValue() for sensor in prox_sensors]  # sensor values
    distances = [val_to_dist(sval) for sval in distances]  # converted to distances
    return distances


l = 53
diam = 40
pos = [50, 50, 2 - (math.pi/2)]
target = [230, 360]


epsilon = 10  # error margin
dist_eps = 40  # distance to obstacle
threshold_sensor = 50

lamda = 0.025  # robot's turning speed  # 0.05
# wavelet params
beta1 = 1.15  # magnitude  # 0.25
beta2 = 5  # the bigger the more rapid the curve  # 20
sigma = 1  # locality of each wavelet

path = robot_path(pos)

while math.hypot(target[1]-pos[1], pos[0]-target[0]) > epsilon:
    psi = math.atan2(target[1] - pos[1], pos[0] - target[0])  # angle to target from robot
    robot.step(timestep)

    # add f_obs if there is any obstacle found
    distances = get_distance()  # e.g. [60,60,60,18]
    boolean_list = [d < dist_eps for d in distances]  # e.g. [False,False,False,True]
    index_rel_sensors = [i for i, x in enumerate(boolean_list) if x]  # e.g.[3]

    # compute f_obs
    f_obs = []
    lamda_obs = []
    for i in index_rel_sensors:
        lamda_obs_i = beta1 * math.exp(-distances[i] / beta2)  # eq 26
        # lamda_obs.append((lamda_obs_i))
        if i == 0:  # eq 27  si0
            psi_obs_i = pos[2] - math.radians(13)
        elif i == 1:  # si 1
            psi_obs_i = pos[2] - math.radians(45)
        elif i == 2:  # si 2
            psi_obs_i = pos[2] - math.radians(90)
        elif i == 3:  # si 5
            psi_obs_i = pos[2] + math.radians(90)
        elif i == 4:  # si 6
            psi_obs_i = pos[2] + math.radians(45)
        else:  # si 7
            psi_obs_i = pos[2] + math.radians(13)
        f_obs.append(lamda_obs_i * (pos[2] - psi_obs_i) * math.exp(-(pos[2] - psi_obs_i)**2 / (2*sigma**2)))  # eq 25

    d_pi = - lamda * math.sin(pos[2] - psi)  # eq 21: how much the robot should turn in a single timestep
    print(distances)
    # print(pos[2] + math.pi/2)
    # print(d_pi, math.degrees(pos[2] - psi), math.sin(pos[2] - psi))
    for f_obs_i in f_obs:  # eq 22
        d_pi += f_obs_i


    # Move and record position
    v = l * d_pi / 2  # motor velocity at current time
    # clip the velocity
    v = min(6, v)
    v = max(-6, v)
    motor_left.setVelocity(0.1-v)
    motor_right.setVelocity(0.1+v)
    pos = path.step(encoder_left, encoder_right)


motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

trace = path.trace
with open('trace_8cm.pickle', 'wb') as f:
    pickle.dump(trace, f)
