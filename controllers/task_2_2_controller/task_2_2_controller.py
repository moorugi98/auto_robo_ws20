"""task_2_2_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from odometry import robot_path
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#A list containing sensor handles
prox_sensors = []
for i in range(0,8):
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



# Start your code here!

def val_to_dist(v):
    '''
    translate the sensor value to the distance
    '''
    v = (v - 272.1955449905647) / (1391.974949347419 - 272.1955449905647)
    if v > 0:
        d = -3.561773132741485 * math.log(v) + 4.2668543946692274
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
pos = [140, 50, math.pi - (math.pi/2)]
target = [50, 360]
lamda = 0.1  # robot's turning speed
psi = math.atan2(target[1] - pos[1], pos[0] - target[0])  # angle to target w.r.t. 0
epsilon = 10  # error margin
dist_eps = 20  # distance to obstacle
threshold_sensor = 50
# wavelet params
beta1 = 1  # magnitude
beta2 = 10  # the bigger the more rapid the curve
sigma = 1  # locality of each wavelet

path = robot_path(pos)

while math.hypot(target[1]-pos[1], pos[0]-target[0]) > epsilon:
    robot.step(timestep)

    # add f_obs if there is any obstacle found
    distances = get_distance()  # [60,60,,..,60,18]
    boolean_list = [d < dist_eps for d in distances]  # [False,...,False,True]
    index_rel_sensors = [i for i, x in enumerate(boolean_list) if x]  # [7]
    # TODO: control for empty case
    f_obs = []
    for i in index_rel_sensors:
        lamda_obs_i = beta1 * math.exp(-distances[i] / beta2)  # eq 26
        if i == 0:  # eq 27
            pi_obs_i = pos[2] - math.radians(13)
        elif i == 1:
            pi_obs_i = pos[2] - math.radians(45)
        # TODO: for all 8 sensors
        else:
            pi_obs_i = pos[2] + math.radians(13)
        f_obs.append(lamda_obs_i * (pos[2] - pi_obs_i) * math.exp((pos[2] - pi_obs_i)**2 / (2*sigma**2)))  # eq 25

    d_pi = - lamda * math.sin(pos[2] - psi)  # eq 21: how much the robot should turn in a single timestep
    for f_obs_i in f_obs:  # eq 22
        d_pi += f_obs_i
    # print(d_pi, f_obs)

    v = l * d_pi / 2  # motor velocity at current time
    motor_left.setVelocity(1-v)
    motor_right.setVelocity(1+v)
    pos = path.step(encoder_left, encoder_right)


motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)
