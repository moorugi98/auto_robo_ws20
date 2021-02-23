"""task_1_2_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import pickle

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
# parameters
diam = 40
num_trials = 10  # number to repeat each distance
num_dist = 20  # number of different locations where distance should be measured
max_pos = 140
start_pos = 60
interval = (max_pos - start_pos) / (num_dist-1)  # increment between each measure
results = []

old_encoder = 0  # to reset the encoder
# repeat num_trials measurements
for n in range(num_trials):
    subres = []
    # for each trial, move back and forth repeatedly
    if n % 2 == 0:
        v = 1.0
    else:
        v = -1.0

    # Vary the distance and measure from the sensor
    for _ in range(num_dist-1):
        # Stop and measure
        motor_right.setVelocity(0.0)
        motor_left.setVelocity(0.0)
        [robot.step(timestep) for _ in range(50)]
        subres.append(prox_sensors[0].getValue())

        # Moving the robot
        accumulated = 0
        motor_right.setVelocity(v)
        motor_left.setVelocity(v)
        while accumulated < interval:
            robot.step(timestep)
            accumulated = abs(abs(encoder_right.getValue()) - old_encoder) * diam / 2
        old_encoder = abs(encoder_right.getValue())
    subres.append(prox_sensors[0].getValue())   # measure one last time
    if n % 2 == 1:
        subres.reverse()
    results.append(subres)
motor_right.setVelocity(0.0)
motor_left.setVelocity(0.0)

# save data
with open("test.txt", "wb") as fp:   # pickling
    pickle.dump(results, fp)




