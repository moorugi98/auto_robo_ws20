"""task_1_2_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
import matplotlib.pyplot as plt

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

# Start your code here!
robot.step(timestep)
print(prox_sensors[0].getValue())

# parameters
diam = 40
num_trials = 10
num_dist = 5
max_pos = 140
current_pos = 60
interval = (max_pos - current_pos) / num_dist  # increment between each measure
results = np.zeros((num_trials, num_dist))

# repeat num_trials measurements
for n in range(num_trials):
    # for each trial, move back and forth repeatedly
    if n % 2 == 0:
        v = 1.0
    else:
        v = -1.0

    # Vary the distance and measure from the sensor
    for pi, pos in enumerate(np.linspace(current_pos, max_pos, num_dist)):
        # Moving the robot
        accumulated = 0
        motor_right.setVelocity(v)
        motor_left.setVelocity(v)
        while accumulated < interval:
            robot.step(timestep)
            accumulated = encoder_right.getValue() * diam / 2

        # Stop and measure
        motor_right.setVelocity(0.0)
        motor_left.setVelocity(0.0)
        results[n, pi] = prox_sensors[0].getValue()


# Plot
mean = np.mean(results, axis=0)
std = np.std(results, axis=0)
plt.plot()
plt.fill_between(range(results.shape[1]), mean-std, mean+std, alpha=0.5)
plt.show()
