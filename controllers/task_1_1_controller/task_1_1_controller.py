"""task_1_1_controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import random_movement_generator
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

#### Write your code here
diam = 40
l = 53
epsilon = 0.00001

old_left = 0
old_right = 0
pos = [190, 250, math.pi/2]  # initial position: x, z, angle
trace = []  # keep the history
trace.append(pos)
####

#Initialize the Random Movement Generator
is_still_moving = True
rmg = random_movement_generator.random_move_generator(robot)
#Do not change anything about this!

#Random Movmement Loop
while robot.step(timestep) != -1 and is_still_moving:
    is_still_moving = rmg.move() #Update Speed Commands

    #### Write your Code here

    # treat each timestep as an individual segment
    dleft = encoder_left.getValue() - old_left
    dright = encoder_right.getValue() - old_right
    old_left = encoder_left.getValue()
    old_right = encoder_right.getValue()
    #print(dleft, dright)

    if abs(abs(dleft) - abs(dright)) < epsilon:
        # Moving in straight line
        if abs(dleft - dright) < epsilon:
            hypotenuse = dleft * diam / 2  # distance traveled by the robot
            pos[0] -= hypotenuse * math.cos(pos[2])  # minus due to flipped x-axis
            pos[1] += hypotenuse * math.sin(pos[2])

        # Rotation on the spot
        else:
            # clw
            if dleft > dright:
                pos[2] -= dleft * diam / l  # l * dtheta /2 == enc * diam /2
            # aclw
            else:
                pos[2] += dright * diam / l

    # Circular arc
    else:
        r = ((dright + dleft) * l) / ((dright-dleft) * 2)  # eq 6
        dtheta = (dright - dleft) * diam / (2 * l)  # eq 7
        dx_ego = r * math.sin(dtheta)  # eq 9
        dy_ego = r * (1 - math.cos(dtheta))  # eq 10

        pos[0] -= dx_ego * math.cos(pos[2]) - dy_ego * math.sin(pos[2])  # eq 12, 13, 11
        pos[1] += dx_ego * math.sin(pos[2]) + dy_ego * math.cos(pos[2])
        pos[2] += dtheta

    # append to history
    trace.append(pos)
    
p=[]

for point in trace:
    #print(point)
    p.append((point[0],point[1]))
    ####
print(p)
### Plot your position here!
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

verts = [
   (100., 200.),  # left, bottom
   (100., 210.),  # left, top
   (110., 210.),  # right, top
   (110., 200.),  # right, bottom
   (100., 200.),  # ignored
]

path = Path(p[:10])

# fig, ax = plt.subplots()
# patch = patches.PathPatch(path, facecolor='white', lw=2)
# ax.add_patch(patch)
#ax.set_xlim(270,10)
#ax.set_ylim(10, 400)
#plt.plot(trace[:,0], trace[:,1], '->')
plt.show()

