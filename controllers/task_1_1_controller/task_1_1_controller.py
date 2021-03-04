"""task_1_1_controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
from copy import deepcopy


class robot_path:

    def __init__(self, position_initial):
        self.diam = 40
        self.l = 53
        self. epsilon = 0.00001
        
        self.old_left = 0
        self.old_right = 0
        
        self.trace = []  # keep the history
        self.pos = position_initial  # initial position: x, z, angle
        self.trace.append(deepcopy(self.pos))
        
    def step(self, encoder_left, encoder_right):
        # treat each timestep as an individual segment
        self.dleft = encoder_left.getValue() - self.old_left
        self.dright = encoder_right.getValue() - self.old_right
        self.old_left = encoder_left.getValue()
        self.old_right = encoder_right.getValue()

        if abs(abs(self.dleft) - abs(self.dright)) < self.epsilon:
            # Moving in straight line
            if abs(self.dleft - self.dright) < self.epsilon:
                hypotenuse = self.dleft * self.diam / 2  # distance traveled by the robot
                self.pos[0] -= hypotenuse * math.cos(self.pos[2])  # minus due to flipped x-axis
                self.pos[1] += hypotenuse * math.sin(self.pos[2])

            # Rotation on the spot
            else:
                # clw
                if self.dleft > self.dright:
                    self.pos[2] -= self.dleft * self.diam / self.l  # l * dtheta /2 == enc * diam /2
                # aclw
                else:
                    self.pos[2] += self.dright * self.diam / self.l
    
        # Circular arc
        else:
            r = ((self.dright + self.dleft) * self.l) / ((self.dright-self.dleft) * 2)  # eq 6

            dtheta = (self.dright - self.dleft) * self.diam / (2 * self.l)  # eq 7
            dx_ego = r * math.sin(dtheta)  # eq 9
            dy_ego = r * (1 - math.cos(dtheta))  # eq 10
    
            self.pos[0] -= dx_ego * math.cos(self.pos[2]) - dy_ego * math.sin(self.pos[2])  # eq 12, 13, 11
            self.pos[1] += dx_ego * math.sin(self.pos[2]) + dy_ego * math.cos(self.pos[2])
            self.pos[2] += dtheta
    
        # append to history
        self.trace.append(deepcopy(self.pos))
        return self.pos
        
        
    def plot(self):
        ### Plot your position here!
        import matplotlib.pyplot as plt
        
        plt.axes().set_aspect('equal')
        plt.xlim(470,-200)
        plt.ylim(-130,540)
        plt.plot(self.trace[:,0], self.trace[:,1], '-')
        plt.show()
        
        
if __name__ == "__main__":

    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    #A handle for each wheel encoder
    encoder_left = robot.getDevice("left wheel sensor")
    encoder_right = robot.getDevice("right wheel sensor")
    
    encoder_left.enable(timestep)
    encoder_right.enable(timestep)
            
    pos = [190, 250, math.pi/2]  # initial position: x, z, angle        
    path=robot_path(pos)
    
    #Initialize the Random Movement Generator
    is_still_moving = True
    rmg = random_movement_generator.random_move_generator(robot)
    #Do not change anything about this!
    #Random Movmement Loop
    while robot.step(timestep) != -1 and is_still_moving:
        is_still_moving = rmg.move() #Update Speed Commands
    
        #### Write your Code here
        path.step(encoder_left, encoder_right) # save the movement of the robot for this step
            
    path.plot() #plot the trace moved by the robot
    
    


