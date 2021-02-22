from random import randint

class random_move_generator:

    arc_commands = [[0.2,0.8],[0.8,0.2],[0.7,0.3],[0.3,0.7],[0.4,0.6],[0.6,0.4]]
    turn_commands = [[-0.2,0.2],[0.2,-0.2],[-0.5,0.5],[0.5,-0.5],[1.0,-1.0],[-1.0,1.0]]
    forward_commands = [[0.4,0.4],[0.5,0.5],[0.6,0.6],[0.7,0.7],[0.8,0.8],[1.0,1.0]]
    command_time = 4 #seconds?
    num_commands = 10

    def __init__(self,robot):
        self.elapsed_time = 0
        self.last_time = robot.getTime()
        self.executed_commands = 0
        self.robot = robot
        self.motorL = robot.getDevice("left wheel motor")
        self.motorR = robot.getDevice("right wheel motor")

        self.motorL.setPosition(float('+inf'))
        self.motorR.setPosition(float('+inf'))
        self.motorL.setVelocity( 0.0 )
        self.motorR.setVelocity( 0.0 )
        
        self.command_code = randint(0,2)
        self.current_command = get_random_command(self)
        
        #print("Initialized robot at: " +str(self.last_time))


    def move(self):
        current_time = self.robot.getTime()
        self.elapsed_time = self.elapsed_time + (current_time-self.last_time)
        self.last_time = current_time
        
        if self.elapsed_time > self.command_time:
            #print("Elapsed Time: " +str(self.elapsed_time) +" Let's switch commands!")
            self.executed_commands = self.executed_commands + 1
            self.elapsed_time = 0
            self.current_command = get_random_command(self)
        current_velocities = self.current_command
        #print("I am moving with left speed: " +str(self.current_command[0]) + " and right speed: " + str(self.current_command[1]))
        
        if self.executed_commands < self.num_commands:
            self.motorL.setVelocity(self.current_command[0])
            self.motorR.setVelocity(self.current_command[1])
        else:
            self.motorL.setVelocity(0.0)
            self.motorR.setVelocity(0.0)
        return self.executed_commands < self.num_commands

def get_random_command(self):
    new_command = [0.0,0.0]
    if self.command_code == 0:
        new_command = self.arc_commands[randint(0,len(self.arc_commands)-1)]
        #print("Chose an Arc Command " +str(new_command))
        possible_command_codes = [1,2]
        self.command_code = possible_command_codes[randint(0,len(possible_command_codes)-1)]
        #print("Next Command Code: " +str(self.command_code))
    elif self.command_code == 1:
        new_command = self.turn_commands[randint(0,len(self.turn_commands)-1)]
        #print("Chose a Turn Command " +str(new_command))
        possible_command_codes = [0,2]
        self.command_code = possible_command_codes[randint(0,len(possible_command_codes)-1)]
        #print("Next Command Code: " +str(self.command_code))
    elif self.command_code == 2:
        new_command = self.forward_commands[randint(0,len(self.forward_commands)-1)]
        #print("Chose an Forward Command " +str(new_command))
        possible_command_codes = [0,1]
        self.command_code = possible_command_codes[randint(0,len(possible_command_codes)-1)]
        #print("Next Command Code: " +str(self.command_code))
    return new_command
           