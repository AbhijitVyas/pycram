import sys
import os
from enum import Enum
sys.path.append(os.getcwd() + "/../../src/")


class ActionType(object):
    def __init__(self):
        super(self)
    
    
    def action_name(self, name):
        self.name = name
    
    def action
    
    PickUp = 1
    MoveForward = 2
    MoveBackward = 3
    MoveLeft = 4
    MoveRight = 5
    PutDown = 6
    MoveTCP = 7
    

class BootstrapCommands(object):
    def __init__(self):
        super(self)


    def add_action(self, actionType: ActionType, action_parameters):
        print("adding new action type")
        # find which type of action is requested
        # based upon the action type use action parameters appropreately         
        if(actionType.)



