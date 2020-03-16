from math import tan

class Speed():
    def __init__(self):
        self.fTarget = 0
    
    def setTarget(self, target):
        self.fTarget = target

    def calcTarget(self, blobs):
        '''
        returns a new speed based off of blob position
        '''
        calcSpeed = 0
        return calcSpeed

    @classmethod
    def joystickToThrottle(cls, joystickVal, m = 1.5, b = 0):
        '''
        throttle voltage = joystickVal * m + b 
        '''
        return joystickVal * m + b

class Steering():
    def __init__(self):
        pass

    def calcAngle(self, posBike, posTarget, distanceTarget):
        '''
        returns angle in radians 
        need to add bounds 
        '''
        return tan(posBike - posTarget) / distanceTarget

    @classmethod
    def joystickToSteeringAngle(cls, joystickVal, m = 45, b = 0):
        '''
        steering angle = joystickVal * m + b 
        '''
        return joystickVal * m + b

