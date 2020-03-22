from math import tan
try:
    from .pid import Pid
except Exception:
    from pid import Pid

class Speed(Pid):
    def __init__(self, objectName = "speed-control"):
        self.objectName = objectName
        super().__init__(objectName)

    def setup(self, windup = 100):
        self.loadJson()
        self.setWindupLimit(windup)
        # Ts = circ (m)
        #  / maxSpeed (m/s)
        # Ts = 0.09 gives 4 samples/sec
        self.enableTime(sampleTime = 0.09)

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

class Steering(Pid):
    def __init__(self, objectName = "steering-control"):
        self.objectName = objectName
        super().__init__(objectName)

    def setup(self, windup = 100):
        self.loadJson()
        self.setWindupLimit(windup)
        self.enableTime(sampleTime = 1/(45*2))

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

if __name__ == "__main__":
    # testing how this would work with PID 
    steering = Steering()
    steering.setup()
    # loop and load data 
    # angle = steering.calcAngle(posBike, posTarget)
    # output = steering.run(input = angle)
