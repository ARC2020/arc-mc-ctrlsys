from math import arctan, degrees
from numpy import amin, where, stack, ndarray
from json import load
try:
    from .pid import Pid
except Exception:
    from pid import Pid


class InterfaceError(Exception):
    pass

class Speed(Pid):
    def __init__(self, circ, objectName = "speed-control"):
        # TODO add changes to main 
        self.circ = circ
        self.objectName = objectName
        self.speedMeas = 0
        self.voltOut = 0
        super().__init__(objectName)

    def setup(self, windupLimUpper = 100, windupLimLower = -100):
        self.loadJson()
        self.loadSpeedParams()
        self.setWindupLimitUpper(windupLimUpper)
        self.setWindupLimitLower(windupLimLower)
        # Ts = circ (m) / speedMax (m/s)
        # Ts = 0.09 gives 4 samples/sec
        self.enableTime(sampleTime = 0.09)

    def loadSpeedParams(self):
        with open(self.filename) as f:
            data = load(f)
            constants = data[self.objectName]
            self.speedMax       = constants["max speed (m/s)"]
            self.speedModScalar = constants["speed mod scalar"]
            self.throttleMax    = constants["max throttle (V)"]
            self.throttleScalar = constants["throttle scalar"]
            self.bikeHalfWidth      = constants["bike half width (px)"]

    

    def calcTarget(self, crashTimes):
        '''
        returns a new target speed based off of blob position
        '''
        speedMod = 0        
        # check crash time 
        # check safe stop based off closest object
        if type(crashTimes) is ndarray:
            if len(crashTimes) != 0:
                crashMin = amin(crashTimes)
                speedMod = self.speedModScalar*self.circ/crashMin
                if speedMod > self.target:
                    speedMod = self.target
        # reset target speed to default if no obstables 
        if speedMod <= 0:
            self.target = self.defaultTarget
        calcSpeed = self.target - speedMod
        return calcSpeed

    def speedToThrottle(self, speedOut):
        '''
        assuming the pid output is speed
        maps speed to voltage 
        '''
        speedRatio = speedOut / self.speedMax
        maxVolt = self.throttleMax * self.throttleScalar
        self.voltOut +=  maxVolt * speedRatio
        # check volt bounds 
        if self.voltOut > maxVolt:
            self.setWindupLimitUpper = self.integral
            self.voltOut = maxVolt
        if self.voltOut < 0:
            self.setWindupLimitUpper = self.integral
            self.voltOut = 0
        return self.voltOut

    def feedInput(self, speedMeas, crashTimes):
        '''
        speedMeas [m/s] = tachometer speed
        crashTimes = np.array of crashTimes for each blob
        '''
        # if blobs are blocking change target speed
        target = self.calcTarget(crashTimes)
        if target != self.target:
            self.setTarget(target)
        output = self.run(speedMeas)
        print(f"\tmeas speed: {speedMeas}, target: {target}, pid output:{output}")
        return self.speedToThrottle(output)

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

    def setup(self, windupLimUpper = 100, windupLimLower = -100):
        self.loadJson()
        self.setWindupLimitUpper(windupLimUpper)
        self.setWindupLimitLower(windupLimLower)
        self.enableTime(sampleTime = 1/(45*2))

    def calcAngle(self, posBike, posTarget, distanceTarget):
        '''
        returns angle in radians 
        need to add bounds 
        '''
        return arctan(posBike - posTarget) / distanceTarget

    def feedInput(self, posBike, posTarget, distanceTarget = 1):
        angle = self.calcAngle(posBike, posTarget, distanceTarget)
        output = self.run(angle)
        return degrees(output)

    @classmethod
    def joystickToSteeringAngle(cls, joystickVal, m = 45, b = 0):
        '''
        steering angle = joystickVal * m + b 
        '''
        return joystickVal * m + b

class Blobs():
    def __init__(self, circ, bikeHalfWidth):
        self.circ = circ
        self.bikeHalfWidth = bikeHalfWidth
        self.xPos = 0
        self.widths = 0
        self.depths = 0
        self.bikePos = 0 
        self.speedMeas = 0

    def update(self, xPos, widths, depths, bikePos):
        self.xPos = xPos
        self.widths = widths
        self.depths = depths
        self.bikePos = bikePos

    def checkBlobPos(self):
        '''
        checks blob positions against current pos and target pos
        returns np.array of indices that are in the way
        '''
        # assuming x,y are center pos
        # only check x
        blobsHalfWidth = self.widths//2
        xBoundLower = self.xPos - blobsHalfWidth
        xBoundUpper = self.xPos + blobsHalfWidth
        xBounds = stack((xBoundLower, xBoundUpper), axis=-1)
        bikePosLower = self.bikePos - self.bikeHalfWidth
        bikePosUpper = self.bikePos + self.bikeHalfWidth
        inBoundIndices = where( (xBounds >= bikePosLower) & (xBounds <= bikePosUpper) )
        return inBoundIndices[0]

    def checkCrash(self, speedMeas):
        crashTimes = 0
        self.speedMeas = speedMeas
        if len(self.xPos) > 0:
            blobIndices = self.checkBlobPos()
            crashTimes =  self.depths[blobIndices] / speedMeas
        return crashTimes

    def checkEmergencyStop(self, crashTimes):
        emergency = 0 
        updatePeriod = self.circ / self.speedMeas
        if type(crashTimes) is ndarray:
            if (updatePeriod > crashTimes).any():
                emergency = 1
                print("emergency stop engaged")
        return emergency

if __name__ == "__main__":
    # example of how to run pid and blobs
    import numpy as np 
    import time 
    # inits
    # set bike half width in pixels
    blobs = Blobs(circ = 2.055, bikeHalfWidth = 100)
    speed = Speed(circ = 2.055)
    speed.setup()
    steering = Steering()
    steering.setup()
    # get new data 
    # TODO make sure this is the new data types
    sizeRand = np.random.randint(0,20)
    blobXpos = np.random.randint(0, 800, size = sizeRand)
    blobWidths = np.random.randint(0, 200, size = sizeRand)
    blobDepths = np.random.uniform(3, 15, size = sizeRand)
    bikeSpeed = np.random.uniform(0.2,1)
    bikePosPx = np.random.randint(200,600)
    targetPosM = np.random.uniform(0,0.5)
    bikePosM = np.random.uniform(0,0.5)
    time.sleep(0.1)
    # update blobs
    startTime = time.perf_counter()
    blobs.update(xPos = blobXpos, widths = blobWidths, depths  = blobWidths, bikePos = bikePosPx)
    crashTimes = blobs.checkCrash(bikeSpeed)
    emergencyStop = blobs.checkEmergencyStop(crashTimes)
    if emergencyStop:
        # set throttle volt to 0/brake 
        # switch to manual drive 
        pass
    else:
        throttleVolt = speed.feedInput(bikeSpeed, crashTimes)
        steeringAngle = steering.feedInput(bikePosM, targetPosM, distanceTarget = 1)
        # apply throttle voltage and steering angle 
    endTime = time.perf_counter()
    print(f"inputs: bikeSpeed {bikeSpeed}, bike pos {bikePosM}, target pos {targetPosM}, crashTimes {crashTimes}")
    print(f"outputs: throttleVolt {throttleVolt}, steeringAngle {steeringAngle}, runtime {endTime - startTime}")
        
    

