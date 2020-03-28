from math import tan
from numpy import amin, where, stack
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

    def checkBlobPos(self):
        '''
        checks blob positions against current pos and target pos
        returns np.array of indices that are in the way
        '''
        # assuming x,y are center pos
        # only check x
        blobsHalfWidth = self.blobs.widths//2
        xBoundLower = self.blobs.xPos - blobsHalfWidth
        xBoundUpper = self.blobs.xPos + blobsHalfWidth
        xBounds = stack((xBoundLower, xBoundUpper), axis=-1)
        # TODO need bikepos and bike width in pixels 
        bikePosLower = self.bikePos - self.bikeHalfWidth
        bikePosUpper = self.bikePos + self.bikeHalfWidth
        inBoundIndices = where( (xBounds >= bikePosLower) & (xBounds <= bikePosUpper) )
        return inBoundIndices[0]

    def checkEmergencyStop(self, crashTimes):
        emergency = 0 
        updatePeriod = self.circ / self.speedMeas
        if (updatePeriod > crashTimes).any():
            emergency = 1
            print("emergency stop engaged")
        return emergency

    def calcTarget(self):
        '''
        returns a new target speed based off of blob position
        '''
        speedMod = 0 
        if self.blobs is not None:
        # check if blob is in the way 
            blobIndices = self.checkBlobPos()
            # check crash time 
            # TODO unpack bike width 
            # TODO figure out if xPos should be pixels
            # TODO blobs.depths is a numpy array
            # TODO this needs to be in m/s
            crashTimes =  self.blobs.depths[blobIndices] / self.speedMeas
            if len(crashTimes) == 0:
                return self.target
            # check emergency stop 
            if self.checkEmergencyStop(crashTimes):
                return -1
            # check safe stop based off closest object
            crashMin = amin(crashTimes)
            speedMod = self.speedModScalar*self.circ/crashMin
            if speedMod > self.target:
                speedMod = self.target
        calcSpeed = self.target - speedMod
        return calcSpeed

    def speedToThrottle(self, speedOut):
        '''
        assuming the pid output is speed
        maps speed to voltage 
        '''
        speedDiff = speedOut / self.speedMax
        # speedDiff = (speedOut - self.speedMeas) / self.speedMax
        maxVolt = self.throttleMax * self.throttleScalar
        self.voltOut +=  maxVolt * speedDiff
        # check volt bounds 
        if self.voltOut > maxVolt:
            self.setWindupLimitUpper = self.integral
            self.voltOut = maxVolt
        if self.voltOut < 0:
            self.setWindupLimitUpper = self.integral
            self.voltOut = 0
        return self.voltOut

    def feedInput(self, speedMeas, blobs, bikePos):
        '''
        speedMeas [m/s] = tachometer speed
        '''
        self.speedMeas = speedMeas
        self.blobs = blobs
        self.bikePos = bikePos
        # if blobs are blocking change target speed
        target = self.calcTarget()
        if target < 0:
            # emergency brake 
            return 0
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
        return tan(posBike - posTarget) / distanceTarget

    def feedInput(self, posBike, posTarget, distanceTarget = 1):
        angle = self.calcAngle(posBike, posTarget, distanceTarget)
        output = self.run(angle)
        return output

    @classmethod
    def joystickToSteeringAngle(cls, joystickVal, m = 45, b = 0):
        '''
        steering angle = joystickVal * m + b 
        '''
        return joystickVal * m + b

class Blobs():
    def __init__(self):
        self.xPos = 0
        self.widths = 0
        self.depths = 0

if __name__ == "__main__":
    # testing how this would work with PID 
    steering = Steering()
    steering.setup()
    # loop and load data 
    # angle = steering.calcAngle(posBike, posTarget)
    # output = steering.run(input = angle)

    # timing test 
    import time 
    import numpy as np 
    speed = Speed(2.055)
    speed.setup()
    speed.voltOut = 1.4
    bikePos = 350
    bikeSpeed = 0.9
    blobs = Blobs()
    xPos = np.array([250, 600, 100, 487])
    widths = np.array([50, 25, 80, 200])
    depths = np.array([3, 1, 10, 8])
    blobs.xPos = xPos
    blobs.widths = widths
    blobs.depths = depths
    time.sleep(0.09)

    print("testing feed input single ")
    startTime = time.perf_counter()
    output = speed.feedInput(bikeSpeed, blobs, bikePos)
    endTime = time.perf_counter()
    diffTime = endTime - startTime
    print(f"diff time: {diffTime}, output : {output}")

    print("\n\ntesting feed input random 10 times")
    totalTime = 0
    blobsRand = blobs
    for i in range(10):
        # rand input
        time.sleep(0.2)
        sizeRand = np.random.randint(0,20)
        xPosRand = np.random.randint(0, 800, size = sizeRand)
        widthsRand = np.random.randint(0, 200, size = sizeRand)
        depthsRand = np.random.uniform(3, 15, size = sizeRand)
        blobsRand.xPos = xPosRand
        blobsRand.widths = widthsRand
        blobsRand.depths = depthsRand

        bikeSpeedRand = np.random.uniform(0.2,1)
        bikePosRand = np.random.randint(200,600)
        print(f"number of blobs: {sizeRand}, bike speed: {bikeSpeedRand}")
        startTime = time.perf_counter()
        output = speed.feedInput(bikeSpeedRand, blobsRand, bikePosRand)
        endTime = time.perf_counter()
        diffTime = endTime - startTime
        totalTime += diffTime
        print(f"diff time: {diffTime}, output : {output}")
    print(f"average runtime: {totalTime/10}")
        
    

