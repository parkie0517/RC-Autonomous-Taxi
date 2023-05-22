class Car:
    # Car system related variables
    carNumber = 0
    state = 0
    """
    state 0: None
    state 1: Stop
    state 2: Stop -> Go
    state 3: Go -> Stop
    state 4: Go
    """
    # Arduino related variables
    dcSpeed = 0
    # 2D map related variables
    vmapY = 0
    vmapX = 0
    vcarY = 0
    vcarX = 0
    direction = 0
    # Localization realted variables
    rmapY = 0
    rmapX = 0
    rcarY = 0
    rcarX = 0
    road = None
    distance = 0
    DT = 0 # Distance travelled

    def moveForward(void):
        # Send signal to RC car
        # code here
        return
    
    def moveBackward(int):
        # Code here
        return
    
    def stop(void):
        # Code here
        return
    
    def turnRight(void):
        # Code here
        return
    
    def turnLeft(void):
        # Code here
        return
    
    def straight(void):
        # give rc car sign to go straight
        return None

    def parking(void):
        # Code here
        return
    
    def moving(void):
        # Code here
        return
    
    def setvmapYX(self, Y, X):
        Car.vmapY = Y
        Car.vmapX = X

    def setvcarYX(self, Y, X):
        Car.vcarY = Y
        Car.vcarX = X

    def setvcarY(self, Y):
        Car.vcarY = Y

    def setvcarX(self, X):
        Car.vcarX= X

    def setvmapY(self, Y):
        Car.vmapY = Y

    def setvmapX(self, X):
        Car.vmapX= X
#######################
    def setrmapYX(self, Y, X):
        Car.rmapY = Y
        Car.rmapX = X

    def setrcarYX(self, Y, X):
        Car.rcarY = Y
        Car.rcarX = X

    def setrcarY(self, Y):
        Car.rcarY = Y

    def setrcarX(self, X):
        Car.rcarX= X

    def setrmapY(self, Y):
        Car.rmapY = Y

    def setrmapX(self, X):
        Car.rmapX= X
########################
    def setDirection(self, D):
        Car.direction = D

    def setSpeed(self, speed):
        Car.dcSpeed = speed

    def setState(self, state):
        Car.state = state

    def setDistance(self, distance):
        Car.distance = distance

    def setDT(self, DT):
        Car.DT = DT

    def setRoad(self, road):
        Car.road = road
##################################
    def getvmapYX(void):
        return Car.vmapY, Car.vmapX

    def getvcarYX(void):
        return Car.vcarY, Car.vcarX
    
    def getvcarY(void):
        return Car.vcarY
    
    def getvcarX(void):
        return Car.vcarX
    
    def getvmapY(void):
        return Car.vmapY
    
    def getvmapX(void):
        return Car.vmapX
################################
    def getrmapYX(void):
        return Car.rmapY, Car.rmapX

    def getrcarYX(void):
        return Car.rcarY, Car.rcarX
    
    def getrcarY(void):
        return Car.rcarY
    
    def getrcarX(void):
        return Car.rcarX
    
    def getrmapY(void):
        return Car.rmapY
    
    def getrmapX(void):
        return Car.rmapX
################################   
    def getSpeed(void):
        return Car.dcSpeed

    def getState(void):
        return Car.state

    def getDistance(void):
        return Car.distance

    def getDT(void):
        return Car.DT   

    def getRoad(void):
        return Car.road
    
    def getDirection(void):
        return Car.direction

class Taxi(Car):
    mode = 0
    starting = ""
    destination = ""
    path = None # Stores directional instructions ex) right, left, straight
    block = True
    # Variables used for random road selection
    nextRoad = ''
    nextRoadInt = None
    nextRoadWay = ''


    def booking(void): # used to get booking information
        # Code here
        return
    
    def getStarting(void):
        return Taxi.starting
    
    def getDestination(void):
        return Taxi.destination

    def getMode(void):# delete if you don't use this
        return Taxi.mode

    def getNextRoad(void):
        return Taxi.nextRoad
    
    def getNextRoadInt(void):
        return Taxi.nextRoadInt
    
    def getNextRoadWay(void):
        return Taxi.nextRoadWay
    
    def setStarting(self, s):
        Taxi.starting = s
    
    def setDestination(slef, d):
        Taxi.destination = d
    
    def setMode(self, mode):
        Taxi.mode = mode

    def setNextRoad(self, nextRoad):
        Taxi.nextRoad = nextRoad

    def setNextRoadInt(self, nextRoadInt):
        Taxi.nextRoadInt = nextRoadInt

    def setNextRoadWay(self, nextRoadWay):
        Taxi.nextRoadWay = nextRoadWay
