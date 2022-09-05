from Agent import Agent
from Obstacle import Obstacle


class Map:
    mapGrid = []
    car = Agent(0, 0, 0)
    obsList = []
    obsDirDict = {}
    borders = set()
    size = 0

    def __init__(self, size, x, y, direction):
        self.size = size + 2
        for i in range(self.size):
            self.borders.add((0, i))
            self.borders.add((self.size - 1, i))
            self.borders.add((i, 0))
            self.borders.add((i, self.size - 1))
        # print(self.borders)
        self.initMap(self.size, x, y, direction)

    def initMap(self, size, x, y, direction):
        self.car.pos = [x, y]
        self.car.direction = direction
        for i in range(self.size):
            templist = []
            for j in range(self.size):
                if ((i, j) in self.borders):
                    templist.append("W")
                else:
                    templist.append("-")
            self.mapGrid.append(templist)
        self.markCar()

    def printMap(self):
        for i in range(self.size):
            for j in range(self.size):
                print(self.mapGrid[j][i], end=" ")
            print()

    def resetCar(self):
        loc = self.car.getPos()
        sizeCar = self.car.getSize()
        # wingspan of the car
        span = int((sizeCar - 1) / 2)

        # Remove car on map
        for i in range(-span, span + 1):
            for j in range(-span, span + 1):
                self.mapGrid[loc[0] + i][loc[1] + j] = '-'

    def markCar(self):
        loc = self.car.getPos()
        front = self.car.getCarFront()
        sizeCar = self.car.getSize()
        # wingspan of the car
        span = int((sizeCar - 1) / 2)

        # Avoid checking more if one fails
        fail = 0
        # Mark car on map
        for i in range(-span, span + 1):
            for j in range(-span, span + 1):
                # Check whether car is in border or in Obstacle
                if (loc[0] + i >= 0 and loc[0] + i <= self.size and loc[1] + j >= 0
                        and loc[1] + j <= self.size and (loc[0] + i, loc[1] + j) not in self.borders
                        and Obstacle(loc[0] + i, loc[1] + j) not in self.obsList):

                    if i == 0 and j == 0:
                        self.mapGrid[loc[0]][loc[1]] = '$'
                    elif (loc[0] + i) == front[0] and (loc[1] + j) == front[1]:
                        self.mapGrid[loc[0] + i][loc[1] + j] = '@'
                    else:
                        self.mapGrid[loc[0] + i][loc[1] + j] = '+'
                # Remove car if in border
                else:
                    fail = 1
                    break

            if fail == 1:
                self.checkCarBorder()
                print("Car in border obstacle")
                break

    def checkCarBorder(self):
        loc = self.car.getPos()
        sizeCar = self.car.getSize()
        # wingspan of the car
        span = int((sizeCar - 1) / 2)

        # Remove car on map
        for i in range(-span, span + 1):
            for j in range(-span, span + 1):
                # Check whether car is in border
                if (loc[0] + i >= 0 and loc[0] + i <= self.size and loc[1] + j >= 0
                        and loc[1] + j <= self.size):
                    if (loc[0] + i, loc[1] + j) in self.borders:
                        self.mapGrid[loc[0] + i][loc[1] + j] = 'W'
                    elif Obstacle(loc[0] + i, loc[1] + j) in self.obsList:
                        self.mapGrid[loc[0] + i][loc[1] + j] = self.obsDirDict[(loc[0] + i, loc[1] + j)]
                    else:
                        self.mapGrid[loc[0] + i][loc[1] + j] = '-'

    def moveCar(self, steps):
        self.resetCar()
        self.car.moveForward(steps)
        self.markCar()

    def turnCar(self, rotate):
        self.resetCar()
        self.car.turn(rotate)
        self.markCar()

    def addObstacle(self, x, y, facing):
        if (x, y) not in self.borders:
            self.obsList.append(Obstacle(x, y))
            self.obsDirDict[(x, y)] = facing
            # print(self.obsDirDict)
            self.mapGrid[x][y] = facing
        else:
            print("Unable to add obstacle due to borders")