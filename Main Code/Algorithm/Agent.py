class Agent:
    pos = [0, 0]
    direction = 0
    size = 3

    def __init__(self, x, y, direction):
        self.pos = [x, y]
        self.direction = direction

    def getSize(self):
        return self.size

    def getPos(self):
        return self.pos

    def getDirection(self):
        return self.direction

    def getCarFront(self):
        # 0=N, 2=E, 4=S, 6=W
        front = self.pos.copy()
        if self.direction == 0:
            front[1] -= 1
        elif self.direction == 1:
            front[0] += 1
            front[1] -= 1
        elif self.direction == 2:
            front[0] += 1
        elif self.direction == 3:
            front[0] += 1
            front[1] += 1
        elif self.direction == 4:
            front[1] += 1
        elif self.direction == 5:
            front[0] -= 1
            front[1] += 1
        elif self.direction == 6:
            front[0] -= 1
        elif self.direction == 7:
            front[0] -= 1
            front[1] -= 1

        return front

    def turn(self, direction):
        self.direction = (self.direction + direction) % 8
        # Example case, turn left from North
        if self.direction < 0:
            self.direction = 8 + self.direction

    def moveForward(self, steps):
        # 0=N, 2=E, 4=S, 6=W
        if self.direction == 0:
            self.pos[1] -= steps
        elif self.direction == 1:
            self.pos[0] += steps
            self.pos[1] -= steps
        elif self.direction == 2:
            self.pos[0] += steps
        elif self.direction == 3:
            self.pos[0] += steps
            self.pos[1] += steps
        elif self.direction == 4:
            self.pos[1] += steps
        elif self.direction == 5:
            self.pos[0] -= steps
            self.pos[1] += steps
        elif self.direction == 6:
            self.pos[0] -= steps
        elif self.direction == 7:
            self.pos[0] -= steps
            self.pos[1] -= steps

