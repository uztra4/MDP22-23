class Obstacle:
    pos = [0, 0]
    face = 0
    size = 1

    def __init__(self, *args):
        if len(args) == 2:
            self.pos = [args[0], args[1]]
        elif len(args) == 3:
            self.pos = [args[0], args[1]]
            self.face = args[2]
        else:
            print("Incorrect argument count")

    def __eq__(self, other):
        if self.pos == other.pos and self.face == other.face:
            return True
        return False

    def getObsFacing(self):
        return self.face