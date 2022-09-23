from entities.commands.command import Command


class ScanCommand(Command):
    def __init__(self, time, obj_index):
        super().__init__(time)
        self.obj_index = obj_index

    def __str__(self):
        return f"ScanCommand(time={self.time, self.obj_index})"

    __repr__ = __str__

    def process_one_tick(self, robot):
        if self.total_ticks == 0:
            return

        self.tick()

    def apply_on_pos(self, curr_pos):
        pass

    def convert_to_message(self):
        # Just return a string of s's
        return "STM:p\n"
        # return f"s{self.obj_index:04}"
