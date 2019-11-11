import redis
import time
import tkinter as tk
from enum import Enum

# View related constants.
CANVAS_HEIGHT = 950
CANVAS_WIDTH = 1100
HALF_WIDTH = 5
CANVAS_SHIFT = 16
SCALING_FACTOR = 10

# Data related constants.
X_LEN = 81
Y_LEN = 101

# Scene related constants.
SHELF_COUNT = 3100
ROBOT_COUNT = 300
ROBOT_INFO_ITEMS = 4
SHELF_INFO_ITEMS = 2
REDIS_KEY = "ks"


def float_equals(a, b):
    return abs(a - b) < 0.01


# Transfer data coordinates to canvas coordinates.
def transfer_coordinates(x, y):
    return y * SCALING_FACTOR + CANVAS_SHIFT, x * SCALING_FACTOR + CANVAS_SHIFT


class Robot(object):
    canvas = None
    display_item = None
    x = 0
    y = 0
    theta = 0
    has_shelf = False

    body = [(4, -4), (-4, -4), (0, 4)]

    def __init__(self, canvas):
        self.canvas = canvas
        self.display_item = canvas.create_polygon(*(self.get_points()), fill='blue')

    def draw(self, x, y, theta, has_shelf):
        if (not float_equals(self.x, x)) or (not float_equals(self.y, y)) or (not float_equals(self.theta, theta)):
            self.canvas.coords(self.display_item, *(self.get_points()))
            self.x = x
            self.y = y
            self.theta = theta
        if self.has_shelf != has_shelf:
            color = "red" if has_shelf else "blue"
            self.canvas.itemconfigure(self.display_item, fill=color)
            self.has_shelf = has_shelf

    def get_points(self):
        offset = complex(transfer_coordinates(self.x, self.y))
        cangle = cmath.exp(self.theta * 1j)  # theta in radians
        new_xy = []
        for a, b in self.body:
            v = cangle * (complex(a, b)) + offset
            new_xy.append(v.real)
            new_xy.append(v.imag)
        return new_xy


class GridType(Enum):
    BLOCK = 0
    SHELF_STORAGE_POINT = 1
    SHELF_OPERATION_POINT = 2
    OTHER = 3


class Grid(object):
    canvas = None
    display_item = None
    has_shelf = False
    type = None

    def __init__(self, canvas):
        self.canvas = canvas

    def init(self, display_item, type_char, has_shelf):
        self.display_item = display_item
        self.has_shelf = has_shelf
        if type_char == 'B':
            self.type = GridType.BLOCK
        elif type_char == 'S':
            self.type = GridType.SHELF_STORAGE_POINT
        elif type_char == 'P':
            self.type = GridType.SHELF_OPERATION_POINT
        else:
            self.type = GridType.OTHER

    def update_shelf(self, has_shelf):
        if self.has_shelf != has_shelf:
            self.has_shelf = has_shelf
            self.update_color()

    def update_color(self):
        if self.has_shelf:
            self.canvas.itemconfigure(self.display_item, fill="red")
            return
        if self.type == GridType.BLOCK:
            self.canvas.itemconfigure(self.display_item, fill="black")
            return
        if self.type == GridType.SHELF_STORAGE_POINT:
            self.canvas.itemconfigure(self.display_item, fill="yellow")
            return
        if self.type == GridType.SHELF_OPERATION_POINT:
            self.canvas.itemconfigure(self.display_item, fill="green")
            return
        if self.type == GridType.OTHER:
            self.canvas.itemconfigure(self.display_item, fill="white")
            return


class ObjManager(object):
    canvas = None
    redis_client = None
    grids = None
    robots = None

    def __init__(self, canvas):
        self.canvas = canvas
        # Draw robots and shelfs according to a default initial configuration.
        self.grids = [[Grid(canvas) for y in range(Y_LEN)] for x in range(X_LEN)]
        self.robots = [Robot(canvas) for x in range(ROBOT_COUNT)]
        self.load_map()

    def load_map(self):
        map_file = open("../data/map_1.map", "r")

        list_of_lines = []
        for raw_line in map_file:
            line = raw_line.strip()
            if line[0] == '#':
                continue
            assert (len(line) == Y_LEN)
            list_of_lines.append(line)

        assert (len(list_of_lines) == X_LEN)

        for x in range(X_LEN):
            for y in range(Y_LEN):
                s_x, s_y = transfer_coordinates(x, y)
                item = self.canvas.create_rectangle(s_x - HALF_WIDTH, s_y + HALF_WIDTH,
                                                    s_x + HALF_WIDTH, s_y - HALF_WIDTH, width=1)
                self.grids[x][y].init(item, list_of_lines[x][y], False)

    def run(self):
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        while True:
            time.sleep(0.05)
            value = self.redis_client.get(REDIS_KEY)
            value_array = value.split(' ')

            # Update robots.
            for robot_index in range(ROBOT_COUNT):
                x = value_array[ROBOT_INFO_ITEMS * robot_index]
                y = value_array[ROBOT_INFO_ITEMS * robot_index + 1]
                theta = value_array[ROBOT_INFO_ITEMS * robot_index + 2]
                has_shelf = value_array[ROBOT_INFO_ITEMS * robot_index + 3]
                self.robots[robot_index].draw(x, y, theta, has_shelf)

            # Update shelves.
            shelf_value_base = ROBOT_COUNT * ROBOT_INFO_ITEMS
            grid_with_shelf = set()
            for shelf_index in range(SHELF_COUNT):
                x = value_array[SHELF_INFO_ITEMS * shelf_index + shelf_value_base]
                y = value_array[SHELF_INFO_ITEMS * shelf_index + 1 + shelf_value_base]
                grid_with_shelf.add((x, y))

            for x in range(X_LEN):
                for y in range(Y_LEN):
                    has_shelf = (x, y) in grid_with_shelf
                    self.grids[x][y].update_shelf(has_shelf)


def main():
    root = tk.Tk()

    canvas = tk.Canvas(root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg="white")
    canvas.pack()

    o = ObjManager(canvas)

    root.mainloop()


if __name__ == '__main__':
    main()
