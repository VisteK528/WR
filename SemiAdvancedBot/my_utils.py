

class States:
    FOLLOW_THE_LINE = 0
    TURN_TO_THE_POINT = 1
    PROBE = 2
    LOAD = 3
    RETURN_TO_THE_LINE = 4
    UNLOAD = 5
    TERMINAL = 6


class Colors:
    WHITE = 0
    BLACK = 1
    GREEN = 2
    YELLOW = 3
    RED = 4
    BLUE = 5
    UNKNOWN = 6


class TurnDirection:
    LEFT = 0
    RIGHT = 1


def my_map(x, in_min, in_max, out_min, out_max):
    scaled_value = ((x - in_min) * (out_max - out_min) / (in_max - in_min) +
                    out_min)
    return scaled_value