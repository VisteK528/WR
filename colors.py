from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_2, INPUT_1
from time import time

def map(x, in_min, in_max, out_min, out_max):
    scaled_value = ((x - in_min) * (out_max - out_min) / (in_max - in_min) +
                    out_min)
    return scaled_value

class PID:
    def __init__(self, kp: float, ki: float, kd: float,
                 integral_reset_count: int):

        # Coefficients
        self._kp = kp
        self._ki = ki
        self._kd = kd

        self._integral = 0
        self._derivative = 0
        self._last_error = 0

        self._steering = 0

        # Other variables
        self._integral_counter = 0
        self._integral_reset_count = integral_reset_count

    def calculate_pid_steering(self, error):
        # Calculate proportional, integral and derivative parts
        self._integral += error
        self._derivative = error - self._last_error
        self._last_error = error

        # Sum them together beforehand multiplying them with corresponding
        # coefficients
        self._steering = ((self._kp * error) + (self._ki * self._integral) +
                          (self._kd * self._derivative))

        # Increment integral counter
        self._integral_counter += 1

        # If integral counter is greater or equal to set integral rest count
        # value then reset the integral part
        if self._integral_counter >= self._integral_reset_count:
            self._integral = 0
            self._integral_counter = 0

        return self._steering

    def set_pid_coefficients(self, kp: float, ki: float, kd: float):
        self._kp = kp
        self._ki = ki
        self._kd = kd

    def get_steering_command(self):
        return self._steering

def _convert_rgb_to_grayscale(r: int, g: int, b: int):
    # 0.30*R + 0.59*G + 0.11*B
    red_coefficient = 0.3
    green_coefficient = 0.59
    blue_coefficient = 0.11

    grayscale = red_coefficient * r + green_coefficient * g + blue_coefficient * b

    # scale grayscale value from range 0 to 255 to range from 0 to 100
    #scaled_grayscale = map(grayscale, 0, 255, 0, 100)
    return grayscale

def _generate_error(left, right, left_tol, right_tol):
    error = (left*left_tol - right*right_tol)

    return error

def rgb(red, green, blue, r_max, g_max, b_max):
    return (min(int((red * 255) / r_max), 255), min(int((green * 255) / g_max),
                                                           255), min(int((blue * 255) / b_max), 255))

if __name__ == "__main__":
    left = ColorSensor(INPUT_2)
    right = ColorSensor(INPUT_1)
    left.mode = ColorSensor.MODE_RGB_RAW
    right.mode = ColorSensor.MODE_RGB_RAW
    """left.calibrate_white()
    print("Calibrated!")"""
    pid = PID(3.8, 0, 0.2, 5)

    while True:
        start = time()
        lr, lg, lb = rgb(left.value(0), left.value(1), left.value(2), left.red_max, left.green_max, left.blue_max)
        rr, rg, rb = rgb(right.value(0), right.value(1), right.value(2), right.red_max, right.green_max, right.blue_max)
        left_gray = 0.30*lr + 0.59*lg + 0.11*lb
        right_gray = 0.30*rr + 0.59*rg + 0.11*rb
        left_gray = map(left_gray, 0, 255, 0, 100)
        right_gray = map(right_gray, 0, 255, 0, 100)
        end_sensors = time()
        error = _generate_error(left_gray, right_gray, 1, 0.9)
        end_error = time()
        steering = pid.calculate_pid_steering(error)
        end = time()
        #print("Colors: ", left_gray, "\t", right_gray, "\t", "Steering: ", steering, "\tExecuted: ", end-start)
        print("Sensors: ", end_sensors-start, "\t", "Error: ", end_error - end_sensors, "\tSteering: ", end - end_error)
