import time

from ev3dev2.motor import OUTPUT_A, OUTPUT_C, MoveTank, SpeedPercent, \
    follow_for_ms, speed_to_speedvalue, SpeedInvalid, SpeedNativeUnits
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.motor import LineFollowErrorTooFast, LineFollowErrorLostLine

"""
General information:
Motors
    - LEFT MOTOR - OUTPUT_C
    - RIGHT MOTOR - OUTPUT_A
Sensors
    - Left Color Sensor - ?
    - Right Color Sensor - ?

"""


def map(x, in_min, in_max, out_min, out_max):
    scaled_value = ((x - in_min) * (out_max - out_min) / (in_max - in_min) +
                    out_min)
    return scaled_value


def rgb_to_hsv(r: int, g: int, b: int):
    r_prim = r / 255.
    g_prim = g / 255.
    b_prim = b / 255.

    c_max = max([r_prim, g_prim, b_prim])
    c_min = min([r_prim, g_prim, b_prim])
    delta = c_max - c_min

    # Hue calculation
    hue = 0
    if delta == 0:
        hue = 0
    elif c_max == r_prim:
        hue = 60 * (((g_prim - b_prim) / delta) % 6)
    elif c_max == g_prim:
        hue = 60 * ((b_prim - r_prim) / delta + 2)
    elif c_max == b_prim:
        hue = 60 * ((r_prim - g_prim) / delta + 4)

    # Saturation calculation
    if c_max == 0:
        sat = 0
    else:
        sat = delta / c_max

    # Value calculation
    value = c_max
    return hue, sat, value


# TODO sprawdzić jak w literaturze nazywa się wartość wyjściowa reguatora


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


class LineFollower2:
    def __init__(self, left_motor: str, right_motor: str, left_sensor: str,
                 right_sensor: str, pid_controller: PID,
                 left_motor_polarity_inversed=False,
                 right_motor_polarity_inversed=False,
                 display_logs=True):

        # Init tank unit
        self.tank = MoveTank(left_motor_port=left_motor,
                             right_motor_port=right_motor)

        # Set polarity of the motors
        if left_motor_polarity_inversed:
            self.tank.left_motor.polarity = "inversed"

        if right_motor_polarity_inversed:
            self.tank.right_motor.polarity = "inversed"

        # Init color sensors
        self._l_cs = ColorSensor(left_sensor)
        self._r_cs = ColorSensor(right_sensor)

        self._left_color = None
        self._right_color = None
        self._l_hsv = (None, None, None)
        self._r_hsv = (None, None, None)

        self._reflected_light_percentage = {"left": None, "right": None}

        # Assign PID controller object to variable
        self._pid = pid_controller

        self._display_logs = display_logs
        if self._display_logs:
            print("Configuration complete")

    def determine_color(self, h: int, s: int, v: int):
        # Hue ranges for red, green and blue
        # red from 320 to 50
        # green from 75 to 170
        # blue from 180 to 280

        sat_threshold = 50              # in percent
        if s < sat_threshold:
            return None
        else:
            if h >= 320 or h <= 50:
                return "red"
            elif 75 <= h <= 170:
                return "green"
            elif 180 <= h <= 280:
                return "blue"
            else:
                return "unknown"

    def updateSensor(self):
        self._l_hsv = self._l_cs.hsv
        self._r_hsv = self._r_cs.hsv

        self._left_color = self.determine_color(*self._l_hsv)
        self._right_color = self.determine_color(*self._r_hsv)

        return self._left_color, self._right_color

    def _generate_error(self, l_cs_tol: float,
                        r_cs_tol: float):
        if self._l_hsv[2] is None or self._r_hsv[2] is None:
            return None
        else:
            self._reflected_light_percentage = {
                "left": self._l_hsv[2],
                "right": self._r_hsv[2]
            }

            error = (self._reflected_light_percentage["left"] * l_cs_tol
                     - self._reflected_light_percentage["right"] * r_cs_tol)

            return error

    def follow_line_for_time(self, speed, follow_time, sleep_time=0.01):
        speed = speed_to_speedvalue(speed)
        speed_native_units = speed.to_native_units(self.tank.left_motor)

        start_time = end_time = time.time()

        if self._display_logs:
            print("Following started!")

        # Optional sensor calibration
        # self._right_color_sensor.calibrate_white()
        # self._left_color_sensor.calibrate_white()

        left_color = None
        right_color = None

        while end_time - start_time <= follow_time:
            error = self._generate_error(l_cs_tol=1,
                                         r_cs_tol=0.96)

            if error is None:
                raise Exception("Invalid error")

            turn_native_units = self._pid.calculate_pid_steering(error)
            log_message = ""

            log_message += "Left: " + str(self._reflected_light_percentage[
                                              "left"]) + " " + "Right: " + str(
                self._reflected_light_percentage["right"]) + "\t"
            log_message += "Turn: " + str(turn_native_units) + "\t"

            # Calculate motors speed
            left_speed = SpeedNativeUnits(
                speed_native_units + turn_native_units)
            right_speed = SpeedNativeUnits(
                speed_native_units - turn_native_units)

            log_message += "LSpeed: " + str(left_speed) + " "
            log_message += "RSpeed: " + str(right_speed)

            if sleep_time:
                time.sleep(sleep_time)

            try:
                self.tank.on(left_speed, right_speed)
            except SpeedInvalid as e:
                self.tank.stop()
                raise LineFollowErrorTooFast(
                    "The robot is moving too fast to follow the line")

            end_time = time.time()
            print(log_message)

        self.tank.stop()

        if self._display_logs:
            print("Following ended!")

    def run(self, speed, sleep_time=0.01):
        speed = speed_to_speedvalue(speed)
        speed_native_units = speed.to_native_units(self.tank.left_motor)

        while True:
            # Update color sensor readings
            self.updateSensor()

            if self._left_color in ["uknown", None] and self._right_color in ["unknown", None]:
                # Line following for next step
                pass


if __name__ == "__main__":
    regulator = PID(kp=0.85, ki=0.01, kd=3.2, integral_reset_count=5)
    follower = LineFollower2(OUTPUT_C, OUTPUT_A, INPUT_2, INPUT_1, regulator,
                             True, True)
    follower.follow_line_for_time(speed=15, follow_time=20, sleep_time=0.005)

    """
    Na razie najlepsze wartosci
    1 miejsce
    kp = 0.85
    ki = 0.05
    kd = 3.2
    counter = 3 lub 5

    2 miejsce
    kp = 0.2
    ki = 0.05
    kd = 3.2
    counter = 3 lub 5
    """
