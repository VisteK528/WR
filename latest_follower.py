import time
from ev3dev2.motor import OUTPUT_B, OUTPUT_C, MoveTank, speed_to_speedvalue, SpeedInvalid, SpeedNativeUnits
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.motor import LineFollowErrorTooFast

"""
General information:
Motors
    - LEFT MOTOR - OUTPUT_C
    - RIGHT MOTOR - OUTPUT_A
Sensors
    - Left Color Sensor - ?
    - Right Color Sensor - ?

"""


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
        else:
            self.tank.left_motor.polarity = "normal"

        if right_motor_polarity_inversed:
            self.tank.right_motor.polarity = "inversed"
        else:
            self.tank.right_motor.polarity = "normal"

        # Init color sensors
        self._l_cs = ColorSensor(left_sensor)
        self._r_cs = ColorSensor(right_sensor)

        self._reflected_light_percentage = {"left": None, "right": None}

        # Assign PID controller object to variable
        self._pid = pid_controller

        self._display_logs = display_logs
        if self._display_logs:
            print("Configuration complete")

    def _generate_error(self, l_cs_tol: float,
                        r_cs_tol: float):
        self._reflected_light_percentage = {
            "left": self._l_cs.reflected_light_intensity,
            "right": self._r_cs.reflected_light_intensity
        }

        error = (self._reflected_light_percentage["left"]*l_cs_tol
                 - self._reflected_light_percentage["right"]*r_cs_tol)

        return error

    def follow_line_for_time(self, speed, follow_time, sleep_time=0.01,
                             l_cs_tol=1., r_cs_tol=1.):
        speed = speed_to_speedvalue(speed)
        speed_native_units = speed.to_native_units(self.tank.left_motor)

        start_time = end_time = time.time()

        if self._display_logs:
            print("Following started!")

        # Optional sensor calibration
        # self._right_color_sensor.calibrate_white()
        # self._left_color_sensor.calibrate_white()

        while end_time - start_time <= follow_time:
            error = self._generate_error(l_cs_tol=l_cs_tol,
                                         r_cs_tol=r_cs_tol)
            turn_native_units = self._pid.calculate_pid_steering(error)
            log_message = ""

            log_message += "Left: " + str(self._reflected_light_percentage["left"]) + " " + "Right: " + str(self._reflected_light_percentage["right"]) + "\t"
            log_message += "Turn: " + str(turn_native_units) + "\t"

            # Calculate motors speed
            left_speed = SpeedNativeUnits(speed_native_units + turn_native_units)
            right_speed = SpeedNativeUnits(speed_native_units - turn_native_units)

            log_message += "LSpeed: " + str(left_speed) + " "
            log_message += "RSpeed: " + str(right_speed)

            if sleep_time:
                time.sleep(sleep_time)

            try:
                self.tank.on(left_speed, right_speed)
            except SpeedInvalid as e:
                self.tank.stop()
                raise LineFollowErrorTooFast("The robot is moving too fast to follow the line")

            end_time = time.time()
            print(log_message)

        self.tank.stop()

        if self._display_logs:
            print("Following ended!")

    def follow_line_for_time_raw(self, speed, follow_time, sleep_time=0.01,
                                 l_cs_tol=1., r_cs_tol=1.):
        speed = speed_to_speedvalue(speed)
        speed_native_units = speed.to_native_units(self.tank.left_motor)

        start_time = end_time = time.time()

        if self._display_logs:
            print("Following started!")

        # Optional sensor calibration
        # self._right_color_sensor.calibrate_white()
        # self._left_color_sensor.calibrate_white()

        while end_time - start_time <= follow_time:
            error = self._generate_error(l_cs_tol=l_cs_tol,
                                         r_cs_tol=r_cs_tol)
            turn_native_units = self._pid.calculate_pid_steering(error)

            # Calculate motors speed
            left_speed = SpeedNativeUnits(speed_native_units + turn_native_units)
            right_speed = SpeedNativeUnits(speed_native_units - turn_native_units)

            if sleep_time:
                time.sleep(sleep_time)

            try:
                self.tank.on(left_speed, right_speed)
            except SpeedInvalid as e:
                self.tank.stop()
                raise LineFollowErrorTooFast("The robot is moving too fast to follow the line")

            end_time = time.time()

        self.tank.stop()

        if self._display_logs:
            print("Following ended!")


if __name__ == "__main__":
    regulator = PID(kp=4, ki=0, kd=1, integral_reset_count=5)
    follower = LineFollower2(OUTPUT_C, OUTPUT_B, INPUT_2, INPUT_1, regulator,
                             False, False)

    follower.follow_line_for_time(speed=20, follow_time=40, sleep_time=0.001,
                                  l_cs_tol=1, r_cs_tol=0.9)

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
    """
    Informacje warte uwagi:
    1. Zwrócić uwagę na naładowanie akumulatorów - niskie napięcie może wpływać na działanie czujników
    2. Rozpocząć kalibrację od Kp, Ki i Kd zainicjalizowane jako 0
    3. Przy zwiększaniu Kd, zmniejszać lekko Kp

    """
