from ev3dev2.motor import OUTPUT_B, OUTPUT_C, MoveTank, speed_to_speedvalue, SpeedInvalid, SpeedNativeUnits, MediumMotor
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.motor import LineFollowErrorTooFast
from pid import PID
from time import sleep, time
from my_utils import my_map, States, TurnDirection, Colors

"""
States:
S1 Follow the line
S2 Turn to the loading / destination point and move for x time / rotations
S3 Probe for the package
S4 Load the package
S5 Return back to the line
S6 Unload the package
S7 Terminal State
"""


class AdvancedBot:
    def __init__(self, left_motor: str, right_motor: str, medium_motor: str, left_sensor: str,
                 right_sensor: str, touch_sensor: str, pid_controller: PID,
                 left_motor_polarity_inversed=False,
                 right_motor_polarity_inversed=False):

        # Variables
        self._state = States.FOLLOW_THE_LINE
        self._loaded = False
        self._turn_direction = None
        self._pick_color = None

        self._pid = pid_controller

        # Init tank unit
        self.tank = MoveTank(left_motor_port=left_motor,
                             right_motor_port=right_motor)

        # Polarity
        if left_motor_polarity_inversed:
            self.tank.left_motor.polarity = "inversed"
        else:
            self.tank.left_motor.polarity = "normal"

        if right_motor_polarity_inversed:
            self.tank.right_motor.polarity = "inversed"
        else:
            self.tank.right_motor.polarity = "normal"

        self._medium_motor = MediumMotor(medium_motor)

        # Init color sensors
        self._l_cs = ColorSensor(left_sensor)
        self._r_cs = ColorSensor(right_sensor)

        # Set color sensors to the RGB_RAW mode
        self._l_cs.mode = ColorSensor.MODE_RGB_RAW
        self._r_cs.mode = ColorSensor.MODE_RGB_RAW

        self._l_color = None
        self._r_color = None

        self._l_rgb = None
        self._r_rgb = None

        self._l_grayscale = 0
        self._r_grayscale = 0

        self._touch_sensor = TouchSensor(touch_sensor)

        # ==================== Parameters =====================
        self._default_pid_parameters = (3.8, 0.0, 0.2)
        self._default_speed = 10

        self._follow_speed = speed_to_speedvalue(self._default_speed)
        self._speed_native_units = self._follow_speed.to_native_units(
            self.tank.left_motor)

        # ==================== Calibration ====================

        self._l_cs.calibrate_white()
        self._r_cs.calibrate_white()
        
        self._max_l_cs = [self._l_cs.red_max, self._l_cs.green_max,
                          self._l_cs.blue_max]

        self._max_r_cs = [self._r_cs.red_max, self._r_cs.green_max,
                          self._r_cs.blue_max]

        print("Configuration complete")

    def turn_off_all_motors(self):
        self.tank.stop()
        self._medium_motor.stop()

    def update_colors(self):
        raw_left = [self._l_cs.value(i) for i in range(3)]
        raw_right = [self._r_cs.value(i) for i in range(3)]

        self._l_rgb = [min(int((color * 255) / max_color), 255) for color,
                       max_color in zip(raw_left, self._max_l_cs)]

        self._r_rgb = [min(int((color * 255) / max_color), 255) for color,
                       max_color in zip(raw_right, self._max_r_cs)]

        # TODO Check the ranges of the colors in the spectrum and assign
        #      readings for self._lcolor and self._r_color using Colors class

        # eg.
        """
        ((color_left[0] > 260 and color_left[1] < 80 and
             color_left[2] < 50 and il > 260 and il < 330),
            (color_right[0] > 200 and color_right[1] < 70 and
             color_right[2] < 50 and ir > 200 and ir < 260))

        """

    def calculate_grayscale(self):
        coefficients = [0.30, 0.59, 0.11]

        self._l_grayscale = sum([coeff * canal for coeff, canal in zip(
            coefficients, self._l_rgb)])
        self._r_grayscale = sum([coeff * canal for coeff, canal in zip(
            coefficients, self._r_rgb)])

        self._l_grayscale = my_map(self._l_grayscale, 0, 255, 0, 100)
        self._r_grayscale = my_map(self._r_grayscale, 0, 255, 0, 100)

    def _turn_by_angle(self, clockwise: bool, angle: float):
        """
        :param clockwise: Direction of the turn, either clockwise (to the right)
         or counter-clockwise (to the left)
        :param angle: Turn's angle in degrees
        :return: None
        """
        pass
    
    def _turn_to_the_point(self):
        pass
    
    def _probe(self):
        pass
    
    def _return_to_the_line(self):
        pass
    
    def _load_the_package(self):
        pass
    
    def _unload_the_package(self):
        pass

    def follow_line_step(self, l_tol: float, r_tol: float):

        # Compute error for PID
        error = self._l_grayscale * l_tol - self._r_grayscale * r_tol

        # Compute steering
        turn_native_units = self._pid.calculate_pid_steering(error)

        # Calculate motors speed
        left_speed = SpeedNativeUnits(
            self._speed_native_units + turn_native_units)
        right_speed = SpeedNativeUnits(
            self._speed_native_units - turn_native_units)

        try:
            self.tank.on(left_speed, right_speed)
        except SpeedInvalid as e:
            self.tank.stop()
            raise LineFollowErrorTooFast(
                "The robot is moving too fast to follow the line")

    def run_simple_follower(self, speed, kp, ki, kd, follow_time,
                            sleep_time=0.01, l_cs_tol=1., r_cs_tol=1.):

        self._pid.set_pid_coefficients(kp, ki, kd)
        self._follow_speed = speed_to_speedvalue(speed)
        self._speed_native_units = self._follow_speed.to_native_units(
            self.tank.left_motor)

        print("Following started!")
        start_time = end_time = time()
        while end_time - start_time <= follow_time:

            self.follow_line_step(l_cs_tol, r_cs_tol)
            sleep(sleep_time)

            end_time = time()

        self.tank.stop()
        print("Following ended!")

    def run_loader_job(self, colors):
            # Set parameters to default values
            self._pid.set_pid_coefficients(*self._default_pid_parameters)
            self._follow_speed = speed_to_speedvalue(self._default_speed)
            self._speed_native_units = self._follow_speed.to_native_units(
                self.tank.left_motor)

            while self._state != States.TERMINAL:
                self.update_colors()
                self.calculate_grayscale()

                if self._state == States.FOLLOW_THE_LINE:
                    if self._l_color in colors:
                        self._pick_color = self._l_color
                        self._turn_direction = TurnDirection.LEFT
                        self._state = States.TURN_TO_THE_POINT
                        continue

                    elif self._r_color in colors:
                        self._pick_color = self._r_color
                        self._turn_direction = TurnDirection.RIGHT
                        self._state = States.TURN_TO_THE_POINT
                        continue

                    else:
                        self._state = States.FOLLOW_THE_LINE
                        self.follow_line_step(l_tol=1, r_tol=0.9)
                elif self._state == States.TURN_TO_THE_POINT:
                    self._turn_to_the_point()

                    if not self._loaded:
                        self._state = States.PROBE
                    else:
                        self._state = States.UNLOAD

                elif self._state == States.PROBE:
                    self._probe()

                    self._state = States.LOAD

                elif self._state == States.LOAD:
                    self._load_the_package()

                    self._state = States.RETURN_TO_THE_LINE

                elif self._state == States.RETURN_TO_THE_LINE:
                    self._return_to_the_line()

                    if self._loaded:
                        self._state = States.FOLLOW_THE_LINE
                    else:
                        self._state = States.TERMINAL

                elif self._state == States.UNLOAD:
                    self._unload_the_package()

                    self._state = States.RETURN_TO_THE_LINE

                # Cool down for 5ms / If not necessary remove (very probable)
                sleep(0.005)




