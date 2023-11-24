from ev3dev2.motor import MoveTank, speed_to_speedvalue, SpeedInvalid, SpeedNativeUnits, MediumMotor, SpeedPercent, MoveSteering
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.motor import LineFollowErrorTooFast
from pid import PID
from time import sleep, time
from my_utils import my_map, States, TurnDirection, Colors
from ev3dev2.button import Button

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

FULL_TURN_TIME = 5.445


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
        
        self.steering_drive = MoveSteering(left_motor_port=left_motor,
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

    def update_colors(self, special_colors=None):
        raw_left = [self._l_cs.value(i) for i in range(3)]
        raw_right = [self._r_cs.value(i) for i in range(3)]

        self._l_rgb = [min(int((color * 255) / max_color), 255) for color,
                       max_color in zip(raw_left, self._max_l_cs)]

        self._r_rgb = [min(int((color * 255) / max_color), 255) for color,
                       max_color in zip(raw_right, self._max_r_cs)]

        # 4 colors to be checked Red, Green, Yellow, Blue

        # Red, Green
        # Red, Yellow
        # Red, Blue
        # Green, Yellow
        # Green, Blue
        # Yellow, Blue

        # RGB values from sensors for different colors after calibration
        # White     ->  (255, 255, 255)
        # Green     ->  ( 41, 120,  61)
        # Yellow    ->  (255, 255,  68)
        # Red       ->  (232,  44,  32)
        # Blue      ->  ( 48, 102, 164)

        saturation_threshold = 0.5
        value_threshold = 0.4

        lhsv = rgb_to_hsv(*self._l_rgb)
        rhsv = rgb_to_hsv(*self._r_rgb)

        self._l_color = Colors.UNKNOWN
        self._r_color = Colors.UNKNOWN

        # Check if there are any special colors to be checked
        if special_colors:
            if special_colors[0] in [Colors.RED, Colors.GREEN] and special_colors[1] in [Colors.RED, Colors.GREEN]:
                if lhsv[1] >= saturation_threshold and lhsv[2] >= value_threshold:
                    # Check RED
                    if (0 <= lhsv[0] <= 26) or (282 <= lhsv[0] <= 359):
                        self._l_color = Colors.RED

                    # Check GREEN
                    if 70 <= lhsv[0] <= 172:
                        self._l_color = Colors.GREEN

                if rhsv[1] >= saturation_threshold and rhsv[2] >= value_threshold:
                    # Check RED
                    if (0 <= rhsv[0] <= 26) or (282 <= rhsv[0] <= 359):
                        self._r_color = Colors.RED

                    # Check GREEN
                    if 70 <= rhsv[0] <= 172:
                        self._r_color = Colors.GREEN

            elif special_colors[0] in [Colors.RED, Colors.YELLOW] and special_colors[1] in [Colors.RED, Colors.YELLOW]:
                if lhsv[1] >= saturation_threshold and lhsv[2] >= value_threshold:
                    # Check RED
                    if (0 <= lhsv[0] <= 26) or (282 <= lhsv[0] <= 359):
                        self._l_color = Colors.RED

                    # Check YELLOW
                    if 30 <= lhsv[0] <= 80:
                        self._l_color = Colors.YELLOW

                if rhsv[1] >= saturation_threshold and rhsv[2] >= value_threshold:
                    # Check RED
                    if (0 <= rhsv[0] <= 26) or (282 <= rhsv[0] <= 359):
                        self._r_color = Colors.RED

                    # Check YELLOW
                    if 30 <= rhsv[0] <= 80:
                        self._r_color = Colors.YELLOW

            elif special_colors[0] in [Colors.RED, Colors.BLUE] and special_colors[1] in [Colors.RED, Colors.BLUE]:
                if lhsv[1] >= saturation_threshold and lhsv[2] >= value_threshold:
                    # Check RED
                    if (0 <= lhsv[0] <= 40) or (282 <= lhsv[0] <= 359):
                        self._l_color = Colors.RED

                    # Check BLUE
                    if 160 <= lhsv[0] <= 260:
                        self._l_color = Colors.BLUE

                if rhsv[1] >= saturation_threshold and rhsv[2] >= value_threshold:
                    # Check RED
                    if (0 <= rhsv[0] <= 40) or (282 <= rhsv[0] <= 359):
                        self._r_color = Colors.RED

                    # Check BLUE
                    if 160 <= rhsv[0] <= 260:
                        self._r_color = Colors.BLUE

            elif special_colors[0] in [Colors.GREEN, Colors.YELLOW] and special_colors[1] in [Colors.GREEN, Colors.YELLOW]:
                if lhsv[1] >= saturation_threshold and lhsv[2] >= value_threshold:
                    # Check GREEN
                    if 70 <= lhsv[0] <= 172:
                        self._l_color = Colors.GREEN

                    # Check YELLOW
                    if 35 <= lhsv[0] <= 69:
                        self._l_color = Colors.YELLOW

                if rhsv[1] >= saturation_threshold and rhsv[2] >= value_threshold:
                    # Check GREEN
                    if 70 <= rhsv[0] <= 172:
                        self._r_color = Colors.GREEN

                    # Check YELLOW
                    if 35 <= rhsv[0] <= 69:
                        self._r_color = Colors.YELLOW

            elif special_colors[0] in [Colors.GREEN, Colors.BLUE] and special_colors[1] in [Colors.GREEN, Colors.BLUE]:
                if lhsv[1] >= saturation_threshold and lhsv[2] >= value_threshold:
                    # Check GREEN
                    if 70 <= lhsv[0] <= 155:
                        self._l_color = Colors.GREEN

                    # Check BLUE
                    if 160 <= lhsv[0] <= 260:
                        self._l_color = Colors.BLUE

                if rhsv[1] >= saturation_threshold and rhsv[2] >= value_threshold:
                    # Check GREEN
                    if 70 <= lhsv[0] <= 155:
                        self._l_color = Colors.GREEN

                    # Check BLUE
                    if 160 <= rhsv[0] <= 260:
                        self._r_color = Colors.BLUE

            elif special_colors[0] in [Colors.YELLOW, Colors.BLUE] and special_colors[1] in [Colors.YELLOW, Colors.BLUE]:
                if lhsv[1] >= saturation_threshold and lhsv[2] >= value_threshold:
                    # Check YELLOW
                    if 35 <= lhsv[0] <= 69:
                        self._l_color = Colors.YELLOW

                    # Check BLUE
                    if 160 <= lhsv[0] <= 260:
                        self._l_color = Colors.BLUE

                if rhsv[1] >= saturation_threshold and rhsv[2] >= value_threshold:
                    # Check YELLOW
                    if 35 <= rhsv[0] <= 69:
                        self._r_color = Colors.YELLOW

                    # Check BLUE
                    if 160 <= rhsv[0] <= 260:
                        self._r_color = Colors.BLUE

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
        self.tank.stop()

        if clockwise:
            self.steering_drive.on_for_seconds(100, SpeedPercent(10), FULL_TURN_TIME/4)
        else:
            self.steering_drive.on_for_seconds(-100, SpeedPercent(10), FULL_TURN_TIME/4)
    
    def _turn_to_the_point(self):
        # Additional offset move forwards (to be calculated on labolatories)
        self.tank.stop()
        time_forward = 0.4
        self.tank.on_for_seconds(SpeedPercent(10), SpeedPercent(10), time_forward, brake=True)

        if self._turn_direction == TurnDirection.RIGHT:
            self._turn_by_angle(clockwise=True, angle=90)
        else:
            self._turn_by_angle(clockwise=False, angle=90)

        #pick_up_point_forward_time = 4
        #self.tank.on_for_seconds(SpeedPercent(10), SpeedPercent(10), pick_up_point_forward_time, brake=True)

        self.run_simple_follower(3, 1.2, 0, 0.2, 12)
        print("Simple following ended")

    
    def _probe_one_path(self, max_probing_time):
        start_time = time()
        self.tank.on(SpeedPercent(3), SpeedPercent(3))
        found = False
        while time() - start_time < max_probing_time:
            if self._touch_sensor.is_pressed:
                elapsed_time = time() - start_time
                self.tank.off()
                found = True
            else:
                elapsed_time = max_probing_time
        self.tank.off()
        return found, elapsed_time
    
    def _probe(self, max_probing_time):
        self.steering_drive.on_for_seconds(100, SpeedPercent(10), FULL_TURN_TIME/18)
        self._medium_motor.on_for_degrees(SpeedPercent(10), -20)
        while True:
            for angle in [-1,0,1]:
                found, elapsed_time = self._probe_one_path(max_probing_time)
                if found:
                    return elapsed_time, angle
                self.tank.on_for_seconds(SpeedPercent(-3), SpeedPercent(-3), elapsed_time)
                self.steering_drive.on_for_seconds(-100, SpeedPercent(10), FULL_TURN_TIME/18)
            self.steering_drive.on_for_seconds(100, SpeedPercent(10), FULL_TURN_TIME/6)

    def _return_to_the_line(self, elapsed_time, angle, pick_up_point_forward_time):
        self.tank.on_for_seconds(SpeedPercent(-3), SpeedPercent(-3), elapsed_time)
        if self._loaded:
            if angle == -1:
                self.steering_drive.on_for_seconds(-100, SpeedPercent(10), FULL_TURN_TIME/18)
            elif angle == 1:
                self.steering_drive.on_for_seconds(100, SpeedPercent(10), FULL_TURN_TIME/18)

        self.steering_drive.on_for_seconds(100, SpeedPercent(10), FULL_TURN_TIME/2)

        print("Returning to the line currently")
        self.tank.on_for_seconds(SpeedPercent(10), SpeedPercent(10), pick_up_point_forward_time, brake=True)

        if self._turn_direction == TurnDirection.RIGHT:
            self._turn_by_angle(clockwise=True, angle=75)
        else:
            self._turn_by_angle(clockwise=False, angle=75)
        print("Next following")

        pick_up_point_forward_time = 1
        self.tank.on_for_seconds(SpeedPercent(10), SpeedPercent(10), pick_up_point_forward_time, brake=True)

    def _load_the_package(self):
        self._medium_motor.on_for_degrees(SpeedPercent(10), -80)
        self._loaded = True
    
    def _unload_the_package(self):
        self._medium_motor.on_for_degrees(SpeedPercent(10), 80)
        self._loaded = False

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

            self.update_colors()

            self.calculate_grayscale()

            self.follow_line_step(l_cs_tol, r_cs_tol)
            sleep(sleep_time)

            end_time = time()

        self.tank.stop()
        print("Following ended!")

    def run_loader_job(self, custom_colors):
            # Set parameters to default values
            kp = 1.2
            ki = 0
            kd = 0.2
            speed = 3

            self._pid.set_pid_coefficients(kp, ki, kd)
            self._follow_speed = speed_to_speedvalue(speed)
            self._speed_native_units = self._follow_speed.to_native_units(
                self.tank.left_motor)

            while self._state != States.TERMINAL:

                # Probe RGB from both sensors and calculate grayscale reading
                self.update_colors(custom_colors)
                self.calculate_grayscale()

                if self._state == States.FOLLOW_THE_LINE:
                    if self._l_color == custom_colors[0]:
                        self._pick_color = self._l_color
                        self._turn_direction = TurnDirection.LEFT
                        self._state = States.TURN_TO_THE_POINT
                        print("Turn state")
                        continue

                    elif self._r_color == custom_colors[0]:
                        self._pick_color = self._r_color
                        self._turn_direction = TurnDirection.RIGHT
                        self._state = States.TURN_TO_THE_POINT
                        print("Turn state")
                        continue

                    else:
                        self._state = States.FOLLOW_THE_LINE
                        self.follow_line_step(l_tol=1, r_tol=1)

                elif self._state == States.TURN_TO_THE_POINT:
                    self._turn_to_the_point()

                    if not self._loaded:
                        self._state = States.PROBE
                        print("Probe")
                    else:
                        self._state = States.UNLOAD
                        print("Unload")

                elif self._state == States.PROBE:
                    elapsed_time, angle = self._probe(6)

                    self._state = States.LOAD
                    print("Load")

                elif self._state == States.LOAD:
                    self._load_the_package()

                    self._state = States.RETURN_TO_THE_LINE
                    print("Return to the line")

                elif self._state == States.RETURN_TO_THE_LINE:

                    if self._loaded:
                        pick_up_forward_time = 2.5
                        self._return_to_the_line(elapsed_time, angle, pick_up_forward_time)
                        self._state = States.FOLLOW_THE_LINE
                        print("Follow the line")
                    else:
                        pick_up_forward_time = 1.5
                        self._return_to_the_line(elapsed_time, angle, pick_up_forward_time)
                        self._state = States.TERMINAL
                        print("Terminal")

                elif self._state == States.UNLOAD:
                    self._unload_the_package()

                    self._state = States.RETURN_TO_THE_LINE
                    print("Return to the line")

                # Cool down for 5ms / If not necessary remove (very probable)
                sleep(0.005)
