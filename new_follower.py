import time

from ev3dev2.motor import OUTPUT_A, OUTPUT_C, MoveTank, SpeedPercent, follow_for_ms, speed_to_speedvalue, SpeedInvalid, SpeedNativeUnits
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


class LineFollower:
    def __init__(self, left_motor: str, right_motor: str, left_sensor: str, right_sensor: str,
                 left_motor_polarity="inversed", right_motor_polarity="inversed"):

        # Init tank unit
        self.tank = MoveTank(left_motor_port=left_motor, right_motor_port=right_motor)

        # Set polarity of the motors
        self.tank.left_motor.polarity = left_motor_polarity
        self.tank.right_motor.polarity = right_motor_polarity

        # Init color sensors
        self._left_color_sensor = ColorSensor(left_sensor)
        self._right_color_sensor = ColorSensor(right_sensor)
        print("Configuration complete")

    def calculate_pid_steering(self, kp: float, ki: float, kd: float):
        pass


    def follow_line_2_sensors(self, kp, ki, kd, speed, follow_time, target_light_intensity=None, follow_left_edge=True, white=60,
                              off_line_count_max=20, sleep_time=0.01):
        integral = 0.0
        last_error = 0.0
        derivative = 0.0
        off_line_count = 0
        speed = speed_to_speedvalue(speed)
        speed_native_units = speed.to_native_units(self.tank.left_motor)
        start_time = time.time()
        end_time = time.time()
        print("Start following")
        # self._right_color_sensor.calibrate_white()
        # self._left_color_sensor.calibrate_white()

        while end_time - start_time <= follow_time:
            error =  self._left_color_sensor.reflected_light_intensity - self._right_color_sensor.reflected_light_intensity - 50
            print("Left: ", self._left_color_sensor.reflected_light_intensity, "Right: ", self._right_color_sensor.reflected_light_intensity )
            integral = integral + error
            derivative = error - last_error
            last_error = error
            turn_native_units = (kp * error) + (ki * integral) + (kd * derivative)
            print("Native units: ", turn_native_units)

            left_speed = SpeedNativeUnits(speed_native_units + turn_native_units)
            right_speed = SpeedNativeUnits(speed_native_units - turn_native_units)
            print("Left speed: ", left_speed, " ", "Right speed: ", right_speed)

            # Have we lost the line?
            if self._left_color_sensor.reflected_light_intensity >= white and self._right_color_sensor.reflected_light_intensity >= white:
                off_line_count += 1

                if off_line_count >= off_line_count_max:
                    self.tank.stop()
                    raise LineFollowErrorLostLine("we lost the line")
            else:
                off_line_count = 0

            if sleep_time:
                time.sleep(sleep_time)

            try:
                pass
                self.tank.on(left_speed, right_speed)
            except SpeedInvalid as e:
                self.tank.stop()
                raise LineFollowErrorTooFast("The robot is moving too fast to follow the line")

            end_time = time.time()

        self.tank.stop()
        print("Following ended")


if __name__ == "__main__":
    follower = LineFollower(OUTPUT_C, OUTPUT_A, INPUT_2, INPUT_1, "inversed", "inversed")
    follower.follow_line_2_sensors(kp=1, ki=0.05, kd=0.05, speed=15, follow_time=4500, sleep_time=0.01)

    try:
        # Follow the line for 4500ms
        follower.tank.follow_line(
            kp=0.5, ki=0.01, kd=4,
            speed=SpeedPercent(30),
            follow_for=follow_for_ms,
            follow_left_edge=True,
            ms=4500
        )
    except LineFollowErrorTooFast:
        follower.tank.stop()
