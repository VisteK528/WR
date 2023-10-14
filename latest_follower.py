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

        while end_time - start_time >= follow_time:
            error = self._left_color_sensor.reflected_light_intensity - self._right_color_sensor.reflected_light_intensity
            integral = integral + error
            derivative = error - last_error
            last_error = error
            turn_native_units = (kp * error) + (ki * integral) + (kd * derivative)

            if not follow_left_edge:
                turn_native_units *= -1

            left_speed = SpeedNativeUnits(speed_native_units - turn_native_units)
            right_speed = SpeedNativeUnits(speed_native_units + turn_native_units)

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
                self.tank.on(left_speed, right_speed)
            except SpeedInvalid as e:
                self.tank.stop()
                raise LineFollowErrorTooFast("The robot is moving too fast to follow the line")

        self.tank.stop()


if __name__ == "__main__":
    follower = LineFollower(OUTPUT_C, OUTPUT_A, INPUT_1, INPUT_2, "inversed", "inversed")

    try:
        # Follow the line for 4500ms
        follower.tank.follow_line(
            kp=11.3, ki=0.05, kd=3.2,
            speed=SpeedPercent(30),
            follow_for=follow_for_ms,
            follow_left_edge=True,
            ms=4500
        )
    except LineFollowErrorTooFast:
        follower.tank.stop()
