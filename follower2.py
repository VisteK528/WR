from ev3dev2.motor import LargeMotor, MoveSteering, OUTPUT_A, OUTPUT_C, SpeedPercent
from ev3dev2.sensor import Colo

steering_drive = MoveSteering(OUTPUT_A, OUTPUT_C)
steering_drive.on_for_seconds(0, SpeedPercent(75))