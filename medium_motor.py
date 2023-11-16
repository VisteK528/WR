from ev3dev2.motor import OUTPUT_D, MediumMotor, SpeedPercent


if __name__ == "__main__":
    motor = MediumMotor(OUTPUT_D)
    motor.on_for_degrees(SpeedPercent(50), -45)