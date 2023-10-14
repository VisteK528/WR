from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_2
import time

sensor1 = ColorSensor(INPUT_1)

while True:
    print(sensor1.color_name)
    time.sleep(0.1)