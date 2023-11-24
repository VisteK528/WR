from moze_dziala import AdvancedBot
from my_utils import Colors
from pid import PID
from ev3dev2.motor import OUTPUT_B, OUTPUT_C, OUTPUT_A
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_4
import time


if __name__ == "__main__":
    regulator = PID(kp=1.2, ki=0, kd=0.2, integral_reset_count=5)
    bot = AdvancedBot(OUTPUT_C, OUTPUT_B, OUTPUT_A, INPUT_2, INPUT_1, INPUT_4, regulator, False, False)
    colors = [Colors.BLUE, Colors.GREEN]

    try:
        # Simple follow line
        #bot.run_simple_follower(speed=3, kp=1.2, ki=0, kd=0.2, follow_time=120, sleep_time=0, l_cs_tol=1., r_cs_tol=1.)
        bot.run_loader_job(colors)
        """while True:
            bot.update_colors([Colors.RED, Colors.GREEN])
            print(bot._l_color, bot._r_color)
            time.sleep(0.1)"""
    except KeyboardInterrupt:
        bot.turn_off_all_motors()
    except Exception as e:
        print(e)

    bot.turn_off_all_motors()
    print("Stopped")
    # Pick up the package and put it down on target
    #bot.run_loader_job()

"""
Ustawienia jakkolwiek dzialajace
bot.run_simple_follower(speed=5, kp=1, ki=0, kd=0, follow_time=40, sleep_time=0.01, l_cs_tol=1., r_cs_tol=0.9)

speed=7
kp=1.5
kd=0.6

# Dla całego toru
speed = 3
kp = 1
kd = 0.2
"""