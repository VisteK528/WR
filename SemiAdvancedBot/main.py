from advanced_bot import AdvancedBot
from pid import PID
from ev3dev2.motor import OUTPUT_B, OUTPUT_C, OUTPUT_D
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_4


if __name__ == "__main__":
    regulator = PID(kp=1, ki=0, kd=0, integral_reset_count=5)
    bot = AdvancedBot(OUTPUT_C, OUTPUT_B, OUTPUT_D, INPUT_2, INPUT_1, INPUT_4, regulator, False, False)

    try:
        # Simple follow line
        bot.run_simple_follower(speed=3, kp=1.2, ki=0, kd=0.2, follow_time=120, sleep_time=0, l_cs_tol=1., r_cs_tol=1.)
    except:
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