from advanced_bot import AdvancedBot
from pid import PID
from ev3dev2.motor import OUTPUT_B, OUTPUT_C, OUTPUT_D
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_4


if __name__ == "__main__":
    regulator = PID(kp=3.8, ki=0, kd=0.2, integral_reset_count=5)
    bot = AdvancedBot(OUTPUT_C, OUTPUT_B, OUTPUT_D, INPUT_2, INPUT_1, INPUT_4, regulator, False, False)

    # Simple follow line
    bot.run_simple_follower(speed=10, kp=3.8, ki=0, kd=0.2, follow_time=20, sleep_time=0.01, l_cs_tol=1., r_cs_tol=0.9)

    # Pick up the package and put it down on target
    #bot.run_loader_job()