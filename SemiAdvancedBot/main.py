from advanced_bot import AdvancedBot, Colors
from pid import PID
from ev3dev2.motor import OUTPUT_B, OUTPUT_C, OUTPUT_A
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_4


if __name__ == "__main__":
    regulator = PID(kp=1.2, ki=0, kd=0.2, integral_reset_count=5)
    bot = AdvancedBot(OUTPUT_C, OUTPUT_B, OUTPUT_A, INPUT_2, INPUT_1, INPUT_4, regulator, False, False)
    colors = [Colors.BLUE, Colors.GREEN]

    try:
        bot.run_loader_job(colors)
    except KeyboardInterrupt:
        bot.turn_off_all_motors()
    except Exception as e:
        print(e)

    bot.turn_off_all_motors()
    print("Stopped")
