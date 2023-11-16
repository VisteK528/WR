from advanced_bot import AdvancedBot
from pid import PID

if __name__ == "__main__":
    bot = AdvancedBot()

    # Simple follow line
    bot.run_simple_follower()

    # Pick up the package and put it down on target
    bot.run_loader_job()