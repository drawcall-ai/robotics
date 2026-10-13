"""Autonomous policy (sim): TASK from the ROS parameter `task` (solve | antagonist)."""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE)); sys.path.insert(0, os.path.join(HERE, "auto"))
from robot import Robot  # noqa: E402
from bot import SimBot  # noqa: E402
import run  # noqa: E402

robot = Robot()
task = robot.param("task", os.environ.get("TASK", "solve")).lower()
seed = robot.param("seed", 0)
logdir = os.path.join(HERE, "auto", "logs", time.strftime("%Y%m%d-%H%M%S") + f"-{task}")
run.main(SimBot(robot), task, logdir, seed)
