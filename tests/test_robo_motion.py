# test_robo_motion.py

import sys
import os
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
from motor_driver import MotorDriver

# Create MotorDriver instance
driver = MotorDriver()

try:
    print("Testing FORWARD motion")
    driver.forward(speed=0.6)
    time.sleep(2)
    driver.stop()
    time.sleep(1)

    print("Testing REVERSE motion")
    driver.reverse(speed=0.6)
    time.sleep(2)
    driver.stop()
    time.sleep(1)

    print("Testing LEFT turn")
    driver.left(speed=0.6)
    time.sleep(2)
    driver.stop()
    time.sleep(1)

    print("Testing RIGHT turn")
    driver.right(speed=0.6)
    time.sleep(2)
    driver.stop()

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    print("Stopping all motors")
    driver.stop()
