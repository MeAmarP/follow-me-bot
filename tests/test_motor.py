# test_motor.py

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
from motor_driver import Motor
import time

# Motor A test setup: ENA -> GPIO12, IN1 -> GPIO20, IN2 -> GPIO21
motor_a = Motor(enable_pin=12, in1_pin=20, in2_pin=21)

# Motor B test setup: ENB -> GPIO13, IN3 -> GPIO19, IN4 -> GPIO26
motor_b = Motor(enable_pin=13, in1_pin=19, in2_pin=26)

try:
    print("Testing Motor A - FORWARD")
    motor_a.move("reverse", speed=0.6)
    time.sleep(2)
    print("Stopping Motor A")
    motor_a.stop()
    time.sleep(1)

    print("Testing Motor A - REVERSE")
    motor_a.move("forward", speed=0.6)
    time.sleep(2)
    print("Stopping Motor A")
    motor_a.stop()
    time.sleep(1)

    print("Testing Motor B - FORWARD")
    motor_b.move("forward", speed=0.6)
    time.sleep(2)
    print("Stopping Motor B")
    motor_b.stop()
    time.sleep(1)

    print("Testing Motor B - REVERSE")
    motor_b.move("reverse", speed=0.6)
    time.sleep(2)
    print("Stopping Motor B")
    motor_b.stop()

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    print("Stopping all motors")
    motor_a.stop()
    motor_b.stop()

