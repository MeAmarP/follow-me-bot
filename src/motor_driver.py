# motor_driver.py

from gpiozero import PWMOutputDevice, DigitalOutputDevice
from typing import Literal
import logging

# Setup basic logger configuration
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Define allowed directions using Literal for type checking
Direction = Literal["forward", "reverse", "stop"]

class Motor:
    """
    Class representing a single DC motor controlled through L298N driver.
    It uses one PWM pin for speed control and two digital pins for direction.
    """

    def __init__(self, enable_pin: int, in1_pin: int, in2_pin: int, pwm_freq: int = 1000) -> None:
        """
        Initialize motor control pins.

        :param enable_pin: GPIO pin for PWM (speed control)
        :param in1_pin: GPIO pin for direction input 1
        :param in2_pin: GPIO pin for direction input 2
        :param pwm_freq: PWM frequency in Hz (default: 1000)
        """
        self._pwm = PWMOutputDevice(enable_pin, frequency=pwm_freq)
        self._in1 = DigitalOutputDevice(in1_pin)
        self._in2 = DigitalOutputDevice(in2_pin)
        logger.info(f"Initialized Motor (EN: GPIO{enable_pin}, IN1: GPIO{in1_pin}, IN2: GPIO{in2_pin})")

    def move(self, direction: Direction, speed: float = 0.5) -> None:
        """
        Move motor in specified direction with given speed.

        :param direction: One of "forward", "reverse", "stop"
        :param speed: PWM duty cycle (0.0 to 1.0)
        """
        if direction == "forward":
            self._in1.on()
            self._in2.off()
            self._pwm.value = speed
            logger.debug(f"Motor moving FORWARD at speed {speed:.2f}")
        elif direction == "reverse":
            self._in1.off()
            self._in2.on()
            self._pwm.value = speed
            logger.debug(f"Motor moving REVERSE at speed {speed:.2f}")
        elif direction == "stop":
            self._pwm.value = 0
            self._in1.off()
            self._in2.off()
            logger.debug("Motor STOPPED")
        else:
            raise ValueError(f"Invalid direction: {direction!r}")

    def stop(self) -> None:
        """
        Stop the motor by disabling both input pins and PWM.
        """
        self.move("stop")


class MotorDriver:
    """
    MotorDriver class handles two motors to control robot movement.
    It abstracts motor commands to implement directional movement.
    """

    def __init__(self) -> None:
        """
        Initialize both motors with appropriate GPIO pins.
        Motor A is assumed to control the left wheel, Motor B the right wheel.
        """
        # Motor A (left motor): ENA -> GPIO12, IN1 -> GPIO20, IN2 -> GPIO21
        self.motor_a = Motor(enable_pin=12, in1_pin=20, in2_pin=21)
        # Motor B (right motor): ENB -> GPIO13, IN3 -> GPIO19, IN4 -> GPIO26
        self.motor_b = Motor(enable_pin=13, in1_pin=19, in2_pin=26)
        logger.info("MotorDriver initialized for 2 motors.")

    def forward(self, speed: float = 0.5) -> None:
        """
        Move robot forward by driving both motors forward.

        :param speed: Speed of movement (0.0 to 1.0)
        """
        logger.info("Robot moving FORWARD")
        self.motor_a.move("forward", speed)
        self.motor_b.move("forward", speed)

    def reverse(self, speed: float = 0.5) -> None:
        """
        Move robot backward by driving both motors in reverse.

        :param speed: Speed of movement (0.0 to 1.0)
        """
        logger.info("Robot moving REVERSE")
        self.motor_a.move("reverse", speed)
        self.motor_b.move("reverse", speed)

    def left(self, speed: float = 0.5) -> None:
        """
        Turn robot left by reversing left motor and forwarding right motor.

        :param speed: Speed of turn (0.0 to 1.0)
        """
        logger.info("Robot turning LEFT")
        self.motor_a.move("reverse", speed)
        self.motor_b.move("forward", speed)

    def right(self, speed: float = 0.5) -> None:
        """
        Turn robot right by forwarding left motor and reversing right motor.

        :param speed: Speed of turn (0.0 to 1.0)
        """
        logger.info("Robot turning RIGHT")
        self.motor_a.move("forward", speed)
        self.motor_b.move("reverse", speed)

    def stop(self) -> None:
        """
        Stop both motors, halting robot movement.
        """
        logger.info("Robot STOPPED")
        self.motor_a.stop()
        self.motor_b.stop()

