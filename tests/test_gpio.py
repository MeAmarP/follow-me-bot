# gpio_toggle_test_gpiozero.py

from gpiozero import LED
from time import sleep

# Use any GPIO pin, e.g., GPIO18
pin = LED(18)

print("Toggling GPIO18 every 500ms. Press Ctrl+C to stop.")

try:
    while True:
        pin.toggle()     # Flip the current state
        sleep(0.5)        # 500 ms delay
except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    pin.off()
    print("GPIO cleanup done.")
