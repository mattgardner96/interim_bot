import RPi.GPIO as GPIO
import time

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the GPIO pin for PWM
pwm_pin = 13

# Set up the GPIO pin as an output
GPIO.setup(pwm_pin, GPIO.OUT)

# Initialize the pin to high
GPIO.output(pwm_pin, GPIO.HIGH)

# Set up PWM with a frequency of 10 kHz
pwm_frequency = 10000  # 10 kHz
pwm = GPIO.PWM(pwm_pin, pwm_frequency)

# Start PWM with a 20% duty cycle (inverted)
duty_cycle = 1  # Inverted duty cycle (0 means fully on, 100 means fully off)
pwm.start(100 - duty_cycle)

try:
    # Spin the motor for 10 seconds
    time.sleep(2)

finally:
    # Stop PWM and set the pin to high
    pwm.stop()
    GPIO.output(pwm_pin, GPIO.HIGH)
    GPIO.cleanup()


