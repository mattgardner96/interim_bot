import RPi.GPIO as GPIO
import time
import threading

# Constants
ENCODER_PPR = 100  # Encoder pulses per revolution
PWM_FREQUENCY = 25000  # Fixed PWM frequency in Hz (25 kHz)
INITIAL_PWM_DUTY_CYCLE = 20  # Initial motor speed as a percentage (e.g., 20%)

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for PWM and encoder
pwm_pin = 13  # PWM output to motor driver
encoder_pin_a = 17
encoder_pin_b = 27

# Set up the GPIO pins
GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(encoder_pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder_pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize PWM
pwm = GPIO.PWM(pwm_pin, PWM_FREQUENCY)
pwm.start(INITIAL_PWM_DUTY_CYCLE)  # Start PWM with the initial duty cycle

# Encoder variables
encoder_count = 0
last_time = time.time()

def encoder_callback(channel):
    global encoder_count
    encoder_count += 1

# Set up encoder callback
GPIO.add_event_detect(encoder_pin_a, GPIO.FALLING, callback=encoder_callback)

def update_duty_cycle(pwm):
    """
    Function to update the PWM duty cycle.
    """
    while True:
        try:
            new_duty_cycle = float(input("Enter new PWM duty cycle (%): "))
            pwm.ChangeDutyCycle(new_duty_cycle)
        except ValueError:
            print("Invalid input. Please enter a numeric value.")
        except KeyboardInterrupt:
            break

# Start a thread to handle user input for duty cycle updates
duty_cycle_thread = threading.Thread(target=update_duty_cycle, args=(pwm,))
duty_cycle_thread.daemon = True
duty_cycle_thread.start()

try:
    while True:
        # Calculate RPM
        current_time = time.time()
        elapsed_time = current_time - last_time
        last_time = current_time
        revolutions = encoder_count / ENCODER_PPR
        rpm = (revolutions / elapsed_time) * 60  # Convert to RPM
        encoder_count = 0

        # Print RPM
        print(f"RPM: {rpm:.2f}")

        # Wait for a short period before updating
        time.sleep(1)

finally:
    # Stop PWM and clean up GPIO
    pwm.stop()
    GPIO.cleanup()
