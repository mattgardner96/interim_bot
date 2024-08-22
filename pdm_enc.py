import RPi.GPIO as GPIO
import time

# Constants
ENCODER_PPR = 100  # Encoder pulses per revolution
PDM_FREQUENCY = 10000  # PDM pulse frequency in Hz
PDM_DUTY_CYCLE = 20  # Desired motor drive in percentage (e.g., 20%)

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for PDM and encoder
pdm_pin = 13
encoder_pin_a = 17
encoder_pin_b = 27

# Set up the GPIO pins
GPIO.setup(pdm_pin, GPIO.OUT, initial=GPIO.HIGH)  # Initialize PDM pin to HIGH (motor off)
GPIO.setup(encoder_pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder_pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Encoder variables
encoder_count = 0
last_time = time.time()

def encoder_callback(channel):
    global encoder_count
    encoder_count += 1

# Set up encoder callback without debounce
GPIO.add_event_detect(encoder_pin_a, GPIO.FALLING, callback=encoder_callback)

def pdm_drive(duty_cycle, frequency):
    """
    PDM motor drive function.
    """
    period = 1.0 / frequency
    on_time = duty_cycle / 100.0 * period
    off_time = period - on_time
    
    while True:
        # Drive the motor (active low)
        GPIO.output(pdm_pin, GPIO.LOW)  # Motor on
        time.sleep(on_time)
        GPIO.output(pdm_pin, GPIO.HIGH)  # Motor off
        time.sleep(off_time)

try:
    # Start PDM drive with the specified duty cycle and frequency
    pdm_drive(PDM_DUTY_CYCLE, PDM_FREQUENCY)
    
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
    # Stop and clean up GPIO
    GPIO.output(pdm_pin, GPIO.HIGH)  # Ensure motor is off (pin HIGH)
    GPIO.cleanup()
