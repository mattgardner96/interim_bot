import pigpio
import time
import threading

# Constants
ENCODER_PPR = 100  # Encoder pulses per revolution
PWM_FREQUENCY = 25000  # Fixed PWM frequency in Hz (25 kHz)
INITIAL_PWM_DUTY_CYCLE = 20  # Initial motor speed as a percentage (e.g., 20%)

# Set up pigpio
pi = pigpio.pi()

# GPIO pins
pwm_pin = 13
encoder_pin_a = 17
encoder_pin_b = 27

# Set up GPIO pins
pi.set_mode(pwm_pin, pigpio.OUTPUT)
pi.set_mode(encoder_pin_a, pigpio.INPUT)
pi.set_mode(encoder_pin_b, pigpio.INPUT)

# Initialize PWM
pi.set_PWM_frequency(pwm_pin, PWM_FREQUENCY)
# Inverted duty cycle: 0% duty cycle = full power, 100% duty cycle = no power
initial_duty_cycle = 255 * (100 - INITIAL_PWM_DUTY_CYCLE) / 100
pi.set_PWM_dutycycle(pwm_pin, initial_duty_cycle)

# Encoder variables
encoder_count = 0
last_time = time.time()

def encoder_callback(gpio, level, tick):
    global encoder_count
    encoder_count += 1

# Set up encoder callback
pi.callback(encoder_pin_a, pigpio.FALLING_EDGE, encoder_callback)

def update_duty_cycle(pi):
    """
    Function to update the PWM duty cycle.
    """
    while True:
        try:
            new_duty_cycle = float(input("Enter new PWM duty cycle (%): "))
            # Inverted duty cycle calculation
            inverted_duty_cycle = 255 * (100 - new_duty_cycle) / 100
            pi.set_PWM_dutycycle(pwm_pin, inverted_duty_cycle)
        except ValueError:
            print("Invalid input. Please enter a numeric value.")
        except KeyboardInterrupt:
            break

# Start a thread to handle user input for duty cycle updates
duty_cycle_thread = threading.Thread(target=update_duty_cycle, args=(pi,))
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
    # Clean up pigpio
    pi.set_PWM_dutycycle(pwm_pin, 0)  # Ensure PWM is stopped
    pi.stop()
