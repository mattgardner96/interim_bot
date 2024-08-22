# motor class for controlling the motors of the robot
# also reads encoders for closed loop velocity control
# FOR MY MOTORS, PWM DUTY CYCLES ARE INVERTED
# 0% duty cycle is full speed forward

import pigpio
import time

ENCODER_PPR=100 # 100 pulses per revolution
PWM_FREQ=25000 # 25kHz PWM frequency
PWM_FULL_SCALE = 255 # 8-bit PWM
WHEEL_DIAMETER = 0.064 # wheel diameter in meters

class Motor_24H():
    
    pi = None # this is the pigpio.pi() object
    
    def __init__(self, rpi, pwm_port:int, dir_port:int, enc_a_port:int, enc_b_port:int, forward_dir:int):
        '''
        rpi: pigpio.pi() object for controlling GPIO pins to drive motors
        pwm_pin: BCM pin number for PWM signal
        dir_pin: BCM pin number for direction signal
        enc_a_pin: BCM pin number for encoder channel A
        enc_b_pin: BCM pin number for encoder channel B
        forward_dir: 1 for CW, 0 for CCW
        '''
        
        init_done = False
        
        # set pins
        self.pi = rpi
        self.pwm_pin   = pwm_port
        self.dir_pin   = dir_port
        self.enc_a_pin = enc_a_port
        self.enc_b_pin = enc_b_port
        self.forward_dir = forward_dir
        
        # init the pins and their init states
        self.pi.set_mode(self.pwm_pin, pigpio.ALT0)
        self.pi.set_mode(self.dir_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.enc_a_pin, pigpio.INPUT)
        self.pi.set_mode(self.enc_b_pin, pigpio.INPUT)
        
        # init PWM to off
        self.pi.set_PWM_range(self.pwm_pin,PWM_FULL_SCALE)
        self.pi.set_PWM_frequency(self.pwm_pin, PWM_FREQ)
        self.pi.write(self.pwm_pin, PWM_FULL_SCALE) # inverted; this is actually off
        
        # init direction to forward
        self.pi.write(self.dir_pin, self.forward_dir)
        
    def set_duty_inv(self,duty:float)->bool:
            '''
            duty percent, not inverted
            '''
            duty = 255 * (100 - duty) / 100
            
            return self.set_PWM_dutycycle(self.pwm_pin, duty)
        
    def get_duty_inv(self)->float:
        '''
        returns the inverted duty cycle as a percentage
        '''
        duty = self.pi.get_PWM_dutycycle(self.pwm_pin)
        return 100 - (duty / 255 * 100)
        
    
    def set_velocity(self, velocity:float):
        '''
        velocity: velocity in m/s
        '''
        pass
        
    def __del__(self):
        self.pi.set_PWM_dutycycle(self.pwm_pin, 255) # inverted; this is actually off

def main():
    pi = pigpio.pi()
    left_motor = Motor_24H(pi,13,26,17,27,0)
    right_motor = Motor_24H(pi,12,16,23,24,1)
    
    left_motor.set_duty_inv(5)
    right_motor.set_duty_inv(5)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        left_motor.set_duty_inv(0)
        right_motor.set_duty_inv(0)
    

if __name__ == '__main__':
    main()
        
        
        
        
    
    