# motor class for controlling the motors of the robot
# also reads encoders for closed loop velocity control
# FOR MY MOTORS, PWM DUTY CYCLES ARE INVERTED
# 0% duty cycle is full speed forward

import pigpio
import time
import os
import threading



class Motor_24H():
    
    ENCODER_PPR=100 # 100 pulses per revolution
    PWM_FREQ=25000 # 25kHz PWM frequency
    PWM_FULL_SCALE = 255 # 8-bit PWM
    WHEEL_DIAMETER = 0.064 # wheel diameter in meters
    MAX_RPM = 3035 # max RPM of the motor, no load
    DEFAULT_ACCEL_RPMPS = 200 # default acceleration in RPM/s
      
    def __init__(self, rpi, pwm_port:int, dir_port:int, enc_a_port:int, enc_b_port:int, forward_dir:int, wheel_diam=WHEEL_DIAMETER):
        '''
        rpi: pigpio.pi() object for controlling GPIO pins to drive motors
        pwm_pin: BCM pin number for PWM signal
        dir_pin: BCM pin number for direction signal
        enc_a_pin: BCM pin number for encoder channel A
        enc_b_pin: BCM pin number for encoder channel B
        forward_dir: 1 for CW, 0 for CCW
        '''
        
        self.pi = rpi
        self.pwm_pin   = pwm_port
        self.dir_pin   = dir_port
        self.enc_a_pin = enc_a_port
        self.enc_b_pin = enc_b_port
        self.forward_dir = forward_dir
        self.wheel_diam = wheel_diam
        
        # init the pins and their init states
        self.pi.set_mode(self.pwm_pin, pigpio.ALT0)
        self.pi.write(self.pwm_pin, 1) # inverted; this is actually off
        self.pi.set_mode(self.dir_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.enc_a_pin, pigpio.INPUT)
        self.pi.set_mode(self.enc_b_pin, pigpio.INPUT)
        
        # init PWM to off
        self.pi.set_PWM_range(self.pwm_pin,self.PWM_FULL_SCALE)
        self.pi.set_PWM_frequency(self.pwm_pin, self.PWM_FREQ)
        self.pi.set_PWM_dutycycle(self.pwm_pin, self.PWM_FULL_SCALE) # inverted; this is actually off
        
        # init direction to forward
        self.pi.write(self.dir_pin, self.forward_dir)
        
        # init encoder variables
        self.encoder_count = 0
        self.last_time = time.time()
        self.pi.callback(self.enc_a_pin, pigpio.FALLING_EDGE, self.__encoder_callback)
        # self.pi.callback(self.enc_b_pin, pigpio.FALLING_EDGE, self.__encoder_callback)

    def __set_duty(self,duty:float)->bool:
            '''
            duty percent, not inverted
            '''
            duty = 255 * (100 - duty) / 100
            
            return self.pi.set_PWM_dutycycle(self.pwm_pin, duty)
        
    def __get_duty(self)->float:
        '''
        returns the inverted duty cycle as a percentage
        '''
        duty = self.pi.get_PWM_dutycycle(self.pwm_pin)
        return 100 - (duty / 255 * 100)
    
    def __encoder_callback(self, gpio, level, tick):
        '''
        callback function for encoder pulses
        '''
        self.encoder_count += 1
    
    def get_curr_vel(self)->float:
        '''
        returns the current angular velocity in RPM
        takes values directly from the encoder
        '''
        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time
        delta_count = self.encoder_count
        self.encoder_count = 0
        
        # TODO handle polarity when using both encoder channels
        return delta_count / self.ENCODER_PPR / delta_time * 60
    
    def stop(self):
        self.__set_duty(0)
        
    def set_velocity(self, velocity_rpm:float, accel_rpm_ps:float=200):
        '''
        velocity_rpm: target angular velocity in RPM
        accel_rpm_ps: acceleration in RPM/s (default 200)
        '''
        self.velocity_rpm = velocity_rpm
        
        if velocity_rpm == 0:
            self.stop()
            return
        
        # set direction
        self.pi.write(self.dir_pin,
                      self.forward_dir if velocity_rpm > 0 else abs(self.forward_dir - 1))
        # set duty cycle
        self.__set_duty(abs(velocity_rpm) / self.MAX_RPM * 100)
        
    def get_setpoint_vel(self)->float:
        '''
        returns the current angular velocity in RPM
        '''
        return self.velocity_rpm
        
    def cl_vel_ctrl(self, setpoint_vel:float, accel_rpm_ps:float=200):
        '''
        setpoint_vel: target angular velocity in RPM
        accel_rpm_ps: acceleration in RPM/s (default 200)
        '''
        current_vel = self.get_curr_vel()
        if setpoint_vel == 0:
            self.stop()
            return
        
        if setpoint_vel > current_vel:
            while current_vel < setpoint_vel:
                try:
                    current_vel = self.get_curr_vel()
                    self.set_velocity(current_vel + accel_rpm_ps)
                    time.sleep(0.1)
                except KeyboardInterrupt:
                    self.stop()
                    return
        else:
            while current_vel > setpoint_vel:
                try:
                    current_vel = self.get_curr_vel()
                    self.set_velocity(current_vel - accel_rpm_ps)
                    time.sleep(0.1)
                except KeyboardInterrupt:
                    self.stop()
                    return
    
    # def __del__(self): # TODO destructor doesn't work
    #     self.stop()

if __name__ == '__main__':
    try:
        os.system('sudo pigpiod') # start pigpio daemon
    except:
        assert RuntimeError("Failed to start pigpio daemon")
    
    pi = pigpio.pi()
    left_motor = Motor_24H(pi,13,26,17,27,0)
    right_motor = Motor_24H(pi,12,16,23,24,1)
    
    left_motor.set_velocity(velocity_rpm=60)
    right_motor.set_velocity(velocity_rpm=60)
    
    def speed_print_daemon():
        while True:
            print(f"Left:{round(left_motor.get_curr_vel(),2)}/Right:{round(right_motor.get_curr_vel(),2)}")
            time.sleep(1)
    
    speed_thread = threading.Thread(target=speed_print_daemon)
    speed_thread.daemon = True
    speed_thread.start()
    
    try:
        while(True):
            time.sleep(1)
    except KeyboardInterrupt:
        left_motor.stop()
        right_motor.stop()
        pi.stop()
