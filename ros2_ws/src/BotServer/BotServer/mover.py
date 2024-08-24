import rclpy
from rclpy.node import Node
import pigpio, os
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from motor import Motor_24H



class Mover(Node):
    def __init__(self):
        super().__init__('mover')
        
        try:
            os.system('sudo pigpiod') # start pigpio daemon
        except:
            assert RuntimeError("Failed to start pigpio daemon")
        
        self.pi = pigpio.pi()
        
        self.left_motor = Motor_24H(self.pi,13,26,17,27,0)
        self.right_motor = Motor_24H(self.pi,12,16,23,24,1)
        
        vel_cmd_srv = self.create_service(Twist, 'velocity_cmd', self.velocity_cmd_callback)
        
    
    def velocity_cmd_callback(self, request, response):
        self.left_motor.set_velocity(request.cmd_vel.linear)
        self.right_motor.set_velocity(request.right_velocity)
        response.success = True
        return response

def main():
    print('Hi from BotServer.')


if __name__ == '__main__':
    main()

