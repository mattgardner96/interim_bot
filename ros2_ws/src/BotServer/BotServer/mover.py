import rclpy
from rclpy.node import Node
import pigpio

from motor import Motor_24H


class Mover(Node):
    def __init__(self):
        super().__init__('mover')
        pi = pigpio.pi()
        left_motor = Motor_24H(pi,13,26,17,27,-1)
        right_motor = Motor_24H(pi,12,16,23,24,1)
        vel_cmd_srv = self.create_service(Velocity, 'velocity_cmd', self.velocity_cmd_callback)
    
    def velocity_cmd_callback(self, request, response):
        left_motor.set_velocity(request.left_velocity)
        right_motor.set_velocity(request.right_velocity)
        response.success = True
        return response

def main():
    print('Hi from BotServer.')


if __name__ == '__main__':
    main()
