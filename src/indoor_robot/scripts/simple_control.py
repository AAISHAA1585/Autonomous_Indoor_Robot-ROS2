
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios

class SimpleControl(Node):
    def __init__(self):
        super().__init__('simple_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Simple Control Started!')
        self.get_logger().info('Controls:')
        self.get_logger().info('  W: Forward')
        self.get_logger().info('  S: Backward')
        self.get_logger().info('  A: Turn Left')
        self.get_logger().info('  D: Turn Right')
        self.get_logger().info('  SPACE: Stop')
        self.get_logger().info('  Q: Quit')
        
    def send_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)
        
    def stop(self):
        self.send_velocity(0.0, 0.0)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rclpy.init()
    node = SimpleControl()
    
    settings = termios.tcgetattr(sys.stdin)
    
    speed = 0.3  # Linear speed
    turn = 0.5   # Angular speed
    
    try:
        while True:
            key = get_key(settings)
            
            if key.lower() == 'w':
                node.send_velocity(speed, 0.0)
                node.get_logger().info('Forward')
            elif key.lower() == 's':
                node.send_velocity(-speed, 0.0)
                node.get_logger().info('Backward')
            elif key.lower() == 'a':
                node.send_velocity(0.0, turn)
                node.get_logger().info('Turn Left')
            elif key.lower() == 'd':
                node.send_velocity(0.0, -turn)
                node.get_logger().info('Turn Right')
            elif key == ' ':
                node.stop()
                node.get_logger().info('STOPPED')
            elif key.lower() == 'q':
                node.stop()
                break
            else:
                node.stop()
                
    except Exception as e:
        print(e)
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
