#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Instructions for the user
msg = """
Control Your Robot!
---------------------------
Moving around:
       w
   a   s   d
       x

w/s : move forward/backward
a/d : turn left/right

space key, x : force stop

CTRL-C to quit
"""

# Dictionary to map key presses to velocities
moveBindings = {
    'w': (1, 0),  # Forward
    's': (-1, 0), # Backward
    'a': (0, 1),  # Turn Left
    'd': (0, -1), # Turn Right
    'x': (0, 0),  # Stop
    ' ': (0, 0),  # Stop
}

# Helper function to get a single key press
def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class RobotKeyTeleop(Node):
    def __init__(self):
        super().__init__('robot_key_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.speed = 0.2
        self.turn = 0.5
        
        self.settings = termios.tcgetattr(sys.stdin)

        self.run_loop()

    def run_loop(self):
        print(msg)
        while rclpy.ok():
            key = getKey(self.settings)

            # Only publish and print if a key is pressed
            if key:
                if key in moveBindings.keys():
                    linear_vel = moveBindings[key][0]
                    angular_vel = moveBindings[key][1]
                    
                    twist = Twist()
                    twist.linear.x = linear_vel * self.speed
                    twist.angular.z = angular_vel * self.turn
                    self.publisher_.publish(twist)

                    # Print current velocities only when a command is sent
                    print(f"Linear Vel: {twist.linear.x:.2f}, Angular Vel: {twist.angular.z:.2f}")

                elif (key == '\x03'): # CTRL-C
                    break

def main(args=None):
    rclpy.init(args=args)
    node = RobotKeyTeleop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()