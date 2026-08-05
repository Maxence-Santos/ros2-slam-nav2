#!/usr/bin/env python3
"""
Dynamic Actors Controller Node.
Publishes velocity commands to person_1, person_2, and person_3
to make them walk continuously back and forth across the warehouse.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class DynamicActorsNode(Node):
    def __init__(self):
        super().__init__(
            'dynamic_actors_controller',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)]
        )

        # Publishers for each dynamic person
        self.pub1 = self.create_publisher(Twist, '/model/person_1/cmd_vel', 10)
        self.pub2 = self.create_publisher(Twist, '/model/person_2/cmd_vel', 10)
        self.pub3 = self.create_publisher(Twist, '/model/person_3/cmd_vel', 10)

        # 20Hz motion update loop
        self.create_timer(0.05, self.update_motion)
        self.t = 0.0

        self.get_logger().info('🚶 Dynamic Actors Controller started! Moving person_1, person_2, person_3...')

    def update_motion(self):
        self.t += 0.05

        # Person 1: Walks back and forth along Y axis (period 20s, speed 0.20 m/s)
        cmd1 = Twist()
        phase1 = math.sin(2.0 * math.pi * self.t / 20.0)
        cmd1.linear.x = 0.20 * (1.0 if phase1 > 0 else -1.0)
        self.pub1.publish(cmd1)

        # Person 2: Walks back and forth along X axis (period 24s, speed 0.20 m/s)
        cmd2 = Twist()
        phase2 = math.sin(2.0 * math.pi * self.t / 24.0)
        cmd2.linear.x = 0.20 * (1.0 if phase2 > 0 else -1.0)
        self.pub2.publish(cmd2)

        # Person 3: Walks back and forth along Y axis inverted (period 20s, speed 0.20 m/s)
        cmd3 = Twist()
        phase3 = math.cos(2.0 * math.pi * self.t / 20.0)
        cmd3.linear.x = 0.20 * (1.0 if phase3 > 0 else -1.0)
        self.pub3.publish(cmd3)

def main():
    rclpy.init()
    node = DynamicActorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
