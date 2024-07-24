#The purpose of this code is to ‘sync’ our nodes by taking advantage of ROS2’s /clock topic. 
# This code creates a node that publishes time to the /clock topic, which every node is subscribed to because of our launch file parameters. 
# Nodes will use this time as their timestamp when constructing messages.
# Authors: Eric Santana, Ethan Elgavish, Julian Ortega
# Clock sim time publisher

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.exceptions import ParameterNotDeclaredException

from rosgraph_msgs.msg import Clock

class SimTimePublisher(Node):

    def __init__(self, update_frequency=100):
        
        super().__init__('sim_time_publisher')

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

        self.sim_time_publisher = self.create_publisher(Clock, 'clock', qos_profile = qos)

        self.zero_time = self.get_clock().now()

        self.timer_period = 1/update_frequency  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        clock_msg = Clock()
        current_time = self.get_clock().now()

        duration = current_time - self.zero_time

        elapsed_time = Time()
        temp_time = Time()
        elapsed_time = temp_time + duration

        clock_msg = Clock()
        clock_msg.clock = elapsed_time.to_msg()
        self.sim_time_publisher.publish(clock_msg)
    
def main(args=None):
    rclpy.init(args=args)
    try:
        sim_time_publisher = SimTimePublisher()
        rclpy.spin(sim_time_publisher)
    except rclpy.exceptions.ROSInterruptException:
        pass

    sim_time_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
