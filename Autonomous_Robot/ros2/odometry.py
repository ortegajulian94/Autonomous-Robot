#This code is necessary for creating a transformation between odom->base_link. 
#In order to use the localization packages a /odom topic must be created with an initial estimate for position change. 
#The SLAM localization technique that is used by ROS corrects the position estimate at a high resolution and is not infallible. 
#One of the major concerns with it is that the process associating landmarks in the view to landmarks in the map can go entirely off the rails, the better the initial estimate each time step the longer the algorithm can run without losing its place.
#This code is a hacky shortcut to get the localization packages to run. A preferred method would be to calculate odometry from encoder data. 
#This code subscribes to the /cmd_vel topic and assumes that the command to the wheels is the same as what the wheels actually did. It then calculates world frame change in pose from the twist message and publishes that to the /odom topic. It then calculates and publishes the transform between odom->base_link.
#All of this happens in a timer callback that is called at the frequency specified by the update_frequency parameter. 
#It keeps time with internal python time or the /clock topic depending on the use_sim_time boolean parameter.

# Authors: Eric Santana, Ethan Elgavish, Julian Ortega
# Odometry for vehicle localization from cmd_vel

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.exceptions import ParameterNotDeclaredException

import math
import time

from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster

def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q

class OdomPublisher(Node):

    def __init__(self, use_sim_time=False, update_frequency=100):
        
        super().__init__('odom_publisher')

        self.use_sim_time = use_sim_time
        
        self.twist_subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.twist_subscription

        self.twist = Twist()

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)

        if(self.use_sim_time):
            self.clock_subscription = self.create_subscription(Clock, 'clock', self.clock_callback, qos)
            self.clock_subscription
            self.clock = Clock().clock

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        self.timer_period = 1/update_frequency  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0
        self.y = 0
        self.th = 0

        

    def timer_callback(self):
        
        # distance traveled is the chord length of an arc
        dth = self.twist.angular.z * self.timer_period
        arc = self.twist.linear.x * self.timer_period
        chord_angle = self.th + dth/2

        if dth == 0:
            dist = arc
        else:
            dist = 2*arc*math.sin(dth/2)/dth

        dx = dist*math.cos(chord_angle)
        dy = dist*math.sin(chord_angle)

        self.x = self.x + dx
        self.y = self.y + dy
        self.th = self.th + dth

        robot_orientation = quaternion_from_euler(0, 0, self.th)
        
        if(self.use_sim_time):
            timestamp = self.clock
        else:
            timestamp = self.get_clock().now().to_msg()

        # transforms
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = '/odom'
        t.child_frame_id = '/base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = robot_orientation
        
        # odometry twist
        odom_msg = Odometry()
        odom_msg.header.frame_id = '/odom'
        odom_msg.child_frame_id = '/base_link'
        odom_msg.header.stamp = timestamp
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = robot_orientation
        odom_msg.twist.twist.linear.x = self.twist.linear.x
        odom_msg.twist.twist.angular.z = self.twist.angular.z

        # broadcast and publish
        self.tf_broadcaster.sendTransform(t)
        self.odom_publisher.publish(odom_msg)

    def twist_callback(self, twist: Twist):
        self.twist = twist

    def clock_callback(self, clock: Clock):
        self.clock = clock.clock
    
def main(args=None):
    rclpy.init(args=args)
    try:
        odom_publisher = OdomPublisher()
        rclpy.spin(odom_publisher)
    except rclpy.exceptions.ROSInterruptException:
        pass

    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
