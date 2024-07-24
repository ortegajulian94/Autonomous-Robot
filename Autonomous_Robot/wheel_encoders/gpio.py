# Reads gpio data from serial

from unittest import result
from serial.serialposix import Serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.exceptions import ParameterNotDeclaredException

import math
import serial
import sys

from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster




class Encoder_GPIO(Node):
    def __init__(self, use_sim_time=True, update_frequency=115200):
        super().__init__('encoder_gpio')

        self.use_sim_time = use_sim_time

        if (len(sys.argv) < 2):
            sys.exit(0)
        else:
            portName = sys.argv[1]
            gpioNum = sys.argv[2]

        self.rightCounter = 0
        self.totalCount = 0
        self.leftCounter = 0

        self.distance_per_count = ((2 * math.pi * .086) / 16384)
        self.gpio = serial.Serial('/dev/ttyACM1',230400,xonxoff=False,rtscts=False,dsrdtr=False)
        self.gpio.write(str.encode("gpio iodir 000000000000000F\n"))
        self.gpio.write(str.encode("gpio notify on\n"))
        self.gpio.write(str.encode("gpio read 00\n"))
        self.gpio.write(str.encode("gpio read 01\n"))
        self.gpio.write(str.encode("gpio read 02\n"))
        self.gpio.write(str.encode("gpio read 03\n"))


        self.timer_period = 1/update_frequency  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        


    def timer_callback(self):


        gpio_data = self.gpio.readline()

        if gpio_data != "" and len(gpio_data) > 1:
            decoded_data = str(gpio_data[0:len(gpio_data)].decode("utf-8"))
            self.get_logger().info("%s" % decoded_data)
            if decoded_data[1] == "#" and len(decoded_data) > 1:

                self.get_logger().info("%s" % decoded_data)
                self.totalCount += 1
                self.get_logger().info("totalCount: %d" % self.totalCount)


                s_current_val = "0x0" + decoded_data[9]
                s_prev_val = "0x0" + decoded_data[18]


                i_current_val = int(s_current_val,16)
                i_prev_val = int(s_prev_val,16)

                #self.get_logger().info("Current: %s:%d" % (s_current_val,i_current_val))
                #self.get_logger().info("Previous: %s:%d" % (s_prev_val,i_prev_val))
                
                
                Roldbits = i_prev_val & int("0x0C",16)
                Loldbits = i_prev_val & int("0x03",16)
                Rnewbits = i_current_val & int("0x0C",16)
                Lnewbits = i_current_val & int("0x03",16)

                
                if(Roldbits != Rnewbits):
                    self.rightCounter += 1
                

                if(Loldbits != Lnewbits):
                    self.leftCounter += 1

                self.get_logger().info("rightCounter: %d" % self.rightCounter)
                self.get_logger().info("leftCounter: %d" % self.leftCounter)
                #self.get_logger().info("%s" % hex(prev_val))


def main(args=None):
    rclpy.init(args=args)
    try:
        encoder_gpio = Encoder_GPIO()
        rclpy.spin(encoder_gpio)
    except rclpy.exceptions.ROSInterruptException:
        pass

    encoder_gpio.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
