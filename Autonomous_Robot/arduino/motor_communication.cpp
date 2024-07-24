/* Author: Julian Ortega */
/* Alphabot's Arduino MEGA core functions. This file is called directly upon bot_init from Raspberry Pi */

// Libraries
#include <ros.h> /* Line 62 ros.h changed to "#define SERIAL_CLASS USBSerial" from "#define SERIAL_CLASS HardwareSerial" */
#include <ZLTech.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>

// Dimensional constants
#define WHEEL_RADIUS 0.08255 /*Meters*/
#define AXIS_RADIUS 0.1905 /*Meters*/
#define PI 3.1415927

// ROS node initialization and msgs variables
ros::NodeHandle node_handle;
std_msgs::Bool init_success;
std_msgs::Float32 current_velocity_left;
std_msgs::Float32 current_velocity_right;
geometry_msgs::Twist command;

// Motor controller driver
ZLTech drv;

// ROS publishers
ros::Publisher init_publisher("init_success", &init_success);
ros::Publisher curr_velocity_left("velocity_left", &current_velocity_left);
ros::Publisher curr_velocity_right("velocity_right", &current_velocity_right);

void updateEncoders(){}

// cmd_vel velocity callback
void velocityCallback(const geometry_msgs::Twist& command)
{
  //drv.setMode(1);
  float wheel_radius = WHEEL_RADIUS;
  float axis_radius = AXIS_RADIUS;
  float rads_to_revm = (60)/(2*PI);
  float linear_x = command.linear.x; // assume [m/s]
  float angular_z = command.angular.z; // assume [rad/s]

  // [rad/s]
  float velocity_left = ((linear_x/wheel_radius) - ((angular_z*axis_radius)/(wheel_radius)));
  float velocity_right = (-(linear_x/wheel_radius) - ((angular_z*axis_radius)/(wheel_radius)));
  
  // [rad/s] -> [rev/min]
  velocity_left = velocity_left*rads_to_revm;
  velocity_right = velocity_right*rads_to_revm;

  // Send velocity instructions to motor controller (left, right)
  drv.targetVelocity((signed int)velocity_right, 0);
  drv.targetVelocity((signed int)velocity_left, 1);
  
  // Posting current velocity from encoders
  updateEncoders();
  current_velocity_left.data = (signed int)(velocity_left);
  current_velocity_right.data = (signed int)(velocity_right);
  curr_velocity_left.publish(&current_velocity_left);
  curr_velocity_right.publish(&current_velocity_right);
}
ros::Subscriber<geometry_msgs::Twist> velocity_subscriber("cmd_vel", &velocityCallback);

void setup() 
{
  // ROS initialization for subscribers and publishers
  node_handle.initNode();
  node_handle.advertise(init_publisher);
  node_handle.advertise(curr_velocity_left);
  node_handle.advertise(curr_velocity_right);
  node_handle.subscribe(velocity_subscriber);

  // Motor controller initialization, initiate velocity mode
  drv.initialize(115200);
  drv.setMode(1);
  node_handle.spinOnce();
}

void loop() 
{
  node_handle.spinOnce();
  delay(10);
}
ROS2 (To be tested)
#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <ZLTech.h>

#include <stdio.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// Dimensional constants
#define WHEEL_RADIUS 0.08255 /*Meters*/
#define AXIS_RADIUS 0.1905 /*Meters*/
#define PI 3.1415927

ZLTech drv;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
void subscription_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  float wheel_radius = WHEEL_RADIUS;
  float axis_radius = AXIS_RADIUS;
  float rads_to_revm = (60)/(2*PI);
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;

   // [rad/s]
  float velocity_left = ((linear_x/wheel_radius) - ((angular_z*axis_radius)/(wheel_radius)));
  float velocity_right = (-(linear_x/wheel_radius) - ((angular_z*axis_radius)/(wheel_radius)));
  
  // [rad/s] -> [rev/min]
  velocity_left = velocity_left*rads_to_revm;
  velocity_right = velocity_right*rads_to_revm;

  drv.targetVelocity((signed int)velocity_right, 0);
  drv.targetVelocity((signed int)velocity_left, 1);
}

int main()
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create executor
  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    rclc_executor_spin(&executor);

  RCCHECK(rcl_subscription_fini(&subscriber, &node));
  RCCHECK(rcl_node_fini(&node));
}
