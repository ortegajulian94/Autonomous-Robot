This is the code we used to initialize both wheel encoders, drive the bot 3 meters, and calculate velocity, distance, and total encoder count.
#include <Arduino_PortentaBreakout.h>

#include <micro_ros_arduino.h>
#include <Arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <mbed.h>
#include <rtos.h>


#include <geometry_msgs/msg/twist.h>

#include <AlphabotEncoder.h>
#include <ZLTech.h>

//===========================================================================================
//begin: test 'new'
#include <stdlib.h> // for malloc and free
//void *operator new (size_t size, void *ptr) { return ptr; }

static uint8_t RAMleftObj[ sizeof(AlphabotEncoder) ];
static uint8_t RAMrightObj[ sizeof(AlphabotEncoder) ];
AlphabotEncoder *leftEncoder;
AlphabotEncoder *rightEncoder;


//end: test 'new'
//===========================================================================================

// Dimensional constants
#define WHEEL_RADIUS 0.086 /*Meters*/
#define AXIS_RADIUS 0.1985 /*Meters*/
#define rads_to_revm (60)/(2*PI)

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

ZLTech drv;
int g_count;
#define LED_PIN LEDG

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.println("Error in RCCHECK");}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.println("Error in RCSOFTCHECK");}}

// Rotary Encoder Inputs
#define AoutL PWM4   //Port J4: Pin 1: Blue
#define BoutL PWM5   //Port J4: Pin 2: White
#define AoutR PWM6   //Port J4: Pin 3: Black
#define BoutR PWM7   //Port J4: Pin 4: Red

//begin: test 'new'
//AlphabotEncoder leftEncoder = AlphabotEncoder();
//AlphabotEncoder rightEncoder = AlphabotEncoder();

//AlphabotEncoder *leftEncoder;
//AlphabotEncoder *rightEncoder;
//C* c; // declare variable
//end: test 'new'

//===========================================================================================
//
void LChangeA(){
  if(digitalRead(AoutL) == digitalRead(BoutL)) {
    leftEncoder->count--;
  } else {
    leftEncoder->count++;
  }
  
}

void LChangeB(){
   if(digitalRead(AoutL) != digitalRead(BoutL)) {
    leftEncoder->count--;
  } else {
    leftEncoder->count++;
  }
}

void RChangeA(){
  //rightEncoder.onChangeA();
  if(digitalRead(AoutR) == digitalRead(BoutR)) {
    rightEncoder->count--;
  } else {
    rightEncoder->count++;
  }
}

void RChangeB(){
  //rightEncoder.onChangeB();
  if(digitalRead(AoutR) != digitalRead(BoutR)) {
    rightEncoder->count--;
  } else {
    rightEncoder->count++;
  }
}

//===========================================================================================
//
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//===========================================================================================
//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;
  
   // [rad/s]
  float velocity_right = ((linear_x/WHEEL_RADIUS) - ((angular_z*AXIS_RADIUS)/(WHEEL_RADIUS)));
  float velocity_left = (-(linear_x/WHEEL_RADIUS) - ((angular_z*AXIS_RADIUS)/(WHEEL_RADIUS)));
  
  // [rad/s] -> [rev/min]
  velocity_left = velocity_left*rads_to_revm;
  velocity_right = velocity_right*rads_to_revm;

  drv.ZLTech::targetVelocity((signed int)velocity_right, 0);
  drv.ZLTech::targetVelocity((signed int)velocity_left, 1);
  //Serial.print("Velocity left: ");Serial.println((signed int)velocity_left);
  //Serial.print("Velocity right: ");Serial.println((signed int)velocity_right);
}

//===========================================================================================
//
// SETUP()
//
//===========================================================================================
void setup() {

  leftEncoder = new (RAMleftObj) AlphabotEncoder();
  rightEncoder = new (RAMrightObj) AlphabotEncoder();
  leftEncoder->positionn = 0.0;
  rightEncoder->positionn = 0.0;
  leftEncoder->sum = 0;
  rightEncoder->sum = 0;
  leftEncoder->velocity = 0.0;
  rightEncoder->velocity = 0.0;
  
  LL_RCC_ForceCM4Boot();
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  delay(2000);
  g_count = 10;
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  delay(500);
  // Motor controller initialization
  drv.ZLTech::initialize(115200); // init baud rate
  drv.ZLTech::setMode(1); // velocity mode
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  //
  // Configure the MotorController's Encoder
  //
  Serial.println("Initializing left");
//note: pulled code from class (in the 2 lines below) and put inline below these 2 rows.
//  leftEncoder.initialize(AoutL, BoutL, LChangeA, LChangeB);
//  rightEncoder.initialize(AoutR, BoutR, RChangeA, RChangeB);

  //==============================
  //configure left encoder
  //==============================
  
  leftEncoder->tmpVar = 2;    //test class obj created.
  leftEncoder->pinA = AoutL;
  leftEncoder->pinB = BoutL;
  // configure pins and attach interrupts
  pinMode(AoutL, INPUT);
  delay(500);
  pinMode(BoutL, INPUT);
  delay(500);
  Serial.println(AoutL);
  Serial.println(BoutL);
  delay(500);
  attachInterrupt(digitalPinToInterrupt(AoutL), LChangeA, CHANGE);
  delay(500);
  attachInterrupt(digitalPinToInterrupt(BoutL), LChangeB, CHANGE);
  delay(500);

  leftEncoder->prevTime = millis();
  delay(500);
  
  //==============================
  //configure right encoder
  //==============================
  rightEncoder->tmpVar = 4;    //test class obj created.
  rightEncoder->pinA = AoutR;
  rightEncoder->pinB = BoutR;

  // configure pins and attach interrupts
  pinMode(AoutR, INPUT);
  delay(500);
  
  pinMode(BoutR, INPUT);
  delay(500);
  //attachInterrupt(digitalPinToInterrupt(AoutR), RChangeA, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(BoutR), RChangeB, CHANGE);
  rightEncoder->prevTime = millis();
  //Serial.println("+-+-+ G");
  
  delay(500);
  Serial.println("SETUP left encorder tmpVar = ");
  delay(500);
  Serial.println(leftEncoder->tmpVar);
  delay(500);
  Serial.println("SETUP right encorder tmpVar = ");
  delay(500);
  Serial.println(rightEncoder->tmpVar);
  delay(500);
  Serial.println("leftEncoderCount: ");
  Serial.print(leftEncoder->tempCount);
  Serial.println("rightEncoderCount: ");
  Serial.println(rightEncoder->tempCount);
  float linear_x = -0.03; // assume [m/s]
  float angular_z = 0.0; // assume [rad/s]
  
  // [rev/m]
  float velocity_left = ((linear_x/WHEEL_RADIUS) - ((angular_z*AXIS_RADIUS)/(WHEEL_RADIUS))) * rads_to_revm;
  float velocity_right = (-(linear_x/WHEEL_RADIUS) - ((angular_z*AXIS_RADIUS)/(WHEEL_RADIUS))) * rads_to_revm;

  Serial.println("Moving!");
  drv.targetVelocity((signed int)velocity_right, 0);
  drv.targetVelocity((signed int)velocity_left, 1);

  // stop driving after x seconds
  for(int i = 0; i < 110; i++) {
    delay(1000); 
    leftEncoder->updateEncoder();
    rightEncoder->updateEncoder();
    Serial.println("leftEncoderCount: ");
    Serial.println(leftEncoder->tempCount);
    Serial.println("rightEncoderCount: ");
    Serial.println(rightEncoder->tempCount);
    Serial.println("leftEncoder Velocity: ");
    Serial.println(leftEncoder->getVelocity());
    Serial.println("rightEncoder Velocity: ");
    Serial.println(rightEncoder->getVelocity());
    Serial.println(leftEncoder->getPosition());
    Serial.println(rightEncoder->getPosition());
     
  }

  drv.stop();
  leftEncoder->updateEncoder();
  rightEncoder->updateEncoder();
  Serial.println(leftEncoder->getPosition());
  Serial.println(rightEncoder->getPosition());
}

void loop() {
  //delay(100);
  //RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  //leftEncoder->updateEncoder();
  //rightEncoder->updateEncoder();
  
  //Serial.println(g_count);
  //Serial.println(leftEncoder.getVelocity());
  //Serial.print("AoutL: "); Serial.print(AoutL); Serial.print(" | BoutL: "); Serial.println(BoutL);

}
