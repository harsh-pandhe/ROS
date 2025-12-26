#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_VL53L0X.h>

// --- CONFIGURATION ---
const int SERVO_PIN = 18;
const int ANGLE_STEP = 2;
const int FALLBACK_DIST = 2000;

// --- ROS OBJECTS ---
rcl_publisher_t angle_pub;
rcl_publisher_t dist_pub;
std_msgs__msg__Int32 angle_msg;
std_msgs__msg__Int32 dist_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// --- SENSOR OBJECTS ---
Servo myServo;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// --- VARIABLES ---
int currentAngle = 0;
int direction = 1;
unsigned long lastScanTime = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void setup() {
  set_microros_transports();
  
  // 1. Setup Servo
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN);
  myServo.write(0);

  // 2. Setup Sensor (I2C)
  Wire.begin();
  delay(100);
  lox.begin(); // We ignore failure for now to keep the node alive

  // 3. Setup Micro-ROS
  delay(2000);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_radar", "", &support));

  // Publisher for Angle
  RCCHECK(rclc_publisher_init_default(
    &angle_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/radar/angle"));

  // Publisher for Distance
  RCCHECK(rclc_publisher_init_default(
    &dist_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/radar/distance"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

void loop() {
  // Move radar every 30ms
  if (millis() - lastScanTime > 30) {
    lastScanTime = millis();

    // 1. Move Servo
    currentAngle += (direction * ANGLE_STEP);
    if (currentAngle >= 180) { currentAngle = 180; direction = -1; }
    if (currentAngle <= 0)   { currentAngle = 0;   direction = 1; }
    myServo.write(currentAngle);

    // 2. Read Sensor
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    
    int dist = FALLBACK_DIST;
    if (measure.RangeStatus != 4) {
      dist = measure.RangeMilliMeter;
    }

    // 3. Publish Data
    angle_msg.data = currentAngle;
    dist_msg.data = dist;
    
    RCSOFTCHECK(rcl_publish(&angle_pub, &angle_msg, NULL));
    RCSOFTCHECK(rcl_publish(&dist_pub, &dist_msg, NULL));
  }

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
