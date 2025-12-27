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

#define SERVO_PIN 18
#define SDA_PIN 21
#define SCL_PIN 22

rcl_publisher_t dist_pub;
rcl_publisher_t angle_pub;
std_msgs__msg__Int32 dist_msg;
std_msgs__msg__Int32 angle_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

Servo myServo;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

int currentAngle = 0;
int direction = 1;
unsigned long lastScan = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void setup() {
  set_microros_transports();

  // 1. Force I2C Pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // 2. Start Sensor (Keep trying until it works)
  // If this fails, the code will hang here (safer than sending garbage)
  while (!lox.begin()) {
    delay(100); 
  }

  // 3. Start Servo
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN);
  myServo.write(0);

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_radar", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &dist_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/radar/distance"));

  RCCHECK(rclc_publisher_init_default(
    &angle_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/radar/angle"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

void loop() {
  if (millis() - lastScan > 30) {
    lastScan = millis();

    // Move Servo
    currentAngle += (direction * 2);
    if (currentAngle >= 180) { currentAngle = 180; direction = -1; }
    if (currentAngle <= 0)   { currentAngle = 0;   direction = 1; }
    myServo.write(currentAngle);

    // Read Sensor
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    // Send Data
    angle_msg.data = currentAngle;
    if (measure.RangeStatus != 4) {
      dist_msg.data = measure.RangeMilliMeter;
    } else {
      dist_msg.data = 2000; // Out of range
    }

    rcl_publish(&angle_pub, &angle_msg, NULL);
    rcl_publish(&dist_pub, &dist_msg, NULL);
  }
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
