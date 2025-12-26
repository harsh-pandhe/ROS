#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>

// Definitions
#define BOOT_BUTTON_PIN 0
#define LED_PIN 2

// ROS Objects
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Bool stop_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

bool emergency_stop_active = false;

// Error handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Callback: Runs when Laptop sends a command to 'emergency_stop'
void stop_callback(const void * msgin)
{  
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  emergency_stop_active = msg->data;
}

void setup() {
  set_microros_transports();
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_safety_node", "", &support));

  // 1. Publisher (Controls Turtle)
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/turtle1/cmd_vel"));

  // 2. Subscriber (Listens for Emergency Stop)
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "emergency_stop"));

  // Executor handles both Publisher and Subscriber
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &stop_msg, &stop_callback, ON_NEW_DATA));

  // Initialize Twist
  twist_msg.linear.x = 0; twist_msg.linear.y = 0; twist_msg.linear.z = 0;
  twist_msg.angular.x = 0; twist_msg.angular.y = 0; twist_msg.angular.z = 0;
}

void loop() {
  // Check for incoming messages (Stop Signal)
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  if (emergency_stop_active) {
    // SAFETY MODE: Stop everything, turn on Warning LED
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    digitalWrite(LED_PIN, HIGH); // LED ON = DANGER/STOP
  } 
  else {
    // NORMAL MODE: Button controls turtle
    digitalWrite(LED_PIN, LOW); // LED OFF = Safe
    
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
      twist_msg.linear.x = 2.0;
      twist_msg.angular.z = 1.5;
    } else {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
    }
  }

  // Send the command
  RCSOFTCHECK(rcl_publish(&publisher, &twist_msg, NULL));
  delay(50);
}
