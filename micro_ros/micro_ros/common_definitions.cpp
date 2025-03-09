#include "common_definitions.h"
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
//motor 관련부
#include <geometry_msgs/msg/twist.h>
#include <my_custom_message/msg/motor.h>
//imu센서 관련부
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <sensor_msgs/msg/imu.h>

void error_loop(){
  while(1){
    Serial.println("not prepare yet");
    delay(100);
  }
}


geometry_msgs__msg__Twist msg;
my_custom_message__msg__Motor motor;
rcl_subscription_t subscriber;
sensor_msgs__msg__Imu imu;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_publisher_t encoder_publisher, imu_publisher; 
rcl_node_t node;
rcl_timer_t timer;

float Kp_L = 4.0;  
float Ki_L = 10.0;   
float Kd_L = 0.0;  

float Kp_R = 3.5; 
float Ki_R = 10.0;   
float Kd_R = 0.7;  

float desired_left_speed = 0.0;
float desired_right_speed = 0.0;

float target_omega_left = 0.0;  
float target_omega_right = 0.0;     // 목표 각속도 (rad/s)
float measured_w_left = 0.0;    
float measured_w_right = 0.0;      // 현재 측정된 바퀴 각속도 (rad/s)
float error_left = 0.0, last_error_left = 0.0, left_integral = 0.0;
float error_right = 0.0, last_error_right = 0.0, right_integral = 0.0;
float control_output_left = 0.0;
float control_output_right = 0.0;
int motor_pwm = 0;

volatile int left_encoder_count = 0;
volatile int right_encoder_count = 0;
unsigned long last_time_left = 0;
unsigned long last_time_right = 0;

float left_offset_count = 0.0;
float right_offset_count = 0.0;

float alpha = 0.4;  // 필터 계수 
float LPF_target_omega_left = 0.0;
float LPF_target_omega_right = 0.0;

float alpha_d = 0.7;  // D항 LPF 계수 
float filtered_derivative_left = 0.0;
float filtered_derivative_right = 0.0;

MPU6050 mpu; 
Quaternion q;
VectorInt16 aa;       
VectorInt16 gg; 
bool DMPReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];
