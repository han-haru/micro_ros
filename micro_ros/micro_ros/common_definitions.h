#ifndef COMMON_DEFINITIONS_H
#define COMMON_DEFINITIONS_H

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

void error_loop();

#define LEFT_MOTOR_PWM  19   // 왼쪽 모터 PWM 핀
#define LEFT_MOTOR_DIR1 18  // 왼쪽 모터 방향 핀 1
#define LEFT_MOTOR_DIR2 5  // 왼쪽 모터 방향 핀 2
#define RIGHT_MOTOR_PWM 4  // 오른쪽 모터 PWM 핀
#define RIGHT_MOTOR_DIR1 16 // 오른쪽 모터 방향 핀 1
#define RIGHT_MOTOR_DIR2 17 // 오른쪽 모터 방향 핀 2
#define LEFT_ENCODER_A 39  // 왼쪽 엔코더 A 채널
#define LEFT_ENCODER_B 36  // 왼쪽 엔코더 B 채널
#define RIGHT_ENCODER_A 35 // 오른쪽 엔코더 A 채널
#define RIGHT_ENCODER_B 34 // 오른쪽 엔코더 B 채널
#define WHEEL_SEPARATION 0.261  // 바퀴 간 거리 (30cm)
#define PPR 11                  // 엔코더 펄스
#define GEAR_RATIO 90           // 감속비
#define ENCODER_FACTOR (2.0 * M_PI / (PPR * GEAR_RATIO * 2)) // 각속도 변환 계수
#define R 0.04375 // 휠의 반지름 (m)
#define MAX_PWM 255 // 9v = 255, 12v = 240
#define MIN_PWM 0  // 모터가 동작하는 최소 PWM 값
#define EARTH_GRAVITY_MS2 9.80665  // m/s²
#define DEG_TO_RAD 0.017453292519943295769236907684886 //imu rad/s변환상수

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

extern geometry_msgs__msg__Twist msg;
extern my_custom_message__msg__Motor motor;
extern rcl_subscription_t subscriber;
extern sensor_msgs__msg__Imu imu;
extern rclc_executor_t executor;
extern rcl_allocator_t allocator;
extern rclc_support_t support;
extern rcl_publisher_t encoder_publisher, imu_publisher; 
extern rcl_node_t node;
extern rcl_timer_t timer;

extern float Kp_L, Ki_L, Kd_L;
extern float Kp_R, Ki_R, Kd_R;
extern float desired_left_speed, desired_right_speed;
extern float target_omega_left, target_omega_right;
extern float measured_w_left, measured_w_right;
extern float error_left, last_error_left, left_integral;
extern float error_right, last_error_right, right_integral;
extern float control_output_left, control_output_right;
extern int motor_pwm;
extern volatile int left_encoder_count, right_encoder_count;
extern unsigned long last_time_left, last_time_right;
extern float left_offset_count, right_offset_count;
extern float alpha, LPF_target_omega_left, LPF_target_omega_right;
extern float alpha_d, filtered_derivative_left, filtered_derivative_right;

extern MPU6050 mpu; 
extern Quaternion q;
extern VectorInt16 aa;       
extern VectorInt16 gg; 
extern bool DMPReady;
extern uint8_t devStatus;
extern uint16_t packetSize;
extern uint8_t FIFOBuffer[64];


#endif // COMMON_DEFINITIONS_H
