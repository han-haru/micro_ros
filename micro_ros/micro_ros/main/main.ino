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

#include "motor_control.h"
#include "encoder_handler.h"
#include "ros2_communication.h"
#include "common_definitions.h"

// **ESP32 초기 설정**
void setup() {
    //set_microros_wifi_transports((char*)"HOTPOT",(char*)"18123030", (char*)"192.168.25.124", 8888);
    //set_microros_wifi_transports("HY-DORM5-658","residence658", "192.168.0.8", 8888);
    //set_microros_wifi_transports("KT_GiGA_5D35","ahb66kz314", "172.30.1.58", 8888);
    set_microros_wifi_transports((char*)"I_phone",(char*)"dlgksrufdlek", (char*)"192.168.166.30", 8888);
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(100000); //100kHZ로 설정
    //imu 초기화 및 연결 확인코드
    delay(2000);
    Serial.println("initializing MPU6050");
    mpu.initialize();
    Serial.println("Testing MPU6050 connection...");
    if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed");
      while (1);
    }
    Serial.println("MPU6050 connection successful");
    Serial.println("Initializing DMP...");
    devStatus = mpu.dmpInitialize();
    // IMU 오프셋 설정
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
  
    if (devStatus == 0) {
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      Serial.println("DMP ready!");
      mpu.setDMPEnabled(true);
      DMPReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      Serial.println("DMP initialization failed!");
    }
      // IMU 메시지 초기화 frame_id설정 동적 string 동적 메모리할당 size,capacity지정해 주어야함.(c언어임으로.)
    imu.header.frame_id.data = (char *)"base_link";
    imu.header.frame_id.size = strlen("base_link");
    imu.header.frame_id.capacity = imu.header.frame_id.size + 1;
  
    // 모터 핀을 출력 모드로 설정
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(LEFT_MOTOR_DIR1, OUTPUT);
    pinMode(LEFT_MOTOR_DIR2, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR1, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR2, OUTPUT);
    //엔코더 핀모드
    pinMode(LEFT_ENCODER_A, INPUT);
    pinMode(LEFT_ENCODER_B, INPUT);
    pinMode(RIGHT_ENCODER_A, INPUT);
    pinMode(RIGHT_ENCODER_B, INPUT);
    //2체배 방식 채널 A의 rising, falling모두 인식
    attachInterrupt(LEFT_ENCODER_A, left_encoder_ISR, CHANGE);
    attachInterrupt(RIGHT_ENCODER_A, right_encoder_ISR, CHANGE);
    //offset();
  
    //micro_ros 설정 부분
    allocator = rcl_get_default_allocator();
  
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
    RCCHECK(rclc_subscription_init_default(&subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),"/cmd_vel"));
    //엔코더를 통한 속도값 퍼블리셔
    RCCHECK(rclc_publisher_init_default(&encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(my_custom_message, msg, Motor), "motor_topic"));
    //imu데이터 퍼블리셔
    RCCHECK(rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Imu), "/imu/data"));
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback));
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    //RCCHECK(rclc_executor_set_trigger(&executor,rclc_executor_trigger_one, &subscriber));
    Serial.println("micro-ROS node started.");
  }

void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
    updateleftControlLoop();
    updaterightControlLoop();
    delay(20);
}