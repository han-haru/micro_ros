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
#define SPEED_CORRECTION_KP 0.1 //오른쪽 모터 속도 보정 값(PID 제어에서 P만)

#define EARTH_GRAVITY_MS2 9.80665  // m/s²
#define DEG_TO_RAD 0.017453292519943295769236907684886 //imu rad/s변환상수

//엔코더 관련 변수 선언
volatile int left_pulse_count = 0;
volatile int right_pulse_count = 0;
//imu 관련 변수 선언
bool DMPReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

MPU6050 mpu; //이 부분은 라이브러리 코드를 참고해봐야 될듯싶다.
//delta t를 위한 변수 선언
unsigned long last_time = 0;

VectorInt16 aa;   // Accel sensor measurements
VectorInt16 gg;   // Gyro sensor measurements

// Offset mode 선택
String offsetMode = "manual"; // 기본 모드 manual로 설정
// 수동 offset 값 입력부
// 현재 Red light imu에 맞춰둠
float offsetXAccel = -0.414487395;
float offsetYAccel = -0.101733549;
float offsetZAccel = 10.938856883;
float offsetXGyro = -0.001060807;
float offsetYGyro = -0.00082012;
float offsetZGyro = -0.000802291;

// 에러 매크로
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// 오류 발생 시 루프
void error_loop(){
  while(1){
    Serial.println("not prepare yet");
    delay(100);
  }
}

// **엔코더 인터럽트 핸들러 (A, B 채널)**
void IRAM_ATTR left_encoder_ISR() {
  if (digitalRead(LEFT_ENCODER_A) != digitalRead(LEFT_ENCODER_B)) {
    left_pulse_count++;
  } else {
    left_pulse_count--;
  }
}

void IRAM_ATTR right_encoder_ISR() {
  if (digitalRead(RIGHT_ENCODER_A) == digitalRead(RIGHT_ENCODER_B)) {
    right_pulse_count++;
  } else {
    right_pulse_count--;
  }
}

// **모터 속도 및 방향 설정 함수**
void setMotor(int pwm_pin, int dir1_pin, int dir2_pin, float speed) {
    int pwm_value;

    // 속도가 0.28m/s 초과하면 최대 PWM 255로 설정
    if (speed > 0.285) {
        pwm_value = 255;
    } 
    // // 속도가 0.1m/s 이하이면 PWM을 0으로 설정 (모터가 움직이지 않도록 함)
    // else if (abs(speed) < 0.1) {  
    //     pwm_value = 0;
    // } 
    // 속도를 0~0.28 m/s 범위로 매핑하여 PWM 값 생성
    else {
        pwm_value = map(abs(speed) * 1000, 0, 285, 0, 255);  // 0.1m/s 이상 속도 매핑
    }

    pwm_value = constrain(pwm_value, 0, 255);  // PWM 값 범위 제한

    // 디버깅 출력
    Serial.print("Motor PWM Pin: "); Serial.print(pwm_pin);
    Serial.print(" | Speed: "); Serial.print(speed);
    Serial.print(" | PWM Value: "); Serial.println(pwm_value);

    // **모터 방향 설정**
    if (speed > 0) { // 전진
        digitalWrite(dir1_pin, HIGH);
        digitalWrite(dir2_pin, LOW);
    } else if (speed < 0) { // 후진
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, HIGH);
    } else { // 정지
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, LOW);
    }

    // **PWM 적용**
    analogWrite(pwm_pin, pwm_value);
}
// **ROS 2에서 `/cmd_vel` 메시지 수신 시 실행되는 콜백 함수**
void subscription_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    float linear_x = msg->linear.x;  
    float angular_z = msg->angular.z;
    //오도메트리의 너무 작은 값 방지.
    if (abs(linear_x) < 0.01) linear_x = 0.0;
    if (abs(angular_z) < 0.01) angular_z = 0.0;

    // 시리얼 출력 (디버깅용)
    Serial.print("Received /cmd_vel | Linear X: ");
    Serial.print(linear_x);
    Serial.print(" | Angular Z: ");
    Serial.println(angular_z);

    float left_speed = linear_x - (angular_z * WHEEL_SEPARATION / 2.0);
    float right_speed = linear_x + (angular_z * WHEEL_SEPARATION / 2.0);
    // 디버깅 출력
    Serial.print("Left Speed: "); Serial.print(left_speed);
    Serial.print(" | Right Speed: "); Serial.println(right_speed);

    setMotor(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR1, LEFT_MOTOR_DIR2, left_speed);
    setMotor(RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR1, RIGHT_MOTOR_DIR2, right_speed);
}

// **엔코더 값을 각속도로 변환하는 함수**
void update_motor_velocity() {
  unsigned long current_time = millis();
  float delta_time = (current_time - last_time) / 1000.0; // 초 단위 변환

  float left_angular_velocity = (left_pulse_count * ENCODER_FACTOR) / delta_time;
  float right_angular_velocity = (right_pulse_count * ENCODER_FACTOR) / delta_time;

  // ROS 2 메시지로 엔코더 값 발행
  motor.left_w = left_angular_velocity;
  motor.right_w = right_angular_velocity;
  motor.linear_vel = (left_angular_velocity + right_angular_velocity)*0.0875/4;
  left_pulse_count = 0;
  right_pulse_count = 0;
  last_time = current_time;
}

// Offset 계산 함수(동적 모드)
void calculateIMUOffsets(){
  // IMU 오프셋 설정
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  // 자동 보정 함수
  mpu.CalibrateAccel(6); // Accel 보정; MPU6050 라이브러리에서 pid로 offset 자동 보정
  mpu.CalibrateGyro(6); // Gyro 보정; MPU6050 라이브러리에서 pid로 offset 자동 보정
  }

  // imu데이터를 읽어서 메시지에 저장
void update_imu_data_dynamic(){
  if(!DMPReady){
    Serial.println("DMP not ready! Skipping IMU update.");
    return;
  }
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)){
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGyro(&gg,FIFOBuffer);
    
    // 가속도 데이터 변환 (m/s^2 단위)
    imu.linear_acceleration.x = -aa.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2; // x,y,z 그리고 부호를 로봇 좌표계와 맞춤 imu 그림과 달리 우리는 y방향이 전진 방향
    imu.linear_acceleration.y = aa.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2; // x,y,z 그리고 부호를 로봇 좌표계와 맞춤 imu 그림과 달리 우리는 y방향이 전진 방향
    imu.linear_acceleration.z = -aa.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2; // x,y,z 그리고 부호를 로봇 좌표계와 맞춤 imu 그림과 달리 우리는 y방향이 전진 방향
    
    // 자이로 데이터 변환(m/s^2 단위)
    imu.angular_velocity.x = gg.x * mpu.get_gyro_resolution() * DEG_TO_RAD;
    imu.angular_velocity.y = gg.y * mpu.get_gyro_resolution() * DEG_TO_RAD;
    imu.angular_velocity.z = gg.z * mpu.get_gyro_resolution() * DEG_TO_RAD;

    // ROS2 메시지 헤더 업데이트
    imu.header.stamp.sec = millis() / 1000;
    imu.header.stamp.nanosec = (millis() % 1000) * 1000000;
  }
}

void update_imu_data_manual(){
  if (!DMPReady) {
    Serial.println("DMP not ready! Skipping IMU update.");
    return;
  }

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGyro(&gg, FIFOBuffer);
    // 가속도 데이터 변환 (m/s² 단위
    // x,y,z 그리고 부호를 로봇 좌표계와 맞춤 imu 그림과 달리 우리는 y방향이 전진 방향
    float robot_ax = -aa.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2 - offsetYAccel;
    float robot_ay = aa.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2 - offsetXAccel;
    float robot_az = -aa.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2 - offsetZAccel;
    imu.linear_acceleration.x = robot_ax; 
    imu.linear_acceleration.y = robot_ay; 
    imu.linear_acceleration.z = robot_az;

    // 자이로 데이터 변환 (rad/s 단위)
    imu.angular_velocity.x = gg.x * mpu.get_gyro_resolution() * DEG_TO_RAD - offsetXGyro;
    imu.angular_velocity.y = gg.y * mpu.get_gyro_resolution() * DEG_TO_RAD - offsetYGyro;
    imu.angular_velocity.z = gg.z * mpu.get_gyro_resolution() * DEG_TO_RAD - offsetZGyro;

    // ROS 2 메시지 헤더 업데이트
    imu.header.stamp.sec = millis() / 1000;
    imu.header.stamp.nanosec = (millis() % 1000) * 1000000;

  }
}

// **모터 속도 정보 발행 콜백함수**
void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    update_motor_velocity();
    if (offsetMode == "manual"){
      update_imu_data_manual();
    }
    else {
      update_imu_data_dynamic();
    }
    RCSOFTCHECK(rcl_publish(&encoder_publisher,&motor,NULL));
    RCSOFTCHECK(rcl_publish(&imu_publisher,&imu, NULL));
  }
}

// **ESP32 초기 설정**
void setup() {
  set_microros_wifi_transports((char*)"I_phone", (char*)"dlgksrufdlek", (char*)"192.168.166.30", 8888);
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
  devStatus = mpu.dmpInitialize(); // dmp 초기화(Digital Motion Processor) 
  
  int time_wait = 0; // offset 입력 기다리는 시간
  Serial.println("Enter offset mode ('manual' or 'dynamic'): ");
  while (Serial.available() == 0) {
    time_wait ++;
    delay(10);
    if (time_wait >= 500){
      break;
    }

  }
  if (Serial.available() != 0){
    offsetMode = Serial.readStringUntil('\n');}
  offsetMode.trim(); // Remove any leading/trailing whitespace

  Serial.print("Offset mode selected: ");
  Serial.println(offsetMode);

  if (devStatus == 0) { // 초기화됐을 때
    if (offsetMode == "dynamic"){
      calculateIMUOffsets();
    }
    Serial.println("DMP ready!");
    mpu.setDMPEnabled(true); // DMP 활성화
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.println("DMP initialization failed!");
  }

  // IMU 메시지 초기화 frame_id설정 동적 string 동적 메모리할당 size,capacity지정해 주어야함.(c언어임으로.)
  imu.header.frame_id.data = (char *)"base_link";
  imu.header.frame_id.size = strlen("base_link");
  imu.header.frame_id.capacity = imu.header.frame_id.size + 1;
  // IMU 데이터 구독 주기
  float fre_imu = 10; // ms -> 0.01 s = 100 Hz
  // 모터 핀을 출력 모드로 설정
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR1, OUTPUT);
  pinMode(LEFT_MOTOR_DIR2, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR1, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR2, OUTPUT);

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

  //micro_ros 설정 부분
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(&subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),"/cmd_vel"));
  //엔코더를 통한 속도값 퍼블리셔
  RCCHECK(rclc_publisher_init_default(&encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(my_custom_message, msg, Motor), "motor_topic"));
  //imu데이터 퍼블리셔
  RCCHECK(rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Imu), "/imu/data"));
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(fre_imu), timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  //RCCHECK(rclc_executor_set_trigger(&executor,rclc_executor_trigger_one, &subscriber));
  Serial.println("micro-ROS node started.");
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}