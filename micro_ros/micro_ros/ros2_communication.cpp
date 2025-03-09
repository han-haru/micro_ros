#include "ros2_communication.h"
#include "common_definitions.h"

void update_motor_velocity() {
  // ROS 2 메시지로 엔코더 값 발행
  motor.left_w = measured_w_left;
  motor.right_w = measured_w_right;
  motor.left_target_w = LPF_target_omega_left;
  motor.right_target_w = LPF_target_omega_right;
}

// imu데이터를 읽어서 메시지에 저장
void update_imu_data(){
    if (!DMPReady) {
      Serial.println("DMP not ready! Skipping IMU update.");
      return;
    }
  
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
      mpu.dmpGetAccel(&aa, FIFOBuffer);
      mpu.dmpGetGyro(&gg, FIFOBuffer);
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      //센서 쿼터니언
      imu.orientation.x = q.x;
      imu.orientation.y = q.y;
      imu.orientation.z = q.z;
      imu.orientation.w = q.w;
  
      // 가속도 데이터 변환 (m/s² 단위)
      imu.linear_acceleration.x = aa.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
      imu.linear_acceleration.y = -aa.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
      // 자이로 데이터 변환 (rad/s 단위)
      imu.angular_velocity.z = gg.z * mpu.get_gyro_resolution() * DEG_TO_RAD;
  
      // ROS 2 메시지 헤더 업데이트
      imu.header.stamp.sec = millis() / 1000;
      imu.header.stamp.nanosec = (millis() % 1000) * 1000000;
  
    }
  }

// ROS 2에서 `/cmd_vel` 메시지 수신 시 실행되는 콜백 함수
void subscription_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    float desired_speed_mps = msg->linear.x;  
    float order_w = msg->angular.z;
    //오도메트리의 너무 작은 값 방지.
    if (abs(desired_speed_mps) < 0.01) desired_speed_mps = 0.0;
    if (abs(order_w) < 0.01) order_w = 0.0;
    // 목표 각속도 계산 (rad/s)
    desired_left_speed = desired_speed_mps - (order_w*WHEEL_SEPARATION)/2;
    desired_right_speed = desired_speed_mps + (order_w*WHEEL_SEPARATION)/2;

    target_omega_left = desired_left_speed / R;
    target_omega_right = desired_right_speed / R;
    
}

// **모터 속도, imu 정보 발행 콜백함수**
void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    if (timer != NULL){
      update_motor_velocity();
      update_imu_data();
      RCSOFTCHECK(rcl_publish(&encoder_publisher,&motor,NULL));
      RCSOFTCHECK(rcl_publish(&imu_publisher,&imu, NULL));
    }
  }
