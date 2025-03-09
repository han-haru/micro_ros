#include "motor_control.h"
#include "common_definitions.h"

// 모터 속도 제어 함수
// LEFT
void leftsetMotorPWM(int pwm) {
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);

    if (target_omega_left > 0) {
        digitalWrite(LEFT_MOTOR_DIR1, HIGH);
        digitalWrite(LEFT_MOTOR_DIR2, LOW);
    } else if (target_omega_left < 0) {
        digitalWrite(LEFT_MOTOR_DIR1, LOW);
        digitalWrite(LEFT_MOTOR_DIR2, HIGH);
    } else {
        digitalWrite(LEFT_MOTOR_DIR1, LOW);
        digitalWrite(LEFT_MOTOR_DIR2, LOW);
    }
    Serial.println(abs(pwm));
    analogWrite(LEFT_MOTOR_PWM, abs(pwm));
}
// RIGHT
void rightsetMotorPWM(int pwm) {
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);

    if (target_omega_right > 0) {
        digitalWrite(RIGHT_MOTOR_DIR1, HIGH);
        digitalWrite(RIGHT_MOTOR_DIR2, LOW);
    } else if (target_omega_right < 0) {
        digitalWrite(RIGHT_MOTOR_DIR1, LOW);
        digitalWrite(RIGHT_MOTOR_DIR2, HIGH);
    } else {
        digitalWrite(RIGHT_MOTOR_DIR1, LOW);
        digitalWrite(RIGHT_MOTOR_DIR2, LOW);
    }
    Serial.println(abs(pwm));
    analogWrite(RIGHT_MOTOR_PWM, abs(pwm));
}
// 속도 측정 및 PID 제어 실행
// left
void updateleftControlLoop() {
    unsigned long current_time = millis();
    float dt = (current_time - last_time_left) / 1000.0; // 초 단위 변환
    if (dt < 0.01) return; // 너무 짧은 시간 간격 방지

    // 디버깅: 엔코더 카운트 확인
    Serial.print("Encoder Count: ");
    Serial.println(left_encoder_count);

    // 현재 속도 측정 (rad/s)
    measured_w_left = ((left_encoder_count-left_offset_count) * ENCODER_FACTOR) / dt;
    if (abs(measured_w_left) >= 0.0 && abs(measured_w_left) < 0.05) { //노이즈 무시
        measured_w_left = 0.0;
    }
    left_encoder_count = 0; // 카운트 리셋
    LPF_target_omega_left = alpha*target_omega_left + (1-alpha)*LPF_target_omega_left;    
    //  오차 계산
    error_left = LPF_target_omega_left - measured_w_left;

    Serial.print("target_left");
    Serial.println(LPF_target_omega_left);
    Serial.print("error_left");
    Serial.println(error_left);
    Serial.print("measured_left");
    Serial.println(measured_w_left);
    // PID 계산

    if (abs(error_left) < 1.5){
      left_integral += error_left * dt;
    } else if (abs(error_left < 0.05)) {
      left_integral = 0.0;
    } else if (abs(LPF_target_omega_left) < 0.01) {
      left_integral = 0.0;
    } else {
      left_integral = 0.0;
    }
    // 미분을 dt로 표현
    //float derivative = (error_left - last_error_left) / dt;
    // 미분을 LPF로 표현
    float derivative_raw = (error_left - last_error_left);
    filtered_derivative_left = alpha_d * filtered_derivative_left + (1 - alpha_d) * derivative_raw;
    control_output_left = (Kp_L * error_left) + (Ki_L * left_integral) + (Kd_L * filtered_derivative_left);
    Serial.print("pid_output_L");
    Serial.println(control_output_left);
    
    last_error_left = error_left;
    last_time_left = current_time;

    // **디버깅: PID 출력을 확인**
    //Serial.print("Control Output_left: ");
    //Serial.println(control_output_left);

    // PWM 변환 (새로운 공식 사용)
    //무부하 mapping
    //float left_pwm = 2.4425*control_output_left*control_output_left - 2.6619*control_output_left + 46.902 ; 
    //지면 주행 mapping
    float left_pwm = 1.4111*control_output_left*control_output_left + 6.4437*control_output_left + 50.688 ; 
    int motor_pwm = round(left_pwm);
    // 모터에 PWM 적용
    leftsetMotorPWM(motor_pwm);
}
// right
void updaterightControlLoop() {
    unsigned long current_time = millis();
    float dt = (current_time - last_time_right) / 1000.0; // 초 단위 변환
    if (dt < 0.01) return; // 너무 짧은 시간 간격 방지

    // **디버깅: 엔코더 카운트 확인**
    //Serial.print("Encoder Count: ");
    //Serial.println(right_encoder_count);

    // 1. 현재 속도 측정 (rad/s)
    measured_w_right = ((right_encoder_count-right_offset_count) * ENCODER_FACTOR) / dt;
    if (abs(measured_w_right) >= 0.0 && abs(measured_w_right) < 0.05) { //노이즈 무시
        measured_w_right = 0.0;
    }
    right_encoder_count = 0; // 카운트 리셋

    // 2. 오차 계산
    LPF_target_omega_right = alpha*target_omega_right + (1-alpha)*LPF_target_omega_right;  
    error_right = LPF_target_omega_right - measured_w_right;
    Serial.print("target_right");
    Serial.println(target_omega_right);
    Serial.print("error_right");
    Serial.println(error_right);
    Serial.print("measured_right");
    Serial.println(measured_w_right);
    
    // 3. PID 계산
    if (abs(error_right)< 1.5) {
      right_integral += error_right*dt;
    } else if (abs(error_right)<0.05) {
      right_integral = 0.0;
    } else if (abs(LPF_target_omega_right<0.1)) {
      right_integral = 0.0;
    } else {
      right_integral = 0.0;
    }
    // 미분을 dt로 표현
    //float derivative = (error_right - last_error_right) / dt;
    // 미분을 LPF로 표현
    float derivative_raw = (error_left - last_error_right);
    filtered_derivative_right = alpha_d * filtered_derivative_right + (1 - alpha_d) * derivative_raw;

    control_output_right = (Kp_R * error_right) + (Ki_R * right_integral) + (Kd_R * filtered_derivative_right);
    Serial.print("pid_output_R");
    Serial.println(control_output_right);
    
    last_error_right = error_right;
    last_time_right = current_time;

    // 4. PWM 변환 
    // 무부하 mapping
    //float right_pwm = 2.3216*control_output_right*control_output_right - 3.1183*control_output_right + 46.695 ; 
    float right_pwm = 2.16*control_output_right*control_output_right + 0.7162*control_output_right + 52.622 ; 
    int motor_pwm = round(right_pwm);

    // 5. 모터에 PWM 적용
    rightsetMotorPWM(motor_pwm);
}
