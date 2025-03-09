#include "encoder_handler.h"
#include "common_definitions.h"

// 인터럽트 서비스 루틴(ISR)
// left
void IRAM_ATTR left_encoder_ISR() {
    if (digitalRead(LEFT_ENCODER_A) != digitalRead(LEFT_ENCODER_B)) {
        left_encoder_count++; 
    } else {
        left_encoder_count--; 
    }
}
// right
void IRAM_ATTR right_encoder_ISR() {
    if (digitalRead(RIGHT_ENCODER_A) != digitalRead(RIGHT_ENCODER_B)) {
        right_encoder_count--; 
    } else {
        right_encoder_count++; 
    }
}
// 엔코더 초기 offset보정 (필요할때만 사용)
void offset() {
    Serial.println(" 엔코더 오프셋 보정 시작...");
    delay(1000); // 안정화 대기

    int sum_left = 0, sum_right = 0;
    int sample_count = 20;  // 20개 샘플 측정
    int interval = 50; // 50ms 간격


    for (int i = 0; i < sample_count; i++) {
        sum_left += left_encoder_count;
        sum_right += right_encoder_count;
        delay(interval);
    }

    left_offset_count = sum_left / sample_count;
    right_offset_count = sum_right / sample_count;

    Serial.print("Left Encoder Offset: ");
    Serial.println(left_offset_count);
    Serial.print(" Right Encoder Offset: ");
    Serial.println(right_offset_count);
}