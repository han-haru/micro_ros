#ifndef ROS2_COMMUNICATION_H
#define ROS2_COMMUNICATION_H

#include <rcl/rcl.h>
#include <rcl/timer.h>

void update_motor_velocity();
void update_imu_data();
void subscription_callback(const void *msgin);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

#endif // ROS2_COMMUNICATION_H
