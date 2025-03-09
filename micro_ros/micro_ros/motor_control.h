#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

void leftsetMotorPWM(int pwm);
void rightsetMotorPWM(int pwm);
void updateleftControlLoop();
void updaterightControlLoop();

#endif // MOTOR_CONTROL_H