/**
 * @file Motor.cpp
 * @brief 电机控制模块实现文件
 * @author 喵屋
 * @date 2025.12.4
 */

#include "Motor.h"

Motor::Motor() {
    // 构造函数
}

void Motor::init() {
    // 设置所有电机控制引脚为输出模式
    pinMode(IO_M1PWM, OUTPUT);
    pinMode(IO_M2PWM, OUTPUT);
    pinMode(IO_M3PWM, OUTPUT);
    pinMode(IO_M4PWM, OUTPUT);

    pinMode(IO_M1IN1, OUTPUT);
    pinMode(IO_M1IN2, OUTPUT);
    pinMode(IO_M2IN1, OUTPUT);
    pinMode(IO_M2IN2, OUTPUT);
    pinMode(IO_M3IN1, OUTPUT);
    pinMode(IO_M3IN2, OUTPUT);
    pinMode(IO_M4IN1, OUTPUT);
    pinMode(IO_M4IN2, OUTPUT);
}

void Motor::setSpeed(int speed1, int speed2, int speed3, int speed4) {
    setMotorSpeed(1, speed1);
    setMotorSpeed(2, speed2);
    setMotorSpeed(3, speed3);
    setMotorSpeed(4, speed4);
}

void Motor::setMotorSpeed(int motorNum, int speed) {
    int pwmPin, in1Pin, in2Pin;
    bool reverseLogic = false;

    // 根据电机编号选择对应引脚
    switch(motorNum) {
        case 1:
            pwmPin = IO_M1PWM;
            in1Pin = IO_M1IN1;
            in2Pin = IO_M1IN2;
            break;
        case 2:
            pwmPin = IO_M2PWM;
            in1Pin = IO_M2IN1;
            in2Pin = IO_M2IN2;
            reverseLogic = true;  // 电机2方向逻辑相反
            break;
        case 3:
            pwmPin = IO_M3PWM;
            in1Pin = IO_M3IN1;
            in2Pin = IO_M3IN2;
            break;
        case 4:
            pwmPin = IO_M4PWM;
            in1Pin = IO_M4IN1;
            in2Pin = IO_M4IN2;
            reverseLogic = true;  // 电机4方向逻辑相反
            break;
        default:
            return;
    }

    // 控制电机方向和速度
    int absSpeed = abs(speed);
    
    if (motorNum == 4) {
        // 电机4使用特殊的方向逻辑
        if (speed < 0) {
            digitalWrite(in2Pin, LOW);
            digitalWrite(in1Pin, HIGH);
        } else {
            digitalWrite(in2Pin, HIGH);
            digitalWrite(in1Pin, LOW);
        }
    } else if (reverseLogic) {
        // 电机2的反向逻辑
        if (speed < 0) {
            digitalWrite(in1Pin, HIGH);
            digitalWrite(in2Pin, LOW);
        } else {
            digitalWrite(in1Pin, LOW);
            digitalWrite(in2Pin, HIGH);
        }
    } else {
        // 电机1和3的正常逻辑
        if (speed < 0) {
            digitalWrite(in1Pin, LOW);
            digitalWrite(in2Pin, HIGH);
        } else {
            digitalWrite(in1Pin, HIGH);
            digitalWrite(in2Pin, LOW);
        }
    }
    
    analogWrite(pwmPin, absSpeed);
}

void Motor::stop() {
    setSpeed(0, 0, 0, 0);
}

void Motor::test() {
    setSpeed(100, 0, 0, 0);
    delay(200);
    setSpeed(0, 100, 0, 0);
    delay(200);
    setSpeed(0, 0, 100, 0);
    delay(200);
    setSpeed(0, 0, 0, 100);
    delay(200);
    stop();
}
