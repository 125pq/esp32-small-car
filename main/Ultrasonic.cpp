/**
 * @file Ultrasonic.cpp
 * @brief 超声波测距模块实现文件
 * @author 喵屋
 * @date 2025.12.4
 */

#include "Ultrasonic.h"

Ultrasonic::Ultrasonic() : trigPin(IO_TRIG), echoPin(IO_ECHO) {
    // 构造函数
}

void Ultrasonic::init() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

long Ultrasonic::getRawPulse() {
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // 使用 20ms 超时 (20000 微秒)，对应约 3.4米 距离
    // 如果超时返回 0，避免程序卡死
    return pulseIn(echoPin, HIGH, 20000);
}

float Ultrasonic::getDistance() {
    long duration = getRawPulse();
    // 声速340m/s，往返距离，转换为厘米
    float distance = (duration / 2.0) * 0.0343;
    return distance;
}
