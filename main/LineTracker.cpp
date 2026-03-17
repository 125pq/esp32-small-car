/**
 * @file LineTracker.cpp
 * @brief 巡线传感器模块实现文件
 * @author 黄竞亿
 * @date 2025.12.4
 */

#include "LineTracker.h"

LineTracker::LineTracker() : 
    sensor1Pin(IO_X1),
    sensor2Pin(IO_X2),
    sensor3Pin(IO_X3),
    sensor4Pin(IO_X4) {
    // 构造函数
}

void LineTracker::init() {
    pinMode(sensor1Pin, INPUT);
    pinMode(sensor2Pin, INPUT);
    pinMode(sensor3Pin, INPUT);
    pinMode(sensor4Pin, INPUT);
}

uint8_t LineTracker::getState() {
    uint8_t x1 = digitalRead(sensor1Pin);
    uint8_t x2 = digitalRead(sensor2Pin);
    uint8_t x3 = digitalRead(sensor3Pin);
    uint8_t x4 = digitalRead(sensor4Pin);
    
    // 将四个传感器状态组合成一个4位二进制数
    uint8_t state = x2 | (x1 << 1) | (x3 << 2) | (x4 << 3);
    return state;
}

bool LineTracker::getSensor(int sensorNum) {
    switch(sensorNum) {
        case 1: return digitalRead(sensor1Pin);
        case 2: return digitalRead(sensor2Pin);
        case 3: return digitalRead(sensor3Pin);
        case 4: return digitalRead(sensor4Pin);
        default: return false;
    }
}
