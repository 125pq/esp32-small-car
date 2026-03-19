/**
 * @file ObstacleAvoidance.cpp
 * @brief 超声波避障控制模块实现文件
 * @author 125pq
 * @date 2026.3.11
 */

#include "ObstacleAvoidance.h"
#include "Config.h"

// 避障距离阈值 (cm)
#define OBSTACLE_DISTANCE 5.0
// 后退持续时间 (ms)
#define BACKWARD_DURATION 500
// 转向持续时间 (ms)
#define TURN_DURATION 400

ObstacleAvoidance::ObstacleAvoidance(Ultrasonic& us, MecanumControl& control) 
    : ultrasonic(us), mecanumControl(control), running(false) {
    baseSpeed = 0.4 * MAX_LINEAR_SPEED;   // 默认避障速度较慢，安全第一
    turnSpeed = 0.5 * MAX_ROTATION_SPEED;
    currentState = FORWARD;
    lastMeasureTime = 0;
}

void ObstacleAvoidance::start() {
    running = true;
    currentState = FORWARD;
    stateStartTime = millis();
    Serial.println("Obstacle Avoidance Started");
}

void ObstacleAvoidance::stop() {
    running = false;
    mecanumControl.setTargetVelocity(0, 0, 0);
    Serial.println("Obstacle Avoidance Stopped");
}

void ObstacleAvoidance::setSpeed(float speed) {
    baseSpeed = speed * MAX_LINEAR_SPEED;
    turnSpeed = speed * MAX_ROTATION_SPEED;
}

bool ObstacleAvoidance::isRunning() {
    return running;
}

void ObstacleAvoidance::update() {
    if (!running) return;

    // 定期测量距离 (每100ms)
    // 如果没有返回 (超时)，则距离保持不变
    if (millis() - lastMeasureTime > 100) {
        float d = ultrasonic.getDistance();
        if (d > 0.0) {
            currentDistance = d;
        } else {
             // 0 表示超时或错误，通常意味着没有障碍物或障碍物很远
             currentDistance = 400.0; // 设置为一个大值
        }
        lastMeasureTime = millis();
        // Serial.print("Distance: ");
        // Serial.println(currentDistance);
    }

    switch (currentState) {
        case FORWARD:
            if (currentDistance > 0 && currentDistance < OBSTACLE_DISTANCE) {
                // 遇到障碍物，切换到后退
                mecanumControl.setTargetVelocity(0, 0, 0); // 先停
                currentState = BACKWARD;
                stateStartTime = millis();
            } else {
                // 前方无障碍，直行
                mecanumControl.setTargetVelocity(baseSpeed, 0, 0);
            }
            break;

        case BACKWARD:
            if (millis() - stateStartTime < BACKWARD_DURATION) {
                mecanumControl.setTargetVelocity(-baseSpeed, 0, 0);
            } else {
                // 后退完成，随机转向
                currentState = (random(0, 2) == 0) ? TURN_LEFT : TURN_RIGHT;
                stateStartTime = millis();
            }
            break;

        case TURN_LEFT:
            if (millis() - stateStartTime < TURN_DURATION) {
                mecanumControl.setTargetVelocity(0, 0, turnSpeed);
            } else {
                // 转向完成，恢复直行检测
                currentState = FORWARD;
            }
            break;

        case TURN_RIGHT:
            if (millis() - stateStartTime < TURN_DURATION) {
                mecanumControl.setTargetVelocity(0, 0, -turnSpeed);
            } else {
                // 转向完成，恢复直行检测
                currentState = FORWARD;
            }
            break;
            
        case STOP:
            mecanumControl.setTargetVelocity(0, 0, 0);
            break;
    }
}
