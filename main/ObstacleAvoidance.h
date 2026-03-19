/**
 * @file ObstacleAvoidance.h
 * @brief 超声波避障控制模块头文件
 * @author 黄竞亿
 * @date 2026.3.11
 */

#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H

#include "Ultrasonic.h"
#include "MecanumControl.h"

class ObstacleAvoidance {
public:
    /**
     * @brief 构造函数
     * @param ultrasonic 超声波传感器引用
     * @param control 麦轮控制引用
     */
    ObstacleAvoidance(Ultrasonic& ultrasonic, MecanumControl& control);

    /**
     * @brief 开始避障
     */
    void start();

    /**
     * @brief 停止避障
     */
    void stop();

    /**
     * @brief 设置避障速度
     * @param speed 速度值 (0.0 - 1.0)
     */
    void setSpeed(float speed);

    /**
     * @brief 更新避障状态（在loop中调用）
     */
    void update();

    /**
     * @brief 检查是否正在运行
     */
    bool isRunning();

private:
    Ultrasonic& ultrasonic;
    MecanumControl& mecanumControl;
    bool running;
    float baseSpeed;
    float turnSpeed;
    float currentDistance;
    unsigned long lastMeasureTime;
    
    // 状态机状态
    enum State {
        FORWARD,
        BACKWARD,
        TURN_LEFT,
        TURN_RIGHT,
        STOP
    };
    State currentState;
    unsigned long stateStartTime;
};

#endif // OBSTACLEAVOIDANCE_H
 