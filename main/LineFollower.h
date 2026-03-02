/**
 * @file LineFollower.h
 * @brief 寻线控制模块头文件
 * @author Copilot
 * @date 2026.3.2
 */

#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include "LineTracker.h"
#include "MecanumControl.h"

class LineFollower {
public:
    /**
     * @brief 构造函数
     * @param tracker 巡线传感器引用
     * @param control 麦轮控制引用
     */
    LineFollower(LineTracker& tracker, MecanumControl& control);

    /**
     * @brief 开始寻线
     */
    void start();

    /**
     * @brief 停止寻线
     */
    void stop();

    /**
     * @brief 更新寻线状态（在loop中调用）
     */
    void update();

    /**
     * @brief 检查是否正在运行
     */
    bool isRunning();

private:
    LineTracker& lineTracker;
    MecanumControl& mecanumControl;
    bool running;
    
    // 基础速度参数
    float baseSpeed;
    float turnSpeed; 

    // 虚线处理
    unsigned long lastLineTime;  // 上次检测到线的时间
    bool isLost;                 // 是否丢失线条
};

#endif // LINEFOLLOWER_H
