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
     * @brief 设置巡线速度
     * @param speed 速度值 (0.0 - 1.0)
     */
    void setSpeed(float speed);

    /**
     * @brief 设置巡线算法参数
     * @param key 参数键
     * @param value 参数值
     * @return true-设置成功, false-参数不存在
     */
    bool setTuning(const String& key, float value);

    /**
     * @brief 恢复巡线参数默认值
     */
    void resetTuningToDefault();

    /**
     * @brief 获取巡线参数JSON
     */
    String getTuningJson() const;

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

    // 可运行时调节参数
    float errorFilterAlpha;
    float dampingGain;
    float omegaSmoothAlpha;
    float speedReductionGain;
    float minForwardRatio;
    float lostOmegaDecay;
    float searchOmegaRatio;
    float searchSpeedRatio;
    float turnGain;
    float dErrorClamp;
    unsigned long lostHoldMs;
    unsigned long lostTimeoutMs;

    // 平滑循迹状态
    float filteredError;
    float prevError;
    float lastOmegaCmd;
    unsigned long lastControlTime;

    // 虚线处理
    unsigned long lastLineTime;  // 上次检测到线的时间
    bool isLost;                 // 是否丢失线条
};

#endif // LINEFOLLOWER_H
