/**
 * @file LineTracker.h
 * @brief 巡线传感器模块头文件
 * @author 125pq
 * @date 2025.12.4
 */

#ifndef LINETRACKER_H
#define LINETRACKER_H

#include <Arduino.h>
#include "Config.h"

/**
 * @class LineTracker
 * @brief 四路巡线传感器类
 */
class LineTracker {
public:
    /**
     * @brief 构造函数
     */
    LineTracker();

    /**
     * @brief 初始化巡线传感器引脚
     */
    void init();

    /**
     * @brief 获取巡线传感器数据
     * @return 返回4位二进制数表示四个传感器的状态
     */
    uint8_t getState();

    /**
     * @brief 获取单个传感器状态
     * @param sensorNum 传感器编号 (1-4)
     * @return 传感器状态 (0或1)
     */
    bool getSensor(int sensorNum);

private:
    int sensor1Pin;
    int sensor2Pin;
    int sensor3Pin;
    int sensor4Pin;
};

#endif // LINETRACKER_H
