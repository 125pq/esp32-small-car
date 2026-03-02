/**
 * @file SerialCommander.h
 * @brief 串口调试与控制模块头文件
 * @author Copilot
 * @date 2026.3.2
 */

#ifndef SERIALCOMMANDER_H
#define SERIALCOMMANDER_H

#include <Arduino.h>
#include "MecanumControl.h"

/**
 * @class SerialCommander
 * @brief 处理串口命令，用于调整PID参数和监控状态
 */
class SerialCommander {
public:
    /**
     * @brief 构造函数
     * @param control 麦轮控制对象引用
     */
    SerialCommander(MecanumControl& control);

    /**
     * @brief 在主循环中调用，处理串口输入
     */
    void update();

    /**
     * @brief 设置是否自动上报数据
     * @param enable 是否开启
     */
    void setReportEnable(bool enable);

private:
    MecanumControl& mecanumControl;
    String inputString;
    bool reportEnable;
    unsigned long lastReportTime;

    /**
     * @brief 解析并执行命令
     * @param cmd 命令字符串
     */
    void processCommand(String cmd);
    
    /**
     * @brief 上报状态数据
     */
    void reportStatus();
};

#endif // SERIALCOMMANDER_H
