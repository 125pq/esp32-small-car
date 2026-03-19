/**
 * @file WebControl.h
 * @brief Web服务器控制模块头文件
 * @author 125pq
 * @date 2025.12.4
 */

#ifndef WEBCONTROL_H
#define WEBCONTROL_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "Config.h"
#include "MecanumControl.h"
#include "LineFollower.h"
#include "ObstacleAvoidance.h"

/**
 * @class WebControl
 * @brief Web服务器控制类
 */
class WebControl {
public:
    /**
     * @brief 构造函数
     * @param mecanumCtrl 麦轮控制对象引用
     */
    WebControl(MecanumControl& mecanumCtrl);

    /**
     * @brief 初始化WiFi和Web服务器
     * @return true-成功连接WiFi, false-失败
     */
    bool init();

    /**
     * @brief 处理客户端请求（在loop中调用）
     */
    void handleClient();

    /**
     * @brief 获取WiFi连接状态
     * @return true-已连接, false-未连接
     */
    bool isConnected();

    /**
     * @brief 获取IP地址
     * @return IP地址字符串
     */
    String getIPAddress();

    /**
     * @brief 设置LineFollower对象
     */
    void setLineFollower(LineFollower* lf) { lineFollower = lf; }

    /**
     * @brief 设置ObstacleAvoidance对象
     */
    void setObstacleAvoidance(ObstacleAvoidance* oa) { obstacleAvoidance = oa; }

private:
    MecanumControl& mecanumControl;
    LineFollower* lineFollower = nullptr;
    ObstacleAvoidance* obstacleAvoidance = nullptr;
    WebServer* server;

    /**
     * @brief 处理根路径请求（显示控制页面）
     */
    void handleRoot();

    /**
     * @brief 处理控制指令
     */
    void handleControl();

    /**
     * @brief 获取巡线参数
     */
    void handleLineParams();

    /**
     * @brief 生成HTML控制页面
     * @return HTML字符串
     */
    String generateHTML();

    float currentSpeed = 0.4; // 当前速度设定
};

#endif // WEBCONTROL_H
