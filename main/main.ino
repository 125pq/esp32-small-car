/*
 * ESP32 四轮小车控制程序
 * 功能：通过WiFi接收控制指令，控制电机运动，集成多种传感器（超声波、巡线、MPU6050）
 * 硬件：ESP32开发板、L298N电机驱动、OLED显示屏、MPU6050陀螺仪、超声波传感器、四路巡线传感器
 * 作者：喵屋
 * 日期：25.9.18
 */

#include <WiFi.h>          // WiFi连接库
#include <WiFiUdp.h>       // UDP通信库
#include <Wire.h>          // I2C通信库
#include <Adafruit_GFX.h>  // Adafruit图形库
#include <Adafruit_SSD1306.h> // Adafruit OLED库
#include <MPU6050_tockn.h> // MPU6050传感器库

#include <WebServer.h>
WebServer server(80); // 端口80

MPU6050 mpu6050(Wire);     // 初始化MPU6050对象

/**
 * @brief 巡线传感器引脚定义
 */
#define IO_X1 35
#define IO_X2 34
#define IO_X3 39
#define IO_X4 36

/**
 * @brief 电机接口定义
 * 使用L298N电机驱动模块，每个电机需要PWM信号和两个方向控制引脚
 */
#define IO_M1PWM 32  // 电机1 PWM速度控制
#define IO_M2PWM 18  // 电机2 PWM速度控制
#define IO_M3PWM 33  // 电机3 PWM速度控制
#define IO_M4PWM 19  // 电机4 PWM速度控制
#define IO_M1IN1 14  // 电机1 方向控制1
#define IO_M1IN2 13  // 电机1 方向控制2
#define IO_M2IN1 17  // 电机2 方向控制1
#define IO_M2IN2 5   // 电机2 方向控制2
#define IO_M3IN1 26  // 电机3 方向控制1
#define IO_M3IN2 27  // 电机3 方向控制2
#define IO_M4IN1 16  // 电机4 方向控制1
#define IO_M4IN2 4   // 电机4 方向控制2

/**
 * @brief 超声波接口定义
 */
#define IO_TRIG 23   // 超声波触发引脚
#define IO_ECHO 25   // 超声波回波引脚

/*-----OLED显示定义-------*/
#define SCREEN_WIDTH 128   // OLED宽度
#define SCREEN_HEIGHT 64   // OLED高度

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // 初始化OLED对象

// WiFi连接凭据
const char *sta_ssid = "park";     // WiFi名称
const char *sta_password = "86534633";     // WiFi密码

#define WheelBase 85  //轴距，用于运动学计算
#define TrackWidth 80 //轮距，用于运动学计算
//#define MOTOR_DISTANCE_TO_CENTER = 0.1 // 电机到中心点的距离，用于运动学计算

/*--------遥控数据解析变量--------*/
int index1;     // 字符串解析索引
int index2;     // 字符串解析索引
int index3;     // 字符串解析索引
int index4;     // 字符串解析索引
int index5;     // 字符串解析索引
int index6;     // 字符串解析索引
int index7;     // 字符串解析索引

String joy_x;   // 摇杆X轴数据
String joy_y;   // 摇杆Y轴数据
String but_a;   // A按钮状态
String but_b;   // B按钮状态
String but_x;   // X按钮状态
String but_y;   // Y按钮状态
String but_l;   // L按钮状态
String but_r;   // R按钮状态

uint8_t send_buff[20]; // 发送缓冲区

WiFiUDP Udp;    // UDP对象

int open_flag = 0; // 开启标志

/*------函数声明--------*/
/**
 * @brief 电机引脚初始化
 */
void MotorInit(void);

/**
 * @brief 设置四个电机的方向和速度
 * @param speed1 电机1速度 (-255 to 255)
 * @param speed2 电机2速度 (-255 to 255)
 * @param speed3 电机3速度 (-255 to 255)
 * @param speed4 电机4速度 (-255 to 255)
 */
void SetDirectionAndSpeed(int speed1, int speed2, int speed3, int speed4);

/**
 * @brief 根据速度向量设置小车运动
 * @param vx_set X轴速度
 * @param vy_set Y轴速度
 * @param wz_set 旋转速度
 */
void SetSpeed(float vx_set, float vy_set, float wz_set);

/**
 * @brief 超声波引脚初始化
 */
void UltrasonicInit(void);

/**
 * @brief 超声波测距
 * @return 返回距离值（单位：厘米）
 */
int UltrasonicDistence(void);

/**
 * @brief 巡线传感器初始化
 */
void LineInit(void);

/**
 * @brief 获取巡线传感器数据
 * @return 返回4位二进制数表示四个传感器的状态
 */
uint8_t GetLine(void);

// 全局变量
long duration;    // 超声波持续时间
float distance;   // 超声波测量距离

void OLEDdisplay(String text); // 修复函数声明

void setup()
{
    Serial.begin(115200); // 初始化串口通信
    Wire.begin();       // 初始化I2C通信

    delay(500);         // 延时等待系统稳定
    UltrasonicInit();   // 初始化超声波传感器

    /**各种外设初始化**/
    MotorInit();    // 初始化电机控制引脚
    LineInit();     // 初始化巡线传感器引脚

    pinMode(2, OUTPUT); // 设置板载LED引脚为输出
    digitalWrite(2, LOW); // 初始关闭LED
    
    // 初始化OLED显示屏
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // 如果初始化失败，停止程序
    }
    Serial.println("oled done");
    display.display(); // 显示Adafruit启动画面
    delay(2000);       // 延时2秒

    // MPU6050初始化
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    Serial.println("mpu6050 done");

    display.clearDisplay(); // 清除显示屏
    
    /**---------------------WiFi连接初始化---------------------------**/
    WiFi.mode(WIFI_STA); // 设置为STA模式（连接其他WiFi）
    /*------------------连接到WiFi网络-------------------*/
    WiFi.begin(sta_ssid, sta_password);

    OLEDdisplay(sta_ssid, sta_password, "Connecting");

    // 等待WiFi连接
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        display.display(); // 更新显示
        delay(500);
    }

    OLEDdisplay(WiFi.localIP().toString()); // 显示IP地址

    Udp.begin(3000); // 启动UDP监听端口3000

//网页模块
    server.on("/", HTTP_GET, []() {
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>button{width:100px;height:100px;margin:10px;font-size:20px;}</style></head>";
    html += "<body><center>";
    html += "<h1>ESP32 Car Control</h1>";
    html += "<button onclick=\"control('F')\">W</button><br>";
    html += "<button onclick=\"control('L')\">A</button>";
    html += "<button onclick=\"control('R')\">D</button><br>";
    html += "<button onclick=\"control('B')\">S</button>";
    html += "<button onclick=\"control('S')\">STOP</button>";
    html += "<script>function control(cmd){fetch('/control?cmd='+cmd);}</script>";
    html += "</center></body></html>";
    server.send(200, "text/html", html);
    });

    server.on("/control", HTTP_GET, []() {
    String cmd = server.arg("cmd");
    if (cmd == "F") SetDirectionAndSpeed(100, 100, 100, 100);
    else if (cmd == "B") SetDirectionAndSpeed(-100, -100, -100, -100);
    else if (cmd == "L") SetDirectionAndSpeed(-100, 100, 100, -100);
    else if (cmd == "R") SetDirectionAndSpeed(100, -100, -100, 100);
    else if (cmd == "S") SetDirectionAndSpeed(0, 0, 0, 0);
    server.send(200, "text/plain", "OK");
    });

    server.begin();
    
    // LED闪烁表示初始化完成
    for(int i=0;i<3;i++){
        digitalWrite(2, HIGH);
        delay(500);
        digitalWrite(2, LOW);
        delay(500);
        }
    
    
}

void loop()
{
    server.handleClient(); // 处理Web请求

    static int err_cnt = 0; // 错误计数器
    delay(2); // 短延时

    SetSpeed(100,25,0);
    
    OLEDdisplay(GetLine());

    // 接收UDP数据包
    /*
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
        err_cnt = 0; // 重置错误计数器
        digitalWrite(2, HIGH); // 点亮LED表示正在接收数据
        
        char buf[packetSize + 1]; // 创建缓冲区
        String rx;                // 存储接收到的字符串
        Udp.read(buf, packetSize); // 读取UDP数据
        rx = buf;                 // 转换为字符串

        // 解析接收到的控制数据（格式：joy_x:joy_y:but_a:but_b:but_x:but_y:but_l:but_r）
        index1 = rx.indexOf(':', 0);
        index2 = rx.indexOf(':', index1 + 1);
        index3 = rx.indexOf(':', index2 + 1);
        index4 = rx.indexOf(':', index3 + 1);
        index5 = rx.indexOf(':', index4 + 1);
        index6 = rx.indexOf(':', index5 + 1);
        index7 = rx.indexOf(':', index6 + 1);

        // 提取各部分数据
        joy_x = rx.substring(0, index1);
        joy_y = rx.substring(index1 + 1, index2);
        but_a = rx.substring(index2 + 1, index3);
        but_b = rx.substring(index3 + 1, index4);
        but_x = rx.substring(index4 + 1, index5);
        but_y = rx.substring(index5 + 1, index6);
        but_l = rx.substring(index6 + 1, index7);
        but_r = rx.substring(index7 + 1, packetSize + 1);

        // 根据按钮状态控制小车运动
        if (but_y.toInt()) // Y按钮按下 - 前进
        {
            SetDirectionAndSpeed(100, 100, 100, 100);
        }
        else if (but_a.toInt()) // A按钮按下 - 后退
        {
            SetDirectionAndSpeed(-100, -100, -100, -100);
        }
        else if (but_x.toInt()) // X按钮按下 - 左转
        {
            SetDirectionAndSpeed(-100, 100, 100, -100);
        }
        else if (but_b.toInt()) // B按钮按下 - 右转
        {
            SetDirectionAndSpeed(100, -100, -100, 100);
        }
        else if (but_l.toInt()) // L按钮按下 - 左平移
        {
            SetDirectionAndSpeed(-100, 100, -100, 100);
        }
        else if (but_r.toInt()) // R按钮按下 - 右平移
        {
            SetDirectionAndSpeed(100, -100, 100, -100);
        }
        else // 无按钮按下 - 停止
        {
            SetDirectionAndSpeed(0, 0, 0, 0);
        }
    }
    else
    {
        err_cnt++; // 增加错误计数
        if (err_cnt > 200) // 如果长时间没有收到数据
            digitalWrite(2, LOW); // 关闭LED
    }
    */
}


/**
 * @brief 电机引脚初始化
 */
void MotorInit(void)
{
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

/**
 * @brief 设置四个电机的方向和速度
 */
void SetDirectionAndSpeed(int speed1, int speed2, int speed3, int speed4)
{
    // 电机1控制
    if (speed1 < 0)
    {
        speed1 *= -1; // 取绝对值
        digitalWrite(IO_M1IN1, LOW);
        digitalWrite(IO_M1IN2, HIGH);
        analogWrite(IO_M1PWM, speed1);
    }
    else
    {
        digitalWrite(IO_M1IN1, HIGH);
        digitalWrite(IO_M1IN2, LOW);
        analogWrite(IO_M1PWM, speed1);
    }
    
    // 电机2控制（注意：方向逻辑与其他电机不同）
    if (speed2 < 0)
    {
        speed2 *= -1;
        digitalWrite(IO_M2IN1, HIGH);
        digitalWrite(IO_M2IN2, LOW);
        analogWrite(IO_M2PWM, speed2);
    }
    else
    {
        digitalWrite(IO_M2IN1, LOW);
        digitalWrite(IO_M2IN2, HIGH);
        analogWrite(IO_M2PWM, speed2);
    }
    
    // 电机3控制
    if (speed3 < 0)
    {
        speed3 *= -1;
        digitalWrite(IO_M3IN1, LOW);
        digitalWrite(IO_M3IN2, HIGH);
        analogWrite(IO_M3PWM, speed3);
    }
    else
    {
        digitalWrite(IO_M3IN1, HIGH);
        digitalWrite(IO_M3IN2, LOW);
        analogWrite(IO_M3PWM, speed3);
    }
    
    // 电机4控制（注意：方向逻辑与其他电机不同）
    if (speed4 < 0)
    {
        speed4 *= -1;
        digitalWrite(IO_M4IN2, LOW);
        digitalWrite(IO_M4IN1, HIGH);
        analogWrite(IO_M4PWM, speed4);
    }
    else
    {
        digitalWrite(IO_M4IN2, HIGH);
        digitalWrite(IO_M4IN1, LOW);
        analogWrite(IO_M4PWM, speed4);
    }
}

/**
 * @brief 超声波引脚初始化
 */
void UltrasonicInit(void)
{
    pinMode(IO_TRIG, OUTPUT); // 触发引脚设为输出
    pinMode(IO_ECHO, INPUT);  // 回波引脚设为输入
}

/**
 * @brief 超声波测距
 * @return 返回距离值（单位：厘米）
 */
int UltrasonicDistence(void)
{
    digitalWrite(IO_TRIG, HIGH); // 发送10微秒高脉冲
    delayMicroseconds(10);
    digitalWrite(IO_TRIG, LOW);
    return pulseIn(IO_ECHO, HIGH); // 测量高脉冲持续时间并返回
}

/**
 * @brief 巡线传感器初始化
 */
void LineInit(void)
{
    // 设置所有巡线传感器引脚为输入模式
    pinMode(IO_X1, INPUT);
    pinMode(IO_X2, INPUT);
    pinMode(IO_X3, INPUT);
    pinMode(IO_X4, INPUT);
}

/**
 * @brief 获取巡线传感器数据
 * @return 返回4位二进制数表示四个传感器的状态
 */
uint8_t GetLine(void)
{
    uint8_t x1, x2, x3, x4;
    uint8_t tmp = 0;
    x1 = digitalRead(IO_X1); // 读取传感器1
    x2 = digitalRead(IO_X2); // 读取传感器2
    x3 = digitalRead(IO_X3); // 读取传感器3
    x4 = digitalRead(IO_X4); // 读取传感器4
    // 将四个传感器状态组合成一个4位二进制数
    tmp = x2 | (x1 << 1) | (x3 << 2) | (x4 << 3);
    return tmp;
}

/**
 * @brief 根据速度向量设置小车运动
 * 使用运动学模型计算每个电机的速度
 */
void SetSpeed(float vx_set, float vy_set, float wz_set)
{
    int speed1, speed2, speed3, speed4;
    // 根据运动学模型计算四个电机的速度
    speed1 = vx_set + vy_set - (WheelBase/2 + TrackWidth/2) * wz_set;
    speed2 = vx_set - vy_set - (WheelBase/2 + TrackWidth/2) * wz_set;
    speed3 = vx_set + vy_set + (WheelBase/2 + TrackWidth/2) * wz_set;
    speed4 = vx_set - vy_set + (WheelBase/2 + TrackWidth/2) * wz_set;

    // 设置电机速度
    SetDirectionAndSpeed(speed1, speed2, speed3, speed4);
}

/**
 * @brief OLED显示函数
 * @param text 要显示的文本
 */
void OLEDdisplay(String text){
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(text); // 显示文本
    display.display();
}
void OLEDdisplay(String text1, String text2) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(text1);
    display.println(text2);
    display.display();
}
void OLEDdisplay(String text1, String text2, String text3) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(text1);
    display.println(text2);
    display.println(text3);
    display.display();
}