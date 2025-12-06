/**
 * @file WebControl.cpp
 * @brief Web服务器控制模块实现文件
 * @author 喵屋
 * @date 2025.12.4
 */

#include "WebControl.h"

WebControl::WebControl(MecanumControl& mecanumCtrl) 
    : mecanumControl(mecanumCtrl) {
    server = new WebServer(WEB_PORT);
}

bool WebControl::init() {
    // 设置WiFi模式
    WiFi.mode(WIFI_STA);
    
    // 连接到WiFi网络
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        Serial.print(".");
        delay(500);
        attempts++;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi connection failed!");
        return false;
    }
    
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // 设置Web服务器路由
    server->on("/", [this]() { this->handleRoot(); });
    server->on("/control", [this]() { this->handleControl(); });
    
    // 启动Web服务器
    server->begin();
    Serial.println("Web server started");
    
    return true;
}

void WebControl::handleClient() {
    server->handleClient();
}

bool WebControl::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

String WebControl::getIPAddress() {
    return WiFi.localIP().toString();
}

void WebControl::handleRoot() {
    String html = generateHTML();
    server->send(200, "text/html", html);
}

void WebControl::handleControl() {
    String cmd = server->arg("cmd");
    String val = server->arg("val");
    
    float speed = 0.5; // 默认速度
    
    if (cmd == "SP") {
        speed = val.toFloat() / 100.0;
    } else if (cmd == "F") {
        mecanumControl.setTargetVelocity(speed, 0, 0);
    } else if (cmd == "B") {
        mecanumControl.setTargetVelocity(-speed, 0, 0);
    } else if (cmd == "L") {
        mecanumControl.setTargetVelocity(0, speed, 0);
    } else if (cmd == "R") {
        mecanumControl.setTargetVelocity(0, -speed, 0);
    } else if (cmd == "FL") {
        mecanumControl.setTargetVelocity(speed, speed, 0);
    } else if (cmd == "FR") {
        mecanumControl.setTargetVelocity(speed, -speed, 0);
    } else if (cmd == "BL") {
        mecanumControl.setTargetVelocity(-speed, speed, 0);
    } else if (cmd == "BR") {
        mecanumControl.setTargetVelocity(-speed, -speed, 0);
    } else if (cmd == "CL") {
        mecanumControl.setTargetVelocity(0, 0, speed);
    } else if (cmd == "CR") {
        mecanumControl.setTargetVelocity(0, 0, -speed);
    } else if (cmd == "S") {
        mecanumControl.setTargetVelocity(0, 0, 0);
    }
    
    server->send(200, "text/plain", "OK");
}

String WebControl::generateHTML() {
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial;text-align:center;}";
    html += ".control{display:inline-block;margin:10px;}";
    html += "button{width:80px;height:80px;margin:5px;font-size:16px;}";
    html += ".slider{width:300px;margin:10px;}</style></head>";
    html += "<body><h1>Mecanum Wheel Car Control</h1>";
    
    // 方向控制
    html += "<div class='control'><h2>Direction</h2>";
    html += "<button onclick=\"control('F')\">Forward</button><br>";
    html += "<button onclick=\"control('L')\">Left</button>";
    html += "<button onclick=\"control('R')\">Right</button><br>";
    html += "<button onclick=\"control('B')\">Backward</button>";
    html += "<button onclick=\"control('S')\">Stop</button></div>";
    
    // 斜向移动
    html += "<div class='control'><h2>Diagonal</h2>";
    html += "<button onclick=\"control('FL')\">Front-Left</button>";
    html += "<button onclick=\"control('FR')\">Front-Right</button><br>";
    html += "<button onclick=\"control('BL')\">Back-Left</button>";
    html += "<button onclick=\"control('BR')\">Back-Right</button></div>";
    
    // 旋转控制
    html += "<div class='control'><h2>Rotation</h2>";
    html += "<button onclick=\"control('CL')\">Rotate Left</button><br>";
    html += "<button onclick=\"control('CR')\">Rotate Right</button></div>";
    
    // 速度控制
    html += "<div class='control'><h2>Speed</h2>";
    html += "<input type='range' id='speed' class='slider' min='0' max='100' value='50' onchange='updateSpeed()'>";
    html += "<p>Speed: <span id='speedValue'>50</span>%</p></div>";
    
    html += "<script>";
    html += "function control(cmd){fetch('/control?cmd='+cmd);}";
    html += "function updateSpeed(){";
    html += "var speed = document.getElementById('speed').value;";
    html += "document.getElementById('speedValue').innerHTML = speed;";
    html += "fetch('/control?cmd=SP&val='+speed);}";
    html += "</script></body></html>";
    
    return html;
}
