# ESP32 小车项目

## 介绍
这是一个基于 ESP32 的小车项目，旨在通过 ESP32 模块实现小车的无线控制。通过 Wi-Fi 连接，用户可以使用手机热点控制小车，并通过浏览器访问遥控页面进行操作。

## 软件架构
本项目采用 Arduino 框架进行开发，主要功能代码集中在 `main/main.ino` 文件中。整体架构围绕 ESP32 的 Wi-Fi 功能和小车的电机控制模块展开。

## 安装教程
1. 使用 Arduino IDE 打开并烧录 `main` 文件夹中的代码。
2. 在 Arduino IDE 中安装所有必要的库文件。
3. 在 Arduino IDE 中选择开发板为 **ESP32 Dev Module**。
4. 如果无法连接开发板，请安装 **CP210X 驱动**。

## 使用说明
1. 请将代码提交到 `master` 分支以外的其他分支。
2. 在代码中修改 Wi-Fi 名称和密码，以便使用手机热点连接小车。
3. 将代码烧录到 ESP32 后，设备会显示一个 IP 地址，通过浏览器访问该 IP 地址即可进入遥控页面。

## 参与贡献
1. Fork 本仓库。
2. 创建新的功能分支（如 `Feat_xxx`）。
3. 提交您的代码更改。
4. 创建 Pull Request 并等待审核。

## 特技
1. 使用 `Readme_XXX.md` 文件支持多语言文档，例如 `Readme_en.md` 和 `Readme_zh.md`。
2. 了解更多关于 Gitee 的信息，请访问 [Gitee 官方博客](https://blog.gitee.com)。
3. 探索 Gitee 上的优秀开源项目，请访问 [https://gitee.com/explore](https://gitee.com/explore)。
4. GVP（Gitee 最有价值开源项目）是 Gitee 上的高质量开源项目评选栏目。
5. Gitee 官方使用手册请访问 [https://gitee.com/help](https://gitee.com/help)。
6. 想了解 Gitee 会员风采吗？请访问 [Gitee 封面人物](https://gitee.com/gitee-stars/) 页面。