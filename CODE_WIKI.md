# 小智 ESP32 AI 聊天机器人 - Code Wiki

## 目录

1. [项目概述](#项目概述)
2. [项目架构](#项目架构)
3. [核心模块详解](#核心模块详解)
4. [关键类与函数说明](#关键类与函数说明)
5. [依赖关系](#依赖关系)
6. [项目构建与运行](#项目构建与运行)
7. [扩展开发指南](#扩展开发指南)

---

## 项目概述

### 项目简介

小智 ESP32 AI 聊天机器人是一个基于 ESP32 系列芯片的开源 AI 语音交互设备固件项目。该项目利用大语言模型（如 Qwen、DeepSeek）的 AI 能力，通过 MCP（Model Context Protocol）协议实现多端控制，构建了一个完整的语音交互入口。

**当前版本**: v2.2.6  
**开发框架**: ESP-IDF 5.4+  
**编程语言**: C++ (主要) / C  
**许可证**: MIT

### 核心特性

- **网络支持**: Wi-Fi / ML307 Cat.1 4G / NT26
- **离线唤醒**: 基于 ESP-SR 的离线语音唤醒
- **通信协议**: WebSocket 或 MQTT+UDP
- **音频编解码**: OPUS 格式
- **语音交互**: 流式 ASR + LLM + TTS 架构
- **声纹识别**: 基于 3D Speaker
- **显示支持**: OLED / LCD 显示屏，支持表情显示
- **多语言**: 支持中文、英文、日文等 30+ 种语言
- **芯片平台**: ESP32-C3、ESP32-S3、ESP32-P4
- **MCP 协议**: 设备端和云端 MCP 实现

---

## 项目架构

### 整体架构图

```
┌─────────────────────────────────────────────────────────────┐
│                        Application Layer                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ Application  │  │Settings/OTA  │  │Device State  │      │
│  │   (主应用)    │  │ (设置/升级)   │  │  (状态机)     │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                        Service Layer                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │AudioService  │  │  McpServer   │  │  Protocol    │      │
│  │  (音频服务)   │  │ (MCP服务器)   │  │  (通信协议)   │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                        Hardware Layer                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │    Board     │  │  AudioCodec  │  │   Display    │      │
│  │  (开发板抽象) │  │  (音频编解码) │  │   (显示设备)  │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │     Led      │  │   Camera     │  │   Network    │      │
│  │   (LED灯)    │  │   (摄像头)    │  │   (网络接口)  │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
```

### 目录结构

```
xiaozhi-esp32/
├── main/                      # 主程序源码
│   ├── application.h/cc       # 应用主类
│   ├── main.cc                # 程序入口
│   ├── audio/                 # 音频处理模块
│   │   ├── audio_codec.h/cc   # 音频编解码抽象
│   │   ├── audio_service.h/cc # 音频服务
│   │   ├── codecs/            # 音频编解码器实现
│   │   ├── demuxer/           # 音频解复用器
│   │   ├── processors/        # 音频处理器
│   │   └── wake_words/        # 唤醒词检测
│   ├── boards/                # 开发板支持
│   │   ├── common/            # 通用板级支持
│   │   └── [board-name]/      # 各开发板实现
│   ├── display/               # 显示模块
│   │   ├── display.h/cc       # 显示抽象
│   │   ├── lcd_display.cc     # LCD 显示
│   │   ├── oled_display.cc    # OLED 显示
│   │   └── lvgl_display/      # LVGL 显示
│   ├── led/                   # LED 模块
│   ├── protocols/             # 通信协议
│   │   ├── protocol.h/cc      # 协议抽象
│   │   ├── websocket_protocol.cc
│   │   └── mqtt_protocol.cc
│   ├── assets/                # 资源文件
│   │   ├── common/            # 通用音频
│   │   └── locales/           # 多语言资源
│   ├── mcp_server.h/cc        # MCP 服务器
│   ├── settings.h/cc          # 设置管理
│   ├── device_state.h         # 设备状态定义
│   ├── device_state_machine.cc
│   ├── ota.h/cc               # OTA 升级
│   └── system_info.h/cc       # 系统信息
├── docs/                      # 文档
├── scripts/                   # 构建脚本
├── partitions/                # 分区表
├── CMakeLists.txt             # CMake 配置
├── sdkconfig.defaults         # SDK 默认配置
└── idf_component.yml          # 组件依赖
```

---

## 核心模块详解

### 1. Application 模块

**位置**: `main/application.h/cc`

**职责**: 应用程序核心控制器，管理整个设备的生命周期和事件循环。

**关键功能**:
- 初始化所有子系统
- 运行主事件循环
- 管理设备状态转换
- 处理网络事件
- 协调音频服务、协议层和 MCP 服务器

**事件类型**:
```cpp
#define MAIN_EVENT_SCHEDULE             (1 << 0)  // 调度回调
#define MAIN_EVENT_SEND_AUDIO           (1 << 1)  // 发送音频
#define MAIN_EVENT_WAKE_WORD_DETECTED   (1 << 2)  // 唤醒词检测
#define MAIN_EVENT_VAD_CHANGE           (1 << 3)  // VAD 变化
#define MAIN_EVENT_NETWORK_CONNECTED    (1 << 7)  // 网络连接
#define MAIN_EVENT_NETWORK_DISCONNECTED (1 << 8)  // 网络断开
#define MAIN_EVENT_TOGGLE_CHAT          (1 << 9)  // 切换聊天
```

### 2. Board 模块

**位置**: `main/boards/`

**职责**: 硬件抽象层，为不同开发板提供统一的接口。

**核心类**: `Board` (基类)

**主要方法**:
```cpp
class Board {
public:
    virtual std::string GetBoardType() = 0;
    virtual AudioCodec* GetAudioCodec() = 0;
    virtual Display* GetDisplay();
    virtual Camera* GetCamera();
    virtual NetworkInterface* GetNetwork() = 0;
    virtual void StartNetwork() = 0;
    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging);
    virtual void SetPowerSaveLevel(PowerSaveLevel level) = 0;
};
```

**支持的开发板类型** (70+):
- 立创实战派 ESP32-S3
- 乐鑫 ESP32-S3-BOX3
- M5Stack CoreS3 / AtomS3R
- 神奇按钮 2.4
- 微雪电子系列
- LILYGO T-Circle-S3
- 等等...

### 3. Audio 模块

**位置**: `main/audio/`

**职责**: 音频采集、处理、编解码和播放。

**数据流**:
```
录音流: (MIC) → [Processors] → {Encode Queue} → [Opus Encoder] → {Send Queue} → (Server)
播放流: (Server) → {Decode Queue} → [Opus Decoder] → {Playback Queue} → (Speaker)
```

**核心组件**:
- `AudioService`: 音频服务主类
- `AudioCodec`: 音频编解码器抽象
- `AudioProcessor`: 音频处理器（AEC、VAD等）
- `WakeWord`: 唤醒词检测

**支持的音频编解码器**:
- ES8311 / ES8374 / ES8388 / ES8389
- Box Audio Codec
- Dummy Audio Codec

### 4. Protocol 模块

**位置**: `main/protocols/`

**职责**: 定义和实现设备与服务器之间的通信协议。

**支持的协议**:
1. **WebSocket 协议** (`websocket_protocol.cc`)
   - 实时双向通信
   - 低延迟
   - 适合流式音频传输

2. **MQTT + UDP 协议** (`mqtt_protocol.cc`)
   - MQTT 用于控制信令
   - UDP 用于音频数据传输
   - 适合网络不稳定环境

**协议抽象**:
```cpp
class Protocol {
public:
    virtual bool Start() = 0;
    virtual bool OpenAudioChannel() = 0;
    virtual void CloseAudioChannel(bool send_goodbye = true) = 0;
    virtual bool SendAudio(std::unique_ptr<AudioStreamPacket> packet) = 0;
    virtual void SendWakeWordDetected(const std::string& wake_word);
    virtual void SendStartListening(ListeningMode mode);
    virtual void SendStopListening();
    virtual void SendMcpMessage(const std::string& message);
};
```

### 5. MCP Server 模块

**位置**: `main/mcp_server.h/cc`

**职责**: 实现 MCP (Model Context Protocol) 服务器，允许 AI 调用设备功能。

**核心类**:
- `McpServer`: MCP 服务器单例
- `McpTool`: MCP 工具定义
- `Property`: 工具参数属性
- `PropertyList`: 参数列表

**内置工具**:
- `self.get_device_status`: 获取设备状态
- `self.audio_speaker.set_volume`: 设置音量
- `self.audio_speaker.get_volume`: 获取音量
- `self.led.set_brightness`: 设置 LED 亮度
- `self.camera.take_photo`: 拍照
- 等等...

### 6. Display 模块

**位置**: `main/display/`

**职责**: 管理显示屏，显示表情、状态和聊天内容。

**核心类**:
- `Display`: 显示抽象基类
- `LcdDisplay`: LCD 显示实现
- `OledDisplay`: OLED 显示实现
- `LvglDisplay`: 基于 LVGL 的显示实现

**显示功能**:
- 状态栏显示
- 表情动画
- 聊天消息
- 通知提示

### 7. Device State Machine 模块

**位置**: `main/device_state_machine.h/cc`

**职责**: 管理设备状态转换。

**设备状态**:
```cpp
enum DeviceState {
    kDeviceStateUnknown,          // 未知
    kDeviceStateStarting,         // 启动中
    kDeviceStateWifiConfiguring,  // WiFi 配置中
    kDeviceStateIdle,             // 空闲
    kDeviceStateConnecting,       // 连接中
    kDeviceStateListening,        // 监听中
    kDeviceStateSpeaking,         // 说话中
    kDeviceStateUpgrading,        // 升级中
    kDeviceStateActivating,       // 激活中
    kDeviceStateAudioTesting,     // 音频测试中
    kDeviceStateFatalError        // 致命错误
};
```

---

## 关键类与函数说明

### Application 类

**文件**: [main/application.h](main/application.h)

**单例模式**: 通过 `Application::GetInstance()` 获取实例

**关键方法**:

| 方法 | 说明 |
|------|------|
| `Initialize()` | 初始化应用，设置显示、音频、网络回调 |
| `Run()` | 运行主事件循环，永不返回 |
| `SetDeviceState(DeviceState)` | 请求状态转换 |
| `Schedule(std::function<void()>)` | 在主任务中调度回调 |
| `Alert(const char*, const char*, const char*, const std::string_view&)` | 显示警告 |
| `ToggleChatState()` | 切换聊天状态 |
| `StartListening()` | 开始监听 |
| `StopListening()` | 停止监听 |
| `WakeWordInvoke(const std::string&)` | 唤醒词触发 |
| `UpgradeFirmware(const std::string&, const std::string&)` | OTA 升级 |
| `SendMcpMessage(const std::string&)` | 发送 MCP 消息 |

### AudioService 类

**文件**: [main/audio/audio_service.h](main/audio/audio_service.h)

**职责**: 管理音频输入输出、编解码和唤醒词检测

**关键方法**:

| 方法 | 说明 |
|------|------|
| `Initialize(AudioCodec*)` | 初始化音频服务 |
| `Start()` | 启动音频服务 |
| `Stop()` | 停止音频服务 |
| `EnableWakeWordDetection(bool)` | 启用/禁用唤醒词检测 |
| `EnableVoiceProcessing(bool)` | 启用/禁用语音处理 |
| `PushPacketToDecodeQueue(...)` | 推送音频包到解码队列 |
| `PopPacketFromSendQueue()` | 从发送队列弹出音频包 |
| `PlaySound(const std::string_view&)` | 播放音效 |
| `IsVoiceDetected()` | 检测是否有语音 |

### Board 类

**文件**: [main/boards/common/board.h](main/boards/common/board.h)

**设计模式**: 工厂模式 + 单例模式

**关键方法**:

| 方法 | 说明 |
|------|------|
| `GetBoardType()` | 获取开发板类型 |
| `GetAudioCodec()` | 获取音频编解码器 |
| `GetDisplay()` | 获取显示设备 |
| `GetCamera()` | 获取摄像头 |
| `GetNetwork()` | 获取网络接口 |
| `StartNetwork()` | 启动网络 |
| `GetBatteryLevel(...)` | 获取电池电量 |
| `SetPowerSaveLevel(...)` | 设置省电级别 |

### McpServer 类

**文件**: [main/mcp_server.h](main/mcp_server.h)

**设计模式**: 单例模式

**关键方法**:

| 方法 | 说明 |
|------|------|
| `AddTool(McpTool*)` | 添加工具 |
| `AddTool(name, description, properties, callback)` | 添加工具（简化版） |
| `AddUserOnlyTool(...)` | 添加仅用户可见的工具 |
| `ParseMessage(const cJSON*)` | 解析 MCP 消息 |
| `ParseMessage(const std::string&)` | 解析 MCP 消息（字符串版本） |

### McpTool 类

**文件**: [main/mcp_server.h](main/mcp_server.h)

**用途**: 定义一个 MCP 工具

**构造函数**:
```cpp
McpTool(
    const std::string& name,           // 工具名称
    const std::string& description,    // 工具描述
    const PropertyList& properties,    // 参数列表
    std::function<ReturnValue(const PropertyList&)> callback  // 回调函数
)
```

### Protocol 类

**文件**: [main/protocols/protocol.h](main/protocols/protocol.h)

**设计模式**: 抽象基类

**关键方法**:

| 方法 | 说明 |
|------|------|
| `Start()` | 启动协议 |
| `OpenAudioChannel()` | 打开音频通道 |
| `CloseAudioChannel(bool)` | 关闭音频通道 |
| `SendAudio(...)` | 发送音频数据 |
| `SendWakeWordDetected(...)` | 发送唤醒词检测事件 |
| `SendStartListening(...)` | 发送开始监听 |
| `SendStopListening()` | 发送停止监听 |
| `SendMcpMessage(...)` | 发送 MCP 消息 |

---

## 依赖关系

### 外部组件依赖

项目使用 ESP-IDF 组件管理器管理依赖，主要依赖包括：

| 组件 | 版本 | 用途 |
|------|------|------|
| `lvgl/lvgl` | ~9.5.0 | 图形界面库 |
| `espressif/esp-sr` | ~2.3.0 | 语音识别和唤醒词 |
| `espressif/esp_codec_dev` | ~1.5.6 | 音频编解码设备 |
| `espressif/esp32-camera` | ^2.1.6 | 摄像头驱动 |
| `espressif/led_strip` | ~3.0.2 | LED 灯带控制 |
| `espressif/button` | ~4.1.5 | 按钮驱动 |
| `78/xiaozhi-fonts` | ~1.6.0 | 小智字体库 |
| `78/esp-ml307` | ~3.6.5 | ML307 4G 模块支持 |
| `espressif/esp_audio_codec` | ~2.4.1 | 音频编解码 |
| `espressif/esp_audio_effects` | ~1.2.1 | 音频效果处理 |

### 模块间依赖关系

```
Application
    ├── AudioService
    │   ├── AudioCodec
    │   ├── AudioProcessor
    │   └── WakeWord
    ├── Board
    │   ├── AudioCodec
    │   ├── Display
    │   ├── Camera
    │   ├── Led
    │   └── NetworkInterface
    ├── Protocol
    │   └── NetworkInterface
    ├── McpServer
    │   └── McpTool
    ├── DeviceStateMachine
    ├── Settings
    └── Ota
```

---

## 项目构建与运行

### 开发环境要求

- **IDE**: Cursor 或 VSCode
- **插件**: ESP-IDF 插件
- **SDK 版本**: ESP-IDF 5.4 或以上
- **操作系统**: Linux（推荐）/ Windows

### 构建步骤

1. **安装 ESP-IDF**
   ```bash
   # 参考 https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32/get-started/
   ```

2. **克隆项目**
   ```bash
   git clone https://github.com/78/xiaozhi-esp32.git
   cd xiaozhi-esp32
   ```

3. **配置项目**
   ```bash
   idf.py menuconfig
   # 选择开发板类型和其他配置
   ```

4. **编译项目**
   ```bash
   idf.py build
   ```

5. **烧录固件**
   ```bash
   idf.py -p PORT flash
   ```

6. **查看日志**
   ```bash
   idf.py -p PORT monitor
   ```

### 配置选项

主要配置项位于 `main/Kconfig.projbuild`：

- **开发板类型**: 选择目标开发板
- **语言**: 选择界面语言
- **通信协议**: WebSocket 或 MQTT+UDP
- **音频处理器**: 是否启用 AEC 等
- **唤醒词**: 默认唤醒词或自定义唤醒词

### 分区表

项目使用自定义分区表，位于 `partitions/v2/`：

- **16m.csv**: 16MB Flash 设备
- **32m.csv**: 32MB Flash 设备
- **4m.csv**: 4MB Flash 设备
- **8m.csv**: 8MB Flash 设备

分区包括：
- factory: 工厂分区
- ota_0/ota_1: OTA 分区
- assets: 资源分区（字体、表情等）
- storage: 存储分区

---

## 扩展开发指南

### 添加新的开发板支持

1. **创建开发板目录**
   ```
   main/boards/your-board/
   ├── config.h          # 配置头文件
   ├── config.json       # 配置 JSON
   ├── your_board.cc     # 实现文件
   └── README.md         # 说明文档
   ```

2. **实现 Board 子类**
   ```cpp
   class YourBoard : public Board {
   public:
       YourBoard();
       ~YourBoard();
       
       std::string GetBoardType() override { return "your-board"; }
       AudioCodec* GetAudioCodec() override;
       Display* GetDisplay() override;
       NetworkInterface* GetNetwork() override;
       void StartNetwork() override;
       void SetPowerSaveLevel(PowerSaveLevel level) override;
       std::string GetBoardJson() override;
       std::string GetDeviceStatusJson() override;
   };
   ```

3. **添加 CMake 配置**
   
   在 `main/CMakeLists.txt` 中添加：
   ```cmake
   elseif(CONFIG_BOARD_TYPE_YOUR_BOARD)
       set(BOARD_TYPE "your-board")
       set(BUILTIN_TEXT_FONT font_puhui_basic_16_4)
       set(BUILTIN_ICON_FONT font_awesome_16_4)
   ```

4. **添加 Kconfig 选项**
   
   在 `main/Kconfig.projbuild` 中添加配置选项。

### 添加新的 MCP 工具

1. **定义工具**
   ```cpp
   auto tool = new McpTool(
       "self.your_tool",
       "工具描述",
       PropertyList({
           Property("param1", kPropertyTypeString),
           Property("param2", kPropertyTypeInteger, 0, 100)  // 0-100 范围
       }),
       [](const PropertyList& props) -> ReturnValue {
           std::string param1 = props["param1"].value<std::string>();
           int param2 = props["param2"].value<int>();
           // 执行工具逻辑
           return "执行结果";
       }
   );
   McpServer::GetInstance().AddTool(tool);
   ```

### 添加新的通信协议

1. **创建协议类**
   ```cpp
   class YourProtocol : public Protocol {
   public:
       bool Start() override;
       bool OpenAudioChannel() override;
       void CloseAudioChannel(bool send_goodbye) override;
       bool IsAudioChannelOpened() const override;
       bool SendAudio(std::unique_ptr<AudioStreamPacket> packet) override;
   protected:
       bool SendText(const std::string& text) override;
   };
   ```

2. **在 Application 中集成**
   
   修改 `Application::InitializeProtocol()` 方法。

---

## 附录

### 设备状态转换图

```
┌─────────────┐
│   Starting  │
└──────┬──────┘
       │
       v
┌─────────────┐     ┌──────────────┐
│WifiConfiguring│◄────┤   Idle       │
└─────────────┘     └──────┬───────┘
                          │
                          v
                    ┌──────────────┐
                    │  Connecting  │
                    └──────┬───────┘
                           │
              ┌────────────┼────────────┐
              v            v            v
        ┌──────────┐ ┌──────────┐ ┌──────────┐
        │Listening │ │Speaking  │ │Activating│
        └──────────┘ └──────────┘ └──────────┘
```

### 音频数据流图

```
┌─────────────────────────────────────────────────────────────┐
│                        音频输入流                            │
│                                                              │
│  ┌─────┐    ┌──────────┐    ┌────────┐    ┌──────────┐     │
│  │ MIC │───►│Processor │───►│ Encode │───►│Send Queue│──►  │
│  └─────┘    │(AEC/VAD) │    │ Queue  │    └──────────┘     │
│             └──────────┘    └────────┘                      │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                        音频输出流                            │
│                                                              │
│  ┌──────────┐    ┌────────┐    ┌───────────┐    ┌─────┐    │
│  │Decode    │───►│Playback│───►│  Opus     │───►│Speaker   │
│  │Queue     │    │ Queue  │    │  Decoder  │    │     │    │
│  └──────────┘    └────────┘    └───────────┘    └─────┘    │
└─────────────────────────────────────────────────────────────┘
```

### 参考文档

- [自定义开发板指南](docs/custom-board_zh.md)
- [MCP 协议物联网控制用法说明](docs/mcp-usage_zh.md)
- [MCP 协议交互流程](docs/mcp-protocol_zh.md)
- [MQTT + UDP 混合通信协议文档](docs/mqtt-udp_zh.md)
- [WebSocket 通信协议文档](docs/websocket_zh.md)
- [代码风格指南](docs/code_style_zh.md)

---

**文档版本**: 1.0  
**最后更新**: 2026-05-07  
**维护者**: 小智开源社区
