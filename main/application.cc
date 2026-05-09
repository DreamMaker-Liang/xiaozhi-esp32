/**
 * @file application.cc
 * @brief 主应用类实现文件
 *
 * Application 类是 ESP32 智能设备的核心控制器，负责管理设备的完整生命周期，包括：
 * - 设备状态机管理
 * - 音频服务初始化与控制
 * - 网络事件处理
 * - 协议通信（MQTT/WebSocket）
 * - 语音唤醒与语音交互
 * - OTA 固件升级
 * - 用户界面显示控制
 */

#include "application.h"
#include "assets.h"
#include "assets/lang_config.h"
#include "audio_codec.h"
#include "board.h"
#include "display.h"
#include "mcp_server.h"
#include "mqtt_protocol.h"
#include "settings.h"
#include "system_info.h"
#include "websocket_protocol.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <arpa/inet.h>
#include <cJSON.h>
#include <font_awesome.h>
#include <mqtt_client.h>
#include <cstring>

#define TAG "Application"

// MQTT 配置常量
static const char* MQTT_BROKER = "mqtts://h1eee66a.ala.cn-hangzhou.emqxsl.cn:8883";
static const char* MQTT_USERNAME = "xiaozhi";
static const char* MQTT_PASSWORD = "2495288449";
static const char* MQTT_TOPIC_light = "devices/light";
static const char* MQTT_TOPIC_fan = "devices/fan";
static const char* MQTT_TOPIC_humidifier = "devices/humidifier";
static const char* MQTT_TOPIC_ac = "devices/ac";
static const char* MQTT_SUBSCRIBE_xiaozhi = "devices/xiaozhi";
static const char* MQTT_CERT =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n"
    "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
    "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n"
    "QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n"
    "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
    "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n"
    "9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n"
    "CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n"
    "nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n"
    "43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n"
    "T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n"
    "gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n"
    "BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n"
    "TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n"
    "DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n"
    "hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n"
    "06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n"
    "PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n"
    "YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n"
    "CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n"
    "-----END CERTIFICATE-----";

/**
 * @brief Application 构造函数
 *
 * 初始化事件组、AEC（声学回声消除）模式配置和时钟定时器。
 * AEC 模式根据编译配置选择：设备端 AEC、服务端 AEC 或关闭 AEC。
 */
Application::Application() {
    // 创建 FreeRTOS 事件组，用于事件通知和同步
    event_group_ = xEventGroupCreate();

    // 根据编译配置设置 AEC（声学回声消除）模式
#if CONFIG_USE_DEVICE_AEC && CONFIG_USE_SERVER_AEC
#error "CONFIG_USE_DEVICE_AEC and CONFIG_USE_SERVER_AEC cannot be enabled at the same time"
#elif CONFIG_USE_DEVICE_AEC
    aec_mode_ = kAecOnDeviceSide;
#elif CONFIG_USE_SERVER_AEC
    aec_mode_ = kAecOnServerSide;
#else
    aec_mode_ = kAecOff;
#endif

    // 创建时钟定时器，每秒触发一次 MAIN_EVENT_CLOCK_TICK 事件
    esp_timer_create_args_t clock_timer_args = {.callback =
                                                    [](void* arg) {
                                                        Application* app = (Application*)arg;
                                                        xEventGroupSetBits(app->event_group_,
                                                                           MAIN_EVENT_CLOCK_TICK);
                                                    },
                                                .arg = this,
                                                .dispatch_method = ESP_TIMER_TASK,
                                                .name = "clock_timer",
                                                .skip_unhandled_events = true};
    esp_timer_create(&clock_timer_args, &clock_timer_handle_);
}

/**
 * @brief Application 析构函数
 *
 * 清理资源：停止并删除时钟定时器，删除事件组。
 */
Application::~Application() {
    if (clock_timer_handle_ != nullptr) {
        esp_timer_stop(clock_timer_handle_);
        esp_timer_delete(clock_timer_handle_);
    }
    vEventGroupDelete(event_group_);
}

/**
 * @brief 设置设备状态
 * @param state 目标设备状态
 * @return 状态转换是否成功
 */
bool Application::SetDeviceState(DeviceState state) { return state_machine_.TransitionTo(state); }

/**
 * @brief 初始化应用程序
 *
 * 执行设备启动时的初始化流程：
 * 1. 设置设备状态为启动中
 * 2. 初始化显示界面
 * 3. 初始化音频服务并设置回调函数
 * 4. 设置状态机监听器
 * 5. 启动时钟定时器（每秒更新状态栏）
 * 6. 注册 MCP 工具
 * 7. 设置网络事件回调
 * 8. 启动网络连接
 */
void Application::Initialize() {
    auto& board = Board::GetInstance();
    SetDeviceState(kDeviceStateStarting);

    // 初始化显示界面
    auto display = board.GetDisplay();
    display->SetupUI();
    // 显示设备名称和版本信息
    display->SetChatMessage("system", SystemInfo::GetUserAgent().c_str());

    // 初始化音频服务
    auto codec = board.GetAudioCodec();
    audio_service_.Initialize(codec);
    audio_service_.Start();

    // 设置音频服务回调函数
    AudioServiceCallbacks callbacks;
    callbacks.on_send_queue_available = [this]() {
        xEventGroupSetBits(event_group_, MAIN_EVENT_SEND_AUDIO);
    };
    callbacks.on_wake_word_detected = [this](const std::string& wake_word) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_WAKE_WORD_DETECTED);
    };
    callbacks.on_vad_change = [this](bool speaking) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_VAD_CHANGE);
    };
    audio_service_.SetCallbacks(callbacks);

    // 添加状态变化监听器
    state_machine_.AddStateChangeListener([this](DeviceState old_state, DeviceState new_state) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_STATE_CHANGED);
    });

    // 启动时钟定时器（每秒触发一次，用于更新状态栏）
    esp_timer_start_periodic(clock_timer_handle_, 1000000);

    // 注册 MCP（Model Context Protocol）工具
    auto& mcp_server = McpServer::GetInstance();
    mcp_server.AddCommonTools();
    mcp_server.AddUserOnlyTools();

    // 设置网络事件回调，用于更新 UI 和处理网络状态变化
    board.SetNetworkEventCallback([this](NetworkEvent event, const std::string& data) {
        auto display = Board::GetInstance().GetDisplay();

        switch (event) {
            case NetworkEvent::Scanning:
                display->ShowNotification(Lang::Strings::SCANNING_WIFI, 30000);
                xEventGroupSetBits(event_group_, MAIN_EVENT_NETWORK_DISCONNECTED);
                break;
            case NetworkEvent::Connecting: {
                if (data.empty()) {
                    // 蜂窝网络 - 尚未获取运营商信息
                    display->SetStatus(Lang::Strings::REGISTERING_NETWORK);
                } else {
                    // WiFi 或已获取运营商信息的蜂窝网络
                    std::string msg = Lang::Strings::CONNECT_TO;
                    msg += data;
                    msg += "...";
                    display->ShowNotification(msg.c_str(), 30000);
                }
                break;
            }
            case NetworkEvent::Connected: {
                std::string msg = Lang::Strings::CONNECTED_TO;
                msg += data;
                display->ShowNotification(msg.c_str(), 30000);
                xEventGroupSetBits(event_group_, MAIN_EVENT_NETWORK_CONNECTED);
                break;
            }
            case NetworkEvent::Disconnected:
                xEventGroupSetBits(event_group_, MAIN_EVENT_NETWORK_DISCONNECTED);
                break;
            case NetworkEvent::WifiConfigModeEnter:
                // WiFi 配置模式进入由 WifiBoard 内部处理
                break;
            case NetworkEvent::WifiConfigModeExit:
                // WiFi 配置模式退出由 WifiBoard 内部处理
                break;
            // 蜂窝调制解调器特定事件
            case NetworkEvent::ModemDetecting:
                display->SetStatus(Lang::Strings::DETECTING_MODULE);
                break;
            case NetworkEvent::ModemErrorNoSim:
                Alert(Lang::Strings::ERROR, Lang::Strings::PIN_ERROR, "triangle_exclamation",
                      Lang::Sounds::OGG_ERR_PIN);
                break;
            case NetworkEvent::ModemErrorRegDenied:
                Alert(Lang::Strings::ERROR, Lang::Strings::REG_ERROR, "triangle_exclamation",
                      Lang::Sounds::OGG_ERR_REG);
                break;
            case NetworkEvent::ModemErrorInitFailed:
                Alert(Lang::Strings::ERROR, Lang::Strings::MODEM_INIT_ERROR, "triangle_exclamation",
                      Lang::Sounds::OGG_EXCLAMATION);
                break;
            case NetworkEvent::ModemErrorTimeout:
                display->SetStatus(Lang::Strings::REGISTERING_NETWORK);
                break;
        }
    });

    // 异步启动网络连接
    board.StartNetwork();

    // 立即更新状态栏显示网络状态
    display->UpdateStatusBar(true);

    // 初始化 MQTT 客户端（网络连接成功后自动连接）
    MqttInit();
}

/**
 * @brief 主事件循环
 *
 * 应用程序的核心事件处理循环，负责处理所有系统事件：
 * - 网络连接/断开事件
 * - 激活完成事件
 * - 状态变化事件
 * - 语音交互事件（唤醒词检测、开始/停止监听）
 * - 音频发送事件
 * - VAD（语音活动检测）变化事件
 * - 定时任务调度
 * - 时钟tick（每秒更新状态栏）
 */
void Application::Run() {
    // 设置主任务优先级为 10
    vTaskPrioritySet(nullptr, 10);

    // 定义需要监听的所有事件
    const EventBits_t ALL_EVENTS =
        MAIN_EVENT_SCHEDULE | MAIN_EVENT_SEND_AUDIO | MAIN_EVENT_WAKE_WORD_DETECTED |
        MAIN_EVENT_VAD_CHANGE | MAIN_EVENT_CLOCK_TICK | MAIN_EVENT_ERROR |
        MAIN_EVENT_NETWORK_CONNECTED | MAIN_EVENT_NETWORK_DISCONNECTED | MAIN_EVENT_TOGGLE_CHAT |
        MAIN_EVENT_START_LISTENING | MAIN_EVENT_STOP_LISTENING | MAIN_EVENT_ACTIVATION_DONE |
        MAIN_EVENT_STATE_CHANGED | MAIN_EVENT_MQTT_CONNECTED | MAIN_EVENT_MQTT_DISCONNECTED |
        MAIN_EVENT_MQTT_DATA;

    // 无限循环处理事件
    while (true) {
        auto bits = xEventGroupWaitBits(event_group_, ALL_EVENTS, pdTRUE, pdFALSE, portMAX_DELAY);

        // 处理错误事件
        if (bits & MAIN_EVENT_ERROR) {
            SetDeviceState(kDeviceStateIdle);
            Alert(Lang::Strings::ERROR, last_error_message_.c_str(), "circle_xmark",
                  Lang::Sounds::OGG_EXCLAMATION);
        }

        // 处理网络连接事件
        if (bits & MAIN_EVENT_NETWORK_CONNECTED) {
            HandleNetworkConnectedEvent();
        }

        // 处理网络断开事件
        if (bits & MAIN_EVENT_NETWORK_DISCONNECTED) {
            HandleNetworkDisconnectedEvent();
        }

        // 处理激活完成事件
        if (bits & MAIN_EVENT_ACTIVATION_DONE) {
            HandleActivationDoneEvent();
        }

        // 处理状态变化事件
        if (bits & MAIN_EVENT_STATE_CHANGED) {
            HandleStateChangedEvent();
        }

        // 处理聊天状态切换事件
        if (bits & MAIN_EVENT_TOGGLE_CHAT) {
            HandleToggleChatEvent();
        }

        // 处理开始监听事件
        if (bits & MAIN_EVENT_START_LISTENING) {
            HandleStartListeningEvent();
        }

        // 处理停止监听事件
        if (bits & MAIN_EVENT_STOP_LISTENING) {
            HandleStopListeningEvent();
        }

        // 处理音频发送事件
        if (bits & MAIN_EVENT_SEND_AUDIO) {
            while (auto packet = audio_service_.PopPacketFromSendQueue()) {
                if (protocol_ && !protocol_->SendAudio(std::move(packet))) {
                    break;
                }
            }
        }

        // 处理唤醒词检测事件
        if (bits & MAIN_EVENT_WAKE_WORD_DETECTED) {
            HandleWakeWordDetectedEvent();
        }

        // 处理 VAD（语音活动检测）变化事件
        if (bits & MAIN_EVENT_VAD_CHANGE) {
            if (GetDeviceState() == kDeviceStateListening) {
                auto led = Board::GetInstance().GetLed();
                led->OnStateChanged();
            }
        }

        // 处理调度任务
        if (bits & MAIN_EVENT_SCHEDULE) {
            std::unique_lock<std::mutex> lock(mutex_);
            auto tasks = std::move(main_tasks_);
            lock.unlock();
            for (auto& task : tasks) {
                task();
            }
        }

        // 处理时钟tick事件（每秒触发）
        if (bits & MAIN_EVENT_CLOCK_TICK) {
            clock_ticks_++;
            auto display = Board::GetInstance().GetDisplay();
            display->UpdateStatusBar();

            // 每10秒打印一次调试信息（堆内存统计）
            if (clock_ticks_ % 10 == 0) {
                SystemInfo::PrintHeapStats();
            }
        }

        // 处理 MQTT 连接事件
        if (bits & MAIN_EVENT_MQTT_CONNECTED) {
            HandleMqttConnectedEvent();
        }

        // 处理 MQTT 断开连接事件
        if (bits & MAIN_EVENT_MQTT_DISCONNECTED) {
            HandleMqttDisconnectedEvent();
        }

        // 处理 MQTT 数据接收事件
        if (bits & MAIN_EVENT_MQTT_DATA) {
            HandleMqttDataEvent();
        }
    }
}

/**
 * @brief 处理网络连接事件
 *
 * 当网络连接成功时触发，主要处理：
 * - 如果设备处于启动或WiFi配置状态，启动激活任务
 * - 更新状态栏显示网络状态
 */
void Application::HandleNetworkConnectedEvent() {
    ESP_LOGI(TAG, "Network connected");
    auto state = GetDeviceState();

    // 如果设备处于启动或WiFi配置状态，启动激活流程
    if (state == kDeviceStateStarting || state == kDeviceStateWifiConfiguring) {
        SetDeviceState(kDeviceStateActivating);
        if (activation_task_handle_ != nullptr) {
            ESP_LOGW(TAG, "Activation task already running");
            return;
        }

        // 创建激活任务
        xTaskCreate(
            [](void* arg) {
                Application* app = static_cast<Application*>(arg);
                app->ActivationTask();
                app->activation_task_handle_ = nullptr;
                vTaskDelete(NULL);
            },
            "activation", 4096 * 2, this, 2, &activation_task_handle_);
    }

    // 立即更新状态栏显示网络状态
    auto display = Board::GetInstance().GetDisplay();
    display->UpdateStatusBar(true);
}

/**
 * @brief 处理网络断开事件
 *
 * 当网络断开时触发，主要处理：
 * - 如果设备正在连接、监听或说话状态，关闭音频通道
 * - 更新状态栏显示网络状态
 */
void Application::HandleNetworkDisconnectedEvent() {
    auto state = GetDeviceState();
    // 网络断开时关闭当前的音频通道
    if (state == kDeviceStateConnecting || state == kDeviceStateListening ||
        state == kDeviceStateSpeaking) {
        ESP_LOGI(TAG, "Closing audio channel due to network disconnection");
        protocol_->CloseAudioChannel();
    }

    // 立即更新状态栏显示网络状态
    auto display = Board::GetInstance().GetDisplay();
    display->UpdateStatusBar(true);
}

/**
 * @brief 处理激活完成事件
 *
 * 当设备激活完成后触发，主要处理：
 * - 设置设备状态为空闲
 * - 显示版本信息
 * - 释放OTA对象
 * - 设置低功耗模式
 * - 播放成功提示音
 */
void Application::HandleActivationDoneEvent() {
    ESP_LOGI(TAG, "Activation done");

    SystemInfo::PrintHeapStats();
    SetDeviceState(kDeviceStateIdle);

    has_server_time_ = ota_->HasServerTime();

    auto display = Board::GetInstance().GetDisplay();
    std::string message = std::string(Lang::Strings::VERSION) + ota_->GetCurrentVersion();
    display->ShowNotification(message.c_str());
    display->SetChatMessage("system", "");

    // 激活完成后释放OTA对象
    ota_.reset();
    auto& board = Board::GetInstance();
    board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);

    // 播放成功提示音，表示设备已就绪
    Schedule([this]() { audio_service_.PlaySound(Lang::Sounds::OGG_SUCCESS); });
}

/**
 * @brief 激活任务
 *
 * 设备激活流程的核心任务，按顺序执行：
 * 1. 创建OTA对象
 * 2. 检查并更新资源文件版本
 * 3. 检查并更新固件版本
 * 4. 初始化通信协议
 * 5. 通知主循环激活完成
 */
void Application::ActivationTask() {
    // 创建OTA对象用于激活过程
    ota_ = std::make_unique<Ota>();

    // 检查资源文件版本
    CheckAssetsVersion();

    // 检查固件版本
    CheckNewVersion();

    // 初始化通信协议
    InitializeProtocol();

    // 通知主循环激活完成
    xEventGroupSetBits(event_group_, MAIN_EVENT_ACTIVATION_DONE);
}

/**
 * @brief 检查资源文件版本
 *
 * 检查是否有新的资源文件需要下载更新，包括：
 * - 检查资源分区是否有效
 * - 从设置中读取下载URL
 * - 如果有新资源，下载并应用
 */
void Application::CheckAssetsVersion() {
    // 该函数只允许调用一次
    if (assets_version_checked_) {
        return;
    }
    assets_version_checked_ = true;

    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto& assets = Assets::GetInstance();

    // 检查资源分区是否有效
    if (!assets.partition_valid()) {
        ESP_LOGW(TAG, "Assets partition is disabled for board %s", BOARD_NAME);
        return;
    }

    Settings settings("assets", true);
    // 检查是否有需要下载的新资源
    std::string download_url = settings.GetString("download_url");

    if (!download_url.empty()) {
        settings.EraseKey("download_url");

        char message[256];
        snprintf(message, sizeof(message), Lang::Strings::FOUND_NEW_ASSETS, download_url.c_str());
        Alert(Lang::Strings::LOADING_ASSETS, message, "cloud_arrow_down",
              Lang::Sounds::OGG_UPGRADE);

        // 等待音频服务空闲3秒
        vTaskDelay(pdMS_TO_TICKS(3000));
        SetDeviceState(kDeviceStateUpgrading);
        board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
        display->SetChatMessage("system", Lang::Strings::PLEASE_WAIT);

        // 下载资源文件
        bool success =
            assets.Download(download_url, [this, display](int progress, size_t speed) -> void {
                char buffer[32];
                snprintf(buffer, sizeof(buffer), "%d%% %uKB/s", progress, speed / 1024);
                Schedule([display, message = std::string(buffer)]() {
                    display->SetChatMessage("system", message.c_str());
                });
            });

        board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 下载失败处理
        if (!success) {
            Alert(Lang::Strings::ERROR, Lang::Strings::DOWNLOAD_ASSETS_FAILED, "circle_xmark",
                  Lang::Sounds::OGG_EXCLAMATION);
            vTaskDelay(pdMS_TO_TICKS(2000));
            SetDeviceState(kDeviceStateActivating);
            return;
        }
    }

    // 应用资源文件
    assets.Apply();
    display->SetChatMessage("system", "");
    display->SetEmotion("microchip_ai");
}

/**
 * @brief 检查固件版本
 *
 * 检查是否有新的固件版本，包含重试机制：
 * - 最多重试10次，重试间隔从10秒开始翻倍
 * - 如果有新版本，执行固件升级
 * - 如果需要激活码，显示激活码并尝试激活
 */
void Application::CheckNewVersion() {
    const int MAX_RETRY = 10;
    int retry_count = 0;
    int retry_delay = 10;  // 初始重试延迟（秒）

    auto& board = Board::GetInstance();
    while (true) {
        auto display = board.GetDisplay();
        display->SetStatus(Lang::Strings::CHECKING_NEW_VERSION);

        esp_err_t err = ota_->CheckVersion();
        if (err != ESP_OK) {
            retry_count++;
            if (retry_count >= MAX_RETRY) {
                ESP_LOGE(TAG, "Too many retries, exit version check");
                return;
            }

            char error_message[128];
            snprintf(error_message, sizeof(error_message), "code=%d, url=%s", err,
                     ota_->GetCheckVersionUrl().c_str());
            char buffer[256];
            snprintf(buffer, sizeof(buffer), Lang::Strings::CHECK_NEW_VERSION_FAILED, retry_delay,
                     error_message);
            Alert(Lang::Strings::ERROR, buffer, "cloud_slash", Lang::Sounds::OGG_EXCLAMATION);

            ESP_LOGW(TAG, "Check new version failed, retry in %d seconds (%d/%d)", retry_delay,
                     retry_count, MAX_RETRY);
            for (int i = 0; i < retry_delay; i++) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                if (GetDeviceState() == kDeviceStateIdle) {
                    break;
                }
            }
            retry_delay *= 2;  // 重试延迟翻倍
            continue;
        }
        retry_count = 0;
        retry_delay = 10;  // 重置重试延迟

        // 如果有新版本，执行固件升级
        if (ota_->HasNewVersion()) {
            if (UpgradeFirmware(ota_->GetFirmwareUrl(), ota_->GetFirmwareVersion())) {
                return;  // 升级后会重启，此代码不会执行
            }
            // 升级失败则继续正常操作
        }

        // 没有新版本，标记当前版本为有效
        ota_->MarkCurrentVersionValid();
        if (!ota_->HasActivationCode() && !ota_->HasActivationChallenge()) {
            // 检查完成，退出循环
            break;
        }

        display->SetStatus(Lang::Strings::ACTIVATION);
        // 如果有激活码，显示给用户等待输入
        if (ota_->HasActivationCode()) {
            ShowActivationCode(ota_->GetActivationCode(), ota_->GetActivationMessage());
        }

        // 循环等待激活完成或超时（最多尝试10次）
        for (int i = 0; i < 10; ++i) {
            ESP_LOGI(TAG, "Activating... %d/%d", i + 1, 10);
            esp_err_t err = ota_->Activate();
            if (err == ESP_OK) {
                break;
            } else if (err == ESP_ERR_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(3000));
            } else {
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
            if (GetDeviceState() == kDeviceStateIdle) {
                break;
            }
        }
    }
}

/**
 * @brief 初始化通信协议
 *
 * 根据OTA配置选择并初始化通信协议（MQTT或WebSocket），设置各种回调函数：
 * - OnConnected: 连接成功时关闭警告
 * - OnNetworkError: 网络错误处理
 * - OnIncomingAudio: 接收音频数据
 * - OnAudioChannelOpened/Closed: 音频通道状态变化
 * - OnIncomingJson: 处理JSON消息（tts/stt/llm/mcp/system/alert/custom）
 */
void Application::InitializeProtocol() {
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto codec = board.GetAudioCodec();

    display->SetStatus(Lang::Strings::LOADING_PROTOCOL);

    // 根据OTA配置选择协议类型
    if (ota_->HasMqttConfig()) {
        protocol_ = std::make_unique<MqttProtocol>();
    } else if (ota_->HasWebsocketConfig()) {
        protocol_ = std::make_unique<WebsocketProtocol>();
    } else {
        ESP_LOGW(TAG, "No protocol specified in the OTA config, using MQTT");
        protocol_ = std::make_unique<MqttProtocol>();
    }

    // 连接成功回调
    protocol_->OnConnected([this]() { DismissAlert(); });

    // 网络错误回调
    protocol_->OnNetworkError([this](const std::string& message) {
        last_error_message_ = message;
        xEventGroupSetBits(event_group_, MAIN_EVENT_ERROR);
    });

    // 接收到音频数据回调
    protocol_->OnIncomingAudio([this](std::unique_ptr<AudioStreamPacket> packet) {
        if (GetDeviceState() == kDeviceStateSpeaking) {
            audio_service_.PushPacketToDecodeQueue(std::move(packet));
        }
    });

    // 音频通道打开回调
    protocol_->OnAudioChannelOpened([this, codec, &board]() {
        board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
        // 检查采样率是否匹配
        if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
            ESP_LOGW(TAG,
                     "Server sample rate %d does not match device output sample rate %d, "
                     "resampling may cause distortion",
                     protocol_->server_sample_rate(), codec->output_sample_rate());
        }
    });

    // 音频通道关闭回调
    protocol_->OnAudioChannelClosed([this, &board]() {
        board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
        Schedule([this]() {
            auto display = Board::GetInstance().GetDisplay();
            display->SetChatMessage("system", "");
            SetDeviceState(kDeviceStateIdle);
        });
    });

    // 处理JSON消息
    protocol_->OnIncomingJson([this, display](const cJSON* root) {
        auto type = cJSON_GetObjectItem(root, "type");
        if (strcmp(type->valuestring, "tts") == 0) {
            // 文字转语音消息
            auto state = cJSON_GetObjectItem(root, "state");
            if (strcmp(state->valuestring, "start") == 0) {
                Schedule([this]() {
                    aborted_ = false;
                    SetDeviceState(kDeviceStateSpeaking);
                });
            } else if (strcmp(state->valuestring, "stop") == 0) {
                Schedule([this]() {
                    if (GetDeviceState() == kDeviceStateSpeaking) {
                        if (listening_mode_ == kListeningModeManualStop) {
                            SetDeviceState(kDeviceStateIdle);
                        } else {
                            SetDeviceState(kDeviceStateListening);
                        }
                    }
                });
            } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                auto text = cJSON_GetObjectItem(root, "text");
                if (cJSON_IsString(text)) {
                    ESP_LOGI(TAG, "<< %s", text->valuestring);
                    Schedule([display, message = std::string(text->valuestring)]() {
                        display->SetChatMessage("assistant", message.c_str());
                    });
                }
            }
        } else if (strcmp(type->valuestring, "stt") == 0) {
            // 语音转文字消息
            auto text = cJSON_GetObjectItem(root, "text");
            if (cJSON_IsString(text)) {
                ESP_LOGI(TAG, ">> %s", text->valuestring);
                Schedule([display, message = std::string(text->valuestring)]() {
                    display->SetChatMessage("user", message.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "llm") == 0) {
            // LLM情感消息
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(emotion)) {
                Schedule([display, emotion_str = std::string(emotion->valuestring)]() {
                    display->SetEmotion(emotion_str.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "mcp") == 0) {
            // MCP协议消息
            auto payload = cJSON_GetObjectItem(root, "payload");
            if (cJSON_IsObject(payload)) {
                McpServer::GetInstance().ParseMessage(payload);
            }
        } else if (strcmp(type->valuestring, "system") == 0) {
            // 系统命令
            auto command = cJSON_GetObjectItem(root, "command");
            if (cJSON_IsString(command)) {
                ESP_LOGI(TAG, "System command: %s", command->valuestring);
                if (strcmp(command->valuestring, "reboot") == 0) {
                    Schedule([this]() { Reboot(); });
                } else {
                    ESP_LOGW(TAG, "Unknown system command: %s", command->valuestring);
                }
            }
        } else if (strcmp(type->valuestring, "alert") == 0) {
            // 警告消息
            auto status = cJSON_GetObjectItem(root, "status");
            auto message = cJSON_GetObjectItem(root, "message");
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(status) && cJSON_IsString(message) && cJSON_IsString(emotion)) {
                Alert(status->valuestring, message->valuestring, emotion->valuestring,
                      Lang::Sounds::OGG_VIBRATION);
            } else {
                ESP_LOGW(TAG, "Alert command requires status, message and emotion");
            }
#if CONFIG_RECEIVE_CUSTOM_MESSAGE
        } else if (strcmp(type->valuestring, "custom") == 0) {
            // 自定义消息（需启用CONFIG_RECEIVE_CUSTOM_MESSAGE）
            auto payload = cJSON_GetObjectItem(root, "payload");
            ESP_LOGI(TAG, "Received custom message: %s", cJSON_PrintUnformatted(root));
            if (cJSON_IsObject(payload)) {
                Schedule(
                    [this, display, payload_str = std::string(cJSON_PrintUnformatted(payload))]() {
                        display->SetChatMessage("system", payload_str.c_str());
                    });
            } else {
                ESP_LOGW(TAG, "Invalid custom message format: missing payload");
            }
#endif
        } else {
            ESP_LOGW(TAG, "Unknown message type: %s", type->valuestring);
        }
    });

    // 启动协议
    protocol_->Start();
}

/**
 * @brief 显示激活码
 *
 * 显示激活码并播放对应的语音提示，用于设备激活流程。
 * @param code 激活码字符串
 * @param message 提示消息
 */
void Application::ShowActivationCode(const std::string& code, const std::string& message) {
    struct digit_sound {
        char digit;
        const std::string_view& sound;
    };
    // 数字与对应语音文件的映射表
    static const std::array<digit_sound, 10> digit_sounds{
        {digit_sound{'0', Lang::Sounds::OGG_0}, digit_sound{'1', Lang::Sounds::OGG_1},
         digit_sound{'2', Lang::Sounds::OGG_2}, digit_sound{'3', Lang::Sounds::OGG_3},
         digit_sound{'4', Lang::Sounds::OGG_4}, digit_sound{'5', Lang::Sounds::OGG_5},
         digit_sound{'6', Lang::Sounds::OGG_6}, digit_sound{'7', Lang::Sounds::OGG_7},
         digit_sound{'8', Lang::Sounds::OGG_8}, digit_sound{'9', Lang::Sounds::OGG_9}}};

    // 显示激活提示（此操作占用约9KB SRAM，需等待完成）
    Alert(Lang::Strings::ACTIVATION, message.c_str(), "link", Lang::Sounds::OGG_ACTIVATION);

    // 逐位播放激活码语音
    for (const auto& digit : code) {
        auto it = std::find_if(digit_sounds.begin(), digit_sounds.end(),
                               [digit](const digit_sound& ds) { return ds.digit == digit; });
        if (it != digit_sounds.end()) {
            audio_service_.PlaySound(it->sound);
        }
    }
}

/**
 * @brief 显示警告信息
 *
 * 在屏幕上显示警告状态、消息和表情，并播放提示音。
 * @param status 状态文本
 * @param message 警告消息
 * @param emotion 表情图标名称
 * @param sound 提示音文件
 */
void Application::Alert(const char* status, const char* message, const char* emotion,
                        const std::string_view& sound) {
    ESP_LOGW(TAG, "Alert [%s] %s: %s", emotion, status, message);
    auto display = Board::GetInstance().GetDisplay();
    display->SetStatus(status);
    display->SetEmotion(emotion);
    display->SetChatMessage("system", message);
    if (!sound.empty()) {
        audio_service_.PlaySound(sound);
    }
}

/**
 * @brief 关闭警告显示
 *
 * 当设备处于空闲状态时，恢复显示为待机状态。
 */
void Application::DismissAlert() {
    if (GetDeviceState() == kDeviceStateIdle) {
        auto display = Board::GetInstance().GetDisplay();
        display->SetStatus(Lang::Strings::STANDBY);
        display->SetEmotion("neutral");
        display->SetChatMessage("system", "");
    }
}

/**
 * @brief 切换聊天状态
 *
 * 触发 MAIN_EVENT_TOGGLE_CHAT 事件，用于切换设备的聊天状态。
 */
void Application::ToggleChatState() { xEventGroupSetBits(event_group_, MAIN_EVENT_TOGGLE_CHAT); }

/**
 * @brief 开始监听
 *
 * 触发 MAIN_EVENT_START_LISTENING 事件，使设备进入监听状态。
 */
void Application::StartListening() { xEventGroupSetBits(event_group_, MAIN_EVENT_START_LISTENING); }

/**
 * @brief 停止监听
 *
 * 触发 MAIN_EVENT_STOP_LISTENING 事件，使设备退出监听状态。
 */
void Application::StopListening() { xEventGroupSetBits(event_group_, MAIN_EVENT_STOP_LISTENING); }

/**
 * @brief 处理聊天状态切换事件
 *
 * 根据当前设备状态执行不同的操作：
 * - 激活中 -> 切换到空闲状态
 * - WiFi配置中 -> 进入音频测试模式
 * - 音频测试中 -> 退出音频测试模式，返回WiFi配置
 * - 空闲 -> 打开音频通道，进入监听状态
 * - 说话中 -> 中止说话
 * - 监听中 -> 关闭音频通道
 */
void Application::HandleToggleChatEvent() {
    auto state = GetDeviceState();

    // 如果正在激活，取消激活并返回空闲
    if (state == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (state == kDeviceStateWifiConfiguring) {
        // WiFi配置模式下切换到音频测试
        audio_service_.EnableAudioTesting(true);
        SetDeviceState(kDeviceStateAudioTesting);
        return;
    } else if (state == kDeviceStateAudioTesting) {
        // 音频测试模式下返回WiFi配置
        audio_service_.EnableAudioTesting(false);
        SetDeviceState(kDeviceStateWifiConfiguring);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }

    if (state == kDeviceStateIdle) {
        ListeningMode mode = GetDefaultListeningMode();
        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            // 调度执行，让状态变化先处理（更新UI）
            Schedule([this, mode]() { ContinueOpenAudioChannel(mode); });
            return;
        }
        SetListeningMode(mode);
    } else if (state == kDeviceStateSpeaking) {
        AbortSpeaking(kAbortReasonNone);
    } else if (state == kDeviceStateListening) {
        protocol_->CloseAudioChannel();
    }
}

/**
 * @brief 继续打开音频通道
 *
 * 在连接状态下打开音频通道并设置监听模式。
 * 此函数由 Schedule 调度执行，确保状态变化已处理完成。
 * @param mode 监听模式
 */
void Application::ContinueOpenAudioChannel(ListeningMode mode) {
    // 再次检查状态，防止调度期间状态已改变
    if (GetDeviceState() != kDeviceStateConnecting) {
        return;
    }

    // 如果音频通道未打开，尝试打开
    if (!protocol_->IsAudioChannelOpened()) {
        if (!protocol_->OpenAudioChannel()) {
            return;
        }
    }

    SetListeningMode(mode);
}

/**
 * @brief 处理开始监听事件
 *
 * 根据当前设备状态执行不同的操作：
 * - 激活中 -> 切换到空闲状态
 * - WiFi配置中 -> 进入音频测试模式
 * - 空闲 -> 打开音频通道，进入手动停止监听模式
 * - 说话中 -> 中止说话，进入手动停止监听模式
 */
void Application::HandleStartListeningEvent() {
    auto state = GetDeviceState();

    if (state == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (state == kDeviceStateWifiConfiguring) {
        audio_service_.EnableAudioTesting(true);
        SetDeviceState(kDeviceStateAudioTesting);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }

    if (state == kDeviceStateIdle) {
        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            // 调度执行，让状态变化先处理（更新UI）
            Schedule([this]() { ContinueOpenAudioChannel(kListeningModeManualStop); });
            return;
        }
        SetListeningMode(kListeningModeManualStop);
    } else if (state == kDeviceStateSpeaking) {
        AbortSpeaking(kAbortReasonNone);
        SetListeningMode(kListeningModeManualStop);
    }
}

/**
 * @brief 处理停止监听事件
 *
 * 根据当前设备状态执行不同的操作：
 * - 音频测试中 -> 退出音频测试，返回WiFi配置模式
 * - 监听中 -> 发送停止监听命令，切换到空闲状态
 */
void Application::HandleStopListeningEvent() {
    auto state = GetDeviceState();

    if (state == kDeviceStateAudioTesting) {
        audio_service_.EnableAudioTesting(false);
        SetDeviceState(kDeviceStateWifiConfiguring);
        return;
    } else if (state == kDeviceStateListening) {
        if (protocol_) {
            protocol_->SendStopListening();
        }
        SetDeviceState(kDeviceStateIdle);
    }
}

/**
 * @brief 处理唤醒词检测事件
 *
 * 根据当前设备状态处理唤醒词检测：
 * - 空闲状态：编码唤醒词，打开音频通道，进入监听状态
 * - 说话/监听状态：中止当前操作，重新开始监听
 * - 激活状态：取消激活，返回空闲状态
 */
void Application::HandleWakeWordDetectedEvent() {
    if (!protocol_) {
        return;
    }

    auto state = GetDeviceState();
    auto wake_word = audio_service_.GetLastWakeWord();
    ESP_LOGI(TAG, "Wake word detected: %s (state: %d)", wake_word.c_str(), (int)state);

    if (state == kDeviceStateIdle) {
        // 编码唤醒词数据
        audio_service_.EncodeWakeWord();
        auto wake_word = audio_service_.GetLastWakeWord();

        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            // 调度执行，让状态变化先处理（更新UI），然后打开音频通道（可能阻塞约1秒）
            Schedule([this, wake_word]() { ContinueWakeWordInvoke(wake_word); });
            return;
        }
        // 通道已打开，直接继续
        ContinueWakeWordInvoke(wake_word);
    } else if (state == kDeviceStateSpeaking || state == kDeviceStateListening) {
        // 中止当前会话（唤醒词检测导致）
        AbortSpeaking(kAbortReasonWakeWordDetected);
        // 清空发送队列，避免发送残留数据到服务器
        while (audio_service_.PopPacketFromSendQueue())
            ;

        if (state == kDeviceStateListening) {
            protocol_->SendStartListening(GetDefaultListeningMode());
            audio_service_.ResetDecoder();
            audio_service_.PlaySound(Lang::Sounds::OGG_POPUP);
            // 重新启用唤醒词检测（检测本身会停止检测）
            audio_service_.EnableWakeWordDetection(true);
        } else {
            // 播放弹窗声音并重新开始监听
            play_popup_on_listening_ = true;
            SetListeningMode(GetDefaultListeningMode());
        }
    } else if (state == kDeviceStateActivating) {
        // 激活过程中检测到唤醒词，重新开始激活检查
        SetDeviceState(kDeviceStateIdle);
    }
}

/**
 * @brief 继续处理唤醒词触发
 *
 * 在连接状态下打开音频通道并处理唤醒词数据发送。
 * @param wake_word 检测到的唤醒词
 */
void Application::ContinueWakeWordInvoke(const std::string& wake_word) {
    // 再次检查状态，防止调度期间状态已改变
    if (GetDeviceState() != kDeviceStateConnecting) {
        return;
    }

    // 如果音频通道未打开，尝试打开
    if (!protocol_->IsAudioChannelOpened()) {
        if (!protocol_->OpenAudioChannel()) {
            audio_service_.EnableWakeWordDetection(true);
            return;
        }
    }

    ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
#if CONFIG_SEND_WAKE_WORD_DATA
    // 编码并发送唤醒词数据到服务器
    while (auto packet = audio_service_.PopWakeWordPacket()) {
        protocol_->SendAudio(std::move(packet));
    }
    // 设置聊天状态为唤醒词检测到
    protocol_->SendWakeWordDetected(wake_word);
    SetListeningMode(GetDefaultListeningMode());
#else
    // 设置标志位，在状态变为监听后播放弹窗声音
    // （在此处调用PlaySound会被EnableVoiceProcessing中的ResetDecoder清除）
    play_popup_on_listening_ = true;
    SetListeningMode(GetDefaultListeningMode());
#endif
}

/**
 * @brief 处理状态变化事件
 *
 * 根据新状态执行相应的操作：
 * - 空闲状态：设置待机显示，禁用语音处理，启用唤醒词检测
 * - 连接状态：设置连接中显示
 * - 监听状态：启用语音处理，发送开始监听命令
 * - 说话状态：禁用语音处理（非实时模式），重置解码器
 * - WiFi配置状态：禁用语音处理和唤醒词检测
 */
void Application::HandleStateChangedEvent() {
    DeviceState new_state = state_machine_.GetState();
    clock_ticks_ = 0;

    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto led = board.GetLed();
    led->OnStateChanged();

    switch (new_state) {
        case kDeviceStateUnknown:
        case kDeviceStateIdle:
            display->SetStatus(Lang::Strings::STANDBY);
            display->ClearChatMessages();    // 先清除消息
            display->SetEmotion("neutral");  // 然后设置表情（微信模式检查子节点数量）
            audio_service_.EnableVoiceProcessing(false);
            audio_service_.EnableWakeWordDetection(true);
            break;
        case kDeviceStateConnecting:
            display->SetStatus(Lang::Strings::CONNECTING);
            display->SetEmotion("neutral");
            display->SetChatMessage("system", "");
            break;
        case kDeviceStateListening:
            display->SetStatus(Lang::Strings::LISTENING);
            display->SetEmotion("neutral");

            // 确保音频处理器正在运行
            if (play_popup_on_listening_ || !audio_service_.IsAudioProcessorRunning()) {
                // 自动模式下，在启用语音处理前等待播放队列清空
                // 这可以防止STOP因网络抖动延迟到达时导致音频截断
                if (listening_mode_ == kListeningModeAutoStop) {
                    audio_service_.WaitForPlaybackQueueEmpty();
                }

                // 发送开始监听命令
                protocol_->SendStartListening(listening_mode_);
                audio_service_.EnableVoiceProcessing(true);
            }

#ifdef CONFIG_WAKE_WORD_DETECTION_IN_LISTENING
            // 在监听模式下启用唤醒词检测（通过Kconfig配置）
            audio_service_.EnableWakeWordDetection(audio_service_.IsAfeWakeWord());
#else
            // 在监听模式下禁用唤醒词检测
            audio_service_.EnableWakeWordDetection(false);
#endif

            // 在EnableVoiceProcessing中的ResetDecoder调用后播放弹窗声音
            if (play_popup_on_listening_) {
                play_popup_on_listening_ = false;
                audio_service_.PlaySound(Lang::Sounds::OGG_POPUP);
            }
            break;
        case kDeviceStateSpeaking:
            display->SetStatus(Lang::Strings::SPEAKING);

            if (listening_mode_ != kListeningModeRealtime) {
                audio_service_.EnableVoiceProcessing(false);
                // 说话模式下只能检测AFE唤醒词
                audio_service_.EnableWakeWordDetection(audio_service_.IsAfeWakeWord());
            }
            audio_service_.ResetDecoder();
            break;
        case kDeviceStateWifiConfiguring:
            audio_service_.EnableVoiceProcessing(false);
            audio_service_.EnableWakeWordDetection(false);
            break;
        default:
            // 其他状态不做处理
            break;
    }
}

/**
 * @brief 调度任务到主任务执行
 *
 * 将回调函数添加到任务队列，并触发 MAIN_EVENT_SCHEDULE 事件，
 * 确保任务在主事件循环中执行，保证线程安全。
 * @param callback 要执行的回调函数
 */
void Application::Schedule(std::function<void()>&& callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        main_tasks_.push_back(std::move(callback));
    }
    xEventGroupSetBits(event_group_, MAIN_EVENT_SCHEDULE);
}

/**
 * @brief 中止说话
 *
 * 设置中止标志并通知协议层发送中止说话命令。
 * @param reason 中止原因
 */
void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking");
    aborted_ = true;
    if (protocol_) {
        protocol_->SendAbortSpeaking(reason);
    }
}

/**
 * @brief 设置监听模式
 *
 * 设置当前监听模式并切换到监听状态。
 * @param mode 监听模式
 */
void Application::SetListeningMode(ListeningMode mode) {
    listening_mode_ = mode;
    SetDeviceState(kDeviceStateListening);
}

/**
 * @brief 获取默认监听模式
 *
 * 根据AEC模式返回默认监听模式：
 * - AEC关闭：自动停止模式（kListeningModeAutoStop）
 * - AEC启用：实时模式（kListeningModeRealtime）
 * @return 默认监听模式
 */
ListeningMode Application::GetDefaultListeningMode() const {
    return aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime;
}

/**
 * @brief 重启设备
 *
 * 执行设备重启前的清理工作：
 * - 关闭音频通道
 * - 释放协议对象
 * - 停止音频服务
 * - 延迟1秒后重启设备
 */
void Application::Reboot() {
    ESP_LOGI(TAG, "Rebooting...");
    // 断开音频通道
    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        protocol_->CloseAudioChannel();
    }
    protocol_.reset();
    audio_service_.Stop();

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

/**
 * @brief 升级固件
 *
 * 执行固件升级流程：
 * - 关闭音频通道
 * - 显示升级提示
 * - 停止音频服务
 * - 执行OTA升级
 * - 根据升级结果处理成功或失败情况
 * @param url 固件下载URL
 * @param version 固件版本信息
 * @return 升级是否成功
 */
bool Application::UpgradeFirmware(const std::string& url, const std::string& version) {
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();

    std::string upgrade_url = url;
    std::string version_info = version.empty() ? "(Manual upgrade)" : version;

    // 如果音频通道打开，先关闭
    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        ESP_LOGI(TAG, "Closing audio channel before firmware upgrade");
        protocol_->CloseAudioChannel();
    }
    ESP_LOGI(TAG, "Starting firmware upgrade from URL: %s", upgrade_url.c_str());

    Alert(Lang::Strings::OTA_UPGRADE, Lang::Strings::UPGRADING, "download",
          Lang::Sounds::OGG_UPGRADE);
    vTaskDelay(pdMS_TO_TICKS(3000));

    SetDeviceState(kDeviceStateUpgrading);

    std::string message = std::string(Lang::Strings::NEW_VERSION) + version_info;
    display->SetChatMessage("system", message.c_str());

    board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
    audio_service_.Stop();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 执行OTA升级
    bool upgrade_success = Ota::Upgrade(upgrade_url, [this, display](int progress, size_t speed) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%d%% %uKB/s", progress, speed / 1024);
        Schedule([display, message = std::string(buffer)]() {
            display->SetChatMessage("system", message.c_str());
        });
    });

    if (!upgrade_success) {
        // 升级失败，重启音频服务并继续运行
        ESP_LOGE(TAG,
                 "Firmware upgrade failed, restarting audio service and continuing operation...");
        audio_service_.Start();                              // 重启音频服务
        board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);  // 恢复低功耗模式
        Alert(Lang::Strings::ERROR, Lang::Strings::UPGRADE_FAILED, "circle_xmark",
              Lang::Sounds::OGG_EXCLAMATION);
        vTaskDelay(pdMS_TO_TICKS(3000));
        return false;
    } else {
        // 升级成功，立即重启
        ESP_LOGI(TAG, "Firmware upgrade successful, rebooting...");
        display->SetChatMessage("system", "Upgrade successful, rebooting...");
        vTaskDelay(pdMS_TO_TICKS(1000));  // 短暂延迟显示消息
        Reboot();
        return true;
    }
}

/**
 * @brief 唤醒词触发接口
 *
 * 外部调用的唤醒词触发函数，根据当前设备状态执行不同操作：
 * - 空闲状态：编码唤醒词，打开音频通道，进入监听状态
 * - 说话状态：中止说话
 * - 监听状态：关闭音频通道
 * @param wake_word 唤醒词字符串
 */
void Application::WakeWordInvoke(const std::string& wake_word) {
    if (!protocol_) {
        return;
    }

    auto state = GetDeviceState();

    if (state == kDeviceStateIdle) {
        audio_service_.EncodeWakeWord();

        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            // 调度执行，让状态变化先处理（更新UI）
            Schedule([this, wake_word]() { ContinueWakeWordInvoke(wake_word); });
            return;
        }
        // 通道已打开，直接继续
        ContinueWakeWordInvoke(wake_word);
    } else if (state == kDeviceStateSpeaking) {
        Schedule([this]() { AbortSpeaking(kAbortReasonNone); });
    } else if (state == kDeviceStateListening) {
        Schedule([this]() {
            if (protocol_) {
                protocol_->CloseAudioChannel();
            }
        });
    }
}

/**
 * @brief 检查是否可以进入睡眠模式
 *
 * 检查设备是否满足进入睡眠模式的条件：
 * - 设备状态必须是空闲
 * - 音频通道必须关闭
 * - 音频服务必须空闲
 * @return 是否可以进入睡眠模式
 */
bool Application::CanEnterSleepMode() {
    if (GetDeviceState() != kDeviceStateIdle) {
        return false;
    }

    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        return false;
    }

    if (!audio_service_.IsIdle()) {
        return false;
    }

    // 现在可以安全进入睡眠模式
    return true;
}

/**
 * @brief 发送MCP消息
 *
 * 通过协议发送MCP（Model Context Protocol）消息，
 * 始终调度到主任务执行以保证线程安全。
 * @param payload MCP消息负载
 */
void Application::SendMcpMessage(const std::string& payload) {
    // 始终调度到主任务执行以保证线程安全
    Schedule([this, payload = std::move(payload)]() {
        if (protocol_) {
            protocol_->SendMcpMessage(payload);
        }
    });
}

/**
 * @brief 设置AEC模式
 *
 * 设置声学回声消除模式，并更新相关配置：
 * - kAecOff: 关闭AEC
 * - kAecOnServerSide: 服务端AEC
 * - kAecOnDeviceSide: 设备端AEC
 * AEC模式改变时会关闭音频通道。
 * @param mode AEC模式
 */
void Application::SetAecMode(AecMode mode) {
    aec_mode_ = mode;
    Schedule([this]() {
        auto& board = Board::GetInstance();
        auto display = board.GetDisplay();
        switch (aec_mode_) {
            case kAecOff:
                audio_service_.EnableDeviceAec(false);
                display->ShowNotification(Lang::Strings::RTC_MODE_OFF);
                break;
            case kAecOnServerSide:
                audio_service_.EnableDeviceAec(false);
                display->ShowNotification(Lang::Strings::RTC_MODE_ON);
                break;
            case kAecOnDeviceSide:
                audio_service_.EnableDeviceAec(true);
                display->ShowNotification(Lang::Strings::RTC_MODE_ON);
                break;
        }

        // AEC模式改变时，关闭音频通道
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
    });
}

/**
 * @brief 播放声音
 *
 * 通过音频服务播放指定的声音文件。
 * @param sound 声音文件路径/名称
 */
void Application::PlaySound(const std::string_view& sound) { audio_service_.PlaySound(sound); }

/**
 * @brief 重置协议
 *
 * 关闭音频通道并释放协议对象，用于重新初始化协议。
 */
void Application::ResetProtocol() {
    Schedule([this]() {
        // 如果音频通道打开，先关闭
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
        // 重置协议对象
        protocol_.reset();
    });
}

/**
 * @brief MQTT 事件处理函数（静态）
 *
 * 处理 MQTT 客户端的各种事件：
 * - MQTT_EVENT_CONNECTED: 连接成功，发布上线消息
 * - MQTT_EVENT_DISCONNECTED: 连接断开
 * - MQTT_EVENT_SUBSCRIBED: 订阅成功
 * - MQTT_EVENT_PUBLISHED: 消息发布成功
 * - MQTT_EVENT_DATA: 接收到数据
 * - MQTT_EVENT_ERROR: 发生错误
 */
void Application::MqttEventHandler(void* handler_args, esp_event_base_t base, int32_t event_id,
                                   void* event_data) {
    Application* app = static_cast<Application*>(handler_args);
    esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);
    (void)base;  // 避免未使用参数警告

    switch (static_cast<esp_mqtt_event_id_t>(event_id)) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED: 客户端连接到服务器");
            xEventGroupSetBits(app->event_group_, MAIN_EVENT_MQTT_CONNECTED);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED: 客户端断开连接");
            xEventGroupSetBits(app->event_group_, MAIN_EVENT_MQTT_DISCONNECTED);
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED: 订阅成功, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED: 取消订阅, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED: 消息发布成功, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA: 收到消息, topic=%.*s, data_len=%d", event->topic_len,
                     event->topic, event->data_len);
            // 保存接收到的数据
            app->mqtt_received_topic_ = std::string(event->topic, event->topic_len);
            app->mqtt_received_data_ = std::string(event->data, event->data_len);
            xEventGroupSetBits(app->event_group_, MAIN_EVENT_MQTT_DATA);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR: 发生错误");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGI(TAG, "Last errno: %s",
                         strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGI(TAG, "MQTT 其他事件 id:%d", event->event_id);
            break;
    }
}

/**
 * @brief 初始化 MQTT 客户端
 *
 * 配置并启动 MQTT 客户端，使用 MQTT v5 协议和 TLS 加密连接。
 */
void Application::MqttInit() {
    ESP_LOGI(TAG, "初始化 MQTT 客户端...");

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = MQTT_BROKER;
    mqtt_cfg.broker.verification.certificate = MQTT_CERT;
    mqtt_cfg.credentials.username = MQTT_USERNAME;
    mqtt_cfg.credentials.authentication.password = MQTT_PASSWORD;
    mqtt_cfg.session.protocol_ver = MQTT_PROTOCOL_V_5;
    mqtt_cfg.network.disable_auto_reconnect = false;
    mqtt_cfg.network.timeout_ms = 5000;

    mqtt_client_ = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client_ == nullptr) {
        ESP_LOGE(TAG, "MQTT 客户端初始化失败");
        return;
    }

    // 注册事件处理函数
    esp_mqtt_client_register_event(mqtt_client_, MQTT_EVENT_ANY, MqttEventHandler, this);

    // 启动 MQTT 客户端
    esp_err_t err = esp_mqtt_client_start(mqtt_client_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT 客户端启动失败: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "MQTT 客户端启动成功");
    }
}

/**
 * @brief 发布 MQTT 消息
 *
 * 向指定主题发布消息。
 * @param topic 主题名称
 * @param data 消息内容
 * @param qos 服务质量等级 (0, 1, 2)
 * @param retain 是否保留消息
 */
void Application::MqttPublish(const std::string& topic, const std::string& data, int qos,
                              int retain) {
    if (mqtt_client_ == nullptr) {
        ESP_LOGE(TAG, "MQTT 客户端未初始化");
        return;
    }

    int msg_id = esp_mqtt_client_publish(mqtt_client_, topic.c_str(), data.c_str(), data.length(),
                                         qos, retain);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "MQTT 发布消息失败, error=%d", msg_id);
    } else {
        ESP_LOGI(TAG, "MQTT 发布消息成功, msg_id=%d", msg_id);
    }
}

/**
 * @brief 订阅 MQTT 主题
 *
 * 订阅指定的 MQTT 主题。
 * @param topic 主题名称
 * @param qos 服务质量等级 (0, 1, 2)
 */
void Application::MqttSubscribe(const std::string& topic, int qos) {
    if (mqtt_client_ == nullptr) {
        ESP_LOGE(TAG, "MQTT 客户端未初始化");
        return;
    }

    int msg_id = esp_mqtt_client_subscribe(mqtt_client_, topic.c_str(), qos);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "MQTT 订阅主题失败, error=%d", msg_id);
    } else {
        ESP_LOGI(TAG, "MQTT 订阅主题成功, topic=%s, msg_id=%d", topic.c_str(), msg_id);
    }
}

/**
 * @brief 处理 MQTT 连接成功事件
 *
 * MQTT 连接成功后，发布上线消息并订阅主题。
 */
void Application::HandleMqttConnectedEvent() {
    ESP_LOGI(TAG, "MQTT 已连接");

    // 发布上线消息
    // MqttPublish(MQTT_TOPIC1, "Hi EMQX I'm ESP32 ^^", 1, 0);

    // 订阅主题
    MqttSubscribe(MQTT_SUBSCRIBE_xiaozhi, 1);

    // 订阅主题
    MqttSubscribe(MQTT_TOPIC_fan, 1);
    MqttSubscribe(MQTT_TOPIC_humidifier, 1);
    MqttSubscribe(MQTT_TOPIC_ac, 1);
    MqttSubscribe(MQTT_TOPIC_light, 1);

    // 显示通知
    auto display = Board::GetInstance().GetDisplay();
    display->ShowNotification("MQTT Connected", 3000);
}

/**
 * @brief 处理 MQTT 断开连接事件
 *
 * MQTT 断开连接后的处理。
 */
void Application::HandleMqttDisconnectedEvent() {
    ESP_LOGI(TAG, "MQTT 已断开连接");

    // 显示通知
    auto display = Board::GetInstance().GetDisplay();
    display->ShowNotification("MQTT Disconnected", 3000);
}

/**
 * @brief 处理 MQTT 数据接收事件
 *
 * 处理从 MQTT 接收到的数据。
 */
void Application::HandleMqttDataEvent() {
    ESP_LOGI(TAG, "MQTT 收到数据: topic=%s, data=%s", mqtt_received_topic_.c_str(),
             mqtt_received_data_.c_str());

    // 在聊天界面显示收到的消息
    auto display = Board::GetInstance().GetDisplay();
    std::string message = "MQTT: " + mqtt_received_data_;
    display->SetChatMessage("system", message.c_str());
}
