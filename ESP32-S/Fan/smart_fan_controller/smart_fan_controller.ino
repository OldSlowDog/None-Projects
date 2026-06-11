/*
 * ESP32-S Professional Refrigeration Controller V2.6
 * 
 * 功能:
 * 1. 无级调速0-100% (CPU风扇 + 主风扇) - 全硬件MCPWM
 * 2. CPU风扇物理开关 + 简化逻辑
 * 3. CPU风扇转速显示 (理论转速)
 * 4. 双DS18B20温度传感器 (冷端 + 热端)
 * 5. PWM MOS继电器控制半导体制冷片 (TEC)
 * 6. I2C OLED单按键菜单显示
 * 7. 3个GND替代引脚
 * 8. WIFI热点 + TCP服务器
 * 9. 串口+TCP统一JSON指令集
 * 
 * V2.6更新: 简化所有安全逻辑，简化指令集，JSON响应
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/mcpwm.h>
#include <WiFi.h>
#include <WebServer.h>

/* ==================== 引脚定义 ==================== */
/* PWM输出 */
#define PIN_FAN_CPU      15       /* GPIO15 -> CPU散热风扇PWM (25kHz) */
#define PIN_FAN_MAIN     13       /* GPIO13 -> 主风扇电调信号 (50Hz) */
#define PIN_TEC          2        /* GPIO2 -> TEC半导体制冷片MOS继电器 (开关) */

/* CPU风扇物理开关 (MOS/继电器, 控制风扇电源或PWM使能) */
#define PIN_FAN_CPU_SW   12      /* GPIO12 -> CPU风扇电源开关 (高电平=开, 低电平=关) */

/* 测速输入 (FG信号, 上拉) */
#define PIN_FG_CPU       17      /* GPIO17 -> CPU风扇测速脉冲输入 */
#define PIN_FG_MAIN      18      /* GPIO18 -> 主风扇测速脉冲输入(可选) */

/* 温度传感器 DS18B20 */
#define PIN_TEMP_COLD    19      /* GPIO19 -> 冷端温度传感器 */
#define PIN_TEMP_HOT     16      /* GPIO16 -> 热端温度传感器 */

/* I2C OLED */
#define OLED_SDA         21      /* GPIO21 SDA */
#define OLED_SCL         22      /* GPIO22 SCL */
#define OLED_ADDR        0x3C    /* SSD1306地址 */
#define SCREEN_WIDTH     128
#define SCREEN_HEIGHT    64

/* LED状态指示 - 与TEC同步显示, 使用同一引脚 */
#define PIN_LED          PIN_TEC    /* GPIO17 板载LED - 与TEC同步 */

/* 物理按键 (单键菜单) */
#define PIN_BTN          4      /* GPIO4 -> 菜单按键 (上拉输入, 低电平有效) */

/* GND替代引脚 */
#define PIN_GND_REPLACEMENT1   23   /* GPIO23 -> GND替代引脚1 (OUTPUT LOW) */
#define PIN_GND_REPLACEMENT2   14   /* GPIO14 -> GND替代引脚2 (OUTPUT LOW) */
#define PIN_GND_REPLACEMENT3   26   /* GPIO26 -> GND替代引脚3 (OUTPUT LOW) */

/* 雾化器控制引脚 */
#define PIN_FOGGER 25 /* GPIO25 -> 雾化器模拟按键输出 (推挽) */

/* ==================== 按键参数 ==================== */
#define BTN_DEBOUNCE_MS  20      /* 按键去抖动 ms */
#define BTN_SHORT_MIN    50      /* 短按最小时间 ms */
#define BTN_SHORT_MAX    300     /* 短按最大时间 ms */
#define BTN_LONG_MIN     500     /* 长按最小时间 ms */

/* ==================== WIFI和HTTP参数 ==================== */
#define WIFI_SSID        "ESP32_FAN_CTRL"  /* WIFI热点名称 */
#define WIFI_PASS        "12345678"        /* WIFI热点密码 */
#define HTTP_PORT        80                /* HTTP服务器端口 */

/* ==================== PWM参数 ==================== */
#define CPU_FAN_FREQ     25000   /* CPU风扇 25kHz标准频率 */
#define MAIN_FAN_FREQ    50      /* 主风扇电调 50Hz舵机协议 */
#define TEC_FREQ         1000    /* TEC控制 1kHz PWM */
#define PULSE_PERIOD     20000   /* 电调周期 20ms */
#define PULSE_MIN        1000    /* 电调最小脉宽 us */
#define PULSE_MAX        1500    /* 电调最大脉宽 us */

/* 占空比0-100% */

/* ==================== 测速参数 ==================== */
#define FG_SAMPLE_MS     1000    /* 测速采样周期 ms */
#define PULSES_PER_REV   2       /* 每转FG脉冲数 (标准4线风扇=2) */
#define FG_DEBOUNCE_US   2000    /* 最小脉冲间隔 us (2ms) */
#define MAX_VALID_RPM    8000    /* 最大有效转速 */

/* 开环控制参数 - 理论转速映射 */
#define CPU_FAN_MAX_RPM  2000    /* CPU风扇最大额定转速 RPM */
#define MAIN_FAN_MAX_RPM 3000    /* 主风扇最大额定转速 RPM */

/* ==================== 菜单档位定义 ==================== */
enum FanSpeed {
    FAN_OFF = 0,                  /* 关闭 */
    FAN_LOW,                      /* 低速 (~30%) */
    FAN_MID,                      /* 中速 (~60%) */
    FAN_HIGH,                     /* 高速 (100%) */
    FAN_SPEED_COUNT               /* 档位总数 */
};

/* 档位对应的占空比 (%) */
const uint8_t CPU_DUTY_TABLE[FAN_SPEED_COUNT] = {0, 30, 60, 100};
const uint8_t MAIN_DUTY_TABLE[FAN_SPEED_COUNT] = {0, 30, 60, 100};
const char* const SPEED_NAMES[FAN_SPEED_COUNT] = {"OFF", "Low", "Mid", "Hi "};

/* TEC开关状态 */
enum TecState : uint8_t {
    TEC_OFF = 0,
    TEC_ON,
    TEC_STATE_COUNT
};
const char* const TEC_NAMES[TEC_STATE_COUNT] = {"OFF", "ON "};
enum FoggerState : uint8_t {
    FOGGER_OFF = 0,
    FOGGER_ON,
    FOGGER_STATE_COUNT
};
const char* const FOGGER_NAMES[FOGGER_STATE_COUNT] = {"OFF", "ON "};

/* ==================== 温度参数 ==================== */
#define TEMP_TARGET      25.0    /* 目标温度 °C */
#define TEMP_MAX_COLD    60.0    /* 冷端最大安全温度 */
#define TEMP_MAX_HOT     70.0    /* 热端最大安全温度 */

/* ==================== 全局变量 ==================== */
/* 风扇控制 */
uint8_t cpuFanDuty = 0;            /* 0-100% */
uint8_t mainFanDuty = 0;           /* 0-100% (映射到1000-1500us) */
uint8_t tecDuty = 0;               /* 0-100% TEC制冷功率 */

/* 测速反馈 */
volatile unsigned long cpuFgCount = 0;
volatile unsigned long mainFgCount = 0;
volatile unsigned long lastCpuFgTime = 0;
volatile unsigned long lastMainFgTime = 0;
unsigned int cpuRpm = 0;
unsigned int mainRpm = 0;
unsigned int cpuTheoreticalRpm = 0;
unsigned int mainTheoreticalRpm = 0;
unsigned long lastFgTime = 0;

/* 温度 */
float tempCold = 0.0;
float tempHot = 0.0;
float tempDelta = 0.0;

/* 控制模式 */
enum ControlMode {
    MODE_MANUAL = 0,
    MODE_AUTO
};
ControlMode ctrlMode = MODE_MANUAL;

/* 菜单状态 */
uint8_t cpuFanSpeedIdx = FAN_OFF;
uint8_t mainFanSpeedIdx = FAN_OFF;
uint8_t tecState = TEC_OFF;

/* CPU风扇物理开关状态 */
bool cpuFanSwState = false;

/* 主风扇PWM Ramp参数 */
#define RAMP_RATE 200    /* 每秒最大脉宽变化 (us/s) */
#define RAMP_UPDATE_INTERVAL 50 /* ramp更新间隔 (ms) */
uint16_t mainFanTargetPulse = 1000;
uint16_t mainFanDesiredPulse = 1000;

/* 按键状态 */
volatile unsigned long btnPressTime = 0;
volatile bool btnPressed = false;
volatile bool btnDebouncing = false;

unsigned long lastClickTime = 0;
const unsigned long DOUBLE_CLICK_WINDOW = 500;

enum BtnEvent {
    BTN_NONE = 0,
    BTN_SHORT_PRESS,
    BTN_LONG_PRESS,
    BTN_DOUBLE_CLICK,
};
BtnEvent lastBtnEvent = BTN_NONE;

/* 菜单行列选择状态 */
uint8_t menuCurrentRow = 0;
uint8_t menuCurrentCol = 0;

const uint8_t MAX_ROWS = 4;
const uint8_t COLS_PER_ROW[] = {4,4,2,2};

/* 显示刷新 */
unsigned long lastOledUpdate = 0;

/* 串口 */
String cmdBuffer = "";
const uint8_t CMD_TIMEOUT = 50;

/* DS18B20对象 */
OneWire oneWireCold(PIN_TEMP_COLD);
OneWire oneWireHot(PIN_TEMP_HOT);
DallasTemperature sensorCold(&oneWireCold);
DallasTemperature sensorHot(&oneWireHot);

/* OLED */
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/* 雾化器状态 */
bool foggerOn = false; /* 初始关 */

/* WIFI和HTTP */
WebServer server(HTTP_PORT);

/* ==================== 函数声明 ==================== */
void toggleFogger();
void initWiFi();
void handleRoot();
void handleApiStatus();
void handleApiSet();
void initHardware();
void initPwm();
void initFgInterrupts();
void initSensors();
void initOled();
void initButton();
void initCpuFanSw();

void setCpuFan(uint8_t duty);
void setMainFan(uint8_t duty);
void setTec(uint8_t duty);
void setCpuFanSw(bool state);

void IRAM_ATTR cpuFgIsr();
void IRAM_ATTR mainFgIsr();
void calculateRpm();
uint16_t getTheoreticalRpm(uint8_t duty, uint16_t maxRpm);

void updateTemperatures();
void autoControlLoop();
void safetyCheck();
void updateMainFanRamp();

void updateOled();
void drawMenuScreen();

BtnEvent readButton();
void handleMenuEvent(BtnEvent event);
void applyMenuRowColSetting(uint8_t row, uint8_t col);
void applyMenuSettings();

void processCommand(String cmd);

/* ==================== 初始化 ==================== */
void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("  ESP32 Pro Refrigeration Controller V2.6"));
    Serial.println(F("  Mode: OPEN-LOOP | Dual MCPWM | Single Button | WIFI TCP"));
    Serial.println(F("========================================"));
    
    // 首先立即设置雾化器引脚为高电平，防止上电初期误触发
    pinMode(PIN_FOGGER, OUTPUT);
    digitalWrite(PIN_FOGGER, HIGH);
    foggerOn = false;
    Serial.print(F("[FOGGER] Pin "));
    Serial.print(PIN_FOGGER);
    Serial.println(F(" initialized HIGH (prevent power-on glitch)"));
    
    // 给足够时间让系统稳定
    delay(1000);
    
    initHardware();
    initButton();
    initCpuFanSw();
    initPwm();
    initFgInterrupts();
    initSensors();
    initOled();
    initWiFi();
    
    cpuFanSpeedIdx = FAN_OFF;
    mainFanSpeedIdx = FAN_OFF;
    tecState = TEC_OFF;
    applyMenuSettings();
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(6, 24);
    display.print(F("Pro Cooler V2.6"));
    display.setCursor(4, 36);
    display.print(F("WIFI TCP"));
    display.setCursor(0, 48);
    display.print(F("^=Row *=Col Long=OK"));
    display.display();
    
    delay(800);
    
    Serial.println(F("[OK] System Ready!"));
    Serial.println();
}

void loop() {
    unsigned long now = millis();
    
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (cmdBuffer.length() > 0) {
                processCommand(cmdBuffer);
                cmdBuffer = "";
            }
        } else if (cmdBuffer.length() < 48) {
            cmdBuffer += c;
            
            char cc = c;
            if (cc >= 'a' && cc <= 'z') cc -= 32;
            
            if (cmdBuffer.length() == 1 && 
                (cc == 'O' || cc == 'F' || cc == 'A' || cc == 'M' ||
                 cc == 'T' || cc == 'S' || cc == '?' || cc == 'H' || cc == 'P')) {
                processCommand(cmdBuffer);
                cmdBuffer = "";
            }
        }
    }
    
    if (now - lastFgTime >= FG_SAMPLE_MS) {
        lastFgTime = now;
        calculateRpm();
    }
    
    static unsigned long lastTemp = 0;
    if (now - lastTemp >= 2000) {
        lastTemp = now;
        updateTemperatures();
        
        if (ctrlMode == MODE_AUTO) {
            autoControlLoop();
        }
        safetyCheck();
    }
    
    BtnEvent btnEvent = readButton();
    if (btnEvent != BTN_NONE) {
        handleMenuEvent(btnEvent);
    }
    
    updateMainFanRamp();
    
    if (now - lastOledUpdate >= 500) {
        lastOledUpdate = now;
        updateOled();
    }
    
    server.handleClient();
    
    delay(5);
}

/* ==================== 硬件初始化 ==================== */
void initHardware() {
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    
    Serial.println(F("[HW] GPIO Init OK"));
}

void initButton() {
    pinMode(PIN_BTN, INPUT_PULLUP);
    
    pinMode(PIN_GND_REPLACEMENT1, OUTPUT);
    digitalWrite(PIN_GND_REPLACEMENT1, LOW);
    
    pinMode(PIN_GND_REPLACEMENT2, OUTPUT);
    digitalWrite(PIN_GND_REPLACEMENT2, LOW);
    
    pinMode(PIN_GND_REPLACEMENT3, OUTPUT);
    digitalWrite(PIN_GND_REPLACEMENT3, LOW);
    
    // 雾化器引脚已在 setup() 最开始初始化，这里不再重复操作，防止误触发
    // 只确保当前状态是高电平
    digitalWrite(PIN_FOGGER, HIGH);
    
    btnPressed = false;
    btnDebouncing = false;
    btnPressTime = 0;
    lastBtnEvent = BTN_NONE;
    
    Serial.print(F("[BTN] BTN(GPIO"));
    Serial.print(PIN_BTN);
    Serial.print(F(")=PU_LOW, GND_EXT(GPIO"));
    Serial.print(PIN_GND_REPLACEMENT1);
    Serial.print(F("/"));
    Serial.print(PIN_GND_REPLACEMENT2);
    Serial.print(F("/"));
    Serial.print(PIN_GND_REPLACEMENT3);
    Serial.println(F(")=LOW"));
}

void initCpuFanSw() {
    pinMode(PIN_FAN_CPU_SW, OUTPUT);
    digitalWrite(PIN_FAN_CPU_SW, LOW);
    cpuFanSwState = false;
    
    Serial.print(F("[CPU_SW] Fan power switch on GPIO"));
    Serial.println(PIN_FAN_CPU_SW);
}

void setCpuFanSw(bool state) {
    cpuFanSwState = state;
    digitalWrite(PIN_FAN_CPU_SW, state ? HIGH : LOW);
}

BtnEvent readButton() {
    bool currentState = (digitalRead(PIN_BTN) == LOW);
    
    unsigned long now = millis();
    static unsigned long lastStateChange = 0;
    static bool lastState = false;
    static bool longPressFired = false;
    
    if (currentState != lastState) {
        if (!btnDebouncing) {
            btnDebouncing = true;
            lastStateChange = now;
        } else if (now - lastStateChange >= BTN_DEBOUNCE_MS) {
            btnDebouncing = false;
            lastState = currentState;
            
            if (currentState) {
                btnPressTime = now;
                btnPressed = true;
                longPressFired = false;
            } else {
                if (btnPressed) {
                    unsigned int pressDuration = now - btnPressTime;
                    btnPressed = false;
                    
                    if (pressDuration >= BTN_SHORT_MIN && pressDuration <= BTN_SHORT_MAX && !longPressFired) {
                        if ((now - lastClickTime) <= DOUBLE_CLICK_WINDOW) {
                            lastClickTime = 0;
                            return BTN_DOUBLE_CLICK;
                        } else {
                            lastClickTime = now;
                            return BTN_SHORT_PRESS;
                        }
                    }
                }
            }
        }
    }
    
    if (btnPressed && currentState) {
        unsigned int pressDuration = now - btnPressTime;
        if (pressDuration >= BTN_LONG_MIN && !longPressFired) {
            longPressFired = true;
            lastClickTime = 0;
            return BTN_LONG_PRESS;
        }
    }
    
    return BTN_NONE;
}

void initPwm() {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_FAN_CPU);
    
    mcpwm_config_t cpu_pwm_config;
    cpu_pwm_config.frequency = CPU_FAN_FREQ;
    cpu_pwm_config.cmpr_a = 0.0;
    cpu_pwm_config.cmpr_b = 0.0;
    cpu_pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    cpu_pwm_config.counter_mode = MCPWM_UP_COUNTER;
    
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cpu_pwm_config);
    
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, PIN_FAN_MAIN);
    
    mcpwm_config_t esc_pwm_config;
    esc_pwm_config.frequency = MAIN_FAN_FREQ;
    esc_pwm_config.cmpr_a = 0.0;
    esc_pwm_config.cmpr_b = 0.0;
    esc_pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    esc_pwm_config.counter_mode = MCPWM_UP_COUNTER;
    
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &esc_pwm_config);
    
    ledcDetachPin(PIN_TEC);
    pinMode(PIN_TEC, OUTPUT);
    digitalWrite(PIN_TEC, LOW);
    
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
    
    float initial_pulse_pct = (float)PULSE_MIN * 100.0 / PULSE_PERIOD;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, initial_pulse_pct);
    mainFanTargetPulse = PULSE_MIN;
    mainFanDesiredPulse = PULSE_MIN;
    
    Serial.print(F("[PWM] CPU=25kHz(MCPWM0_T0_GPIO"));
    Serial.print(PIN_FAN_CPU);
    Serial.print(F("), Main=50Hz(MCPWM0_T1_GPIO"));
    Serial.print(PIN_FAN_MAIN);
    Serial.print(F("), TEC=GPIO"));
    Serial.print(PIN_TEC);
    Serial.println(F("(开关)"));
}

void initFgInterrupts() {
    pinMode(PIN_FG_CPU, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_FG_CPU), cpuFgIsr, FALLING);
    
    pinMode(PIN_FG_MAIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_FG_MAIN), mainFgIsr, FALLING);
    
    Serial.print(F("[FG] CPU=GPIO"));
    Serial.print(PIN_FG_CPU);
    Serial.print(F(", Main=GPIO"));
    Serial.println(PIN_FG_MAIN);
}

void initSensors() {
    pinMode(PIN_TEMP_COLD, INPUT_PULLUP);
    pinMode(PIN_TEMP_HOT, INPUT_PULLUP);
    
    sensorCold.begin();
    sensorHot.begin();
    
    sensorCold.setResolution(12);
    sensorHot.setResolution(12);
    
    int nCold = sensorCold.getDeviceCount();
    int nHot = sensorHot.getDeviceCount();
    
    Serial.print(F("[TEMP] Cold DS18B20(GPIO"));
    Serial.print(PIN_TEMP_COLD);
    Serial.print(F("): "));
    Serial.print(nCold);
    Serial.print(F(" device(s), Hot DS18B20(GPIO"));
    Serial.print(PIN_TEMP_HOT);
    Serial.print(F("): "));
    Serial.println(nHot);
    
    if (nCold == 0 && nHot == 0) {
        Serial.println(F("[WARN] No DS18B20 detected!"));
    }
}

void initOled() {
    Wire.begin(OLED_SDA, OLED_SCL);
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("[ERR] SSD1306 failed!"));
    } else {
        display.clearDisplay();
        display.display();
        Serial.print(F("[OLED] SSD1306@0x"));
        Serial.print(OLED_ADDR, HEX);
        Serial.println(F(" OK"));
    }
}

/* ==================== WIFI和HTTP ==================== */
void handleRoot() {
    String html;
    html += "<!DOCTYPE html>";
    html += "<html lang=\"en\">";
    html += "<head>";
    html += "<meta charset=\"UTF-8\">";
    html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
    html += "<title>Cooling Fan Control</title>";
    html += "<style>";
    html += "* { box-sizing: border-box; margin: 0; padding: 0; }";
    html += "body { font-family: sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; padding: 20px; }";
    html += ".container { max-width: 600px; margin: 0 auto; background: white; border-radius: 20px; box-shadow: 0 20px 60px rgba(0,0,0,0.3); padding: 30px; }";
    html += "h1 { text-align: center; color: #333; margin-bottom: 10px; font-size: 24px; }";
    html += ".lock-status { text-align: center; margin-bottom: 20px; font-weight: 600; }";
    html += ".status-card { background: #f0f4ff; border-radius: 12px; padding: 20px; margin-bottom: 20px; }";
    html += ".status-item { display: flex; justify-content: space-between; padding: 10px 0; border-bottom: 1px solid #e0e0e0; }";
    html += ".status-item:last-child { border-bottom: none; }";
    html += ".label { color: #666; font-weight: 500; }";
    html += ".value { color: #333; font-weight: 600; }";
    html += ".control-group { margin-bottom: 25px; }";
    html += ".control-label { display: block; margin-bottom: 10px; font-weight: 600; color: #333; }";
    html += ".slider-container { display: flex; align-items: center; gap: 15px; }";
    html += "input[type=range] { flex: 1; height: 8px; border-radius: 4px; -webkit-appearance: none; background: #e0e0e0; outline: none; }";
    html += "input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; width: 24px; height: 24px; border-radius: 50%; background: #667eea; cursor: pointer; }";
    html += "input[type=range]:disabled { opacity: 0.5; cursor: not-allowed; }";
    html += "input[type=range]:disabled::-webkit-slider-thumb { cursor: not-allowed; background: #999; }";
    html += ".slider-value { min-width: 50px; text-align: center; font-weight: 600; color: #667eea; }";
    html += ".btn-group { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 20px; }";
    html += ".btn { padding: 15px 20px; border: none; border-radius: 10px; font-size: 16px; font-weight: 600; cursor: pointer; }";
    html += ".btn:disabled { opacity: 0.5; cursor: not-allowed; }";
    html += ".btn-primary { background: #667eea; color: white; }";
    html += ".btn-danger { background: #ff6b6b; color: white; }";
    html += ".btn-warning { background: #f0ad4e; color: white; }";
    html += ".btn-success { background: #5cb85c; color: white; }";
    html += ".tec-buttons { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }";
    html += ".refresh { text-align: center; margin-top: 20px; }";
    html += ".refresh-btn { background: #48bb78; color: white; padding: 12px 30px; border: none; border-radius: 8px; font-size: 14px; font-weight: 600; cursor: pointer; }";
    html += ".lock-btn { width: 100%; margin-bottom: 20px; }";
    html += ".locked { color: #d9534f; }";
    html += ".unlocked { color: #5cb85c; }";
    html += ".fogger-btn { width: 100%; margin-top: 10px; }";
    html += "</style>";
    html += "</head>";
    html += "<body>";
    html += "<div class=\"container\">";
    html += "<h1>Cooling Fan Control</h1>";
    html += "<div class=\"lock-status\" id=\"lock-status\">Locked</div>";
    html += "<button class=\"btn btn-warning lock-btn\" id=\"lock-btn\" onclick=\"toggleLock()\">Unlock</button>";
    html += "<div class=\"status-card\">";
    html += "<div class=\"status-item\"><span class=\"label\">Main Fan</span><span class=\"value\" id=\"mf-value\">0%</span></div>";
    html += "<div class=\"status-item\"><span class=\"label\">CPU Fan</span><span class=\"value\" id=\"c-value\">0%</span></div>";
    html += "<div class=\"status-item\"><span class=\"label\">TEC</span><span class=\"value\" id=\"t-value\">OFF</span></div>";
    html += "<div class=\"status-item\"><span class=\"label\">Fogger</span><span class=\"value\" id=\"f-value\">OFF</span></div>";
    html += "<div class=\"status-item\"><span class=\"label\">Hot Temp</span><span class=\"value\" id=\"h-value\">--C</span></div>";
    html += "</div>";
    html += "<div class=\"control-group\">";
    html += "<label class=\"control-label\">Main Fan</label>";
    html += "<div class=\"slider-container\">";
    html += "<input type=\"range\" id=\"mf-slider\" min=\"0\" max=\"100\" value=\"0\" disabled>";
    html += "<span class=\"slider-value\" id=\"mf-display\">0%</span>";
    html += "</div>";
    html += "</div>";
    html += "<div class=\"control-group\">";
    html += "<label class=\"control-label\">CPU Fan</label>";
    html += "<div class=\"slider-container\">";
    html += "<input type=\"range\" id=\"c-slider\" min=\"0\" max=\"100\" value=\"0\" disabled>";
    html += "<span class=\"slider-value\" id=\"c-display\">0%</span>";
    html += "</div>";
    html += "</div>";
    html += "<div class=\"control-group\">";
    html += "<label class=\"control-label\">TEC Control</label>";
    html += "<div class=\"tec-buttons\">";
    html += "<button class=\"btn btn-danger\" id=\"tec-off-btn\" onclick=\"setTec(0)\" disabled>TEC OFF</button>";
    html += "<button class=\"btn btn-primary\" id=\"tec-on-btn\" onclick=\"setTec(100)\" disabled>TEC ON</button>";
    html += "</div>";
    html += "</div>";
    html += "<div class=\"control-group\">";
    html += "<label class=\"control-label\">Fogger Control</label>";
    html += "<button class=\"btn btn-primary fogger-btn\" id=\"fogger-btn\" onclick=\"toggleFogger()\" disabled>Toggle Fogger</button>";
    html += "</div>";
    html += "<div class=\"btn-group\">";
    html += "<button class=\"btn btn-primary\" id=\"all-on-btn\" onclick=\"setAll(30, 50, 100)\" disabled>ALL ON</button>";
    html += "<button class=\"btn btn-danger\" id=\"all-off-btn\" onclick=\"setAll(0, 0, 0)\" disabled>ALL OFF</button>";
    html += "</div>";
    html += "<div class=\"refresh\">";
    html += "<button class=\"refresh-btn\" onclick=\"refreshStatus()\">Refresh</button>";
    html += "</div>";
    html += "</div>";
    html += "<script>";
    html += "let isLocked = true;";
    html += "let lockTimer = null;";
    html += "let tecOn = false;";
    html += "function refreshStatus() {";
    html += "fetch('/api/status').then(r=>r.json()).then(d=>{";
    html += "document.getElementById('mf-value').textContent=d.MF+'%';";
    html += "if(isLocked) document.getElementById('mf-slider').value=d.MF;";
    html += "document.getElementById('mf-display').textContent=d.MF+'%';";
    html += "document.getElementById('c-value').textContent=d.C+'%';";
    html += "if(isLocked) document.getElementById('c-slider').value=d.C;";
    html += "document.getElementById('c-display').textContent=d.C+'%';";
    html += "tecOn = (d.T>0);";
    html += "document.getElementById('t-value').textContent=tecOn?'ON':'OFF';";
    html += "document.getElementById('f-value').textContent=d.F=='1'?'ON':'OFF';";
    html += "document.getElementById('h-value').textContent=d.H+'C';";
    html += "if(tecOn) { document.getElementById('c-slider').min='50'; if(parseInt(document.getElementById('c-slider').value)<50) document.getElementById('c-slider').value='50'; } else { document.getElementById('c-slider').min='0'; }";
    html += "});";
    html += "}";
    html += "function setValue(type,value){if(isLocked)return;fetch('/api/set?type='+type+'&value='+value).then(r=>r.json()).then(d=>refreshStatus());resetLockTimer();}";
    html += "function toggleFogger(){if(isLocked)return;fetch('/api/set?type=F').then(r=>r.json()).then(d=>refreshStatus());resetLockTimer();}";
    html += "function setAll(mf,c,t){if(isLocked)return;setValue('MF',mf);setValue('C',c);setValue('T',t);}";
    html += "function setTec(value){if(isLocked)return;setValue('T',value);}";
    html += "function toggleLock(){if(isLocked){unlockControls();}else{lockControls();}}";
    html += "function lockControls(){";
    html += "isLocked=true;clearTimeout(lockTimer);document.getElementById('lock-status').textContent='Locked';document.getElementById('lock-status').className='lock-status locked';document.getElementById('lock-btn').textContent='Unlock';document.getElementById('lock-btn').className='btn btn-warning lock-btn';document.getElementById('mf-slider').disabled=true;document.getElementById('c-slider').disabled=true;document.getElementById('tec-off-btn').disabled=true;document.getElementById('tec-on-btn').disabled=true;document.getElementById('fogger-btn').disabled=true;document.getElementById('all-on-btn').disabled=true;document.getElementById('all-off-btn').disabled=true;}";
    html += "function unlockControls(){";
    html += "isLocked=false;document.getElementById('lock-status').textContent='Unlocked';document.getElementById('lock-status').className='lock-status unlocked';document.getElementById('lock-btn').textContent='Lock';document.getElementById('lock-btn').className='btn btn-success lock-btn';document.getElementById('mf-slider').disabled=false;document.getElementById('c-slider').disabled=false;document.getElementById('tec-off-btn').disabled=false;document.getElementById('tec-on-btn').disabled=false;document.getElementById('fogger-btn').disabled=false;document.getElementById('all-on-btn').disabled=false;document.getElementById('all-off-btn').disabled=false;resetLockTimer();}";
    html += "function resetLockTimer(){";
    html += "clearTimeout(lockTimer);if(!isLocked)lockTimer=setTimeout(lockControls,3000);}";
    html += "document.getElementById('mf-slider').addEventListener('input',function(){if(isLocked)return;document.getElementById('mf-display').textContent=this.value+'%';resetLockTimer();});";
    html += "document.getElementById('mf-slider').addEventListener('change',function(){if(isLocked)return;setValue('MF',this.value);resetLockTimer();});";
    html += "document.getElementById('c-slider').addEventListener('input',function(){if(isLocked)return;let val=parseInt(this.value);if(tecOn&&val<50)val=50;this.value=val;document.getElementById('c-display').textContent=val+'%';resetLockTimer();});";
    html += "document.getElementById('c-slider').addEventListener('change',function(){if(isLocked)return;let val=parseInt(this.value);if(tecOn&&val<50)val=50;setValue('C',val);resetLockTimer();});";
    html += "window.onload=function(){refreshStatus();setInterval(refreshStatus,5000);};";
    html += "</script>";
    html += "</body>";
    html += "</html>";
    server.send(200, "text/html", html);
}

void handleApiStatus() {
    String json = "{";
    json += "\"MF\":\"" + String(mainFanDuty) + "\",";
    json += "\"C\":\"" + String(cpuFanDuty) + "\",";
    json += "\"T\":\"" + String(tecDuty) + "\",";
    json += "\"H\":\"";
    if (!isnan(tempHot)) {
        json += String(tempHot, 1);
    } else {
        json += "--";
    }
    json += "\",";
    json += "\"F\":\"" + String(foggerOn ? "1" : "0") + "\"}";
    
    server.send(200, "application/json", json);
}

void handleApiSet() {
    if (server.hasArg("type")) {
        String type = server.arg("type");
        
        if (type == "MF") {
            if (server.hasArg("value")) {
                int value = server.arg("value").toInt();
                value = constrain(value, 0, 100);
                ctrlMode = MODE_MANUAL;
                setMainFan((uint8_t)value);
            }
        } else if (type == "C") {
            if (server.hasArg("value")) {
                int value = server.arg("value").toInt();
                value = constrain(value, 0, 100);
                ctrlMode = MODE_MANUAL;
                setCpuFan((uint8_t)value);
            }
        } else if (type == "T") {
            if (server.hasArg("value")) {
                int value = server.arg("value").toInt();
                value = constrain(value, 0, 100);
                setTec((uint8_t)value);
            }
        } else if (type == "F") {
            /* 雾化器翻转，不需要value参数 */
            toggleFogger();
        }
        
        handleApiStatus();
        return;
    }
    
    server.send(400, "application/json", "{\"error\":\"Invalid parameters\"}");
}

void toggleFogger() {
    /* 雾化器状态翻转，通过模拟微动按键实现 */
    /* 完整序列: 高 -> 低 -> 高，保持低电平一定时间 */
    
    Serial.print(F("[FOGGER] Toggle: "));
    Serial.print(foggerOn ? F("ON") : F("OFF"));
    Serial.print(F(" -> "));
    
    /* 翻转状态 */
    foggerOn = !foggerOn;
    
    Serial.println(foggerOn ? F("ON") : F("OFF"));
    
    /* 模拟按键：高 -> 低 -> 高 */
    digitalWrite(PIN_FOGGER, LOW);
    delay(50); /* 低电平保持50ms */
    digitalWrite(PIN_FOGGER, HIGH);
}

void initWiFi() {
    Serial.print(F("[WIFI] Starting AP: "));
    Serial.println(WIFI_SSID);
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print(F("[WIFI] AP IP: "));
    Serial.println(IP);
    
    // 设置路由
    server.on("/", handleRoot);
    server.on("/api/status", handleApiStatus);
    server.on("/api/set", handleApiSet);
    
    server.begin();
    Serial.print(F("[WIFI] HTTP Server started on port "));
    Serial.println(HTTP_PORT);
    
    Serial.println(F("[WIFI] Connect to: ESP32_FAN_CTRL"));
    Serial.print(F("[WIFI] Open browser to: http://"));
    Serial.println(IP);
}

/* ==================== 中断服务程序 ==================== */
void IRAM_ATTR cpuFgIsr() {
    unsigned long now = micros();
    if (now - lastCpuFgTime < FG_DEBOUNCE_US) return;
    lastCpuFgTime = now;
    cpuFgCount++;
}

void IRAM_ATTR mainFgIsr() {
    unsigned long now = micros();
    if (now - lastMainFgTime < FG_DEBOUNCE_US) return;
    lastMainFgTime = now;
    mainFgCount++;
}

void calculateRpm() {
    noInterrupts();
    unsigned long cpuCnt = cpuFgCount;
    unsigned long mainCnt = mainFgCount;
    cpuFgCount = 0;
    mainFgCount = 0;
    interrupts();
    
    if (cpuCnt > 0) {
        unsigned int rawRpm = (unsigned int)(cpuCnt * 60000UL / (PULSES_PER_REV * FG_SAMPLE_MS));
        if (rawRpm > MAX_VALID_RPM) {
            cpuRpm = rawRpm;
            static unsigned long lastNoiseWarn = 0;
            if (millis() - lastNoiseWarn >= 5000) {
                lastNoiseWarn = millis();
                Serial.print(F("[WARN] CPU FG noise detected: "));
                Serial.print(cpuCnt);
                Serial.print(F("p -> "));
                Serial.print(rawRpm);
                Serial.println(F("rpm"));
            }
        } else {
            cpuRpm = rawRpm;
        }
    } else {
        cpuRpm = 0;
    }
    
    if (mainCnt > 0) {
        unsigned int rawMainRpm = (unsigned int)(mainCnt * 60000UL / (PULSES_PER_REV * FG_SAMPLE_MS));
        if (rawMainRpm > MAX_VALID_RPM) {
            mainRpm = rawMainRpm;
        } else {
            mainRpm = rawMainRpm;
        }
    } else {
        mainRpm = 0;
    }
    
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug >= 10000) {
        lastDebug = millis();
        Serial.print(F("[FG] CPU: "));
        Serial.print(cpuCnt);
        Serial.print(F("p -> "));
        Serial.print(cpuRpm);
        Serial.println(F("rpm"));
        
        Serial.print(F("     Main: "));
        Serial.print(mainCnt);
        Serial.print(F(" -> "));
        Serial.println(mainRpm);
        
        static unsigned long lastTempErr = 0;
        if (isnan(tempCold) || isnan(tempHot)) {
            if (millis() - lastTempErr >= 60000) {
                lastTempErr = millis();
                Serial.println(F("[WARN] DS18B20 not connected (ignoring...)"));
            }
        }
    }
}

/* ==================== 控制函数 ==================== */
void setCpuFan(uint8_t duty) {
    if (duty > 100) duty = 100;
    // 如果TEC打开，CPU风扇不能低于50%
    if (tecDuty > 0 && duty < 50) {
        duty = 50;
    }
    cpuFanDuty = duty;
    
    if (duty > 0 && !cpuFanSwState) {
        setCpuFanSw(true);
    } else if (duty == 0 && cpuFanSwState) {
        setCpuFanSw(false);
    }
    
    if (cpuFanSwState) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (float)duty);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
    }
    
    cpuTheoreticalRpm = getTheoreticalRpm(duty, CPU_FAN_MAX_RPM);
}

void setMainFan(uint8_t duty) {
    if (duty > 100) duty = 100;
    mainFanDuty = duty;
    
    if (duty == 0) {
        mainFanDesiredPulse = PULSE_MIN;
    } else {
        mainFanDesiredPulse = PULSE_MIN + (uint16_t)((float)(PULSE_MAX - PULSE_MIN) * duty / 100.0);
    }
    
    mainTheoreticalRpm = getTheoreticalRpm(duty, MAIN_FAN_MAX_RPM);
}

void updateMainFanRamp() {
    static unsigned long lastRampUpdate = 0;
    unsigned long now = millis();
    
    if (now - lastRampUpdate >= RAMP_UPDATE_INTERVAL) {
        lastRampUpdate = now;
        
        unsigned long delta_ms = RAMP_UPDATE_INTERVAL;
        int32_t max_change = (int32_t)(RAMP_RATE * delta_ms / 1000);
        int32_t delta_pulse = (int32_t)mainFanDesiredPulse - (int32_t)mainFanTargetPulse;
        
        if (delta_pulse > 0) {
            if (delta_pulse > max_change) {
                mainFanTargetPulse += (uint16_t)max_change;
            } else {
                mainFanTargetPulse = mainFanDesiredPulse;
            }
        } else if (delta_pulse < 0) {
            if (-delta_pulse > max_change) {
                mainFanTargetPulse -= (uint16_t)max_change;
            } else {
                mainFanTargetPulse = mainFanDesiredPulse;
            }
        }
        
        float pulsePercent = (float)mainFanTargetPulse * 100.0 / PULSE_PERIOD;
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, pulsePercent);
    }
}

void setTec(uint8_t duty) {
    if (duty > 100) duty = 100;
    tecDuty = duty;
    
    // 如果打开TEC，确保CPU风扇至少50%
    if (duty > 0 && cpuFanDuty < 50) {
        setCpuFan(50);
    }
    
    pinMode(PIN_TEC, OUTPUT);
    digitalWrite(PIN_TEC, (duty > 0) ? HIGH : LOW);
    digitalWrite(PIN_LED, (duty > 0) ? HIGH : LOW);
}

uint16_t getTheoreticalRpm(uint8_t duty, uint16_t maxRpm) {
    if (duty == 0) return 0;
    if (duty > 100) duty = 100;
    return (uint16_t)((unsigned long)duty * maxRpm / 100);
}

/* ==================== 温度采集 ==================== */
void updateTemperatures() {
    sensorCold.requestTemperatures();
    sensorHot.requestTemperatures();
    
    tempCold = sensorCold.getTempCByIndex(0);
    tempHot = sensorHot.getTempCByIndex(0);
    tempDelta = tempHot - tempCold;
    
    if (tempCold == -127.0 || tempCold == 85.0) {
        tempCold = NAN;
    }
    if (tempHot == -127.0 || tempHot == 85.0) {
        tempHot = NAN;
    }
}

/* ==================== 自动控制逻辑 ==================== */
void autoControlLoop() {
    return;
}

/* ==================== 安全保护 ==================== */
void safetyCheck() {
    bool fault = false;
    
    if (!isnan(tempCold) && tempCold > TEMP_MAX_COLD) {
        Serial.print(F("[!!!] COLD OVERTEMP: "));
        Serial.print(tempCold, 1);
        Serial.println(F(" C"));
        fault = true;
    }
    
    if (!isnan(tempHot) && tempHot > TEMP_MAX_HOT) {
        Serial.print(F("[!!!] HOT OVERTEMP: "));
        Serial.print(tempHot, 1);
        Serial.println(F(" C"));
        fault = true;
    }
    
    if (isnan(tempCold) || isnan(tempHot)) {
        static unsigned long lastErr = 0;
        if (millis() - lastErr > 10000) {
            lastErr = millis();
            Serial.println(F("[ERR] Sensor disconnected!"));
        }
    }
    
    if (fault) {
        static bool ledState = false;
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState ? HIGH : LOW);
    }
}

/* ==================== 菜单处理 ==================== */
void handleMenuEvent(BtnEvent event) {
    if (event == BTN_NONE) return;
    
    Serial.print(F("[BTN] "));
    switch(event) {
        case BTN_SHORT_PRESS: Serial.print(F("CLICK")); break;
        case BTN_DOUBLE_CLICK: Serial.print(F("DOUBLE")); break;
        case BTN_LONG_PRESS: Serial.print(F("LONG")); break;
        default: Serial.print(F("NONE")); break;
    }
    Serial.print(F(" | Row="));
    Serial.print(menuCurrentRow);
    Serial.print(F(" | Col="));
    Serial.println(menuCurrentCol);
    
    switch (event) {
        case BTN_SHORT_PRESS:
            // 切换列时跳过禁用的选项
            uint8_t nextCol;
            do {
                nextCol = (menuCurrentCol + 1) % COLS_PER_ROW[menuCurrentRow];
                // 检查是否是 CPU 菜单且 TEC 打开且目标列小于2
                bool isDisabled = (menuCurrentRow == 0 && tecState == TEC_ON && nextCol < 2);
                if (!isDisabled) {
                    menuCurrentCol = nextCol;
                    break;
                }
                menuCurrentCol = nextCol;
            } while (true);
            break;
            
        case BTN_DOUBLE_CLICK:
            menuCurrentRow = (menuCurrentRow + 1) % MAX_ROWS;
            if (menuCurrentCol >= COLS_PER_ROW[menuCurrentRow]) {
                menuCurrentCol = 0;
            }
            // 如果是 CPU 菜单且 TEC 打开且当前列小于2，自动跳到第2列
            if (menuCurrentRow == 0 && tecState == TEC_ON && menuCurrentCol < 2) {
                menuCurrentCol = 2;
            }
            break;
            
        case BTN_LONG_PRESS:
            applyMenuRowColSetting(menuCurrentRow, menuCurrentCol);
            break;
            
        default:
            break;
    }
}

void applyMenuRowColSetting(uint8_t row, uint8_t col) {
    switch (row) {
        case 0:
            // 如果 TEC 打开且选择的列小于2，不允许
            if (tecState == TEC_ON && col < 2) {
                Serial.println(F("[SET] Disabled when TEC ON"));
                return;
            }
            cpuFanSpeedIdx = col;
            break;
            
        case 1:
            mainFanSpeedIdx = col;
            break;
            
        case 2:
            tecState = (col == 0) ? TEC_OFF : TEC_ON;
            if (tecState == TEC_ON && cpuFanSpeedIdx < 2) {
                cpuFanSpeedIdx = FAN_LOW;
            }
            // 如果现在是 CPU 菜单且列小于2，自动跳
            if (menuCurrentRow == 0 && menuCurrentCol < 2) {
                menuCurrentCol = 2;
            }
            break;
            
        case 3:
            // Fogger 菜单
            if (col == 0) {
                if (foggerOn) toggleFogger();
            } else {
                if (!foggerOn) toggleFogger();
            }
            break;
    }
    
    applyMenuSettings();
    
    Serial.print(F("[SET] Row="));
    Serial.print(row);
    Serial.print(F(" Col="));
    Serial.print(col);
    Serial.print(F(" | CPU="));
    Serial.print(SPEED_NAMES[cpuFanSpeedIdx]);
    Serial.print(F(" FAN="));
    Serial.print(SPEED_NAMES[mainFanSpeedIdx]);
    Serial.print(F(" TEC="));
    Serial.print(TEC_NAMES[tecState]);
    Serial.print(F(" FOG="));
    Serial.println(foggerOn ? F("ON") : F("OFF"));
}

void applyMenuSettings() {
    setCpuFan(CPU_DUTY_TABLE[cpuFanSpeedIdx]);
    setMainFan(MAIN_DUTY_TABLE[mainFanSpeedIdx]);
    setTec((tecState == TEC_ON) ? 100 : 0);
}

/* ==================== OLED菜单显示 ==================== */
void updateOled() {
    display.clearDisplay();
    drawMenuScreen();
    display.display();
}

void drawMenuScreen() {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    display.setCursor(0, 0);
    display.print(F("C:"));
    if (!isnan(tempCold)) {
        if (tempCold < 100) display.print(F(" "));
        display.print(tempCold, 1);
        display.print(F("C"));
    } else {
        display.print(F("--.-"));
    }
    
    display.print(F(" H:"));
    if (!isnan(tempHot)) {
        if (tempHot < 100) display.print(F(" "));
        display.print(tempHot, 1);
        display.print(F("C"));
    } else {
        display.print(F("--.-"));
    }
    
    display.setCursor(0, 10);
    display.print((menuCurrentRow == 0) ? F("^") : F(" "));
    display.print(F("CPU["));
    
    for (int i = 0; i < FAN_SPEED_COUNT; i++) {
        if (menuCurrentRow == 0 && menuCurrentCol == i) {
            display.print(F("*"));
        }
        
        bool isDisabled = (tecState == TEC_ON && i < 2);
        
        if (isDisabled) {
            display.print(F(" X "));
        } else if (cpuFanSpeedIdx == i) {
            display.print(SPEED_NAMES[i]);
        } else {
            display.print(F("   "));
        }
        
        if (i < FAN_SPEED_COUNT - 1) display.print(F("]["));
    }
    display.print(F("]"));
    
    display.setCursor(0, 20);
    display.print((menuCurrentRow == 1) ? F("^") : F(" "));
    display.print(F("FAN["));
    
    for (int i = 0; i < FAN_SPEED_COUNT; i++) {
        if (menuCurrentRow == 1 && menuCurrentCol == i) {
            display.print(F("*"));
        }
        
        if (mainFanSpeedIdx == i) {
            display.print(SPEED_NAMES[i]);
        } else {
            display.print(F("   "));
        }
        
        if (i < FAN_SPEED_COUNT - 1) display.print(F("]["));
    }
    display.print(F("]"));
    
    display.setCursor(0, 30);
    display.print((menuCurrentRow == 2) ? F("^") : F(" "));
    display.print(F("TEC["));
    
    for (int i = 0; i < TEC_STATE_COUNT; i++) {
        if (menuCurrentRow == 2 && menuCurrentCol == i) {
            display.print(F("*"));
        }
        
        if (tecState == i) {
            display.print(TEC_NAMES[i]);
        } else {
            display.print(F("   "));
        }
        
        if (i < TEC_STATE_COUNT - 1) display.print(F("]["));
    }
    display.print(F("]"));
    
    display.setCursor(0, 40);
    display.print((menuCurrentRow == 3) ? F("^") : F(" "));
    display.print(F("FOG["));
    
    for (int i = 0; i < FOGGER_STATE_COUNT; i++) {
        if (menuCurrentRow == 3 && menuCurrentCol == i) {
            display.print(F("*"));
        }
        
        uint8_t foggerStateIdx = foggerOn ? FOGGER_ON : FOGGER_OFF;
        if (foggerStateIdx == i) {
            display.print(FOGGER_NAMES[i]);
        } else {
            display.print(F("   "));
        }
        
        if (i < FOGGER_STATE_COUNT - 1) display.print(F("]["));
    }
    display.print(F("]"));
    
    display.setCursor(0, 54);
    display.print(F("^=Row *=Col Long=OK"));
}

/* ==================== 串口命令处理 ==================== */
void processCommand(String cmd) {
    cmd.trim();
    
    String upperCmd = cmd;
    upperCmd.toUpperCase();
    
    if (upperCmd == "OFF") {
        ctrlMode = MODE_MANUAL;
        setMainFan(0);
        setCpuFan(0);
        setTec(0);
    } else if (upperCmd.startsWith("C ")) {
        String val = cmd.substring(2);
        int d = val.toInt();
        if (d >= 0 && d <= 100) {
            ctrlMode = MODE_MANUAL;
            setCpuFan((uint8_t)d);
        }
    } else if (upperCmd.startsWith("MF ")) {
        String val = cmd.substring(3);
        int d = val.toInt();
        if (d >= 0 && d <= 100) {
            ctrlMode = MODE_MANUAL;
            setMainFan((uint8_t)d);
        }
    } else if (upperCmd.startsWith("T ")) {
        String val = cmd.substring(2);
        int d = val.toInt();
        if (d >= 0 && d <= 100) {
            setTec((uint8_t)d);
        }
    }
    
    String response = "{MF:\"";
    response += String(mainFanDuty);
    response += "\",C:\"";
    response += String(cpuFanDuty);
    response += "\",T:\"";
    response += String(tecDuty);
    response += "\",H:\"";
    if (!isnan(tempHot)) {
        response += String(tempHot, 1);
    } else {
        response += "--";
    }
    response += "\"}";
    
    Serial.println(response);
}
