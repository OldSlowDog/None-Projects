# ESP32-S Smart Fan Controller (智能风扇/制冷控制器)

基于 ESP32-S 的专业制冷风扇控制器，支持双风扇无级调速、TEC 半导体制冷片控制、温度监测、OLED 菜单操作以及 WiFi 远程控制。

---

## 功能特性

| 功能 | 描述 |
|------|------|
| **CPU 风扇控制** | GPIO15 输出 25kHz PWM，0-100% 无级调速，带物理电源开关 (GPIO12) |
| **主风扇控制** | GPIO13 输出 50Hz 舵机协议 PWM，支持电调（ESC）控制，带软启动斜坡 |
| **TEC 制冷片** | GPIO2 MOS 继电器开关控制，开启时自动强制 CPU 风扇 ≥ 50% |
| **双温度传感** | 两路 DS18B20（冷端 GPIO19 + 热端 GPIO16），12 位分辨率 |
| **转速反馈** | 两路 FG 信号输入（GPIO17/GPIO18），中断计数，理论转速映射 |
| **OLED 显示** | I2C SSD1306 128×64（SDA=21, SCL=22），单按键菜单系统 |
| **按键控制** | GPIO4 单按键，支持短按（换列）/ 双击（换行）/ 长按（确认） |
| **雾化器控制** | GPIO25 模拟微动按键序列（高→低→高） |
| **WiFi 控制** | AP 模式（ESP32_FAN_CTRL）+ HTTP 服务器（端口 80）Web 界面 |
| **串口控制** | 115200bps，JSON 响应格式（MF/C/T/H/F） |
| **硬件 MCPWM** | ESP32 原生 MCPWM 外设，无需手动模拟 |
| **安全保护** | 冷端/热端温度超限告警，LED 闪烁指示 |

---

## 硬件引脚定义

| 引脚 | 功能 | 备注 |
|------|------|------|
| GPIO2 | TEC 半导体制冷片 + LED 状态指示 | MOS 继电器，开关量 |
| GPIO4 | 物理按键输入 | INPUT_PULLUP，低电平有效 |
| GPIO12 | CPU 风扇电源开关 | 高电平=开 |
| GPIO13 | 主风扇 PWM（电调 50Hz） | MCPWM1B |
| GPIO15 | CPU 风扇 PWM（25kHz） | MCPWM0A |
| GPIO17 | CPU 风扇 FG 测速 | 上拉，下降沿中断 |
| GPIO18 | 主风扇 FG 测速 | 上拉，下降沿中断 |
| GPIO19 | DS18B20 冷端温度 | OneWire |
| GPIO16 | DS18B20 热端温度 | OneWire |
| GPIO21 | OLED SDA | I2C |
| GPIO22 | OLED SCL | I2C |
| GPIO25 | 雾化器控制 | 推挽输出，模拟按键 |
| GPIO14/23/26 | GND 替代引脚 | 输出低电平 |

---

## 目录结构

```
ESP32-S/Fan/
├── smart_fan_controller/
│   └── smart_fan_controller.ino    # Arduino 主程序 (V2.6)
├── 3D-Models/                      # 外壳/结构件 3D 打印文件
│   ├── 底座.stp                    # 底座
│   ├── 顶盖.stp                    # 顶盖
│   ├── 上风筒.stp                  # 上风筒
│   ├── 末风筒(上).stp              # 末风筒-上部
│   ├── 末风筒(下).stp              # 末风筒-下部
│   ├── 进风口.stp                  # 进风口
│   ├── 电机固定板.stp              # 电机固定板
│   ├── 半导体嵌装层.stp            # TEC 制冷片嵌装层
│   └── 隔离柱.stp                  # 隔离柱
├── README.md                       # 项目说明文档
└── .gitignore                      # Git 忽略规则
```

---

## 软件依赖

- **开发板**: ESP32 (Arduino Core for ESP32)
- **库依赖**:
  - `Wire` (内置) - I2C
  - `Adafruit GFX Library` - OLED 图形
  - `Adafruit SSD1306` - OLED 驱动
  - `OneWire` - DS18B20 总线
  - `DallasTemperature` - DS18B20 温度读取
  - `driver/mcpwm.h` (ESP32 SDK 内置) - 硬件 PWM
  - `WiFi` (内置) - WiFi
  - `WebServer` (内置) - HTTP 服务器

---

## 使用方法

### 1. 烧录固件

1. 安装 Arduino IDE + ESP32 开发板支持
2. 安装上述依赖库
3. 打开 `smart_fan_controller.ino`
4. 选择开发板：**ESP32 Dev Module**（或你的型号）
5. 选择正确的串口
6. 点击上传

### 2. OLED 菜单操作

```
C: 25.3°C  H: 32.1°C     ← 温度显示
^CPU[OFF][  ][Mid][Hi ]  ← CPU 风扇 (行0)
 FAN[OFF][Low][  ][Hi ]  ← 主风扇 (行1)
 TEC[OFF][ON ]            ← TEC 制冷 (行2)
 FOG[OFF][ON ]            ← 雾化器 (行3)
^=Row *=Col Long=OK       ← 操作提示
```

- **短按** → 切换当前行的选项（列）
- **双击** → 切换下一行
- **长按** → 确认选中的设置
- 当 TEC 为 ON 时，CPU 风扇 OFF/Low 被禁用（自动跳至 Mid 及以上）

### 3. WiFi Web 控制

1. 连接 WiFi **ESP32_FAN_CTRL**，密码 **12345678**
2. 浏览器打开 `http://192.168.4.1`
3. 点击 **Unlock** 解锁操作（3 秒无操作自动锁定）
4. 拖动滑块/点击按钮进行控制
5. 每 5 秒自动刷新状态

### 4. 串口命令

波特率 **115200**

| 命令 | 功能 |
|------|------|
| `OFF` | 全部关闭 |
| `C 50` | 设置 CPU 风扇占空比 (0-100) |
| `MF 50` | 设置主风扇占空比 (0-100) |
| `T 100` | 设置 TEC 开关 (0=OFF, 100=ON) |

响应示例：`{MF:"50",C:"60",T:"100",H:"35.2"}`

---

## 参数调整

在 `smart_fan_controller.ino` 顶部宏定义区域修改：

```cpp
#define WIFI_SSID   "ESP32_FAN_CTRL"   // WiFi 名称
#define WIFI_PASS   "12345678"          // WiFi 密码
#define TEMP_MAX_COLD  60.0             // 冷端安全温度
#define TEMP_MAX_HOT   70.0             // 热端安全温度
#define PULSE_MIN  1000                 // 电调最小脉宽（us）
#define PULSE_MAX  1500                 // 电调最大脉宽（us）
#define RAMP_RATE  200                  // 主风扇软启动速率（us/s）
```

---

## 版本历史

- **V2.6** - 简化安全逻辑，统一 JSON 响应，添加雾化器控制
- **V2.x** - 硬件 MCPWM，WiFi HTTP 界面，单按键菜单
- **V1.x** - 基础风扇+TEC 控制

---

## 注意事项

1. **TEC 开启后 CPU 风扇不会低于 50%** — 保护制冷片不致过热损坏
2. **电调信号** 使用标准 1000-1500μs 脉宽（可根据实际电调调整）
3. **DS18B20 未连接** 时显示 `--.-`，程序继续运行（每 60 秒警告一次）
4. **雾化器控制** 为模拟微动按键（50ms 低电平脉冲），需配合支持的雾化器模块
5. STP 文件为 STEP 格式，可直接在主流 CAD 软件（SolidWorks、Fusion 360、FreeCAD）中打开

## License

项目归属于 [OldSlowDog/None-Projects](https://github.com/OldSlowDog/None-Projects) 仓库
