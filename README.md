# None-Projects

一个存放各种**小项目 / 实验 / 工具**的集合仓库。

---

## 项目索引

### 已收录项目

| 项目路径 | 名称 | 描述 |
|----------|------|------|
| [`ESP32-S/Fan/`](./ESP32-S/Fan/) | ESP32-S 自制制冷风扇 | ESP32 驱动的 DIY 制冷风扇控制器（初设，安全性无法保障，仅供参考） |

### 目录结构

```
None-Projects/
├── ESP32-S/
│   └── Fan/                      # ESP32-S 风扇控制器项目
│       ├── smart_fan_controller/ # Arduino 固件
│       ├── 3D-Models/            # 结构件 3D 模型
│       ├── README.md
│       └── .gitignore
├── README.md                     # 本文件（项目总索引）
└── .gitignore                    # 全局忽略规则
```

---

## 仓库说明

本仓库用于收录各种**小型项目**，按**硬件平台 / 功能分类**组织目录。

每个独立项目拥有：
- 自己的 `README.md` — 说明项目详情、使用方法、依赖
- 自己的 `.gitignore` — 项目级忽略规则
- 独立的源代码 / 设计文件

---

## 新增项目指南

1. 选择合适的分类目录（如 `ESP32-S/`、`STM32/`、`Tools/` 等）
2. 在分类目录下创建项目文件夹
3. 编写项目级 `README.md` 和 `.gitignore`
4. 提交 Pull Request 或直接推送
