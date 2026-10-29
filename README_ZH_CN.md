<div align="center">

[![한국어](https://img.shields.io/badge/Language-한국어-gray?style=for-the-badge)](README.md)
[![English](https://img.shields.io/badge/Language-English-gray?style=for-the-badge)](README_EN.md)
[![日本語](https://img.shields.io/badge/Language-日本語-gray?style=for-the-badge)](README_JA.md)
[![简体中文](https://img.shields.io/badge/Language-简体中文-0078D7?style=for-the-badge)](#)
[![繁體中文](https://img.shields.io/badge/Language-繁體中文-gray?style=for-the-badge)](README_ZH_TW.md)

</div>

# ⚡ 特斯拉行车记录仪播放器 - CT Dashcam Studio

<div align="center">

![Platform](https://img.shields.io/badge/Platform-Windows-0078D7?style=for-the-badge&logo=windows&logoColor=white)
![Release](https://img.shields.io/badge/Release-v1.3.0-00E6FF?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-39FF14?style=for-the-badge)

**专为特斯拉原厂行车记录仪和哨兵模式视频打造的完全免费多机位播放器与车辆数据分析工作台**

厌倦了网络免费工具强制添加水印或画质压缩？CT Dashcam Studio 为您提供真正无损的高性能回放与剪辑体验。

[📥 下载 (v1.3.0)](https://github.com/mhsong282828221212123123123/CT-dashcam-studio/releases) • [主要功能](#-主要功能) • [软件截图](#-软件截图) • [使用方法](#-使用方法) • [快捷键说明](#-快捷键说明)

</div>

---

## 📸 软件截图

<div align="center">

| 2x2 多机位 & 实时 HUD 数据仪表 | 哨兵模式动态侦测分析 |
| :---: | :---: |
| ![主视图](screenshot1.png) | ![哨兵分析](screenshot2.png) |

| 多片段连续时间线剪辑 | AI 车牌与人脸隐私遮蔽 |
| :---: | :---: |
| ![时间线裁剪](screenshot3.png) | ![隐私保护](screenshot_privacy.png) |

| PC 端一键生成无线 QR 码 | 手机扫码直存画廊 |
| :---: | :---: |
| <img src="screenshot4.png" width="450" alt="QR传图"/> | <img src="screenshot5.png" width="220" alt="手机视图"/> |

</div>

---

## ✨ 主要功能

- 🎥 **多机位同步播放**: 前视、后视、左侧、右侧 4 路摄像头画面毫秒级同步回放（支持 1:3、2x2、单镜头全屏布局切换）
- 📊 **实时 HUD 数据仪表**: 将行驶数据（时速、FSD自动辅助驾驶状态、转向灯、方向盘转角、电门与刹车开度）可视化叠印于画面中
- 🗺️ **GPS 迷你地图**: 结合 OpenStreetMap 实时展示行车轨迹与行进方向
- 🛡️ **AI 隐私保护遮蔽**: 自动识别画面中的车牌和人脸并添加动态马赛克（方便分享到社交平台）
- 🔎 **车牌画质增强**: 针对模糊车牌区域局部优化亮度、对比度与锐度，显著提升夜间及远距离车牌识别率
- 🔍 **哨兵模式动态侦测**: 自动扫描并标记停车期间的可疑人员或车辆靠近事件，支持时间线一键跳转
- ✂️ **多片段无缝连续剪辑**: 自由拼接多个 1 分钟短视频片段，快速修剪并导出高清 MP4
- 📱 **手机无线免线互传 (QR)**: PC 渲染完成后自动生成二维码，手机原生相机扫描即可在局域网内直接保存至相册
- 🔒 **加密视频识别 (EncryptedClips)**: 自动检测特斯拉加密安全文件夹并弹出横幅提示，指引车内播放方式
- ☀️ **画面亮度补偿与倍速**: 针对深色贴膜或夜间场景提供画面亮度和对比度微调，支持 0.5x ~ 5.0x 变速播放

---

## 🚀 使用方法

### 便携免安装版（推荐）
1. 前往 [Releases](https://github.com/mhsong282828221212123123123/CT-dashcam-studio/releases) 下载 `CT_Dashcam_Studio_v1.3.0_Portable.zip` 并解压。
2. 双击运行 `CT_Dashcam_Studio.exe`。
3. 点击左上角的 **[📁 指定 TeslaCam 文件夹]** 按钮，选择 U 盘中的 `TeslaCam` 目录。
4. *注：由于未购买商业数字签名证书，Windows 可能会弹出安全提示，本项目代码完全开源透明。如有顾虑亦可使用 Python 源码运行。*

### 使用 Python 源码运行
```bash
git clone https://github.com/mhsong282828221212123123123/CT-dashcam-studio.git
cd CT-dashcam-studio
pip install PyQt6 opencv-python numpy Pillow qrcode imageio-ffmpeg
python CT_Dashcam_Studio.py
```

---

## ⌨️ 快捷键与鼠标操作说明

| 功能 | 快捷键 / 鼠标操作 | 详细说明 |
| :--- | :--- | :--- |
| **播放 / 暂停** | `Space` | 切换视频播放或暂停 |
| **微调 1 秒** | `◀` / `▶` | 快退 / 快进 1 秒 |
| **快进 5 秒** | `Shift + ◀` / `Shift + ▶` | 快速后退 / 前进 5 秒 |
| **回到开头** | `Home` | 回到当前片段起始点 (0秒) |
| **切换片段** | `[` / `]` | 快速切换至上一个 / 下一个片段 |
| **标记剪辑起点/终点** | `I` / `O` | 标记导出选区的入点 (In) 与出点 (Out) |
| **侦测事件跳转** | `⚡ 上一个` / `⚡ 下一个` | 快速跳转至下一个哨兵动态侦测位置 |
| **单镜头放大/还原** | `双击对应摄像头画面` | 切换该机位全屏独占，再次双击还原多机位布局 |
| **调整数据项顺序** | `拖拽底部数据栏` | 左右拖动时速、时间等数据项自定义排布顺序 |
| **重置数据项顺序** | `双击底部数据栏` | 恢复 HUD 数据项的默认排布顺序 |
| **重置画面与倍速** | `双击对应滑块或标签` | 快速将倍速(1.0x)、亮度(0)、对比度(100%)重置为默认值 |
| **快捷键指南** | `F1` 或 `?` | 打开快捷键与操作说明窗口 |

---

## 📄 开源许可证

本项目基于 [MIT LICENSE](LICENSE) 协议发布。
