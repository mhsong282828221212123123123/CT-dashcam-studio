<div align="center">

[![한국어](https://img.shields.io/badge/Language-한국어-gray?style=for-the-badge)](README.md)
[![English](https://img.shields.io/badge/Language-English-0078D7?style=for-the-badge)](#)
[![日本語](https://img.shields.io/badge/Language-日本語-gray?style=for-the-badge)](README_JA.md)
[![简体中文](https://img.shields.io/badge/Language-简体中文-gray?style=for-the-badge)](README_ZH_CN.md)
[![繁體中文](https://img.shields.io/badge/Language-繁體中文-gray?style=for-the-badge)](README_ZH_TW.md)

</div>

# ⚡ Tesla Dashcam Viewer - CT Dashcam Studio

<div align="center">

![Platform](https://img.shields.io/badge/Platform-Windows-0078D7?style=for-the-badge&logo=windows&logoColor=white)
![Release](https://img.shields.io/badge/Release-v1.3.0-00E6FF?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-39FF14?style=for-the-badge)

**Completely free multi-cam viewer & telemetry studio for Tesla OEM dashcam and Sentry Mode videos**

Frustrated by existing viewers stamping watermarks or downgrading quality? CT Dashcam Studio is built to give you full control.

[📥 Download (v1.3.0)](https://github.com/mhsong282828221212123123123/CT-dashcam-studio/releases) • [Features](#-features) • [Screenshots](#-screenshots) • [How to Use](#-how-to-use) • [Shortcuts](#-shortcuts)

</div>

---

## 📸 Screenshots

<div align="center">

| 2x2 Multi-Cam & Telemetry HUD | Sentry Mode Motion Analysis |
| :---: | :---: |
| ![Main View](screenshot1.png) | ![Sentry Analysis](screenshot2.png) |

| Multi-Clip Continuous Trimming | AI Plate & Face Privacy Blur |
| :---: | :---: |
| ![Clip Trimming](screenshot3.png) | ![Privacy Protection](screenshot_privacy.png) |

| One-Click PC QR Generation | Mobile Direct Download Page |
| :---: | :---: |
| <img src="screenshot4.png" width="450" alt="QR Transfer"/> | <img src="screenshot5.png" width="220" alt="Mobile View"/> |

</div>

---

## ✨ Features

- 🎥 **Synchronized Multi-Cam Playback**: Simultaneous playback of Front, Rear, Left, and Right cameras (1:3 / 2x2 / Single Fullscreen layouts)
- 📊 **Real-Time Telemetry HUD**: Visualizes driving data (Speed, FSD status, Turn signals, Steering wheel angle, Accelerator/Brake pedals)
- 🗺️ **GPS Minimap**: Real-time trajectory map synced with vehicle movement via OpenStreetMap
- 🛡️ **AI Privacy Protection**: Automatic detection and blurring of license plates and faces for YouTube/social sharing
- 🔎 **License Plate Enhancement**: Localized brightness, contrast, and sharpening adjustments to reveal indistinct license plates in nighttime or distant footage
- 🔍 **Sentry Motion Detection**: Automatically detects motion event segments during parking and enables 1-click quick jumps
- ✂️ **Multi-Clip Continuous Trimming**: Stitch and trim multiple clips into a seamless, high-quality MP4 export
- 📱 **Wireless Smartphone Transfer (QR)**: Scan the on-screen QR code with your phone camera to download videos directly over Wi-Fi without cables
- 🔒 **EncryptedClips Guidance**: Immediately notifies via toast banner when Tesla's encrypted drive folder is detected
- ☀️ **Night Enhancement & Speed Control**: Brightness and contrast adjustments tailored for dark or tinted footage, plus variable playback speeds (0.5x – 5.0x)

---

## 🚀 How to Use

### Portable Standalone Executable (Recommended)
1. Download `CT_Dashcam_Studio_v1.3.0_Portable.zip` from [Releases](https://github.com/mhsong282828221212123123123/CT-dashcam-studio/releases) and extract it.
2. Launch `CT_Dashcam_Studio.exe`.
3. Click the top-left **[📁 Select TeslaCam Folder]** button and choose the `TeslaCam` folder on your USB drive.
4. *Note: Windows SmartScreen may show an untrusted publisher warning because the binary is not code-signed. This project is 100% open source; you may also run directly from Python if preferred.*

### Running from Python Source
```bash
git clone https://github.com/mhsong282828221212123123123/CT-dashcam-studio.git
cd CT-dashcam-studio
pip install PyQt6 opencv-python numpy Pillow qrcode imageio-ffmpeg
python CT_Dashcam_Studio.py
```

---

## ⌨️ Shortcuts & Mouse Controls

| Action | Shortcut / Mouse Interaction | Description |
| :--- | :--- | :--- |
| **Play / Pause** | `Space` | Toggle playback |
| **Step 1s** | `◀` / `▶` | Seek 1 second backward / forward |
| **Step 5s** | `Shift + ◀` / `Shift + ▶` | Seek 5 seconds fast backward / forward |
| **To Beginning** | `Home` | Jump to 00:00 of the current clip |
| **Prev / Next Clip** | `[` / `]` | Switch between clips |
| **In / Out Trim Points** | `I` / `O` | Mark start and end points for export |
| **Motion Event Jump** | `⚡ Prev` / `⚡ Next` | Jump to previous / next detected motion event |
| **Toggle Single Camera** | `Double-click camera view` | Switch that camera to fullscreen / return to previous layout |
| **Reorder Telemetry** | `Drag bottom telemetry bar` | Drag telemetry items (TIME, SPEED, etc.) left or right to reorder |
| **Reset Telemetry Order** | `Double-click bottom bar` | Reset telemetry layout to default order |
| **Reset Adjustments** | `Double-click slider/label` | Reset speed (1.0x), brightness (0), or contrast (100%) to defaults |
| **Shortcuts Guide** | `F1` or `?` | Open shortcuts and mouse control help dialog |

---

## 📄 License

This project is licensed under the [MIT LICENSE](LICENSE).
