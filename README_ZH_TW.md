<div align="center">

[![한국어](https://img.shields.io/badge/Language-한국어-gray?style=for-the-badge)](README.md)
[![English](https://img.shields.io/badge/Language-English-gray?style=for-the-badge)](README_EN.md)
[![日本語](https://img.shields.io/badge/Language-日本語-gray?style=for-the-badge)](README_JA.md)
[![简体中文](https://img.shields.io/badge/Language-简体中文-gray?style=for-the-badge)](README_ZH_CN.md)
[![繁體中文](https://img.shields.io/badge/Language-繁體中文-0078D7?style=for-the-badge)](#)

</div>

# ⚡ 特斯拉行車記錄器播放器 - CT Dashcam Studio

<div align="center">

![Platform](https://img.shields.io/badge/Platform-Windows-0078D7?style=for-the-badge&logo=windows&logoColor=white)
![Release](https://img.shields.io/badge/Release-v1.3.0-00E6FF?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-39FF14?style=for-the-badge)

**專為特斯拉原廠行車記錄器與哨兵模式影片打造的完全免費多鏡頭播放器與行車數據分析工作台**

厭倦了現有免費工具強制加註浮水印或壓縮畫質？CT Dashcam Studio 專為車主打造，提供無損畫質與極致流暢的操作體驗。

[📥 下載 (v1.3.0)](https://github.com/mhsong282828221212123123123/CT-dashcam-studio/releases) • [主要功能](#-主要功能) • [軟體截圖](#-軟體截圖) • [使用方法](#-使用方法) • [快捷鍵說明](#-快捷鍵說明)

</div>

---

## 📸 軟體截圖

<div align="center">

| 2x2 多鏡頭 & 即時 HUD 行車數據 | 哨兵模式動態偵測分析 |
| :---: | :---: |
| ![主視圖](screenshot1.png) | ![哨兵分析](screenshot2.png) |

| 多片段連續時間軸剪輯 | AI 車牌與人臉隱私遮蔽 |
| :---: | :---: |
| ![時間軸裁剪](screenshot3.png) | ![隱私保護](screenshot_privacy.png) |

| PC 端一鍵產生無線 QR 碼 | 手機掃碼直存相簿 |
| :---: | :---: |
| <img src="screenshot4.png" width="450" alt="QR傳輸"/> | <img src="screenshot5.png" width="220" alt="手機視圖"/> |

</div>

---

## ✨ 主要功能

- 🎥 **多鏡頭同步播放**: 前方、後方、左側、右側 4 路鏡頭毫秒級同步播放（支援 1:3、2x2、單鏡頭全螢幕切換）
- 📊 **即時 HUD 數據儀表**: 行車數據（時速、FSD自動輔助駕駛狀態、方向燈、方向盤轉角、電門與煞車踏板）即時疊印於畫面
- 🗺️ **GPS 迷你地圖**: 結合 OpenStreetMap 即時連動車輛行駛軌跡與行進方向
- 🛡️ **AI 隱私保護遮蔽**: 自動偵測車牌與人臉並加上動態馬賽克（便於社群論壇與 YouTube 分享）
- 🔎 **車牌清晰化增強**: 針對模糊車牌區域局部優化亮度、對比度與銳利度，提升夜間與遠距離車牌辨識度
- 🔍 **哨兵模式動態偵測**: 自動掃描並標記停車期間可疑動態事件，支援時間軸一鍵快速跳轉
- ✂️ **多片段無縫連續剪輯**: 將多個 1 分鐘短影片拼接於同一時間軸，自由裁剪並匯出無損 MP4
- 📱 **手機無線免線傳輸 (QR)**: 匯出後自動產生 QR Code，手機內建相機掃碼即可透過 Wi-Fi 直接儲存至手機相簿
- 🔒 **加密影片識別 (EncryptedClips)**: 自動偵測特斯拉加密資料夾並彈出橫幅提醒，指引車內螢幕播放方式
- ☀️ **畫面亮度補償與倍速**: 針對隔熱紙或夜間場景微調畫面亮度與對比度，支援 0.5x ~ 5.0x 變速播放

---

## 🚀 使用方法

### 可攜式免安裝版（推薦）
1. 至 [Releases](https://github.com/mhsong282828221212123123123/CT-dashcam-studio/releases) 下載 `CT_Dashcam_Studio_v1.3.0_Portable.zip` 並解壓縮。
2. 點擊執行 `CT_Dashcam_Studio.exe`。
3. 點選左上角 **[📁 指定 TeslaCam 資料夾]** 按鈕，選擇隨身碟中的 `TeslaCam` 目錄。
4. *註：因未購買微軟數位簽章憑證，Windows 可能會彈出防護提醒，本專案為完全開源專案。若有疑慮亦可直接透過 Python 原始碼執行。*

### 透過 Python 原始碼執行
```bash
git clone https://github.com/mhsong282828221212123123123/CT-dashcam-studio.git
cd CT-dashcam-studio
pip install PyQt6 opencv-python numpy Pillow qrcode imageio-ffmpeg
python CT_Dashcam_Studio.py
```

---

## ⌨️ 快捷鍵與滑鼠操作說明

| 功能 | 快捷鍵 / 滑鼠操作 | 詳細說明 |
| :--- | :--- | :--- |
| **播放 / 暫停** | `Space` | 切換影片播放或暫停 |
| **微調 1 秒** | `◀` / `▶` | 倒退 / 前進 1 秒 |
| **快轉 5 秒** | `Shift + ◀` / `Shift + ▶` | 快速倒退 / 前進 5 秒 |
| **回到開頭** | `Home` | 回到目前片段起始點 (0秒) |
| **切換片段** | `[` / `]` | 快速切換至上一個 / 下一個片段 |
| **標記剪輯起點/終點** | `I` / `O` | 標記匯出選取的起點 (In) 與終點 (Out) |
| **動態事件跳轉** | `⚡ 上一個` / `⚡ 下一個` | 快速跳轉至下一個哨兵動態偵測位置 |
| **單鏡頭全螢幕/復原** | `雙擊對應鏡頭畫面` | 切換該視角全螢幕放大，再次雙擊還原多鏡頭分割佈局 |
| **調整數據項順序** | `拖曳底部數據欄` | 左右拖曳時速、時間等數據項自訂排列順序 |
| **重設數據項順序** | `雙擊底部數據欄` | 恢復 HUD 數據項的預設排列順序 |
| **重設畫面與倍速** | `雙擊對應滑桿或標籤` | 快速將倍速(1.0x)、亮度(0)、對比度(100%)重設為預設值 |
| **快捷鍵指南** | `F1` 或 `?` | 開啟快捷鍵與操作說明視窗 |

---

## 📄 開源授權

本專案採用 [MIT LICENSE](LICENSE) 授權。
