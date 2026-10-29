<div align="center">

[![한국어](https://img.shields.io/badge/Language-한국어-gray?style=for-the-badge)](README.md)
[![English](https://img.shields.io/badge/Language-English-gray?style=for-the-badge)](README_EN.md)
[![日本語](https://img.shields.io/badge/Language-日本語-0078D7?style=for-the-badge)](#)
[![简体中文](https://img.shields.io/badge/Language-简体中文-gray?style=for-the-badge)](README_ZH_CN.md)
[![繁體中文](https://img.shields.io/badge/Language-繁體中文-gray?style=for-the-badge)](README_ZH_TW.md)

</div>

# ⚡ テスラ ドライブレコーダー ビューア - CT Dashcam Studio

<div align="center">

![Platform](https://img.shields.io/badge/Platform-Windows-0078D7?style=for-the-badge&logo=windows&logoColor=white)
![Release](https://img.shields.io/badge/Release-v1.3.0-00E6FF?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-39FF14?style=for-the-badge)

**テスラ純正ドライブレコーダーおよびセントリーモード映像のための完全無料マルチカムビューア＆テレメトリスタジオ**

既存の無料ビューアやWebサイトの透かし（ウォーターマーク）や画質劣化に不満を感じて開発されたオープンソースソフトです。

[📥 ダウンロード (v1.3.0)](https://github.com/mhsong282828221212123123123/CT-dashcam-studio/releases) • [主な機能](#-主な機能) • [スクリーンショット](#-スクリーンショット) • [使用方法](#-使用方法) • [ショートカット](#-ショートカット)

</div>

---

## 📸 スクリーンショット

<div align="center">

| 2x2 マルチカム & テレメトリ HUD | セントリー動体検知分析 |
| :---: | :---: |
| ![メインビュー](screenshot1.png) | ![セントリー分析](screenshot2.png) |

| 複数クリップ連続トリミング編集 | AI ナンバー＆顔モザイク |
| :---: | :---: |
| ![クリップ編集](screenshot3.png) | ![プライバシー保護](screenshot_privacy.png) |

| ワンタッチ PC QR コード生成 | スマホ直接ダウンロード画面 |
| :---: | :---: |
| <img src="screenshot4.png" width="450" alt="QR転送"/> | <img src="screenshot5.png" width="220" alt="モバイルビュー"/> |

</div>

---

## ✨ 主な機能

- 🎥 **マルチカム同期再生**: フロント・リア・左右の4チャンネルを同時再生（1:3 / 2x2 / 単独フルスクリーン レイアウト）
- 📊 **リアルタイム テレメトリ HUD**: 走行データ（車速、FSD状態、ウインカー、ステアリング操舵角、アクセル/ブレーキ）のHUD可視化
- 🗺️ **GPS ミニマップ**: 車両の走行軌跡と進行方向をリアルタイム連携（OpenStreetMap）
- 🛡️ **AI プライバシー保護**: ナンバープレートおよび顔の自動認識・モザイク処理（SNSやYouTube共有に最適）
- 🔎 **ナンバープレート鮮明化**: 検出されたナンバー領域の明るさ・コントラスト・鮮鋭度を局所補正し、夜間や遠方の視認性を向上
- 🔍 **セントリー動体検知**: 駐車中のイベント区間を自動検知し、タイムラインからワンクリックでスキップ
- ✂️ **複数クリップ連続編集**: 複数の1分クリップを1つのタイムラインに結合し、自由な区間をMP4として書き出し
- 📱 **スマホ QR ワイヤレス転送**: 画面のQRコードをスマホの標準カメラで読み取るだけで、ケーブル不要・同一Wi-Fi経由で即時ダウンロード
- 🔒 **暗号化フォルダ（EncryptedClips）案内**: 暗号化フォルダを検出した場合、トースト通知で車内再生を案内
- ☀️ **映像補正＆変速再生**: スモークフィルムや夜間撮影のための明るさ・コントラスト調整および可変倍速再生（0.5x〜5.0x）

---

## 🚀 使用方法

### ポータブル版の実行（推奨）
1. [Releases](https://github.com/mhsong282828221212123123123/CT-dashcam-studio/releases) から `CT_Dashcam_Studio_v1.3.0_Portable.zip` をダウンロードして解凍します。
2. `CT_Dashcam_Studio.exe` を実行します。
3. 画面左上の **[📁 TeslaCam フォルダ選択]** ボタンをクリックし、USBドライブ内の `TeslaCam` フォルダを選択します。
4. *※ コード署名証明書がないためWindowsの警告画面が表示される場合がありますが、本ソフトはオープンソースプロジェクトです。不安な場合は下記のPython環境から実行してください。*

### Python環境での実行
```bash
git clone https://github.com/mhsong282828221212123123123/CT-dashcam-studio.git
cd CT-dashcam-studio
pip install PyQt6 opencv-python numpy Pillow qrcode imageio-ffmpeg
python CT_Dashcam_Studio.py
```

---

## ⌨️ ショートカット＆マウス操作

| 機能 | ショートカット / マウス操作 | 説明 |
| :--- | :--- | :--- |
| **再生 / 一時停止** | `Space` | 動画の再生/一時停止切り替え |
| **1秒移動** | `◀` / `▶` | 1秒前/後へシーク |
| **5秒移動** | `Shift + ◀` / `Shift + ▶` | 5秒前/後へ高速シーク |
| **先頭へ** | `Home` | 現在のクリップの先頭（0秒）へ移動 |
| **前 / 次のクリップ** | `[` / `]` | クリップの切り替え |
| **区間開始点 / 終了点** | `I` / `O` | 書き出し区間の始点・終点をマーク |
| **動体検知ジャンプ** | `⚡ 前へ` / `⚡ 次へ` | 検知された動体イベント区間へ直接ジャンプ |
| **カメラ単独表示/復帰** | `カメラ画面ダブルクリック` | そのカメラを単独全画面表示、または元の分割表示に復帰 |
| **テレメトリ順序変更** | `下部バーをドラッグ` | テレメトリ項目（TIME, SPEED等）を左右にドラッグして表示順を変更 |
| **テレメトリ初期化** | `下部バーをダブルクリック` | テレメトリ項目の並び順をデフォルト状態にリセット |
| **補正値リセット** | `スライダー/ラベル ダブルクリック` | 倍速(1.0x)、明るさ(0)、コントラスト(100%)を初期値にリセット |
| **操作ヘルプ** | `F1` または `?` | ショートカット及び操作ガイドポップアップを表示 |

---

## 📄 ライセンス

本プロジェクトは [MIT LICENSE](LICENSE) のもとで公開されています。
