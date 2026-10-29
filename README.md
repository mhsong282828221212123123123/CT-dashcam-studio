<div align="center">

[![한국어](https://img.shields.io/badge/Language-한국어-0078D7?style=for-the-badge)](#)
[![English](https://img.shields.io/badge/Language-English-gray?style=for-the-badge)](README_EN.md)
[![日本語](https://img.shields.io/badge/Language-日本語-gray?style=for-the-badge)](README_JA.md)
[![简体中文](https://img.shields.io/badge/Language-简体中文-gray?style=for-the-badge)](README_ZH_CN.md)
[![繁體中文](https://img.shields.io/badge/Language-繁體中文-gray?style=for-the-badge)](README_ZH_TW.md)

</div>

# ⚡ 테슬라 블랙박스 뷰어 - CT Dashcam Studio

<div align="center">

![Platform](https://img.shields.io/badge/Platform-Windows-0078D7?style=for-the-badge&logo=windows&logoColor=white)
![Release](https://img.shields.io/badge/Release-v1.3.0-00E6FF?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-39FF14?style=for-the-badge)

**테슬라(Tesla) 순정 블랙박스 및 센트리 모드 영상을 위한 완전 무료 멀티캠 뷰어 & 텔레메트리 스튜디오**

무료 뷰어나 사이트에서 열심히 뽑았더니 워터마크 박히고 개판내서 만들었습니다.

[📥 다운로드 (v1.3.0)](https://github.com/mhsong282828221212123123123/CT-dashcam-studio/releases) • [주요 기능](#-주요-기능) • [스크린샷](#-스크린샷) • [사용 방법](#-사용-방법) • [단축키](#-단축키)

</div>

---

## 📸 스크린샷

<div align="center">

| 2x2 멀티캠 & 텔레메트리 HUD | 센트리 모드 모션 감지 |
| :---: | :---: |
| ![메인 뷰](screenshot1.png) | ![센트리 분석](screenshot2.png) |

| 다중 클립 연속 구간 편집 | AI 번호판 & 안면 블러 |
| :---: | :---: |
| ![구간 편집](screenshot3.png) | ![개인정보 보호](screenshot_privacy.png) |

| PC 원터치 QR 생성 | 스마트폰 다운로드 웹 |
| :---: | :---: |
| <img src="screenshot4.png" width="450" alt="QR 전송"/> | <img src="screenshot5.png" width="220" alt="모바일 뷰"/> |

</div>

---

## ✨ 주요 기능

- 🎥 **멀티캠 동기화 재생**: 전방·후방·좌·우 4채널 동시 재생 (1:3 / 2x2 / 단일 카메라 풀스크린 레이아웃)
- 📊 **실시간 텔레메트리 HUD**: 주행 데이터(속도, FSD 상태, 방향지시등, 조향각, 가속/브레이크 페달) 시각화
- 🗺️ **GPS 미니맵**: 차량 이동 궤적 및 주행 방향 실시간 연동 지도
- 🛡️ **AI 개인정보 보호**: 차량 번호판 및 얼굴 자동 인식 모자이크 (유튜브/커뮤니티 공유용)
- 🔎 **번호판 선명화**: 검출된 번호판의 밝기·대비·선명도를 국소 보정하여 야간/원거리 식별 개선 (번호판 모자이크와 택일)
- 🔍 **센트리 모션 감지**: 주차 중 이벤트 구간 자동 탐지 및 타임라인 퀵 점프
- ✂️ **다중 클립 연속 편집**: 여러 클립을 하나로 이어붙여 원하는 구간만 MP4로 내보내기
- 📱 **스마트폰 QR 무선 전송**: 케이블 없이 화면의 QR 코드 스캔으로 폰에서 즉시 다운로드
- 🔒 **암호화 클립(EncryptedClips) 안내**: 테슬라 보안 암호화 폴더 감지 시 즉시 토스트 알림으로 차량 내 재생 안내
- ☀️ **화면 보정 & 배속**: 야간/틴팅 영상을 위한 밝기·대비 조절 및 가변 배속 재생

---

## 🚀 사용 방법

### 포터블 무설치 실행 (추천)
1. [Releases](https://github.com/mhsong282828221212123123123/CT-dashcam-studio/releases)에서 `CT_Dashcam_Studio_v1.3.0_Portable.zip` 다운로드 후 압축 해제
2. `CT_Dashcam_Studio.exe` 실행
3. 좌측 상단 **[📁 TeslaCam 폴더 지정]** 버튼으로 USB의 `TeslaCam` 폴더 선택
4. 인증서가 없어 경고가 뜰 수 있으나, 본 프로그램은 소스코드가 공개된 프로젝트입니다.
   신뢰하기 어려울 경우 아래 파이썬 환경에서 실행하시면 됩니다.

### 파이썬 환경에서 실행
```bash
git clone https://github.com/mhsong282828221212123123123/CT-dashcam-studio.git
cd CT-dashcam-studio
pip install PyQt6 opencv-python numpy Pillow qrcode imageio-ffmpeg
python CT_Dashcam_Studio.py
```

---

## ⌨️ 단축키 및 마우스 조작

| 기능 | 단축키 / 마우스 조작 | 설명 |
| :--- | :--- | :--- |
| **재생 / 일시정지** | `Space` | 영상 재생/정지 |
| **1초 이동** | `◀` / `▶` | 1초 전/후 탐색 |
| **5초 이동** | `Shift + ◀` / `Shift + ▶` | 5초 전/후 고속 탐색 |
| **처음으로** | `Home` | 현재 클립 0초로 이동 |
| **이전 / 다음 클립** | `[` / `]` | 클립 전환 |
| **구간 시작점 / 끝점** | `I` / `O` | 내보낼 구간 설정 |
| **모션 이벤트 이동** | `⚡ 이전` / `⚡ 다음` | 센트리 감지 구간 점프 |
| **단독 카메라 전환/복귀** | `카메라 화면 더블클릭` | 해당 카메라 단독 풀스크린 전환 및 이전 분할 화면 복귀 |
| **텔레메트리 순서 변경** | `하단 바 마우스 드래그` | 텔레메트리 항목을 좌우로 끌어 표시 순서 변경 |
| **텔레메트리 순서 초기화** | `하단 바 더블클릭` | 텔레메트리 표시 순서를 기본 배치로 초기화 |
| **배속·밝기·대비 초기화** | `슬라이더/라벨 더블클릭` | 배속(1.0x), 밝기(0), 대비(100%) 기본값 즉시 복원 |
| **단축키 안내** | `F1` 또는 `?` | 단축키 및 조작 가이드 팝업 |

---

## 📄 라이선스

본 프로젝트는 [MIT LICENSE](LICENSE)를 따릅니다.
