# ⚡ CT Dashcam Studio (TeslaCam Multi-Cam & Telemetry Studio)

<div align="center">

![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![PyQt6](https://img.shields.io/badge/GUI-PyQt6-41CD52?style=for-the-badge&logo=qt&logoColor=white)
![FFmpeg](https://img.shields.io/badge/Engine-FFmpeg_libx264-007808?style=for-the-badge&logo=ffmpeg&logoColor=white)
![OpenCV](https://img.shields.io/badge/Vision-OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-Windows-0078D7?style=for-the-badge&logo=windows&logoColor=white)
![Release](https://img.shields.io/badge/Latest_Release-v1.1.0-00E6FF?style=for-the-badge)

**테슬라(Tesla) 순정 블랙박스(TeslaCam / Sentry Mode)의 4채널 멀티캠 영상과<br>H.264 SEI 메타데이터(속도·FSD·스티어링·페달·GPS)를 완벽하게 디코딩하고 시각화하는 고성능 스튜디오입니다.**

[📥 최신 릴리즈 다운로드 (v1.1.0)](https://github.com/mhsong282828221212123123123/CT-dashcam-studio/releases) • [✨ 주요 기능](#-주요-핵심-기능) • [📸 스크린샷](#-스크린샷-미리보기) • [🚀 사용 방법](#-빠른-시작-가이드)

</div>

---

## 📸 스크린샷 미리보기

<div align="center">

### 1. 메인 4채널 동기화 재생 & 실시간 텔레메트리 HUD 대시보드
![메인 4채널 뷰 및 텔레메트리](screenshot1.png)

### 2. 센트리 모드 & AI 모션/충격 이벤트 자동 감지 분석
![센트리 모드 분석](screenshot2.png)

### 3. 실시간 오버레이 & 고효율 libx264 렌더링
![영상 저장 렌더링](screenshot3.png)

### 4. 📱 스마트폰 원터치 QR 무선 고속 전송 (Zero-Cable)
| PC 화면 (원터치 QR 생성) | 스마트폰 화면 (모바일 다운로드 웹) |
| :---: | :---: |
| ![스마트폰 QR 무선 전송](screenshot4.png) | <img src="screenshot5.png" width="280" alt="스마트폰 모바일 뷰"/> |

</div>

---

## ✨ 주요 핵심 기능

### 🎥 1. 완벽한 4채널 멀티캠 동기화 뷰어
- **전방(Main) + 후방(Rear) + 좌/우 리피터(Repeaters)** 영상을 1밀리초의 오차도 없이 완벽한 멀티스레드로 동기화 재생합니다.
- 필러 카메라(PiP) 지원 및 0.5x ~ 5.0x 가변 배속 탐색, 자동 다음 영상 연속 재생 지원.

### 📊 2. 테슬라 순정 H.264 SEI 텔레메트리 실시간 디코딩
- 별도의 GPS/OBD 장비 없이, 테슬라 순정 비디오 스트림 내부의 **SEI(Supplemental Enhancement Information) Protobuf** 바이너리 데이터를 직접 파싱합니다.
  - ⚡ **실제 주행 속도 (km/h)**
  - 🤖 **자율주행 모드 (MANUAL / AUTOPILOT / FSD ACTIVE / TACC)** 및 **FSD 주행 프로파일 (Chill / Standard / Assertive / Hurry)**
  - 🔄 **스티어링 휠 조향 각도 (Steering Angle deg)**
  - 🚦 **방향지시등 좌/우 점멸 상태 (Blinkers)**
  - 🦶 **가속 페달(ACC %) 및 브레이크(BRK) 작동 상태**
  - 🕒 **테슬라 시스템 밀리초 단위 정확한 타임스탬프**

### 🗺️ 3. 테슬라 벡터 마커가 탑재된 정밀 GPS 미니맵
- 비디오에 기록된 GPS 위도/경도/방위각(Heading)을 OpenStreetMap(OSM) 타일과 실시간 결합.
- 차량의 실제 이동 궤적 및 주행 방향에 따라 **실시간으로 부드럽게 회전하는 테슬라 전용 벡터 아이콘**을 표시합니다.

### 🔍 4. 센트리 모드(Sentry Mode) AI 모션 분석기
- 주차 중 발생한 수십 분의 영상 중 **물체 접근, 문콕 등 움직임이 발생한 구간을 프레임 차분 알고리즘으로 자동 탐지**.
- 타임라인 컨트롤룸에 **빨간색 이벤트 블록**으로 하이라이트 표기하며, `[이전 이벤트] / [다음 이벤트]` 원클릭 퀵 점프로 즉시 확인할 수 있습니다.

### ☀️ 5. 실시간 밝기 & 대비(Contrast) 보정 게이지
- 어두운 틴팅(썬팅)이나 야간 주행 영상도 직관적인 슬라이더 조절로 환하고 선명하게 보정.
- 일시정지 상태에서도 화면 프레임이 튀지 않고 프리뷰와 렌더링에 즉각 동기화 반영됩니다.

### 🎬 6. FFmpeg libx264 고효율 렌더링 엔진
- 용량만 크고 화질이 떨어지는 기존 윈도우 코덱(`avc1`)을 걷어내고, **고효율 `libx264` 인코딩 파이프라인으로 전면 개편**.
- 화질 손실 없이 파일 용량을 10분의 1 수준으로 극적 압축하며, 렌더링 전 **실제 파일 크기(MB)를 소수점 단위로 정확하게 예측**합니다.
- **해상도 프리셋**: QHD (2560x1440), FHD (1920x1080), HD (1280x720), 모바일용 Compact (960x540).

### 📱 7. 스마트폰 원터치 QR 무선 고속 전송 (Zero-Cable)
- 렌더링된 결과물이나 블랙박스 원본 클립을 스마트폰으로 옮기기 위해 USB를 뽑을 필요가 없습니다.
- 로컬 Wi-Fi 내장 초경량 스트리밍 서버를 구동하여, **스마트폰 기본 카메라로 화면의 QR 코드만 스캔하면 즉시 아이폰/안드로이드 갤러리로 다운로드**됩니다.

### 🔔 8. 작업표시줄 번쩍임 & 시스템 트레이 토스트 알림
- 장시간 렌더링 중 다른 작업을 하고 있어도, 완료 시 **작업표시줄 아이콘 번쩍임** 및 **윈도우 우측 하단 토스트 팝업 알림**으로 작업 완료를 즉시 알려줍니다.

### 🚀 9. GitHub 릴리즈 연동 스마트 업데이트 알림
- 프로그램 실행 시 백그라운드에서 최신 릴리즈 버전을 확인하고, 새 버전 출시 시 좌측 상단에 눈에 띄는 **신규 버전 다운로드 링크 버튼**을 활성화합니다.

---

## 🚀 빠른 시작 가이드

### 방법 1. 포터블 무설치 버전 사용 (가장 추천 ⭐)
> 파이썬이나 외부 코덱 설치 없이 압축만 풀면 즉시 실행할 수 있는 독립형 패키지입니다.

1. [Releases 페이지](https://github.com/mhsong282828221212123123123/CT-dashcam-studio/releases)에서 최신 `v1.1.0` 압축 파일(`CT_Dashcam_Studio_v1.1.0_Portable.zip`) 또는 `CT_Dashcam_Studio.exe`를 다운로드합니다.
2. 다운로드한 파일의 압축을 풀고 `CT_Dashcam_Studio.exe`를 실행합니다.
3. 좌측 상단 **[📁 테슬라 폴더 지정 (TeslaCam)]** 버튼을 눌러 USB의 `TeslaCam` 폴더를 선택하면 모든 영상이 즉시 로드됩니다.

---

### 방법 2. 파이썬 소스코드에서 실행

```bash
# 1. 저장소 클론
git clone https://github.com/mhsong282828221212123123123/CT-dashcam-studio.git
cd CT-dashcam-studio

# 2. 필수 라이브러리 설치
pip install PyQt6 opencv-python numpy Pillow qrcode imageio-ffmpeg

# 3. 애플리케이션 실행
python CT_Dashcam_Studio.py
```

---

## ⌨️ 단축키 및 컨트롤 가이드

| 기능 | 조작 | 설명 |
| :--- | :--- | :--- |
| **재생 / 일시정지** | `Spacebar` / `▶ 재생` | 현재 클립을 실시간으로 재생하거나 일시정지 |
| **구간 시작점 지정** | `[ 시작점` | 현재 프레임을 내보내기 시작 위치로 설정 |
| **구간 끝점 지정** | `] 끝점` | 현재 프레임을 내보내기 종료 위치로 설정 |
| **구간 초기화** | `↺ 구간 초기화` | 설정된 시작/끝 지점을 초기화하여 전체 클립으로 복구 |
| **배속 조절** | `배속 슬라이더` | 0.5x ~ 5.0x 까지 부드러운 가변 배속 탐색 |
| **밝기 / 대비 조절** | `☀️ / ◑ 슬라이더` | 야간/틴팅 영상의 노출 및 명암 대비 실시간 보정 |
| **모션 분석 점프** | `◀ 이전 이벤트 / 다음 이벤트 ▶` | 센트리 감지된 충격/모션 구간으로 1초 만에 이동 |
| **스마트폰 QR 공유** | `📱 스마트폰 무선 전송` | 모바일 Wi-Fi 다운로드용 QR 모달 팝업 열기 |

---

## 🛠️ 기술 스택 & 아키텍처
 
- **Binary Compiler**: Nuitka C++ Native Compiler (GCC MinGW-w64, Zstandard Compression)
- **GUI Framework**: PyQt6 (Fusion Dark Theme & Custom Cyber-Dashboard UI)
- **Computer Vision**: OpenCV (OpenCL 가속 활성화, Frame Difference Motion Detector)
- **Video Codec**: FFmpeg `libx264` (Raw Video Pipe Stream, CRF Rate Control, YUV420p)
- **Telemetry Parser**: Tesla H.264 NAL SEI Binary Parser & Protocol Buffers Wire Type Engine
- **Map System**: OpenStreetMap XYZ Tile Caching Engine with Vector Vehicle Transformation
- **Wireless Share**: Python Native Threading HTTP Server & Responsive Mobile Web Interface

---

## 📄 라이선스 (License)

본 프로젝트는 [LICENSE](LICENSE)에 명시된 오픈소스 라이선스를 따릅니다.

---

<div align="center">
  <sub>Developed with ❤️ for Tesla & Cybertruck Owners worldwide.</sub>
</div>
