# WP04/05 차량 추적·후보정·첫 3D 프로토타입

2026-11-14. 건물 보정 실험은 중지했다. 최신 실행 기준은 `../../scene-rebuild-plan-20261114/EXECUTION_SCOPE_UPDATE.md`다. 이번 결과는 **선행 차량 한 대의 추정 3D 프로토타입**이며 주변 차량의 카메라 통합·도로 완성으로 보고하지 않는다.

## 실제 구현과 결과

- `tracking.py`: camera-local 최소비용 전역 할당, 높은/낮은 검출 점수의 2단계 연결, 이동/크기/색상 appearance, forward-backward LK 보조 관측. 3D 위치 실패와 무관하게 ID를 유지한다.
- 전체 구간 후처리: 같은 카메라의 모호하지 않은 tracklet을 연결하고 양쪽 관측이 있는 짧은 누락만 채운다. detector / 실제 flow 지지 / 양끝 보간을 별도로 기록한다. 카메라 사이 pixel 보간과 관측 끝의 무제한 외삽은 하지 않는다.
- `offline_paths.py`: 연결된 world anchor의 양끝 위치·속도/차로 근거로 공백을 복원하는 기반과 전체 구간 acceleration fit. 미래 관측이 바뀌면 과거 공백도 다시 계산된다. 같은 road/lane key의 양끝은 같은 차로를 지지한다. 이 world gap 함수는 현재 synthetic 검증이며 실제 cross-camera association 연결은 아직 없다.
- `prototype_front.py`: 지속 관측된 중앙 선행 차량을 자동 선택하고 추정 지면 ray/contact로 배치한다. ego는 자체 SEI signed speed와 heading만 사용한다. 전체 시간의 actor world 관측을 함께 평활화한 뒤 후측 상방 시점에서 사용자 GLB를 렌더링한다.

### 쉬운 pilot

`pilot-offline-tracks-003`: 4채널 700 source frame. 2,087 detector 관측, 53 flow 지지 관측, 35개 raw-box 양끝 보완 sample이다. 후방은 identity 참고이며 3D geometry는 사용하지 않는다. 원본 detector cache를 새로운 2D adapter에서 재사용했고 fresh 3D inference라고 부르지 않는다.

첫 실행에서 우측 SUV가 작은 다른 검출 상자에 붙어 ID가 끊겼다. 급격한 bbox 크기 변화에 대한 연결 비용/거부를 추가하고 재실행했다. 특정 clip/ID별 패치는 없다.

| 선택 차량 | source 존재 reference | 같은 seed ID의 관측/보완 |
|---|---:|---|
| 선행 흰 van | 175 | detector 175 |
| 우측 SUV | 168 | detector 165 + 양끝 보완 3 |

0/15/33초의 두 차량 sparse bbox/identity reference 6개에서 ID가 유지됐고 IoU는 약 0.92~0.98이다. 이것은 선택 차량의 sparse identity와 presence coverage이며 full-scene IDF1·모든 frame identity 정답률이 아니다.

`pilot-tracking-review-002/tracking-review.mp4`: **35초 H.264**, 175 frame 전부 decode 확인. 메인 front와 하단 4채널. 녹색 detector, 청록색 flow, 황색 양끝 보완이다.

### 첫 3D

`pilot-single-lead-3d-002/single-lead-3d.mp4`: **34.8초 / 20fps / 696 frame / H.264**. ego sedan과 선행 van 한 대만 표시한다. 하단 원본 4채널은 비교용이며 side/rear 차량을 융합했다고 주장하지 않는다.

- front 렌즈는 기존 새 엔진의 추정 후보다. 지면 높이 1.3m, pitch 0, front-camera 수평 원점→ego 모델 중심 -1.2m는 명시적인 prototype prior다.
- 원본 bbox 하단 rear-contact를 모델 중심으로 잘못 쓰지 않도록 GLB 길이의 절반을 중심 offset으로 반영했다.
- raw signed speed는 반올림/0 처리하지 않는다. 화면에만 정수 km/h를 표시한다. 정지 중 noisy heading으로 ego가 회전하지 않도록 자기 속도에 따른 heading hold를 사용한다.
- ego trajectory 계산에는 주변 차량 관측을 넣지 않는다. actor world 좌표는 별도다. camera view가 world state를 수정하지 않는다.
- model +Z 앞/+Y 위를 W +X 앞/+Z 위로 변환한다. GLB 원본은 변경하지 않았다. Lambert/opaque material 기반의 단순 렌더링이며 완전한 lamp/ghost material은 아직 아니다.
- 1초 간격 **35개 3D/원본 비교 tile 전부**를 직접 검수했다. 접근→정차→출발, 방향 유지, 화면 밖 이탈 여부를 확인했다. 정밀 거리 검증이나 전체 연속 재생 검수로 확대 해석하지 않는다.
- 696 frame 전체 decode, 175 world 상태의 finite/상대 전방 위치/급격한 위치 이탈·방향 반전 검사를 수행했다. `pilot-single-lead-review-001`에 비교 tile과 숫자 audit가 있다. source 기반 계측 정확도를 뜻하지 않는다.

### 다른 영상 실행

동일 추적 코드/설정으로 August 곡선·교차로 영상도 4채널 700 frame 실행했다(`drive-offline-tracks-001`). 98 tracklet에서 1개 offline 연결, detector 1,274 / flow 21 / 양끝 보완 80 sample이다. 이 구간의 전체 identity GT·3D는 미검증이며 통과로 세지 않는다.

## 남은 범위

WP04/05는 IN_PROGRESS다. 선행 차량 한 대의 납득 가능한 동작을 확인한 첫 prototype이며, 실제 여러 카메라의 global ID/3D 연결, 옆차 world 배치, 차로·교차로·차선 변경, 등화·고스트 재질·UI·독립 날짜 평가는 남아 있다. 다음 작업은 이 차량 관측/오프라인 경로를 바탕으로 주변 차량 연결을 확장하는 것이다. 외벽 정밀 보정으로 돌아가지 않는다.

코드 검사 **68개 통과**. camera assignment, 잘못된 작은 box의 ID 탈취, 실제 flow와 무질감 예측 구분, 두 유사 차량의 모호한 연결 거부, 양끝 공백/차로 유지, 미래 관측에 의한 과거 갱신, whole-interval smoother를 포함한다.

실행(PowerShell, 새 output 경로 필요):

```powershell
$env:PYTHONPATH="$PWD/src"
.venv/Scripts/python -m ct_scene camera-tracks --package runs/pilot-queue-source-002 --detections runs/pilot-raw-masks-001 --output runs/new-tracks
.venv/Scripts/python -m ct_scene single-lead-prototype --package runs/pilot-queue-source-002 --tracks runs/pilot-offline-tracks-003 --telemetry runs/pilot-queue-sei-002 --lens-config configs/multicam-optical-candidates.json --ego-model ../CT-dashcam-studio/assets/scene/tesla_fsd_sedan.glb --actor-model ../CT-dashcam-studio/assets/scene/tesla_fsd_van.glb --output runs/new-single-lead
```
