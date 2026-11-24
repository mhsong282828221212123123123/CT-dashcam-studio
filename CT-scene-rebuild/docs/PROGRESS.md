# 초기 구현 기록

**WP06/07 후속:** [WP06_WP07_PROGRESS.md](WP06_WP07_PROGRESS.md). 두 풀버전의 ID 선택 배타성/관측·위치 분리와 ego 차로 사건을 구현했다. 기존 벤츠/초기 Palisade positive와 다른 차 연결 거부를 확인했다. 차선 변경 차체 완료 October 34.41초, September 19.81초는 raw 사건 window와 일치하고 이후 유지된다. 관측 중 ghost 충돌은 60→0(해당 tracked ID 범위)이며 미확정 위치는 별도로 남긴다. 코드 104개. Rear identity·주변 차 lane events·독립 road correction·곡선/교차로/OSM은 아직 남았다.

**두 번째 날짜 후속:** [SECOND_SCENE_REVIEW.md](SECOND_SCENE_REVIEW.md). October 21 새 원본 4채널 700 frame DirectML → 33초 3D/4채널, 시점별 6~9대. raw 비교에서 발견한 좌측 차로 누락/좁은 가림 조각 겹침/정차 기반 허위 교차로를 수정했다. 660 frame decode/8개 시각 비교, 기존 ego-independent lead와 실제 SUV 인계 회귀, 코드 75개 통과. 일부 가림 차량/후방/곡선/교차로/전체 GT는 남아 있다.

**최신:** [SURROUNDING_WORLD_PROTOTYPE.md](SURROUNDING_WORLD_PROTOTYPE.md). 40.8초 주변 차량 5대 3D·하단 4채널·실제 front→right 인계·양끝 고스트·비균일 차로 prototype 구현/원본 비교. 816 frame decode, 이전 ego prefix 변경 0m, frozen lead 정확 일치, legacy 608개 SHA 일치. 코드 73개 통과. 전체 교차로/후방 identity/여러 날짜 3D는 아직 미완료다. 아래 기록은 각 과거 시점의 상태다.

**현재 우선 작업:** [OFFLINE_TRACKING_PROTOTYPE.md](OFFLINE_TRACKING_PROTOTYPE.md). 건물 보정 실험 중지. 카메라별 ID/flow·양끝 누락 보완을 구현하고, SEI 독립 ego와 전체 시간 actor 후보정으로 첫 34.8초 단일 선행차 GLB 3D + 하단 4채널 영상을 생성·대조했다. 실제 cross-camera global ID·주변 차량 3D·도로는 다음 범위이며 미완료다. 코드 검사 68개 통과.

최신: [MULTIVIEW_GEOMETRY_REVIEW.md](MULTIVIEW_GEOMETRY_REVIEW.md). 수동 reference 001 오류를 원본 확대에서 발견해 002로 교정하고, 잘못된 reference의 수동 품질 판단을 무효화했다. 다중 시점 삼각측량은 아직 FAIL. 전방만으로 계산한 외벽 plane과 side focal 후보는 제한된 분리점 재투영 검사를 통과했으나 앞뒤 145 decoded frame의 유효 coverage가 짧아 optical/rig 프로필로 승인하지 않았다. 총 58개 코드 검사 통과, WP03 IN_PROGRESS다.

최신 검토: [MANUAL_LANDMARK_DIAGNOSIS.md](MANUAL_LANDMARK_DIAGNOSIS.md). 수동 정적 대응 14점으로 상단 ROI 제외 문제를 확인하고 affine 실험 경로를 수정했다. 동일 조건 자동 매칭 대조(17→18점)는 연결 해결 수준이 아니며 평면 시점 보완도 채택하지 않았다. 수동 평면 6점 fit/8점 분리 검사, 총 51개 테스트 통과. 공통 rig와 3D 재현 gate는 미통과다.

최신 후속: [MULTICAMERA_CONNECTIVITY.md](MULTICAMERA_CONNECTIVITY.md). 전방·좌우 리피터의 공통 static map 실험을 구현하고 실제 원본으로 실행했다. side끼리 연결됐으나 front 연결 gate는 FAIL이다. 공통 rig/clock/body pose로 승격하지 않았다. 현재 회귀 검사 46개 통과는 코드 검증이며 3D 품질 통과가 아니다.

최신: [FRONT_OPTICAL_VALIDATION.md](FRONT_OPTICAL_VALIDATION.md). 첫 pilot 전방 학습 66프레임 연결, 제외 4프레임 raw 재투영 p95 1.10~1.83px, 별도 출발 구간의 wheel/visual 이동량 일관성 검사까지 완료했다. 전방 광학 개발 실험만 통과했으며 common rig/body 좌표/metric 정확도/차량 재현은 미검증이다.

최신 구현: [BACKGROUND_MOTION_REVIEW.md](BACKGROUND_MOTION_REVIEW.md). 원본 350칸 검수, 선택 객체 존재 reference 343개, 두 날짜 1,400프레임 DirectML raw mask, 정적 배경 대응과 조건부 두 시점 운동 실험까지 진행했다. 전체 GT·공통 rig·metric ego·3D 재현은 아직 합격하지 않았다.

추가 후속: [PILOT_REFERENCE_REVIEW.md](PILOT_REFERENCE_REVIEW.md). 실제 브라우저 입력/저장을 검사했고 UI 결함과 정지 시 미세 음수 속도 처리 결함을 수정했다. 전 구간 GT와 metric 재현 검수는 아직 미완료다.

최신 후속 상태: [INPUT_REVIEW.md](INPUT_REVIEW.md). 아래 초기 기록 이후 격리 환경/해시 lock, artifact JSON Schema, raw-only annotation 도구, 새 SEI parser, inventory, synthetic distortion/crop roundtrip을 추가했다. 총 22개 테스트가 격리 환경에서 통과했다. WP01/WP02는 아직 IN_PROGRESS이며 annotation 전수 작성과 실제 rig/ego 검증은 완료되지 않았다.

## 완료

- WP00: 608개 보관 member의 현재 원본 SHA-256, ZIP SHA-256/CRC 재검증. 기존 코드 변경 없음.
- 새 독립 루트와 패키지/CLI 생성. 기존 core/ui/pickle import 금지 검사.
- 결과 manifest/payload 해시 검증, 경로 탈출 방지, 새 결과만 atomic publication, 취소 시 임시 결과 제거.
- W/B/C 축·SE3 합성/역변환·pinhole projection·near plane·clock offset/epoch 규약 구현.
- 원본 channel discovery와 실제 decoded frame PTS index, nearest PTS 선택 구현.
- 기초/입력 테스트 13개 통과. 테스트 통과는 재현 품질 판정이 아님.
- 새 .venv 생성. 현재 초기 실행은 bootstrap host interpreter를 사용했고, 의존성 lock은 아직 완료하지 않음.

## 실제 원본 검사

source: E:\tesla\a3\TeslaCam\RecentClips\2026-08-15_17-39-32-front.mp4 와 3개 sibling.
35초 구간의 총 5,044개 frame PTS를 실제 decode로 확인했다.
카메라별 nearest sampling p95: front 17.1ms, back 14.2ms, left 14.6ms, right 15.5ms.
max는 각각 32.9/28.9/29.9/30.7ms다. 임계값 합격 판정은 아직 하지 않았다.

이 숫자는 해당 카메라의 container PTS 선택 오차다. 각 카메라 시작 시각을 같은 노출 시각으로 간주하지 않았고 session_time은 null/unverified다. 센서 동기와 실제 차량 위치 정확도를 입증하지 않는다. 이 source는 legacy regression이며 새 holdout이 아니다.

결과: runs/timing-audit-001. artifact 무결성 PASS, quality_gate NOT_EVALUATED.

## 다음 필수 작업

1. WP01 JSON Schema/typed state, crop/rectification roundtrip, renderer spike, 격리 환경 lock.
2. WP02 전체 원본 inventory/노출 이력, raw-only annotation 도구, pilot 선정·정답 작성.
3. WP02 새 SEI parser와 validity 검증. 기존 decoder import하지 않음.
4. 위 입력 증거를 고정한 뒤 WP03 optical/rig/static ego 검증.

현재 인식·추적·도로·3D 재현 엔진은 미구현이다. 다음 gate를 통과하기 전 기존 결과보다 좋아졌다고 보고하지 않는다.
