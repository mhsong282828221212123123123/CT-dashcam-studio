3D 재현 본체 인계: [편집기 연결·실행·파이프라인·소스 지도](docs/GEMINI_3D_MODULE_GUIDE.md)를 먼저 읽으세요. 다중 카메라 3D 엔진은 이미 구현되어 있으며 접점 학습은 그 보조 실험입니다.

다른 AI 인계: [실행·정답 작성·모델 학습 안내](docs/GEMINI_TRAINING_GUIDE.md). 현재 학습 v2는 새 날짜 접점 검증 실패로 기본3D에 미적용입니다.

# CT Scene Rebuild

최신: [영상 목록](docs/LATEST_RESULTS.md) · [입력 검사·장면 재렌더 작업](docs/WORKFLOW_EXECUTION_20261115.md). 화면 설정으로 확대/도로 밝기만 바꿀 때는 저장된 좌표를 재사용하며 추론하지 않습니다.

에디터 내부 탭으로 연결됨: [연결 검사](../CT-dashcam-studio/SCENE_EDITOR_INTEGRATION.md).

사용: **실험실 / Lab → 주행 3D 재현 / 분석·재생·MP4 → 최근 완료 결과 → 선택 결과 열기**. 전체 영상 저장은 **MP4 내보내기**. [사용 연결 검증](docs/LAB_WORKFLOW_20261115.md).

현재 실행/검증: [EXECUTION_20261115.md](docs/EXECUTION_20261115.md). 독립 창은 `Launch-Scene-v2.ps1`, 기존 편집기는 **실험실 / Lab → 주행 3D 재현 / 분석·재생·MP4**. 전체 품질/holdout은 진행 중이다.

테스트 영상 전달: [VIDEO_DELIVERY.md](docs/VIDEO_DELIVERY.md). Google Drive 업로드 대상은 **테스트26** (`19Q8hPBlHfhKbCJdIuGColfhl_SCECX30`). 로컬 재생 영상과 Drive 링크를 함께 제공한다.

독립 재개발 엔진. 기존 CT-dashcam-studio와 원본 영상은 읽기 전용 참고 자료다.

계획서 다음 단계: `docs/WP06_WP07_PROGRESS.md`. 양방향 최우선/배타적 identity 선택, pre-road-prior 연결 검사, 관측/위치/고스트 상태 분리, ego 차선 변경 시작·통과·차체 완료 기록. 두 풀버전 `runs/wp06-wp07-oct21-004`, `runs/wp06-wp07-sep17-004` 및 초기 Palisade 회귀. 코드 104개 통과. WP06/07 전체/OSM은 미완료다.

다른 날짜 풀버전: `docs/SEPTEMBER_FULL_REVIEW.md` / `runs/sep17-full-clay-003/surrounding-world.mp4` — 9월 17일 주행 59.1초, 1,182 frame. 탑차/버스 asset, right paint edge 기반 corridor fallback, 좌측에도 apparent-height 적용. 코드 89개 통과. 전체 road topology/identity 합격은 아니다.

우측 누락 수정 후속: `docs/RIGHT_REPEATER_FIX.md`, `runs/oct21-right-fixed-006/surrounding-world.mp4` (59.9초). 우측 apparent-height 중심 추정, side 경계 오차 처리, family 중복 억제, 후측 화면 범위 확대. 서로 다른 K5는 분리하며 큰 offset으로 co-visible 차를 맞추는 실험은 폐기했다. 코드 86개 통과.

원본 전체 출력: `docs/FULL_SOURCE_REVIEW.md` — `runs/oct21-full-clay-005/surrounding-world.mp4`, 59.9초/1,198 frame. 선행차 ID 종료와 독립적인 source 전체 시간축, paint 기반 도로 방향 보정, 실제 좌측→전방 벤츠 연결. 코드 80개 통과. 도로 topology는 아직 단순 corridor다.

두 번째 날짜 결과/검수: `docs/SECOND_SCENE_REVIEW.md`. 새 10월 21일 영상의 33초 클레이 3D + 4채널: `runs/oct21-traffic-clay-003/surrounding-world.mp4`. 왼쪽 리피터 경계 보완·접지점 불명 조각 제외·허위 교차로 조건 수정, 코드 75개 통과. 곡선/교차로 일반화 완료는 아니다.

최신 실행/검수: `docs/SURROUNDING_WORLD_PROTOTYPE.md`. 주변 차량 5대·실제 front→repeater 연결·고스트 공백·차로 추정을 포함하는 40.8초 결과: `runs/pilot-extended-surrounding-004/surrounding-world.mp4`. `surrounding-prototype` CLI로 재실행한다. 교차로 완성·후방 연결·여러 영상 품질 통과는 미완료다.

이전 단일 차량 단계: `docs/OFFLINE_TRACKING_PROTOTYPE.md`.
`camera-tracks`, `tracking-review`, `single-lead-prototype`으로 추적 검토와 단일 차량 3D를 실행한다.
첫 34.8초 3D 결과: `runs/pilot-single-lead-3d-002/single-lead-3d.mp4`.
기존 scene 모듈 import, legacy pickle 입력, source ID별 위치 패치는 금지한다.
CUDA·빌드·릴리즈·푸시는 하지 않는다.

계획: ../scene-rebuild-plan-20261114/README.md

개발 실행(PowerShell, 이 폴더에서):

```powershell
$env:PYTHONPATH = "$PWD/src"
.venv/Scripts/python -m ct_scene --help
.venv/Scripts/python -m unittest discover -s tests -v
```

현재는 artifact 무결성·시간·좌표 기초 구현 단계다. 인식/3D 재현 품질 합격이 아니다.

격리 환경은 다운로드한 wheel hash로 고정했다. 기본 입력·검증 환경은
`requirements/windows-bootstrap.lock`, CUDA 없는 추론 포함 환경은 `requirements/windows-directml.lock`이다.

현재 CLI: `inventory`, `audit-pts`, `audit-sei`, `prepare-annotations`,
`validate-annotations`, `verify-artifact`.

추가 CLI: `propose-labels`, `raw-masks`, `static-features`, `motion-candidates`.
상세 진행/실패/적용 범위는 `docs/BACKGROUND_MOTION_REVIEW.md`를 확인한다.
추론 환경은 bootstrap lock 대신 `requirements/windows-directml.lock`을 사용한다.

CPU SfM/광학 검증 환경: `requirements/windows-calibration.lock`.
추가 CLI: `optical-sfm`, `wheel-scale-check`.
최신 검수: `docs/FRONT_OPTICAL_VALIDATION.md`.

공통 카메라 연결 실험: `multicam-connectivity`, `affine-connectivity-probe`.
현재 연결 gate FAIL. 결과와 재현 명령은 `docs/MULTICAMERA_CONNECTIVITY.md`.
수동 기준점·상단 ROI 원인 분리: `docs/MANUAL_LANDMARK_DIAGNOSIS.md`.
개발 진단 CLI: `manual-planar-check`, `planar-warp-probe`.
다중 시점·plane·시간 전이 진단: `manual-multiview-probe`, `plane-pose-probe`, `side-transfer-probe`.
수동 reference 001의 오류 교정과 최신 한계는 `docs/MULTIVIEW_GEOMETRY_REVIEW.md`를 먼저 확인한다.

원본 정답 도구: `runs/pilot-queue-source-001/annotate.html`을 브라우저에서 연다.
원본 700프레임만 포함하며 예측 결과를 읽지 않는다. 박스·global ID를 입력하고
프레임 전체 확인을 눌러 JSON으로 저장한다. 저장한 결과는 `validate-annotations`로 검사한다.
프레임 전체 정답 작성과 브라우저 조작 검수는 아직 미완료다.
