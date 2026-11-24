# CT Dashcam Studio 주행 재현 인계

작성 기준: 2026-11-12, 현재 로컬 작업 트리. **실험용 프로토타입이며 상용 품질 미달**이다. 교차로 구조와 횡단 차량 방향이 실패한다. 단위 테스트 통과를 영상 재현 정확도로 해석하면 안 된다. 사용자는 비용 대비 결과에 불만이 있어 개발을 중단하고 다른 모델의 검토를 요청했다. 이 문서는 추가 개발을 자동 승인하는 지시가 아니라 현황과 참고 자료다.

## 1. 경로와 제약

- 저장소: `C:\Users\ddno2\Documents\ChatGPT\테슬라\CT-dashcam-studio`
- 분석 결과/영상: `C:\Users\ddno2\Documents\ChatGPT\테슬라\scene-results`
- 모델: `C:\Users\ddno2\Documents\ChatGPT\테슬라\scene-models`
- 격리 DirectML 런타임: `C:\Users\ddno2\Documents\ChatGPT\테슬라\scene-runtime-directml`
- 원본: `E:\tesla\a2` ~ `a6` 하위. 차량은 2020 Model 3 LR HW3, 주로 front/back/left_repeater/right_repeater 4채널.
- Gemini 제작 3D 원본: `E:\tesla\project\fsd-sedan-3d` (GLB/고스트, 프리뷰, 생성 스크립트). 앱용 복사본은 저장소 `assets/scene`.
- CUDA 제외. CPU 또는 ONNX Runtime DirectML. 원본 영상 수정 금지. 빌드·릴리즈·푸시 금지. 기존 사용자 작업을 되돌리거나 폴더를 정리하며 삭제하지 말 것.
- Git에 scene 관련 파일 대부분이 **untracked**다. 커밋/원격만 받아서는 현재 결과를 재현할 수 없다. 로컬 전체 작업 트리와 저장소 옆 모델/분석 폴더가 필요하다. 기존 `ui/main_window.py` 수정도 함께 존재한다.

## 2. 사용자가 원하는 것 / 잘못 이해하면 안 되는 부분

- 밝은 Tesla식 화면, 자차 후상방 약 45도 고정 시점. 사고 블랙박스 재현 용도의 후처리다.
- 전방과 양 리피터를 함께 사용. 후방은 왜곡 때문에 위치 결정의 주역이 아니라 연관 확인용.
- 잠깐 가려진 기존 차량은 시간축으로 추정하며 고스트 모델로 표현.
- **차량이 주행하는 공간을 차로의 근거로 삼고, 차로를 포함하는 연속 도로를 구성**한다. 픽셀로 검출한 도로 조각만 겹쳐 그리는 접근을 사용자는 거부했다.
- 단순화 표시는 바깥 실선 / 내부 점선. 벽은 미관측 영역에 임의 생성하면 안 된다.
- OSM은 도로 큰 형태 참고용. 세부 차로는 영상으로 판단. 공개 OSM 서버에 촬영 지역 전송은 이전에 허용받았으나 영상 업로드까지 의미하지 않는다.
- 초기 도로 버전의 사용자 지적은 **자차 차선 변경을 선행차 이동으로 오인해 우측 허위 차로를 추가한 것**이었다. 이 문제를 고치면서 초기 기반 전체를 제거한 것이 잘못된 방향 전환이었다.
- **자차 위치를 주변 차량 상대 움직임으로 정하지 말 것.** 차량으로 차로 존재를 추론하는 것과, 차량으로 자차 위치를 정하는 것은 다르다.

## 3. 현재 처리 흐름과 핵심 파일

1. `core/scene_analysis.py`: CameraReader, YOLO ONNX 검출, 카메라별 영상 추적.
2. `core/scene_inference.py`: CPU/DirectML 실행과 CPU fallback. `requirements-scene-directml.txt`는 1.23.0 격리 설치용. 입력 크기는 모델과 맞아야 한다.
3. `core/scene_pipeline.py`: 동명 타임스탬프 채널 분석, `manifest.json`과 `observations.jsonl` 저장. `core/scene_features.py`는 차선/신호 픽셀 관측.
4. `core/scene_fusion.py`: `FusedSceneTimeline`, 추정 카메라 기하, 평면 접지 차량 형상 적합, 다중 카메라 연관, 시간축 후보정 호출.
5. `core/scene_world_tracks.py`: SEI 속도/heading 기반 상대 오도메트리와 월드 추적, 짧은 공백 예측. 좌표 +X=좌, +Z=전방, 렌더 +Y=위. 실측 캘리브레이션이 아니다.
6. `core/scene_postprocess.py`, `scene_handoff.py`: 양방향 복구/보수적 카메라 전환 연결. 고스트/보간은 관측과 구분한다.
7. **현재 기본 도로** `core/scene_vehicle_road.py`: `scene_road.add_travel_corridors`가 호출. 아래 설명 참조.
8. `core/driving_scene.py`: GLB 소프트웨어 렌더, 도로 면/선, 고스트, `road_display_scene`의 공통 표시 변환.
9. `ui/scene_dialog.py`: Scene Lab UI. `tools/scene_lab.py`로 독립 실행 가능.

### 현재 도로와 자차 위치

- 차로 폭 **3.5m 가정**, 최초 자차 차로 중심이 원점. 차로 중심 근처의 지속 관측(최소 5개/0.8초)으로 점유 차로 번호를 구성한다. 순간 오배치와 고스트는 새 차로 근거에서 제외한다.
- 연속 구간의 전체 관측을 사용하는 후처리다. 가려졌다고 도로를 삭제하지 않는다. 오도메트리 단절 시 구간을 나눈다.
- `_paint_frame`: 가까운 차선 조각의 직선 기울기와 폭 주기상 위치를 추정. `_road_frames`: 차선 위상을 시간축에서 풀어 차선 변경을 표현.
- **최신 수정:** `_road_frames`는 주변 차량을 읽지 않는다. 도색 공백에는 자차 오도메트리와 앞뒤 차선 관측에서 얻은 도로 방향을 사용하고, 다음 관측으로 누적 드리프트를 구간 내 역보정한다. 구간 전체에 도색이 없으면 오도메트리 fallback이다.
- `stabilize_road_frames`: 앞뒤 1.2초 내 가중 robust 국소 다항식으로 자차 표시 위치/방향을 안정화. `ego_display_override`에만 적용해 기존 차로 점유 판정과 주변 차량 계산을 바꾸지 않는다. `FusedSceneTimeline.sample`에서 표시 좌표를 보간한다.
- 도로는 여전히 **평행한 연속 차로 묶음**이다. 교차로/분기 토폴로지나 완전한 곡률 모델이 아니다. 이 부분이 사용자 확인 실패의 핵심이다.

### 남겨둔 실험을 기본 구현으로 착각하지 말 것

- `scene_road.add_camera_corridors`: 이전 카메라 분할 기반 도로. `scene_road_surface.py`의 시간축 단면 중앙값도 가장자리 요철을 충분히 해결하지 못했다. 현재 기본에서 호출하지 않는다. 관련 예전 테스트는 명시적으로 이 함수를 검사한다.
- `scene_road_model.py`/`scene_road_evidence.py`/`scene_lane_model.py`: YOLOP 도로/도색 및 UFLDv2 실험. UFLD는 곡선/교차로 오검출이 심해 채택하지 않았다. 도로 증강 결과와 기본 분석 결과는 다르다.
- `scene_trajectory_fit.py`: 방향/이동을 함께 맞추는 **제안 전용** 실험. 실제 교차로 차량 3대는 기준 미달, 자동 적용되지 않았다.
- `scene_image_track.py`: 작은 이륜차 역방향 이미지 추적 제안. 3D 위치 검증 전이라 검출물로 삽입하지 않았다.
- `scene_motion_heading.py`: 이동 방향 추정 실험. 이동 방향이 정지 차량의 차체 앞뒤 방향을 알려주지는 않는다.
- `scene_truck_shapes.py`: 대표 트럭 크기 후보 선택은 기본 적합에 적용됨. 실제 차종 치수를 측정한 것은 아니다.

## 4. 해결되지 않은 문제 — 먼저 원본으로 확인할 것

1. **교차로를 평행 차로로 그린다.** 교차로가 인식됐다는 주장 불가.
2. **횡단 차량이 세로 방향인 채 옆으로 이동한다.** 위치 궤적과 차체 방향이 분리돼 있고 기본 적합이 도로 정렬 형상에 치우쳐 있다. 렌더 yaw만 속도 방향으로 바꾸는 것도 정차/후진/곡선 문제를 해결하지 못한다.
3. 검출됐어도 접지 형상 적합 실패로 3D에 표시되지 않는 차량이 있다. 검출 누락도 있다. 검출 confidence/박스 IoU가 실제 미터 거리 정확도는 아니다.
4. 카메라 자세·왜곡·동기·도로 경사·차종 크기 가정 오차가 남는다. 리피터 horizon은 개발 자료로 경험적 수정했지만 공장 캘리브레이션이 아니다.
5. 자차 위치는 안정화했으나 절대 정확도와 차선 변경 완료 시점은 완전히 검증하지 못했다. 마지막 수정에서도 기존 샘플 21/22초는 -0.624/-0.832차로, 23/24/25초는 -0.972/-1.031/-1.058차로다. 사용자는 이미 21초경 완료했다고 지적했다. 타이밍이 정확하다고 보고하지 말 것.
6. 교차로 차량 누락/방향, 이륜차, 차로 수, 도로 끝과 가림의 구분 등은 전반적으로 미완성이다.

## 5. 원본·결과·검증 자료

아래 경로는 모두 `C:\Users\ddno2\Documents\ChatGPT\테슬라\scene-results` 기준이다.

### 최신 결과

- `vehicle-road-reset/high-host-road-only-37s.mp4`: **최신** 주변 차량 기반 자차 위치 제거 결과. 원본 4채널 포함, 370프레임/10fps/37초 디코딩 확인. `.road-audit.json`, `.json`, `high-host-road-only-24s.jpg`도 있다.
- 같은 폴더 `high-stable-37s.mp4`: 직전 자차 흔들림 보정 결과. 주변 차량 보조 로직이 남아 있으므로 최신으로 쓰지 말 것.
- `high-phase-37s.mp4`: 도로 원점 재구성 초기 결과. `high-other-date-35s.mp4`: 다른 날짜 35초 비교. 둘 다 최신 자차 수정 이전이다.
- `high-original-37s.mp4`: **실패해 폐기한 중간안**. 긴 곡선에서 직선 월드 도로를 유지해 자차가 도로 밖으로 나간다. 정상 결과로 제시하지 말 것.
- `quality-rebuild/delivery-road-consensus/`: 직전의 분할 도로 실패 결과. 최신 도로가 아니다.
- `daylight-delivery/CT-Daylight-high-35sec.mp4`: 초기 차량 기반 연속 도로. `directml-upgrade/road-style/`는 그 후 카메라 전용으로 바꾸며 도로가 사라졌던 버전. `road-history-review/` 비교 이미지 참고.

### 기존 고속 회귀 원본

`E:\tesla\a2\TeslaCam\SavedClips\2026-09-05_14-56-11\2026-09-05_14-54-11-front.mp4`

분석 캐시: `directml-upgrade/high-v11x-analysis` (370시점). 저속 캐시는 같은 상위의 `low-v11x-analysis`.

### 다양한 샘플

`quality-rebuild` 아래:

- `catalog/catalog.json`: a2~a6 목록, 62개 날짜/4,477개 고유 타임스탬프. 타임스탬프 기반 중복 구분이며 내용 해시 전수 중복 제거는 아니다.
- `baseline-cases.json`, `baseline/`: 21구간. 대개 10초이므로 30초 연속 검증으로 계산하면 안 된다.
- `continuous/`, `continuous-roads/`: 2026-09-22_08-59-36 트럭 추월, 2026-10-27_13-43-29 우측 미니밴, 2026-10-31_13-02-31 고속 접근. 각각 35초. `-roads`는 실험 도로 증강 캐시.
- `fresh-validation/analysis/`: 2026-08-18_07-38-46 도심, 2026-08-19_17-16-37 교차로, 2026-08-31_12-57-14 빗길 곡선, 2026-10-16_20-54-27 야간 주차. 각각 35초/350시점. 원본 경로는 `fresh-validation/run-configuration.json`.
- `fresh-validation/source-keyframe-annotations.json`: 원본에서 먼저 수동 표기한 일부 차량/신호. 전수 정답셋 아님.
- 교차로 2026-08-19 15초: 흰 횡단 SUV front:25, 어두운 SUV front:26, 어두운 세단 front:35. 이륜차 누락, 차량 미배치/방향 실패가 확인됨. `crossing-*.json`, `crossing-fit-overlay.jpg` 참고.
- `split-overrides.json`: 튜닝에 사용한 날짜를 개발용으로 이동한 기록. 촬영 날짜 단위 분리를 유지해야 한다. 특히 8/19와 10/31은 독립 평가로 재사용하지 말 것.

## 6. 재현 명령 (저장소에서 PowerShell)

기존 분석 캐시로 UI를 열면 추가 검출 연산 없이 현재 렌더러를 검토할 수 있다.

```powershell
python tools/scene_lab.py --result ../scene-results/directml-upgrade/high-v11x-analysis
python -m unittest discover -s tests
```

현재 코드로 원본 4채널 비교 영상 출력. 출력 파일은 새 이름이어야 한다.

```powershell
python tools/export_scene_comparison_video.py --result ../scene-results/directml-upgrade/high-v11x-analysis --source E:/tesla/a2/TeslaCam/SavedClips/2026-09-05_14-56-11 --output ../scene-results/gemini-review/high-current.mp4
```

재분석이 필요하면 `python tools/analyze_scene.py --help` 참조. 입력은 `*-front.mp4`, 모델 `../scene-models/yolo11x.onnx`는 640, `yolo11x-960.onnx`는 960 입력이다. DirectML 옵션은 `--backend directml`; 라이브러리 경로/환경이 맞아야 실제 GPU를 쓰므로 출력 inference 정보로 확인할 것. CUDA를 추가하지 말 것.

## 7. 테스트가 입증하는 범위

- 최신 전체 실행 기록: **198 tests / 84.908초 통과**. 그 뒤 자차 안정화와 차량 의존 제거를 추가했으므로 최신 전체 스위트 통과라고 쓰면 안 된다.
- 최신 변경 후 대상 검사: vehicle-road 8, ego-stabilization 2, fusion 19 통과. 마지막 영상 370프레임 전부 디코딩, 24초 비교 이미지 확인. git diff --check 통과.
- 합성 검사는 자차 차선 변경/고스트/오배치 배제/차로 유지/좌표 독립성 등을 검사한다. 실제 교차로 인식이나 차량 방향·위치 정확도를 보증하지 않는다.
- `SCENE_ACCEPTANCE.md`는 여전히 **불합격**. `SCENE_QUALITY_REBUILD.md`는 긴 작업 기록으로, 오래된 중간 주장보다 현재 코드와 최신 항목을 우선해야 한다.

## 8. 다음 검토자에게 권하는 검토 순서

먼저 추가 연산보다 원본과 최신 결과를 보고, 교차로 및 차량 방향을 현재 구조에서 해결할 수 있는지 판단해 달라. 작은 수정/테스트 증가를 전체 품질 개선으로 보고하지 말 것. 사용자가 계속 진행을 승인하면 비용·범위를 제한한 대표 사건 검증부터 시작하는 편이 좋다. 코드를 유지할지 교체할지는 재검토 가능하지만, 사용자 요구인 차량 기반 차로 구성과 자차 위치의 독립성을 다시 혼동하지 말 것.
