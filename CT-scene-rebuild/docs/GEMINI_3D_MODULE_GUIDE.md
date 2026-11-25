# 제미나이 필독: 이미 구현된 3D 재현 모듈

**이 프로젝트에는 작동하는 다중 카메라 3D 주행 재현 모듈이 이미 있다.** CT Dashcam Studio에서 파생된 CT-scene-rebuild가 본체다. 영상에서 차량·자차 움직임과 도로를 추정하고, 실제 GLB 차량을 움직여 3D+원본4채널 MP4를 만든다. 별도의 신규3D 엔진을 만드는 작업부터 시작하지 않는다. 최근 접점·방향 학습은 이 본체를 개선하기 위한 보조 실험이며 아직 연결하지 않았다.

## 먼저 기존 결과를 볼 것

로컬 전체3D 결과:

- G:/3d/CT-scene-rebuild/runs/curb-steering-highway-002/surrounding-world.mp4
- G:/3d/CT-scene-rebuild/runs/curb-steering-oct02-002/surrounding-world.mp4
- G:/3d/CT-scene-rebuild/runs/curb-steering-junction-001/surrounding-world.mp4
- G:/3d/CT-scene-rebuild/runs/curb-steering-ramp-001/surrounding-world.mp4

고속 영상의 [Drive 재생](https://drive.google.com/file/d/1a-GXPYfVZ-swnk5k5dAygyaSH9-HhPCo/view?usp=drivesdk). 이 영상들이 본체의 결과다. contact-learning-review의 접점 그림 영상은 학습 검수용 정지 프레임 모음이므로3D 모듈의 결과로 판단하면 안 된다.

화면은 테슬라 다크테마, 후방 위 고정 시점, 확대1.3, 주변 차량 클레이, 자차와 도로·차선·일부 주변 도로를 표시하며 하단에 원본4채널이 들어간다. B필러 미관측 구간에는 앞뒤 관측으로 지지되는 경우 고스트 추정을 표시한다. 전체 정확도 완성은 아니며 일부 위치 미확정·겹침·교차로 일반화 문제는 남아 있다.

## 1. 편집기에서 실제로 연결되는 경로

```text
G:/3d/CT-dashcam-studio/ui/main_window.py
  → ui/driving_scene_panel.py의 DrivingScenePanel
  → 선택한 엔진/src/ct_scene/review_app.py의 ReviewWindow
  → 엔진 .venv Python 별도 프로세스
  → python -m ct_scene analyze ...
  → src/ct_scene/pipeline.py
  → src/ct_scene/surrounding.py
  → GLB 렌더와 surrounding-world.mp4
```

메뉴: **실험실 / Lab → 주행3D 재현 / 분석·재생·MP4**. 기존 결과 열기에는 위 runs/curb-steering-highway-002 같은 **scene 결과 폴더**를 선택한다. MP4만 선택하는 입력이 아니다. 새 분석은 동일 시각의 front 원본을 지정하면 back/left_repeater/right_repeater 원본을 찾는다. 전체 영상 저장은 MP4 내보내기다.

엔진 탐색 우선순위는 편집기 하위 CT-scene-rebuild → 형제 G:/3d/CT-scene-rebuild이며, .venv/Scripts/python.exe가 있는 곳을 선택한다. **인계 시 실제 검사 결과 편집기 하위에는 .venv가 없고 형제 엔진에는 있으므로 G:/3d/CT-scene-rebuild가 실행된다.** 하위 사본만 고치면 현재 실행에 반영되지 않을 수 있다. ui/driving_scene_panel.py에서 확인할 것. Qt 검토 위젯만 편집기에 올라가고 분석은 엔진의 별도 Python에서 돈다.

편집기 안에는 과거 core/driving_scene.py와 ui/scene_dialog.py도 남아 있다. 이는 구형 Scene Lab 경로다. 현재 재개발 엔진을 찾을 때는 ui/driving_scene_panel.py → ct_scene.review_app 연결부터 따라간다.

독립 실행:

```powershell
Set-Location 'G:/3d/CT-scene-rebuild'
./Launch-Scene-v2.ps1
```

## 2. 영상에서 3D까지의 처리 흐름

```text
같은 시각4채널 원본 MP4 + 전방 SEI 텔레매트리
 → 입력/시계 검사와 원본 프레임 패키지
 → 차량 검출 → 카메라별 추적
 → SEI 속도·방향 + 정적 배경 회전으로 자차 타임라인
 → 카메라별 접점/차체 크기 가정으로 대상 차량 위치 후보
 → 카메라 간 동일 차량 연결·오프라인 후보정·사각지대 추정
 → 카메라 도색/도로 인식 + 선택적 OSM 윤곽
 → 차량별 위치·방향·차로 상태 및 표시 충돌 제약
 → SurroundRenderer + GLB 메시 + 도로/차선
 → 전체20fps MP4와 프레임별 world 상태
```

후방은 왜곡이 크므로 정밀 위치 투영이 아니라 참고 역할이다. 전방/리피터가 주요 위치 근거다. OSM은 선택적 주변 형태 참고이며 GPS로 세부 차로를 정하지 않는다.

## 3. 소스 파일 지도

아래 엔진 파일은 모두 G:/3d/CT-scene-rebuild 아래다.

| 파일 | 읽어야 할 기능 |
|---|---|
| src/ct_scene/cli.py | analyze, surrounding-prototype, replay-scene, manual-render 명령 |
| src/ct_scene/pipeline.py | analyze: 입력→검출→추적→자차→도로→3D→검사 전체 조립 |
| src/ct_scene/review_app.py | 기존 결과 열기·새 분석·재생·저장 UI |
| src/ct_scene/surrounding.py | run: 다중 차량3D 조립, project_contact/front/vehicle_height, SurroundRenderer |
| src/ct_scene/prototype_front.py | Renderer·GLB 메시 읽기·기본 렌더 연산 |
| src/ct_scene/full_timeline.py | 자차 시각·속도·방향·좌표와 기준 차량 타임라인 |
| src/ct_scene/ego_turn_rotation.py / steering_cue.py | 정적 배경 회전 보정과 자차 조향 방향 보조 검사 |
| src/ct_scene/tracking.py / identity_graph.py | 영상 추적·카메라 간 연결 근거와 선택 |
| src/ct_scene/blindspot_continuity.py | 미관측 사각지대의 제한적 추정 표시 |
| src/ct_scene/actor_heading.py / image_heading_refine.py | 차량 진행/차체 방향 처리 |
| src/ct_scene/curved_road.py / road_sections.py | 영상 근거의 곡선 도로와 구간별 형태 |
| src/ct_scene/right_edge_profile.py / local_solid_corridor.py | 갓길 제외·반복 실선·지역 도로 폭 |
| src/ct_scene/junctions.py / junction_branches.py / camera_junction_roads.py | 교차로·연결/횡단 도로 근거와 표시 |
| src/ct_scene/osm_context.py / context_surface.py | OSM 참고 도로와 주변 표면 |
| src/ct_scene/collision_constraints.py | 표시 차량 겹침 제약. 잔여 겹침은 여전히 존재 |
| src/ct_scene/scene_replay.py | 저장된3D 상태로 추론 없이 재렌더 |
| src/ct_scene/quality_review.py / scene_audit.py | 전체 프레임·동작·겹침·원본 관측 커버리지 검사 |
| configs/multicam-optical-candidates.json | 추정 카메라 내부 파라미터. 실측 정밀 보정값 아님 |
| configs/surrounding-prototype.json | 카메라 장착 가정·표시 정책·색·시점·차로/추정 설정 |

## 4. 모델이라는 말의 세 가지 의미

1. G:/3d/fsd-sedan-3d/models 및 기존 엔진 assets의 GLB: 화면에 그리는3D 외형이다. 검출 신경망 가중치가 아니다.
2. G:/3d/scene-models/yolo11x.onnx, model-review/yolopv2-roads.onnx, orientation-review/regressor_mobilenetv3.onnx: 기존 분석 파이프라인에서 쓰는 검출/도로/차체 방향 모델이다. 기존 orientation 모델은 파일이 있을 때 pipeline의 image-headings 단계에서 사용된다.
3. 엔진 models/contact-adapter-v1·v2: 최근 별도 학습한 실험 head다. 현재 pipeline.py나 surrounding.py에 연결하지 않았다. v2 실패가3D 본체 부재를 뜻하지 않는다.

학습 실험을 개선하려면 [학습 상세 안내](GEMINI_TRAINING_GUIDE.md)를 이어서 읽는다. 3D 위치 개선은 해당 후보를 어느 좌표계와 어느 위치 추정 단계에 연결할지 검증하는 별도 단계다.

## 5. 새 원본 전체 분석 명령

먼저 --help로 옵션을 확인한다. 아래 analyze는 실제 전체 분석을 수행하며 새 output을 사용해야 한다. front와 같은 시각의4채널 원본이 필요하다.

```powershell
Set-Location 'G:/3d/CT-scene-rebuild'
$env:PYTHONPATH = 'src;tools'
$runTag = 'gemini-scene-' + (Get-Date -Format 'yyyyMMdd-HHmmss')
./.venv/Scripts/python.exe -m ct_scene analyze --front 'E:/tesla/a5/RecentClips/2026-11-01_13-50-00-front.mp4' --output "runs/$runTag" --detector G:/3d/scene-models/yolo11x.onnx --road-model G:/3d/scene-models/model-review/yolopv2-roads.onnx --lens-config configs/multicam-optical-candidates.json --prototype-config configs/surrounding-prototype.json --assets-root runs/curb-steering-engine-002/assets --backend directml
```

--backend cpu도 지원한다. DirectML은 CUDA가 아니다. --osm은 선택적 로컬 OSM 입력이다. 위 예에는 넣지 않아 네트워크 조회가 자동 추가되지 않는다. 원본 주변 OSM 조회는 기존 허용 범위와 사용 도구를 확인한다.

결과는 runs/새이름/scene/surrounding-world.mp4. pipeline.json은 전체 진행, scene/artifact.json은 결과 식별/해시다. 입력이나 알고리즘이 달라졌으면 기존 output으로 이어하려 하지 말고 새 폴더를 쓴다. 전체 분석은 시간이 걸리므로 기존 결과를 볼 때마다 실행하지 않는다.

## 6. 이미 계산된 장면을 보기 좋게 다시 출력

차량 위치를 바꾸지 않고 색·시점만 확인할 때:

```powershell
$replayTag = 'gemini-replay-' + (Get-Date -Format 'yyyyMMdd-HHmmss')
./.venv/Scripts/python.exe -m ct_scene replay-scene --scene runs/curb-steering-highway-002 --assets-root runs/curb-steering-engine-002/assets --prototype-config configs/surrounding-prototype.json --output "runs/$replayTag"
```

이 명령은 저장된 world 상태를 재생한다. 위치 알고리즘을 고쳤는데 이 명령만 돌리면 위치 수정은 반영되지 않는다. 주변 차량 위치/도로 조립을 다시 계산하려면 surrounding-prototype 또는 tools/render_rebased_case.py를 사용한다. 후자는 기존 source/tracks/base/road 입력을 재사용하고 --engine의 고정 소스를 실행한다. 과거 frozen engine을 지정하면 현재 수정 코드가 아니라 과거 코드를 실행한다. --allow-base-change는 자차 기준을 의도적으로 바꿀 때만 검토하며 일반 회귀에 붙이지 않는다.

기존 결과 폴더의 주요 파일:

- surrounding-world.mp4: 최종3D+원본4채널 영상
- render-states.jsonl: 각 프레임 자차/대상 위치·방향·고스트/관측 상태
- actors.json: 차량별 타임라인과 연결 정보
- report.json: 도로·표시·원본 관측·분석 요약
- ego-lane-state.json / actor-lane-state.json: 차로 상태
- blindspot-inference-review.json: 사각지대 추정 근거
- artifact.json: 입력 의존성과 파일 해시. 수정된 파일을 기존 결과인 것처럼 섞지 않는다.

## 7. 제미나이의 첫 작업 순서

1. 위 기존3D MP4를 보고 module이 이미 무엇을 그리는지 파악한다.
2. ui/driving_scene_panel.py → review_app.py → pipeline.py → surrounding.py를 읽는다.
3. 실제 선택되는 엔진 경로를 확인하고, 현재 코드·Git 변경·기준 결과를 기록한다.
4. 차량 위치 문제를 원본 관측·카메라 투영·자차 운동·동일 차량 연결·표시 후처리 중 어느 단계인지 구분한다.
5. 그 뒤 필요한 학습 실험을 진행한다. 새 head를 배우는 것과3D 재현 모듈을 새로 만드는 것을 혼동하지 않는다.

기존 기본 화면을 보존하고 별도 결과로 비교한다. 정중한 한국어, CUDA 제외, 되돌릴 수 있는 Git 기록, 푸시까지만. 빌드·릴리즈하지 않는다. 영상은 로컬 재생과 Google Drive 테스트26을 함께 제공한다.
