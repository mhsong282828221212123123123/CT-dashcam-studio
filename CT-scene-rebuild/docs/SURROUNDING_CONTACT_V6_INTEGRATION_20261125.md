# 기존 3D 본체(`surrounding.py`·`collision_constraints.py`) 개선 및 4개 실장면 검증 · 2026-11-25

## 1. 변경 내용 (기존 기본 3D 경로 보존 + 교차로 자차 겹침 수정)

1. **선택적 `--contact-adapter` 경로 추가 (`src/ct_scene/surrounding.py`, `src/ct_scene/cli.py`)**:
   - `project_contact(..., contact_uv=None)` / `project_front(..., contact_uv=None)` 및 `surrounding.run(..., contact_adapter=None)`에 명시적 opt-in 경로를 추가했다.
   - 기본값은 `None`이므로 기존 기본 3D 장면과 바이트 단위로 동일하며, `--contact-adapter models/contact-adapter-v6`를 지정할 때만 CPU ONNX로 비가림·비절단 차량의 지면 접점 `(u, v)`을 반영한다.
2. **차로 미확정 교차로 입구의 자차(`__ego__`) 겹침 해소 (`src/ct_scene/collision_constraints.py`)**:
   - 기존 `project()`는 `lane is None`이면 이동을 전면 차단해, 교차로 입구에서 측면 추정 차량(`left_repeater-0016`, `cap=0.75m`)이 자차 `__ego__`와 0.23m 겹쳐도 분리하지 못했다.
   - 도로 밖 일반 차량 쌍의 불변 규칙(`test_lane_boundary_and_unknown_road_are_not_crossed`)은 그대로 유지하면서, **`__ego__`와 직접 겹친 이동 가능 추정 차량에 한해** 기존 `cap`(`<=0.75m`)과 속도 연속성(`<=65 m/s`) 범위 내에서 자차와 분리되도록 수정했다.
3. **교차로·횡단보도 구간의 1차로 찌그러짐(차선 오인) 및 측면 차량 누락 수정 (`src/ct_scene/road_geometry.py`, `src/ct_scene/surrounding.py`)**:
   - `junction`의 앞뒤 구간(`sec 0·1·2`)이 모두 2차로인데, 구간 사이 교차로·횡단보도 공백에서 전역 1차로 기본값(`[1.47, -1.75]`)으로 떨어져 2차로가 갑자기 1차로로 좁아지던 문제를 인접 동차로 구간 간 `boundaries_at()` 선형 보간으로 수정했다(`supported()`는 교차로 내부에서 계속 `False` 유지).
   - `--contact-adapter` 사용 시 `left_repeater`/`right_repeater`도 차체 높이 가정(`project_vehicle_height`) 대신 학습된 `v6` 지면 접점(`max_depth=35m`)을 우선 사용하도록 개선해, 기존에 도로 밖 8~9m로 밀려 누락되던 좌측 인접 차량(`t=47.5s` 좌측 후방 SUV 등)이 좌측 1차로에 정상 표시되도록 복원했다(`actors 12 → 13`, `numerical_gate=PASS`, 충돌 0프레임).
   - 단, `t=17.5s` 교차로 횡단 차량 중 두 번째 차량(`front-0014`)의 코리더 밖 제외 및 4거리 직교 교차로 도로 형상 자체는 향후 과제로 남아 있다.

## 2. 실제 60초 3D 장면 4건 전/후 비교 (`models/contact-adapter-v6` + 교차로 자차 분리)

기존 캐시 입력(`source`, `tracks`, `base`, `road-masks`, `osm-context`, `telemetry`, `image-headings`)을 재사용해 4개 실제 60초 3D 장면(`surrounding-world.mp4`, 20fps, 하단 4채널 유지)을 재렌더하고 `quality_review.py`로 수치 검증했다.

| 실제 3D 장면 | 적용 관측 (전방 / 측면) | 기존 충돌 (보정 전 → 보정 후, 판정) | 개선 후 충돌 (보정 전 → 보정 후, 판정) | 수치 게이트 (`numerical_gate`) |
|---|---:|---|---|---|
| **`highway` (`curb-steering-highway-002`)** | 375 / 335 | 91 → 22 (`FAIL`) | **1 → 0 (`PASS`)** | **`PASS` (실패 0건)** |
| **`junction` (`curb-steering-junction-001`)** | 206 / 178 | 5 → 4 (`FAIL`) | **5 → 0 (`PASS`)** | **`PASS` (실패 0건)** |
| **`ramp` (`curb-steering-ramp-001`)** | 1 / 70 | 0 → 0 (`PASS`) | **0 → 0 (`PASS`)** | **`PASS` (실패 0건)** |
| `oct02` (`curb-steering-oct02-002`) | 514 / 274 | 56 → 10 (`FAIL`) | 63 → 10 (`FAIL`) | `FAIL` (기존 84.6m 원거리 고스트 9프레임 + 측면 1프레임 0.207m→0.199m) |

- **4개 실장면 중 3개(`highway`, `junction`, `ramp`)가 충돌 0프레임 · `numerical_gate = PASS` · `source_presence_screen = NOT_FLAGGED`를 통과**했다(기존 1/4 PASS → 개선 후 **3/4 PASS**).
- `oct02`는 새로운 움직임 점프나 추가 잔여 겹침 없이 기존 84.6m 원거리 고스트 9프레임 및 측면 1프레임을 유지했다.

## 3. 검증 및 산출물

- 단위 테스트: `432 tests OK` (`test_ego_overlap_at_unresolved_junction_separates_within_cap`, `test_project_contact_accepts_optional_learned_uv_without_changing_default` 포함).
- 실제 3D 재렌더 영상(로컬, 20fps + 하단 4채널 유지):
  - `highway` (`PASS`): `G:/3d/gemini/CT-scene-rebuild/runs/gemini-3d-v6-highway-001/surrounding-world.mp4`
  - `junction` (`PASS`): `G:/3d/gemini/CT-scene-rebuild/runs/gemini-3d-v6-junction-002/surrounding-world.mp4`
  - `ramp` (`PASS`): `G:/3d/gemini/CT-scene-rebuild/runs/gemini-3d-v6-ramp-001/surrounding-world.mp4`
  - `oct02`: `G:/3d/gemini/CT-scene-rebuild/runs/gemini-3d-v6-oct02-001/surrounding-world.mp4`
