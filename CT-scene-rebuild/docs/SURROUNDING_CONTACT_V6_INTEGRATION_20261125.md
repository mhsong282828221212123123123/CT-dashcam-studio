# 기존 3D 본체(`surrounding.py`) 선택적 `--contact-adapter` 연결 및 실장면 검증 · 2026-11-25

## 1. 변경 범위 (기존 기본 3D 경로 100% 보존)

`docs/GEMINI_3D_MODULE_GUIDE.md`에 따라 기존 작동 중인 3D 본체(`src/ct_scene/surrounding.py`, `src/ct_scene/cli.py`)를 파악한 뒤, 기본 동작(`contact_adapter=None`)은 변경하지 않고 **명시적 opt-in 옵션(`--contact-adapter`)**으로 학습된 지면 접점 모델(`models/contact-adapter-v6`)을 전방(`project_front`) 및 측면 지면 접점(`side_ground_contact_W`)에 연결할 수 있도록 확장했다.

- `project_contact(..., contact_uv=None)` / `project_front(..., contact_uv=None)`: `contact_uv=None`이면 기존 상자 바닥점 `(0.5, 1.0)`과 바이트 단위로 동일하다.
- `surrounding.run(..., contact_adapter=None)` / `cli.py surrounding-prototype --contact-adapter`: CPU 전용 ONNX 추론으로 비가림·비절단 검출(`w>=85, h>=55, margin>=8px`)에 한해 `(u, v)`를 적용하고 `report.json`의 `learned_contact_adapter`에 적용 건수와 평균 이동량을 기록한다.
- 기본값은 `None`이므로 기존 대시캠 편집기 및 기본 3D 장면에는 자동 적용되지 않는다.

## 2. 실제 60초 3D 장면 전/후 비교 (`models/contact-adapter-v6`)

기존 캐시 입력(`source`, `tracks`, `base`, `road-masks`, `osm-context`, `telemetry`, `image-headings`)을 그대로 재사용해 실제 3D 장면(`surrounding-world.mp4`, 20fps, 하단 4채널 유지)을 재렌더하고 `quality_review.py`로 수치 검증했다.

| 장면 | 적용 관측 (전방 / 측면) | 기존 충돌 (보정 전 → 보정 후, 판정) | `v6` 적용 충돌 (보정 전 → 보정 후, 판정) | 수치 게이트 (`numerical_gate`) |
|---|---:|---|---|---|
| **`highway` (`curb-steering-highway-002`)** | 375 / 335 | 91 → 22 (`FAIL`) | **1 → 0 (`PASS`)** | **`PASS` (실패 0건)** |
| `oct02` (`curb-steering-oct02-002`) | 514 / 274 | 56 → 10 (`FAIL`) | 63 → 10 (`FAIL`) | `FAIL` (기존 원거리 84.6m 고스트 9프레임 + 측면 1프레임 0.207m→0.199m) |

- **`highway` 개선 확인**: 실제 60초 고속도로 3D 장면에서 `models/contact-adapter-v6`를 opt-in 적용하자, 보정 전 차량 겹침이 **91프레임 → 1프레임(-98.9%)**, 보정 후 잔여 겹침이 **22프레임 → 0프레임(`FAIL → PASS`)**으로 완전히 해소되었고, `numerical_gate = PASS`, `source_presence_screen = NOT_FLAGGED`를 통과했다.
- **`oct02` 확인**: 새로운 움직임 점프나 추가 잔여 겹침 없이 기존 잔여 10프레임(84.6m 원거리 고스트 겹침 9프레임 포함)을 유지했다.

## 3. 산출물 및 검증

- 단위 테스트: `431 tests OK` (`project_contact` 기본값 불변 및 `contact_uv` 투영 테스트 추가).
- 실제 3D 재렌더 영상(로컬):
  - `G:/3d/gemini/CT-scene-rebuild/runs/gemini-3d-v6-highway-001/surrounding-world.mp4` (`numerical_gate=PASS`, 충돌 0프레임)
  - `G:/3d/gemini/CT-scene-rebuild/runs/gemini-3d-v6-oct02-001/surrounding-world.mp4`
- 기본 3D 적용 여부는 추가 교차로(`junction`)·램프(`ramp`) 및 미사용 날짜 전체 3D 회귀 확인 후 결정하도록 opt-in 상태를 유지한다.
