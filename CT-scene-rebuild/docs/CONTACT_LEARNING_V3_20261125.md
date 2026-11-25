# v2 재현·정답/분할 검토 및 v3 CPU 실험 결과 · 2026-11-25

## 1. 작업 범위와 기존 결과 보존

- 기준 폴더 `G:/3d`는 수정하지 않고, `G:/3d/gemini/CT-scene-rebuild` 및 `G:/3d/gemini/CT-dashcam-studio`의 별도 작업 브랜치(`codex/gemini-contact-v3`)에서 진행했다.
- 기존 `models/contact-adapter-v1`, `models/contact-adapter-v2`, `configs/contact-learning-annotations-v2.json`은 그대로 보존했다.
- CUDA는 사용하지 않았고(CPU 전용), 빌드·릴리즈는 수행하지 않았다.

## 2. 기존 v2 모델 재현 확인

`configs/contact-learning-annotations-v2.json`으로 특징 추출 → 학습 → 패키징 → CPU ONNX 검토(`runs/gemini-repro-v2-001-*`)를 재실행했다.

- `selected` 및 `results` 전 항목이 `models/contact-adapter-v2/training-report.json`과 정확히 일치했다.
- `adapter.npz` 가중치 최대 절대 오차: `0.0` (`EXACT_REPRO_PASS`).
- CPU PyTorch/ONNX 특징 최대 절대 오차: `7.63e-6`, 검토 영상 660프레임 디코드 통과.

## 3. v2 실패 원인 검토와 데이터 분할 개선

v2 실패 원인 분석(개발 세트 Leave-One-Session-Out CV 및 라벨 시트 직접 확인):

1. **공간 해상도 손실과 과적합**: 기존 `AvgPool2d(4, stride=3)`은 7×7 특징을 2×2(2,304차원)로 뭉개 바퀴 접점의 수평 위치를 흐리고, 37개 학습 접점에 `alpha=0.01`이 선택되어 학습 오차 `0.0359`(방향 61/61)로 강하게 과적합했다.
2. **좌우 대칭 미활용과 카메라 상호작용 부재**: `left_repeater`와 `right_repeater`는 거울 대칭인데 수평 반전 증강이 없었고, 메타데이터에 카메라×위치·종횡비 상호작용이 없었다.
3. **평가 세트 누출 방지 및 카메라별 표본 부족**: v2에서 이미 확인한 `oct26`(접점 6개, `front` 1개)은 최종 평가로 재사용할 수 없다.

이에 따라 `configs/contact-learning-annotations-v3.json`(총 142개)을 새로 작성했다.

- **학습(train, 63개)**: `oct02`, `2026-10-18`, `aug26`, `aug28`, `oct22` (접점 37개, 방향 61개).
- **조정(validation, 43개)**: 기존 `aug20`, `2026-11-01`에 v2에서 이미 본 `oct26`을 편입 (접점 25개, 방향 41개).
- **새 최종 평가(test, 36개)**: 미사용 촬영 날짜 3건(`sep02`, `sep08`, `oct21`)에서 크롭 36개를 추출해 Gemini가 직접 바퀴 접점 쌍과 가림 여부를 검수했다(`visual_reviewed`, `reviewer: Gemini`). 가드레일·수풀·차량·보행자·기둥에 가린 20개는 `null`로 제외했다.
  - 카메라별 유효 접점: `front` 4개, `left_repeater` 8개, `right_repeater` 4개 (총 16개, `all_cameras_have_three_test_contacts = true`).
  - 방향 정답: 36개.
  - 최종 평가 수치를 본 뒤 하이퍼파라미터를 재조정하거나 실패 표본을 제외하지 않았다.

## 4. v3 모델 변경과 성적

동일한 고정 KITTI MobileNetV3-small 백본(CPU) 위에서 아래 항목을 개선했다.

- **다중 풀링 CPU ONNX 인코더(`--encoder-version v3`)**: 방향용 전체 2×2 풀링(2,304차원) + 접점용 하단 4행 수평 스트립·좌우 차분 풀링(3,456차원) = 5,760차원.
- **수평 반전 증강 및 TTA(`flip_tta=true`)**: 좌우 반전 시 카메라(`left_repeater↔right_repeater`), `u→1-u`, `left↔right` 대칭을 정확히 적용.
- **차량 높이 단위 접점 회귀(`height_space_contact=true`)** 및 **카메라 상호작용 메타데이터(`rich_meta_contact=true`)**, 과적합 억제를 위한 정규화 구간(`alpha ∈ [0.1, 25.0]`).

접점 오차는 검출 상자 높이로 정규화한 이미지 2D 거리이며 미터 오차가 아니다.

| 자료 | 접점 수 | 상자 바닥점 평균(p90) | v3 모델 평균(p90) | 방향 정확도 |
|---|---:|---:|---:|---:|
| 학습 (train) | 37 | 0.1879 (0.4857) | 0.0676 (0.1623) | 61/61 (100.0%) |
| 조정 (validation, oct26 포함) | 25 | 0.1563 (0.3096) | 0.1273 (0.2739) | 33/41 (80.5%) |
| **새 최종 평가 (sep02·sep08·oct21)** | **16** | **0.1883 (0.4032)** | **0.1896 (0.3375)** | **16/36 (44.4%)** |

### 판정: 기본 3D 미적용 (`REJECTED`, `runtime_adopted=false`)

- **개선된 점**: v2가 새 날짜(`oct26`)에서 평균 +63.2%, p90 +121.8% 악화했던 것과 달리, v3는 완전 미사용 3개 날짜(`sep02·sep08·oct21`)에서 최악 오차(p90)를 `0.4032 → 0.3375`(-16.3%)로 줄였고 측면 차량(`sep02-007`, `oct21-029`, `oct21-031`, `oct21-034`) 오차를 54~64% 감소시켰다.
- **실패한 점**: 새 날짜 평균 접점 오차(`0.1896`)가 상자 바닥점(`0.1883`)을 넘지 못했고(`contact_2d_candidate_pass = false`), 특히 방향 정확도가 조정 세트 80.5%에서 새 날짜 44.4%(가림 크롭 8/20, 비가림 크롭 8/16)로 급락했다(`direction_candidate_pass = false`).
- **원인**: 37개 학습 접점·61개 방향 표본에 고정 KITTI 방향 백본의 선형 ridge head만 얹는 구조는 리피터 카메라의 사전 편향(camera prior)과 도메인 이동(가림·차종 변화)을 극복하기 어렵다.
- 따라서 `models/contact-adapter-v3`는 실험·실패 가중치로 보존하며 기본 3D 장면에 연결하지 않는다(`applied=false`, `metric_position_available=false`).

## 5. 검증 및 산출물

- 단위 테스트: `430 tests OK` (기존 v1/v2 호환 및 v3 대칭·미적용 가드 테스트 추가).
- CPU ONNX 패리티: 142개 표본 최대 절대 오차 `1.17e-5`, 학습/패키지 예측 일치 통과.
- 정지 프레임 검토 영상(새 3D 결과 아님): `G:/3d/gemini/CT-scene-rebuild/runs/gemini-v3-001-review/heldout-contact-review.mp4` (36프레임 × 1.5초 = 1080프레임 디코드 통과, 하단 4채널 유지).
- 정답 파일: `configs/contact-learning-annotations-v3.json`
- 모델 패키지: `models/contact-adapter-v3/`

## 6. 다음 개선 방향

- 선형 ridge head 추가 튜닝보다 **바퀴별 키포인트·가시성 분리 헤드** 또는 **학습 날짜/표본 수 대폭 확장(최소 수백 개 이상, 다양한 리피터 3/4 각도 포함)**이 선행되어야 한다.
- `sep02`, `sep08`, `oct21`은 v3 최종 평가에 사용했으므로 향후 조정에 편입할 경우 `aug16`, `aug17`, `aug25`, `sep17`, `sep20` 등 남은 미사용 날짜를 새 최종 평가로 남겨야 한다.
