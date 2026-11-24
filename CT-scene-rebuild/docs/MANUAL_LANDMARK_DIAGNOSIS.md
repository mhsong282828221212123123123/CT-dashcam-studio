# 수동 정적 기준점으로 분리한 연결 실패 원인

**정정:** 이 문서의 수동 reference 001에는 잘못 찍은 지붕 모서리와 부정확한 창틀 점이 있었다. 수동점에 의존한 residual/ROI 개수와 그 품질 판단은 폐기된 개발 기록이다. [MULTIVIEW_GEOMETRY_REVIEW.md](MULTIVIEW_GEOMETRY_REVIEW.md)의 002 원본 재검수와 재실행 결과가 우선한다. 기존 파일/실행은 이력으로 보존한다.

2026-11-14. 공통 rig gate는 여전히 **FAIL**이다. 아래는 잘못된 입력 제외 영역을 찾아 수정한 개발 진단이며 카메라 보정/3D 재현 합격이 아니다.

## 원본 대조와 기준점

35초 August 개발 영상의 front/left/right를 0/8/16/24/32초에 직접 대조했다. front 24초와 left 32초에서 같은 외벽을 확인했고, 더 명확한 간판·창틀이 있는 부동산 건물의 front 21초 / left 28초를 수치 진단 대상으로 선정했다. front 20/21/22/23/25/26초와 left 28/30/34초 원본도 확인했다. contact sheets는 `runs/common-landmark-review-001`의 검토 보조 이미지다.

`configs/office-landmark-reference.json`에 원본 pixel 기준 지붕·창틀 14점을 기록했다. 이는 사람이 확대 원본에서 지정한 근사치(명목 3px 불확실성)이며 실측 3D/전체 scene GT가 아니다. 6점만 평면 변환 fitting에 쓰고 나머지 8점은 분리했다. withheld 값이 fitting에 들어가지 않는 회귀 검사를 추가했다.

## 확인된 제외 영역 문제와 수정

기존 static 특징점 경로는 카메라 공통으로 위 25%를 제외했다. front 21초의 지붕과 세 창문 윗부분이 실제 정적 배경임에도 제외됐다. 14개 수동 대응점 중 양쪽 mask/ROI를 모두 통과하는 것은 **1개**였다.

새 `optical_feature_roi`는 front의 상단 배경을 남기고 side는 좌/우 미러 쪽을 각각 제외한다. 기존 `feature_roi`와 기존 결과는 변경하지 않아 이전 mono 실험을 보존했다. 새 영역은 수동 대응점 **7개**를 양쪽에 남긴다. 나머지는 주로 움직이는 차량의 보수적인 bbox+padding mask에서 제외된다. 이 mask를 임의로 풀어 차량을 정적 기준점으로 사용하지 않았다.

**적용 범위:** 현재 새 ROI는 affine 연결 실험 경로에 적용했다. 기존 SfM/mono 결과와 실제 사용자 앱의 3D 엔진이 수정됐다고 주장하지 않는다. ROI 자체도 임시 이미지 영역이며 제조사 제공 보정 프로필이 아니다.

## 수동 평면 진단

`runs/office-manual-diagnostic-001`:

| 조건 | fitting 점 | 분리한 점 | 분리한 점 median / p95 |
|---|---:|---:|---:|
| 원본 pixel 평면 변환 | 6 | 8 | 4.52 / 8.67px |
| 후보 렌즈 rectification 후 평면 변환, raw로 복귀해 평가 | 6 | 8 | 4.74 / 8.85px |

proof 이미지에서 원본 창틀·지붕과 기준점 위치를 직접 대조했다. fitting residual도 약 3px로 남아 있다. 정확한 optical acceptance로 승인하지 않았다. 이 작은 영역·근사 수동점·평면 가정만으로 렌즈가 주된 실패 원인이라고 단정할 수도 없다. 광학 후보를 이 자료의 withheld 값에 맞춰 재조정하지 않았다.

## 동일 조건 대조와 추가 실험

front 20/21/22초, side는 각각 5/6/7초 뒤의 동일 18쌍을 비교했다. 매칭 함수가 호출 순서에 따라 다른 난수를 쓰지 않도록 pair마다 OpenCV seed를 고정했다.

| run | ROI | 좌측 최고 자동 inlier | 결과 |
|---|---|---:|---|
| affine-front-side-probe-004 | 새 camera-aware | 18 (front22/left29) | 연결 미승인 |
| affine-front-side-probe-005 | 기존 공통 영역 | 17 (같은 쌍) | 연결 미승인 |

좌측 proof를 검수했다. 주 대응은 같은 간판에 생겼지만 일부 배경 오대응과 좁은 분포가 있어 카메라 공통 좌표로 채택하지 않았다. ROI 제외 문제는 확인됐으나 매칭이 실질적으로 해결됐다고 판단하지 않는다. 우측 역시 미통과다. 앞의 probe-003은 pair별 RNG 고정 전 기록으로 유지한다.

`affine-front-side-probe-006`은 front22 / side29 선택만 같은 설정으로 재실행하고 전체 experiment config를 payload에 남긴 재현용 자료다. `planar-front-side-probe-002`는 수동점을 사용하지 않고 자동 seed에서 평면을 찾아 시점을 펴고 다시 SIFT를 추출했다. 양쪽 원래 pixel에서 중복 제거 후 좌측 14점에 그쳤다. 직접 proof를 확인했으며 간판 일부에 몰려 있다. 우측은 지지되는 평면이 없어 거부했다. 이 경로를 성능 개선으로 채택하지 않는다. 초기 probe-001과 실패 실행도 수정/삭제하지 않았다.

## 이번에 고정한 검사

- 상단의 실제 정적 기준점을 보존하고 미러·기존 dynamic mask를 계속 제외하는 ROI 검사.
- 분리한 수동 점의 오차를 바꾸어도 fitting 변환은 바뀌지 않는 검사.
- 공선/검사점 없는 평면 입력 거부.
- 알려진 synthetic perspective warp의 대응 복원 및 양쪽 물리 pixel 중복 제거.
- 부족한 평면 지지점 거부, cached rectification lens/ROI 설정 불일치 거부 경로.

총 51개 회귀 검사 통과. 이는 코드 검증이며 실패한 연결 gate의 통과를 의미하지 않는다.

## 다음 작업 범위

같은 쌍에서 matcher 수치 조정을 계속하지 않는다. 수동으로 확인한 정적 기준점을 여러 전방 frame에서 추적해 삼각측량하고, 지도 fitting에 쓰지 않은 리피터 관측으로 재투영을 검사한다. 이때 front 지도 자체의 구간별 유효성, 삼각측량 각도·양의 깊이·점 추적 실패를 먼저 검사한다. front 전체 지도는 아직 fragmented FAIL이므로 그 pose를 정답으로 승격하지 않는다.

재투영이 충분히 맞은 뒤에도 여러 시각에서 얻은 front↔side 상대 pose가 고정인지 별도로 검증해야 한다. clock/body origin/metric scale은 계속 unresolved다. 단일 평면/단일 시각에 맞춘 pose만으로 공통 rig를 승인하지 않는다.

재현 예:

```powershell
$env:PYTHONPATH="$PWD/src"
.venv/Scripts/python -m ct_scene manual-planar-check --package runs/calibration-drive-source-001 --masks runs/calibration-drive-masks-001 --lens-config configs/multicam-optical-candidates.json --reference configs/office-landmark-reference.json --output runs/new-manual-check
.venv/Scripts/python -m ct_scene affine-connectivity-probe --package runs/calibration-drive-source-001 --masks runs/calibration-drive-masks-001 --lens-config configs/multicam-optical-candidates.json --front-times 20 21 22 --side-delays 5 6 7 --roi camera_aware_v1 --output runs/new-camera-aware-probe
```
