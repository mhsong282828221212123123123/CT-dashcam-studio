# 정적 기준점의 다중 시점 기하 검토

2026-11-14. **공통 rig/body/metric/차량 3D 재현은 아직 미통과**다. 이번에 통과한 것은 수동으로 확인한 외벽 하나의 제한된 pixel 재투영 실험이다. 새 광학 후보를 앱이나 전역 프로필에 채택하지 않았다.

## 먼저 정정: 수동 기준점 001의 오류

확대 원본 재검수에서 `office-landmark-reference.json`의 target `roof_right`가 같은 물리 모서리 대신 다른 지붕 부분에 찍힌 것을 확인했다. 일부 창틀도 테두리 위치가 일관되지 않았다. 명목 3px 불확실성을 넘는 오류였다. **001을 정확한 reference로 취급해 보고한 수동 재투영 수치와 그에 따른 기하 품질 판단은 무효화한다.** 기존 파일과 실행 결과는 삭제/수정하지 않는다. 자동 SIFT 실험의 전방 연결 실패는 이 수동점과 별개다.

`configs/office-landmark-reference-002.json`은 같은 14개 ID/fit-check 분할을 유지하고 원본 확대·격자·선분을 직접 대조해 수정한 개발 reference다. 코드/pose 예측에 좌표를 맞추지 않았다. `cornerSubPix` 제안은 옆 테두리로 이동하거나 인접 ID가 같은 점으로 합쳐지는 사례가 있어 일괄 채택하지 않았다. 검토 자료는 `runs/reference-correction-review-001`에 있다.

002 역시 근사 수동 pixel이며 실측 3D/독립 journey GT가 아니다. 수정한 기준점에서 기존/새 ROI를 양쪽 모두 통과하는 수는 **0 / 5개**다. 이전 기록의 1 / 7은 001 기준의 폐기된 진단 수치다. front 상단 제외 문제 자체는 실제 이미지와 ROI에서 확인됐으며 유지한다.

002의 평면 대응 검사(`office-manual-diagnostic-002`): 6점 fitting / 8점 분리. 후보 rectification 조건의 분리점 median 약 **2.44px**, p95 약 **6.16px**다. 이 자료의 점 좌표를 다시 이 오차에 맞춰 조정하지 않았다.

## 전방 SfM을 이용한 삼각측량: 실패

새 `manual-multiview-probe`는 raw PTS frame에서 forward/backward LK, dynamic mask, 범위 검사를 수행한다. 없는 pose를 보간하거나 추적 실패점을 관측으로 복구하지 않는다. 최소 3개 유효 view로 DLT 삼각측량하고 양의 깊이·시차 1도·fit p95 3px·별도 point-check view 5px를 검사한다.

check frame은 **해당 landmark 삼각측량에서만 제외**했다. 원본 SfM의 camera pose 학습에는 포함됐으므로 독립적인 SfM heldout이라고 부르지 않는다. 모든 point는 임의 map gauge이며 미터 좌표가 아니다.

- 초기 20.4/21.0/21.4초 fitting과 22초 검사에서 14점 중 9점이 남았으나 side 재투영은 실패했다. 이 초기 run은 잘못된 reference 001을 사용했으므로 품질 판단 자료에서 제외한다.
- 구간 확대와 002 reference로 재실행한 `manual-multiview-probe-005`는 기존 `front-sfm-003`에서 11점을 남겼지만 side 분리점 6개의 median/p95가 **13.89 / 24.51px**로 실패했다. fit point도 5개뿐이다.
- 개선 ROI를 적용한 새 전방 지도 `front-sfm-004`로 실행한 `manual-multiview-probe-006`은 front 검사를 통과한 점이 4개뿐이라 side pose 검사를 승인하지 않았다.

전방 지도에 상단 배경을 남겼을 때, 지도 학습에서 완전히 제외한 4/12/20초 이미지의 query 검사는 모두 통과했다(p95 **2.25 / 2.30 / 2.52px**, 각 215/130/149개 분리 test point). 하지만 지도는 43개/25개 이미지의 성분으로 끊겼고 선택 성분은 67개 중 43개만 등록되어 전체 gate는 **FAIL**이다. 한 camera의 좋은 재투영만으로 넓은 시점 변화에 쓸 3D 깊이를 승인할 수 없었다.

`front-sfm-004`는 같은 영상에서 나온 bootstrap 렌즈를 고정했다. 해당 오래된 report의 `fixed_cross_recording_candidate`라는 문자열은 잘못된 일반 명칭이며 다른 날짜 검증을 뜻하지 않는다. 이후 코드는 `fixed_bootstrap_candidate`로 수정하고 실제 optics refinement 여부를 별도로 기록한다. 원래 artifact는 그대로 보존한다.

## 독립 평면 경로: 제한적인 조건부 통과

불안정한 SfM 점 깊이를 옮기지 않고, 확인한 외벽의 **평면 가정** 아래 전방 raw 대응으로만 평면 normal을 구하는 `plane-pose-probe`를 추가했다.

1. source anchor 이전 전방 view의 homography를 분해한다. 양의 깊이·지지점·서로 다른 3개 이상 view의 normal 일관성을 검사하고 비슷한 지지도의 다른 해가 있으면 거부한다.
2. 이전 전방 view만으로 normal을 정련한다. target pixel과 anchor 이후 전방 pixel은 normal 계산에 넣지 않는다.
3. 이후 전방 4개 시점에서 6개 fit point로 pose만 구하고 나머지 8점을 평가한다.
4. side에서도 지정한 6개 fit point만으로 pose/선택적으로 초점거리 1개를 구하고 나머지 8점을 평가한다. 검사점으로 normal·pose·초점거리를 선택하지 않는다.

최종 재현 실행 `plane-pose-probe-005`:

| 검사 | 분리한 관측의 결과 |
|---|---|
| 이후 front 21.411 / 21.604 / 21.801 / 22.005초 | 각 8점, p95 약 **0.97 / 1.34 / 1.66 / 2.46px** |
| prior side optics 고정 비교(`plane-pose-probe-003`) | 8점 median/p95 **3.69 / 9.49px**, FAIL |
| 6개 fit point로 focal 1개 추정 후 side28초 | 8점 median/p95 **2.51 / 5.44px**, 조건부 평면 검사만 PASS |

focal 후보는 약 **642.8px**(기존 side seed 약 1023.7px)이다. 검색 하한 640px에 가까워 `near_search_bound=true`이며, radial 계수는 여전히 과거 추정 seed를 고정했다. **검증된 렌즈 보정값으로 승인하지 않았다.** 후보는 run 내부 `target-optical-candidate.json`에만 저장했다.

여기서 얻은 pose는 **front21초를 원점으로 한 plane gauge에서의 side28초 pose**다. 서로 다른 시각이므로 카메라 장착 상대 변환이 아니다. body frame, 노출 clock, 실측 scale, 고정 rig 검증은 별개로 남는다.

## 시간 방향 전이: 충분한 검증 구간을 확보하지 못함

초점거리/평면을 고정한 뒤 side28초 원본에서 앞뒤 2초씩 직접 decode했다. source SHA-256과 anchor PTS를 검사하고 모든 실제 PTS를 사용했다. nominal 30fps로 재생성하거나 평균 FPS로 시간을 만들지 않았다.

- `side-transfer-probe-002`: 앞으로 **72프레임**, 조건부 point 검사 통과 6프레임(27.9947~28.1388초).
- `side-transfer-probe-003`: 뒤로 **73프레임**, 조건부 point 검사 통과 10프레임(27.7551~27.9947초).
- 두 방향의 anchor는 같은 frame이므로 독립 표본 2개로 세지 않는다.
- prior optics는 같은 규칙에서 통과 frame이 0개였으나 이것만으로 새 profile을 승인하지 않는다.

앞뒤 원본 crop/contact sheet의 첫 30프레임씩을 직접 검수했다. 전방 차량 지붕과 지주가 외벽 기준점을 가리는 사례, visible point의 추적 상실, 일부 재투영 오차 증가를 확인했다. 모든 원본에 수동 GT를 작성한 것은 아니다. 이후 frame의 flow는 자동 제안이며 완성된 GT로 취급하지 않는다.

가림 뒤 예측점으로 부족한 fit/check 수를 채우지 않았고, 전체 구간을 통과시키기 위해 요구 수(6 fit / 최소 5 check)나 pixel 기준(median 3 / p95 8px)을 내리지 않았다. 유효 coverage가 너무 짧아 **시간 전이/공통 rig 합격은 보류**했다. calibration에서는 영상에 없는 추정점을 검증 관측으로 사용할 수 없다.

## 구현·검사와 다음 gate

- 새 CLI: `manual-multiview-probe`, `plane-pose-probe`, `side-transfer-probe`; `optical-sfm --roi camera_aware_v1`.
- 물체 검출 prepass는 기존 DirectML artifact를 사용했고, 이번 기하 연산은 CPU다. CUDA 없음.
- 총 **58개** 회귀 검사 통과. 알려진 다중 시점 triangulation, 순수 회전/잘못된 pose 거부, 평면 해의 모호성, 알려진 oblique focal 복원, raw forward/backward PTS 및 source mismatch 등을 포함한다.
- 기존 앱·전역 카메라 설정·608개 보관 대상 소스는 변경하지 않았다. 빌드·릴리즈·푸시 없음.

다음 gate는 한 프레임의 fit 성공을 확대하는 것이 아니라, 여러 시각/별도 개발 날짜의 가려지지 않은 static controls로 **공유 intrinsics의 안정성과 충분한 coverage**를 검사하는 것이다. 그 전에 자동 추적 제안을 원본에서 확인하고, 잃은 점의 재검출과 실제 가림을 구별해야 한다. 고정 camera rig는 동일한 body motion 및 clock 아래에서 여러 시각의 상대 변환이 일치해야 승인한다. 현재 차량/도로 3D 구현을 이 후보에 연결하지 않는다.

최종 실행 예(새 output 경로 필요):

```powershell
$env:PYTHONPATH="$PWD/src"
.venv/Scripts/python -m ct_scene manual-multiview-probe --package runs/calibration-drive-source-001 --masks runs/calibration-drive-masks-001 --sfm runs/front-sfm-004 --lens-config configs/multicam-optical-candidates.json --reference configs/office-landmark-reference-002.json --output runs/new-multiview-probe
.venv/Scripts/python -m ct_scene plane-pose-probe --package runs/calibration-drive-source-001 --tracks runs/manual-multiview-probe-006 --lens-config configs/multicam-optical-candidates.json --refine-target-focal --output runs/new-plane-probe
```
