# 전방·리피터 공통 지도 연결 실험

2026-11-14. 현재 판정은 **FAIL — 공통 rig 미성립**이다. 전방 단독 광학 실험의 통과를 다중 카메라 기하 통과로 확대하지 않는다.

## 입력과 재현

- 개발 영상: a3 RecentClips `2026-08-15_17-39-32`, 35초. 이전 개발에 노출된 regression 자료다.
- 원본 package: `runs/calibration-drive-source-001`; 움직이는 물체 제외 mask: `runs/calibration-drive-masks-001`.
- 후보 광학: `configs/multicam-optical-candidates.json`. front는 새 mono SfM 후보, side는 legacy 수치 seed다. 실측/제조사 보정값으로 취급하지 않는다.
- rear는 광학 검증 실패 때문에 이 기하 실험에서 제외했다.
- CPU pycolmap/OpenCV, CUDA 없음. 동적 물체 mask의 기존 DirectML 추론 결과만 입력한다.

```powershell
$env:PYTHONPATH="$PWD/src"
.venv/Scripts/python -m ct_scene multicam-connectivity --package runs/calibration-drive-source-001 --masks runs/calibration-drive-masks-001 --lens-config configs/multicam-optical-candidates.json --output runs/new-connectivity
.venv/Scripts/python -m ct_scene affine-connectivity-probe --package runs/calibration-drive-source-001 --masks runs/calibration-drive-masks-001 --lens-config configs/multicam-optical-candidates.json --output runs/new-affine-probe --side-delays 2 4 6 8 10 12 14 16 18 20 22 24
```

새 output 경로가 필요하다. 완료 artifact를 덮어쓰지 않는다. 각 카메라 상대 PTS를 검색 범위 지정에만 사용하며 센서 노출 동기로 승인하지 않는다.

## 기본 SIFT 실험: 실패

`runs/multicam-connectivity-001`, 1Hz, 1,722쌍 검색. 카메라 내부는 4초, 카메라 사이는 8초까지 검색했다. side의 12/20/28초 frame은 지도 학습에서 제외했다.

| 지도 성분 | 등록 frame | 결과 |
|---|---|---|
| 0 | front 10 | front만 연결 |
| 1 | front 23 | front만 연결 |
| 2 | left 7 + right 3 | side끼리 공유 landmark 82개, front 연결 0개 |

좌우 리피터의 가장 강한 대응 영상에서 같은 도로 표지·배경을 확인했다. 전방을 포함한 하나의 지도는 생성되지 않았다. 성분 사이에 임의 회전·이동·scale을 넣어 합치지 않았다. side holdout pose 검증과 rigid extrinsics/clock 추정은 연결 선행 조건이 실패하여 수행하지 않았다.

## 시점 변화 대응 실험

`runs/affine-front-side-probe-001`: 추정 렌즈로 먼저 rectification하고 ASIFT/RootSIFT로 62쌍을 검색했다. side 시간 지연은 2/4/6/8초였다. 최고 RANSAC inlier는 left 14, right 10개로 부족했다. 직접 proof 영상을 검사했으며 반복 가드레일·나무의 잘못된 대응이 포함되어 채택하지 않았다.

ASIFT가 같은 물리 지점에서 여러 descriptor를 만들기 때문에 양쪽 4px cell 기준으로 중복을 제거한 뒤 기하 검증한다. 중복 descriptor 32개가 물리 지점 32개로 계수되지 않는 회귀 검사를 추가했다. epipolar inlier 수만으로 진짜 공통 landmark라고 판정하지 않는다.

`runs/affine-front-side-probe-002`: side 지연을 24초까지 넓혀 142쌍을 검사했다. 최고 inlier는 left 14, right 12개였다. 양쪽 최고 proof 영상을 직접 재검수했다. left는 같은 결과였고 right는 나무·건물의 다른 지점을 연결한 오대응이었다. **확장 검색도 FAIL**이며 후보 보정값으로 채택하지 않았다. 처리 시간 약 78초는 이번 PC/42개 선택 영상의 CPU 실험 시간이며 전체 분석 성능 수치가 아니다.

결과 artifact 3개의 manifest/payload hash 검사를 통과했다. 테스트 46개를 통과했고 기존 보관 대상 608개 원본의 SHA-256 일치를 재확인했다. 이 검사는 재현성과 코드 회귀 검사이며 실패한 기하 결과를 합격으로 바꾸지 않는다.

## 다음 판정

공통 landmark 연결이 계속 불성립하면 같은 자료에서 matcher 설정만 반복해서 조절하지 않는다. front에서 본 정적 물체가 이후 side에 명확히 보이는 별도 구간을 원본으로 선정하고, 수동 대응 reference로 광학·검색 시점·시점 변화 실패를 구분한다. 상대 pose를 주변 차량 위치로 보정하지 않는다.

계획 WP03에 따라 공통 rig가 통과하기 전 WP05 다중카메라 3D 합격은 보류한다. 독립적인 WP02 정답 작성과 WP04 raw 2D 추적 개발은 진행 가능하다.
