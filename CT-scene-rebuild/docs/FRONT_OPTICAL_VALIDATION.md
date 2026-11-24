# 전방 다중 프레임 기하와 wheel scale 검수

## 이번 결과의 범위

첫 정체/출발 source에서 전방 static optical experiment를 통과했다. 전체 재현·거리 정확도·공통 카메라 rig의 합격은 아니다. 새 엔진으로 만든 결과이며 기존 scene 알고리즘을 import하지 않았다.

최신 결과:

- `runs/pilot-front-sfm-002`: 전방 35초, 2Hz 선택, 학습 66프레임과 완전히 제외한 4 query 프레임.
- `runs/pilot-wheel-scale-001`: 앞 접근 구간에서 맞춘 wheel scale로 이후 출발 구간의 이동량을 검사.
- 원본/마스크/SEI는 기존에 동결한 새 엔진 artifact를 재사용했다. detector 추론은 재실행하지 않았다.

## 제외 프레임 검사

query 이미지는 학습 이미지 폴더·학습 feature database·재구성에서 제외했다. query pose 계산에는 대응점 2/3만 쓰고, 나머지 1/3은 camera intrinsics와 3D map을 고정한 상태로 원본 pixel에서 검사했다.

| 원본 상대 시각 | pose fit points | 제외 test points | 중앙값 px | p95 px |
|---|---:|---:|---:|---:|
| 4초 | 201 | 101 | 0.396 | 1.831 |
| 12초 | 290 | 145 | 0.334 | 1.740 |
| 20초 | 283 | 142 | 0.290 | 1.433 |
| 32초 | 107 | 54 | 0.448 | 1.096 |

학습 66프레임 모두 하나의 선택 재구성에 등록됐다. 부수적인 작은 중복 model은 사용하지 않았다. raw query overlay를 검사했고, 이전에 보이던 긴 오대응 선과 카메라 뒤쪽 test projection은 최신 4개 query에 남지 않았다.

내부 gate는 median<=3px, p95<=8px, query별 test points>=30, 학습 등록률>=80%다. 실측 거리/상용 인증 기준이 아니다. 이 개발 source는 이전부터 본 legacy regression이므로 새 여행의 독립 일반화 평가로 계산하지 않는다. query descriptor의 실제 물리 identity를 사람이 전수 측정한 정답도 아니다.

## 광학 후보를 선택한 과정

초기 자유 최적화는 8월 주행 source를 여러 component로 나누고, 한 component의 focal을 약 107px까지 무너뜨렸다. 학습 잔차가 1px 미만이어도 FAIL이었다.

학습 데이터의 합리적인 focal 후보를 고정해 초기 등록을 수행하고, 연결 뒤 global bundle adjustment로 조정했다. 2Hz로 시간 간격을 좁혀 연결도 보완했다. 8월 source는 아직 하나의 지도에 완전히 합쳐지지 않아 전체 optical gate가 FAIL이다.

이때 학습 지도에서 나온 전방 후보 `f=1653.06777, cx=640, cy=480, k1=-0.3859065`를 **10월 pilot에 고정한 채** 검사했다. pilot별로 focal을 다시 맞춘 결과를 채택하지 않았다. 이 값은 SIMPLE_RADIAL의 개발용 추정이고, Tesla 실측 프로파일이 아니다.

8월 source의 같은 녹화 내에서 얻은 후보이고, 두 영상 모두 이미 본 개발 자료다. 평가를 반복했으므로 이 검사는 development holdout check이며 미사용 독립 평가로 광고하지 않는다.

## 대응점 검수에서 고친 것

- 3D 점당 descriptor 하나만 쓰면 반복되는 차선·전봇대 특징을 잘못 연결했다. 현재는 학습 시점 두 개 이상의 mutual descriptor vote를 요구한다.
- 정지 시점의 반복 관측 수로 과거 road mark가 압도하는 문제가 있었다. query 전후 3초의 학습 view로 제한하고, 양쪽 view가 존재하면 앞/뒤 지지를 모두 요구한다.
- 이 선택은 query pose fitting 전에 수행한다. test reprojection error를 보고 test point를 지우지 않는다.
- 선택 뒤 대응점이 부족하면 FAIL이며 threshold를 낮추지 않는다. 과거 pilot 32초의 43개 대응점 결과도 실패 이력으로 남겼다.

## Wheel/visual scale 일관성

SEI float 속도를 반올림하거나 deadband로 없애지 않고 실제 camera PTS 간격으로 적분했다. gap·시간 외삽은 거부한다. tiny negative noise도 원본 수치 그대로 보존한다.

직진 forward gear에서 이동량이 충분한 구간만 scale fitting 대상으로 삼았다. 이 선택은 저속 데이터를 0으로 바꾸는 것이 아니며 전체 속도 trace는 그대로다.

- fitting: 첫 접근 구간 7개 interval, wheel travel 약 7.858m.
- 별도 check: 출발 구간 8개 interval, wheel 적분 약 18.786m, visual camera path에 fitting scale을 적용한 값 약 18.456m.
- 두 이동량 차이는 약 **1.76%**다.

이는 wheel을 scale prior로 썼을 때 다른 이동 구간에서도 비슷한 이동량이 나오는지의 검사다. 실제 위치 오차 1.76%나 미터 정확도 검증을 뜻하지 않는다. 타이어/센서 bias, front camera lever arm, body origin, gravity와 도로 경사는 독립 검증되지 않았다.

6~26초의 reconstructed camera centre는 해당 scale 아래에서 내부 p95 spread 약 0.003m였다. 이는 비슷한 정지 영상의 내부 일관성일 뿐, 실제 3mm 정확도 주장이 아니다. ego 평활화·보정으로 만들어 낸 값도 아니며 아직 body trajectory로 채택하지 않았다.

## 실행과 환경

CPU pycolmap 4.2.1, CUDA 없이 SIFT/matching/mapping/BA를 수행했다. `requirements/windows-calibration.lock`에 wheel hashes를 고정했다. 기존 runtime은 변경하지 않았다.

CLI: `optical-sfm`, `wheel-scale-check`를 추가했다. 실제 사용한 bootstrap artifact, 해상도, mask/source hash, sampling/holdout 설정이 manifest에 남는다. 서로 다른 camera의 bootstrap과 source/lens 해상도 불일치를 거부한다.

## 아직 통과하지 않은 것

1. front/left/right 공통 rigid geometry와 clock offset.
2. body 원점과 metric/노면 좌표계.
3. 모든 frame의 dense ego pose와 미래 관측을 포함한 공백 보완.
4. 차량 3D 위치·heading·도로·최종 영상.
5. 전체 객체/차선/event GT와 새 날짜 holdout.

다음 우선순위는 전방의 안정화된 기하를 기준으로 리피터를 같은 정적 지도에 연결하는 것이다. front 결과가 통과했다고 rear나 side 기하를 함께 승인하지 않는다.
