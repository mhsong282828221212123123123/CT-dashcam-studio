# October 21, 39초 흰 차 배치 수정

사용자가 지목한 대상은 27초 검은 차가 아니라 **39초 우측 리피터의 흰 세단**이다. 기존 `wp06-wp07-oct21-004`에서는 검출/추적된 `right_repeater-0010`이 world에 있었으나, 다른 차로의 검은 차 `right_repeater-0009`와 같은 대표 차로로 강제되어 겹쳐 보였다. 대표 차로를 전체 시간의 위치 prior로 재사용한 것이 이번 결함이다.

`actor_lane_priors()`는 시간별 기하 관측을 사용한다. 경계에서 0.2m 이상 안쪽의 관측이 0.6초 이상 지속되어야 차로 근거로 채택한다. 측면은 작은 원거리 box, 잘린 box, flow 보간 box로 차로 전환을 확정하지 않는다. 온전한 detector box(폭 120px, 높이 160px 이상)를 우선하고, 첫 지속 관측 이전의 불확실한 구간은 그 미래 관측으로 후보정한다. 이 수치는 1280×960 현재 입력의 경험적 신뢰도 기준이며 카메라 실측/일반적인 정확도 보장은 아니다. 이후에도 지속적인 명확한 근거가 있으면 차로 전환을 허용한다. 특정 영상·시간·ID를 production 분기로 쓰지 않는다.

첫 수정은 원거리 투영의 드리프트를 실제 차로 이동처럼 해석했다. `oct21-local-lanes-002`의 37–40초를 시각 검수해 이 문제를 발견하고 위 미래 관측 정책으로 다시 수정했다. 해당 중간본은 사용자에게 전달하지 않았다.

최종:

- `runs/oct21-local-lanes-003/surrounding-world.mp4`: 59.9초, 1,198 frames, H.264, 하단 4채널.
- 회귀 `runs/sep17-local-lanes-002`: 59.1초, 1,182 frames.
- 검수 `runs/local-lane-review-001/review.json`, `oct21.jpg`, `sep17.jpg`.
- 재실행 감사: `tools/review_local_lanes.py`.

흰 차의 추적 구간(32.41–41.60초)에서 차로 prior가 일관되며, 39초 검은 차와 서로 다른 차로로 분리된다. 수정 전후 자차 lane-state, road, identity edges, ID 목록, frozen lead samples와 종방향 위치 보존 검사가 통과했다. 모든 export frame을 decode했고 현재 source-visible인 객체가 ghost가 되는 충돌은 없다. 코드 테스트 109개 통과. 검수 이미지의 사건 구간을 원본 4채널과 대조했다.

이는 지적된 배치 결함 수정이다. 다른 차량의 위치 편향/겹침, 미연결 ID, 교차로 topology, 미측정 rig의 한계까지 해결한 결과가 아니다. 차로 값은 실제 운전 행위의 확정 판정이 아닌 추정 기하 상태다. OSM/rear geometry는 이번에 추가하지 않았다. 원본/이전 결과/기존 코드 보존, CUDA/빌드/릴리즈/푸시 없음.

Drive 새 파일: https://drive.google.com/file/d/1GP4OEEASCZ3vzY2ywnYU_eE29KWEjXx0/view?usp=drivesdk
