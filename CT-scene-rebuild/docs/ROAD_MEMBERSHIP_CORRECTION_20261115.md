# 도로 밖 차량의 차로 강제 보정 수정

사용자가 제공 영상의 도로 밖 차량 배치를 지적했다. 원본 대조에서 실제 실패를 확인했으며 이전 방향 수정본 두 개는 품질 검수용으로 기각했다. Drive에서도 파일명에 `기각_도로밖차량오배치`를 표시했다. 파일/수치 검사 통과가 원본 일치 통과를 의미하지 않음을 다시 확인한 사례다.

## 확인된 원인과 수정

- `actor_corridor_lane`의 whole-track 대표 차로와 `actor_lane_priors`의 fallback이 관측 시점의 실제 road membership을 대신했다. 알려진 도로 구간이라는 이유만으로, 그 도로 밖 관측까지 차로 중앙으로 끌어당겼다.
- Aug20 `front-0018`, 23.3967초: 원본은 주유소 내부 주차 차량이다. 기존 원래 횡좌표 약 -19.97m가 보정 후 -5.12m로 이동해 차로 1에 들어갔다.
- Aug26 `front-0006`, 15.7904초: 원본은 건너편 접근 차량이다. 약 +19.97m가 +1.63m로 이동해 차로 0에 들어갔다. 원본 증거는 `runs/outside-to-lane-source-001.jpg`다.
- 새 `lane_prior_geometry`는 시점별 관측 차로가 제안 차로와 맞을 때만 중앙 보정을 허용한다. 도로 밖/unknown을 fallback으로 차로 안에 넣지 않는다. 시간축 흔들림 완화는 따로 수행하며, 도로의 어느 쪽에 있었는지에 대한 제약을 유지한다. 도로 밖 좌표가 실측 정답이라는 의미는 아니다.
- 단순히 원래 좌표를 유지하거나 smoothing을 끊는 후보는 움직임 불연속 때문에 기각했다. `full-001` 및 `full-003`은 제공하지 않았다.
- `full-004`는 수치와 두 원본 사건은 통과했지만, 화면 대조에서 정상 우측 차량을 도로 밖으로 밀어내는 회귀를 찾았다. 해당 candidate 역시 기각했다. `runs/outside-lane-side-regression-rejection-004`가 이 실패를 재현한다.
- 최종 `full-005`는 **독점 ID 연결로 같은 전방 기준 차량임이 확인된 리피터 구간에 한해** 기존 side 접점 오차 범위 1m 안에서 차로 보정을 허용한다. 먼 주차장/건너편 차량에는 적용하지 않는다. 원본 관측, 자차, 동결 전방 anchor는 수정하지 않는다. 제약 fit은 ID 연결 이후에만 추가 적용되며 새로운 ID 연결의 입력으로 사용하지 않는다.

## 검증 범위와 결과

- 코드 검사 294개 통과.
- `runs/outside-lane-aug20-full-005`: 전체 59.8907초 / 1,198프레임 / 움직임 불연속 0.
- `runs/outside-lane-aug26-full-005`: 전체 58.8353초 / 1,177프레임 / 움직임 불연속 0.
- `runs/outside-lane-aug25-full-005`: 정상 비교 구간 59.0018초 / 1,181프레임 / 움직임 불연속 0.
- 각 `outside-lane-*-numerical-005` 보고서 및 독립 원본 시계/마지막 5초 검사 통과.
- `runs/outside-lane-source-005`: 주유소/건너편 차량의 차로 강제 삽입 방지와 57초 정상 우측 차량 유지 통과. 기존 제공본은 동일 검사에서 실패한다. 기각 candidate004도 정상 우측 차량 검사에서 실패한다.
- `runs/outside-lane-visible-source-005`: 50/59초 두 차로, 59초 실제 우측 전방 두 차량의 solid/앞쪽/우측 차로 유지 통과.
- `runs/outside-lane-regression-witnesses-005`: 선택 사건 16개 통과. Aug25만 새 candidate 영상으로 바꿔 검사했고 나머지 7개 사건 구간은 기존 비교 기준이다. 전체 14개를 최종 엔진으로 재합성한 결과라고 보고하지 않는다.
- `outside-lane-final-contact-005.jpg` 등 실제 화면을 대조해 candidate004에서 생긴 우측 차량 회귀의 완화를 확인했다. world membership 검사는 frustum 밖 객체까지 포함한다. 수치·선택 사건 합격을 모든 객체의 화면상 위치 합격으로 확대하지 않는다.

두 수정 MP4는 테스트26에 올렸고 파일 크기 및 부모 폴더를 검증했다.

- Aug20: https://drive.google.com/file/d/1PLvF5jI75w_nJrDl7-hnIH2waSECc3vH/view — 12,117,836 bytes.
- Aug26: https://drive.google.com/file/d/1BeO5p_B61vT8kaGMKMLrN7JDk2PYZI8m/view — 11,903,120 bytes.

전체 프로젝트 합격은 아니다. 버스의 전방→리피터 연결 누락, 넓은 교차로/합류 도로 표현 및 다른 입력의 전체 원본 검수는 남아 있다. 빌드·릴리즈·푸시는 하지 않았다.
