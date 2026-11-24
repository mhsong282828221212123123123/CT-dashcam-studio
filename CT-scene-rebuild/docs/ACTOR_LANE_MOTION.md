# WP07: 주변 차량의 허위 차로 변경 억제

상태: **IN_PROGRESS**. 이번에는 검수한 차로 유지 결함을 고쳤다. 실제 주변 차량의 차로 변경을 새 실영상에서 올바르게 재현했다는 검증은 아직 남아 있다.

## 수정

`actor_lane_motion.py`는 fixed world road 좌표의 시간별 관측으로 주변 차량의 lane support와 부드러운 lateral target을 계산한다. ego lane 번호나 상대 차량을 자차 좌표의 기준으로 사용하지 않는다.

- 작은 원거리 전방 box·잘린 box·flow box로 새 차로 전환을 확정하지 않는다. 현재 1280×960 입력에서 온전한 전방 detector box는 폭 100px/높이 120px, 측면은 120px/160px 이상을 사용한다. 경험적 신뢰도 기준이며 실측 오차 보장은 아니다.
- 신뢰도가 낮아지면 앞뒤의 지속적인 명확 관측으로 지원된 차로를 유지한다. 명확한 관측이 전혀 없으면 `fallback_lane_unresolved`로 기록하며 확인된 차로로 부르지 않는다.
- 같은 camera에서 인접한 두 차로의 지속 관측이 있어야 변경 후보를 만든다. camera 전환의 투영 bias만으로 lane change를 만들지 않는다. 전환은 원시 횡좌표의 진행률을 따라 연속적인 target을 사용한다. 단위 테스트에서는 실제 변경 형태의 합성 궤적, 짧은 관측 흔들림, camera 변경, 관측자 이동을 구분했다.
- 불명확한 전방 횡좌표는 강한 기존 차로 prior로 안정화한다. 종방향 smoothing의 weight는 유지하여 이번 수정으로 앞뒤 좌표를 바꾸지 않는다. 기존 자차/도로/identity 연결도 보존한다.
- `actor-lane-state.json`에 actor별 fixed road 기준 상태·추정 사건·camera별 근거를 기록한다. 모든 기하 사건은 원본 검수가 필요한 추정이며 실제 운전 행위의 확정 판정이 아니다.

## 실제 원본 검수

October 21의 `front-0002`, `front-0012`는 원본에서 차로를 유지한다. 기존 결과는 30초 후반의 작은 box와 자차 이동 전후에 이 차들이 차로를 바꾼 것으로 표시했다. 수정 후 29–32.8초 두 차의 도로 횡좌표는 각각 3.06–3.38m, 3.15–3.52m로 기존 차로 경계 1.6–5.17m 안에 남고 lane event도 없다. ID는 검수 fixture로만 사용한다.

`runs/actor-lane-source-review-001/oct21-front-events.jpg`, `sep17-events.jpg`가 원본 비교이며 `oct21-output.jpg`, `sep17-output.jpg`는 수정 화면이다. `tools/review_actor_lanes.py`와 같은 폴더 `review.json`에 자차/도로/기존 handoff 불변, 처음 front anchor 관측 보존, 모든 actor 종방향 좌표 보존, 관측/ghost 충돌 0, 전체 export decode를 기록했다. 코드 테스트 **124개 통과**. 이 결과를 일반적인 추적/계측 정확도 합격으로 해석하지 않는다.

## 새 영상 검증에서 통과시키지 않은 결과

- August 30 `2026-08-30_16-01-49-front.mp4`: 네 채널을 새로 YOLO11x DirectML로 분석했다. 곡선에서 ego가 직선 도로 영역 밖으로 나가는 263/297개 상태가 생겨 `aug30-actor-lanes-001`은 전달 불합격이다. 자차 좌표를 도로에 강제로 맞추지 않았다.
- October 2 `2026-10-02_15-41-33-front.mp4`: 터널 원본에서 차로 유지인데 직선 도로와 월드 pose가 어긋나 자차/anchor가 좌측 차로 변경으로 표시됐다. `oct02-actor-lanes-001`도 불합격이다. 새 영상이 있다는 이유로 성공으로 보고하지 않았다.

원본 연락 시트와 새 source 목록도 같은 review 폴더에 있다. 곡선 도로/pose-to-road 문제는 WP08과 연결된 실제 일반화 한계다. 주변 차량 차로 변경의 실제 positive 사례 검증과 이 한계를 해결하기 전에는 WP07 완료 판정을 하지 않는다. OSM이나 새 도로 모델을 이번 단계에 추가하지 않았다.

## 전달

- `runs/oct21-actor-lanes-001/surrounding-world.mp4`: 59.9초, 1,198 frames. https://drive.google.com/file/d/1-IvPi5jy0R_kYuEAYmz0xWheMcYEqVnr/view?usp=drivesdk
- `runs/sep17-actor-lanes-001/surrounding-world.mp4`: 59.1초, 1,182 frames. https://drive.google.com/file/d/1rsmrdQPLl97yJK7-KPo_IUkaRrznne3e/view?usp=drivesdk

사용자 요청에 따라 앞으로 영상은 **절대 경로의 Markdown 미디어 임베드로 대화 내 재생 + Google Drive 링크**를 함께 제공한다. 기존/실패 산출물과 원본은 보존하며 CUDA/빌드/릴리즈/푸시는 없다.
