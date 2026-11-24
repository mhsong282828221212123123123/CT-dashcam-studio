# WP06/07: 연결 선택·관측 상태·자차 차로 사건

계획서 다음 순서에 따라 차량 identity와 자차 차로 상태를 보강했다. WP06/07 전체 완료는 아니다. WP03 정밀 보정 실험으로 돌아가지 않았고 OSM/건물/신규 모델 추론은 추가하지 않았다. 기존 raw detector/track artifact로 재현한다.

## 이번 구현

`identity_graph.py`는 한 tracklet에 여러 successor/predecessor가 붙던 문제를 막는다. 양쪽에서 최우선이고 비용 차이가 충분한 후보만 채택하며, component의 동시 관측 충돌과 시간 역행을 거부한다. 앞쪽 연결과 뒤쪽 연결이 한 ID를 공유한다는 이유로 올바른 시간 순서 chain을 모호한 것으로 보지 않는다.

연결 거리와 출입 차로는 `unrefined_position_W`로 검사한다. road containment 이후 가까워진 좌표를 ID 연결 근거로 재사용하지 않는다. 외관도 잘리지 않은 detector box에서 모으며, 짧고 잘린 glimpse만으로 다른 차를 확정하지 않는다. 미래 camera에 선명한 관측 3개 이상이 필요하다. 과거가 잘린 경우에는 40개 이상 관측·8초 이상이라는 시간 근거가 추가로 필요하다. 기준은 `configs/surrounding-prototype.json`에 기록했다. 이는 완전한 차량 ReID 모델의 대체나 일반 정확도 합격이 아니다.

`lane_state.py`는 고정된 road와 ego pose에서 별도 상태를 계산한다. 도로 상대 횡좌표, 차로 membership, 기하학적 변경 시작/차량 중심의 경계 통과/차체 전체의 변경 완료를 나눈다. 시작은 이전 차로 중앙에서 0.4m 이상 이탈한 시점이며 방향지시등/운전자의 의도 시작 시각은 아니다. 완료는 길이·폭·차체 heading을 포함한 footprint 전체가 목표 차로 안에 0.12m 여유를 두고 들어와 0.6초 유지될 때다. 짧은 경계 흔들림·정지 heading noise·unknown road 구간·좌측 pocket 제거로 차로를 새로 만들거나 번호를 바꾸지 않는다. 입력 pose와 road를 수정하지 않는다.

`scene_audit.py`는 원본 관측→world 누락 경로를 기록한다. 원본에 보이는 ID와 3D 접지점 계산 성공을 분리했다. 현재 source 관측이 있는데 위치만 후보정인 차는 실체 클레이로 표시하고, source가 없는 공백만 고스트로 표시한다. 현재 보이지만 쓸 수 있는 위치가 없는 ID는 별도 `visible_unlocalized_ids`로 남기고 영상에 개수를 표시한다. 임의 좌표를 만들어 채우지 않는다. 이 집계의 분모에는 반사/배경/반대편/범위 밖/오검출도 포함돼 GT 정확도나 recall로 읽으면 안 된다. frustum visibility는 별도 검사 대상이다.

## 실제 실행과 검수

최종:

- `runs/wp06-wp07-oct21-004` — 59.9초 / 1,198 frame.
- `runs/wp06-wp07-sep17-004` — 59.1초 / 1,182 frame.
- `runs/wp06-pilot-regression-002` — 초기 October 15 front→right Palisade 사례 40.8초 / 816 frame.

당시 `right_repeater-0011`을 Sorento SUV로 기록한 것은 시각 검수 오류였다. 후속 원본 확대에서 세단임을 확인했고, 전방 재등장 `front-0060`의 번호판 56더1869가 처음 선행차 `front-0001`과 같음을 확인했다. 최종 handoff 수정에서는 이 전방→측면→전방을 연결한다. 별개의 어두운 차 `right_repeater-0009`와는 분리한다. 상세 근거와 코드 정책은 `CAMERA_HANDOFF_FIX.md`를 본다. October 21 좌측→전방 벤츠와 초기 Palisade 연결도 유지한다.

원본 front 사건 창을 보고 비교한 결과:

| 원본 | 방향 | 기하 시작 | 중심 경계 통과 | 차체 완료 |
|---|---|---:|---:|---:|
| October 21 | 좌측 | 31.99초 | 32.97초 | 34.41초 |
| September 17 | 우측 | 12.81초 | 17.42초 | 19.81초 |

이는 추정 road와 5Hz pose의 계산값이며 실제 사건 시각을 0.01초 정확도로 잰 결과가 아니다. `runs/lane-event-source-review-001`의 raw front 비교와 `runs/wp06-wp07-review-001`의 3D/4채널 사건 비교, 넓은 수동 reference window, 이후 차로 유지 결과를 대조했다. 두 사례 모두 reference window 안에 들고 완료 후 membership이 유지된다. 차체가 경계에 남았을 때는 완료로 부르지 않는 unit case도 포함한다.

September의 중간 감사에서 현재 detector/flow 관측이 있는데도 ghost였던 60개 observation 충돌을 찾았다. 최종은 두 영상 모두 해당 충돌 0개다. 이는 **연결된 tracked ID 관측 기준**의 일관성 검사이며, 아직 연결하지 못한 별도 ID 조각까지 모두 같은 차로 알아봤다는 뜻은 아니다. 실제 geometry 미확정은 남아 있다. 최종 `visible_unlocalized_ids`가 있는 render frame은 October 58개, September 766개이며 배경/먼 차/부분 관측과 추적 오류도 포함된다. 이를 숨기거나 전체 객체 정확도 합격으로 표현하지 않는다.

출력마다 `ego-lane-state.json`, `association-review.json`, `observation-world-audit.json`, 세분화된 `render-states.jsonl`이 있다. 전체 export decode·finite·마지막 frame, frozen lead 좌표 보존, ego/road 입력 비수정과 알려진 positive/negative identity 사례를 검사했다. 코드 **104개 통과**. 최종 review.json에 각 video hash·길이·검수 창을 기록했다. 기존 소스·원본·모델은 보존하며 CUDA/빌드/릴리즈/푸시는 없다.

Drive:

- October 21: https://drive.google.com/file/d/1Z5g2MMPVgtD4CMyi7dlJWyGczxaFSdWd/view?usp=drivesdk
- September 17: https://drive.google.com/file/d/14ay9wLW5bKjEeH1aZnKtMGV7W2sk7Jdf/view?usp=drivesdk

## 후속: 39초 흰 차 차로 prior 수정

`docs/WHITE_CAR_39_FIX.md`에 원인·재수정·검증을 정리했다. 전체 구간 대표 차로 고정 대신 지속적인 시간별 관측과 미래의 명확한 측면 관측을 사용한다. 최종 October 영상은 `runs/oct21-local-lanes-003`, September 회귀는 `runs/sep17-local-lanes-002`이다. 코드 109개 테스트와 `runs/local-lane-review-001`의 불변량/대상 사건 검사를 통과했다. WP06/07 전체 완료 판정은 아니다.

## 남은 순서

사용자 지시에 따른 자차 lane-change 구간 한정 보정은 `docs/EGO_LANE_CHANGE_ONLY_FIX.md`를 본다. 전후 동일 차로 근거로 주변 횡좌표를 재구성하고, 해당 구간만 도로 기준 시점을 사용해 자차 이동/회전을 보이게 했다. October 21 선행차 출력 흔들림은 2.13→0.16m, 좌/우 두 사례와 비적용 회귀를 검사했고 코드 135개 테스트가 통과했다. 일반 주행 render actor state와 자차·도로·포켓은 유지한다. 전체 자기운동/곡선 문제 해결 판정은 아니다.

좌측 포켓 형상 1차 구현은 `docs/LEFT_POCKET_IMPLEMENTATION.md`를 본다. October 21의 원본 한 사례에서 기하 분기·폭 변화·복귀와 대기 차량 4대를 검수했다. 빈 lane occupancy 규칙을 제거하고 본선 ID와 자차 좌표를 유지했다. 코드 130개 테스트 통과, September/터널 topology 회귀에서 추가 포켓 0개. 곡선/교차로 전체와 빈 포켓의 실영상 일반 검증까지 완료된 것은 아니다.

화면은 사용자 요청대로 1.3배 확대했다(`camera_zoom=1.3`). 좌측 포켓의 최초 계획은 `docs/LEFT_POCKET_LANE_PLAN.md`, 후속 실제 구현/검수는 `docs/LEFT_POCKET_IMPLEMENTATION.md`에 있다. 원본 한 사례에서 구현했으며 일반화 검증은 남아 있다.

WP07 차로 유지 결함 개선과 새 영상의 불합격 검수는 `docs/ACTOR_LANE_MOTION.md`를 본다. 기존 두 영상의 허위 주변 차량 차로 전환을 억제했고, 테스트 124개가 통과했다. 새 곡선/터널 영상은 직선 도로와 pose 불일치로 통과시키지 않았다. 실제 주변 차량 lane-change positive 검증과 곡선 일반화는 미완료다. 이후 영상 전달은 대화 내 재생과 Drive 링크를 병행한다.

후속 카메라 전환/공백 복원은 `docs/CAMERA_HANDOFF_FIX.md`에 정리했다. `oct21-handoff-002`에서 두 세단의 실제 전환을 연결하고 네 공백을 복원했다. 잘못된 SUV 검수 메모를 바로잡았으며, 코드에 fixture ID를 넣지 않았다. 세 영상 불변량/경로 검수와 120개 테스트가 통과했다.

후방 식별 보조 1차 연결을 `docs/REAR_IDENTITY_REFERENCE.md`에 기록했다. 세 영상 검수, 최종 두 차량의 두 구간 채택, 부품(휠)만 일치한 후보 제외, 테스트 116개 통과. 실제 사각 공백 복원 완료는 아니며 October 영상은 직전과 동일하다.

WP06은 모든 global identity와 사각 구간 연결이 미완료다. WP07은 주변 차량의 차로 전환, leave-one-track-out 독립 road refinement, 고속/별도 날짜 검수 범위를 더 진행해야 한다. 현재 ego lane 사건 구현만으로 전체 WP07 PASS를 주지 않는다. 이후 WP08에서 piecewise 곡선/교차로/합류와 OSM 윤곽 참고를 연결한다. OSM은 이번 결과에 사용하지 않았다. actor/road 추정치를 ego 변경으로 숨기지 않는다.
