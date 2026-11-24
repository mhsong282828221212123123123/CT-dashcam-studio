# 카메라 기준 교차로 연결과 후미등 외형 — 2026-11-15

최종 개발 회귀: `runs/branch-orientation-suite-005`. 네 원본의 검출·추적·자차 base·road masks·telemetry는 검증된 기존 artifact를 재사용하고 scene부터 같은 동결 엔진으로 재계산했다. 원본/실패 실험은 보존한다. 모델 재추론이나 네트워크 조회는 이 회귀에 포함되지 않는다. 코드 검사 240개 통과와 별개로 아래 원본 사건을 대조했다.

## 구현과 판단 근거

- OSM graph node의 실제 이웃 방향을 읽고 카메라가 확인한 opening에 짧은 연결 방향을 표시한다. GPS로 road/ego를 등록하지 않고 OSM lanes/width를 사용하지 않는다. node의 작은 단일 선분 대신 약12m tangent를 읽는다. 여러 시각의 반복 topology·단일 높이층·방향 일관성이 없으면 표시하지 않는다. 팔의 폭과 길이는 표시용 추정이며 branch별 lane 복원이 아니다. 기존 차량 좌표를 이 팔에 맞추지 않는다.
- 인접 차도에서 관측된 지면 횡표식도 같은 교차로를 확인하는 보조 근거가 될 수 있다. 그것으로 반대편 차량을 주행 차로에 넣지는 않는다. 다른 OSM node에 대응하는 연속 횡표식을 하나의 큰 opening으로 chain-merge하지 않는다.
- 지원률이65% 미만이라는 이유로 부분적으로 유효한 곡선 reference 전체를 버리던 처리를 고쳤다. 반복 표식·지원 길이·phase alias/ego-change 보호 조건은 유지하고, 관측 공백에서는 차선 prior와 stripe를 생성하지 않는다. 공백의 route-index 보간은 관측 차선으로 부르지 않는다.
- 우측 yellow curb가 반복 관측되면 차량 후보 하나만으로 그 너머에 허위 차로를 추가하지 않는다.
- 회전의 convex scope hull 전체를 아스팔트로 칠하지 않고 실제 자차가 지나간 곡선 띠로 표시한다. 완전한 반복 T topology가 마지막 관측 자차 경로 **앞**에 있을 때만 display road의 정면 연장을 끝낸다. 실제 주행한 경로를 map으로 잘라내지 않는다.
- 중앙선 scope에 원본 픽셀의 별도 보조 검사를 추가했다. 반복된 yellow line 밖에 차체 전체가 있고, 후속 진입/횡단 단서가 없을 때 제외한다. 카메라 회전·부분 가림·미관측은 abstain한다. 원본 선분을 부정확한 지면으로 투영하지 않는다. 첫 실험은 side-view 횡단 차량까지 제외하여 미채택했다. size/contact 변화와 횡방향 이동, 후속 own-side 관측을 함께 검사하도록 수정했다. 이 검사는 road/ego 좌표를 바꾸지 않는다.
- 반복된 front paired-red-corner 관측이 있는 시각에만 주변 모델의 rear material을 붉게 표시한다. **rear red appearance**이며 brake/indicator/control 상태는 unknown이다. 발광·임의 flash phase를 넣지 않는다. 가까운 실제 관측(.11초) 밖이나 ghost에는 붉은 상태를 이어 붙이지 않는다. source frame과 반복 지원 frame ID를 저장한다.

## 검수 중 발견하여 되돌린 것

`branch-orientation-suite-001`은 own-side 횡표식만 허용하다 queue에 가린 교차로를 놓쳤고, 원래 보이던 Oct26 횡단 트럭도 제외했다. 미통과 결과를 보존하고 인접 지면 표식의 역할을 분리한 뒤 다시 검수했다. `suite-002`에서 Oct22 허위 우측 차로와 여러 node의 과도한 opening 병합을 잡았다. `suite-003/004`와 별도 turn-display 결과는 중간 기록이며 전달본은005다.

`pixel-median-probe-002`의 Oct26 front0043/0062는 side-view 차량을 잘못 제외했다. `003`은 이를 보존하지만 Oct22 접근 택시도 그대로 남겼다. `004`/최종은 횡단/접근의 단서를 분리했다. 일부 후보는 기존 metric scope에서 이미 제외되므로 pixel gate의 abstain을 해당 차량의 최종 recall 성공으로 세지 않는다.

과거 Oct22의 '10–20초 우회전' 설명은 잘못됐다. 실제 raw49/52/55/58초와 SEI를 다시 확인했다. 관측된 사건은 **52–55초 좌회전**, 이후58초의 T자 접근이다. 우회전 화살표는 이후 가능한 진행이며 clip 안에서 완료한 우회전이 아니다. 최초 설명은 기존 기록에 보존하고 새 정답으로 사용하지 않는다. 이 검수는 `oct22-raw-turn-review-001`이다.

## 최종 개발 원본/불변식 검수

| 구간 | 전체 길이 / 프레임 | 원본과 대조한 사건 |
|---|---|---|
| Oct26 |58.9초 /1,178|49.5916초 dump-truck source bbox IoU.978, 같은 front0083, 검수25프레임 모두 solid, 원본과 같은 횡단 방향. 선행차 전 구간 pose 보존|
| Aug28 |59.35초 /1,187|13.4115초 좌측 queue 흰SUV/검은sedan IoU.876/.879, 같은 ID·solid·보조 차로 유지. 자차 완료26.9949초 보존|
| Aug26 |59.95초 /1,199|기존 source의 bus/도로 회귀. camera view와 공통 actor sample 전체 보존. 전체 count 정답은 과거 교정대로0–6초만 확정|
| Oct22 |59.4초 /1,188|좌회전 곡선과 T 접근 원본 대조. 반대편 front0089만 제거; 횡단 bus0095·car0093 유지. 다른 공통 actor pose와 camera view 보존|

4,752프레임 전체 decode/최종5초를 포함한 수치 감사 후보0. 수치0은 모든 객체 recall·정밀 위치·교차로 완성의 합격을 뜻하지 않는다. `branch-suite-source-review-005/review.json`의 모든 선택 회귀가 통과했다. 4구간 모두 기존 common actor sample과 camera view가 정확히 같다(Oct22에서 제외한 source 택시는 공통 집합 밖). 이미지 지지 없이 생성된 주변 brake/indicator 상태0, ghost의 후미등 외형 상태 전달0이다. rear material7차종의 synthetic pixel/좌우·후면 위치 검사는 `surrounding-lamp-render-review-001`에 보존했다. Aug28/Oct26 실제 cue crop도 직접 확인했다.

이번 원본 대조는 개발 검수이며 독립 holdout 성공이 아니다. 곡선/합류에서 branch별 차량의 범용 연결, 실제 다른 차량의 차선 변경 정답, 모든 visible-but-unlocalized 차량의 회수, 주변 brake/indicator state, 넓은 수동 교정 UI는 남아 있다. 사람/폴은 낮은 우선순위다.

## 다음 동결 평가

`new-date-selection-006`은 기존 분석 기록이 확인되지 않은4개 날짜(8/16,8/25,9/2,9/7)의 source-only 선정이다. raw4채널 contact를 먼저 직접 보고 source-controls를 저장했다. 아파트 접근로·실제 우회전·빈2차로 도로·혼잡 교차로/측방 접근을 포함한다. 정확한 경계/사건 시각과 bbox는 일부 pending이며 sparse control을 전체 정답으로 쓰지 않는다. `new-date-suite-006`은 같은 코드/모델을 동결해 재추론한다. 이번 실행은 supplied OSM cache가 없으므로 네트워크 요청 없이 map-missing 조건이다.4날짜를6날짜 holdout 통과라고 보고하지 않는다.
