# 좌측 포켓 형상 1차 구현과 원본 검수

대상 원본은 October 21의 약 42–55초이다. 좌측 분리봉 경계가 벌어지고 대기 차로가 생긴 뒤 분리대가 다시 좁아진다. 이전 renderer의 고정 폭 도로에서는 이 추가 공간과 대기 차량이 누락됐다.

## 구현과 수정한 오류

`pocket_lane.py`는 front/left_repeater ground paint를 관측 위치·frame ID와 함께 모아 10m station 단위의 반복 근거를 계산한다. 새 차로는 바깥 경계의 벌어짐, 계속 이어지는 안쪽 분리선, 본선의 다른 분리선, 관측된 바깥 경계의 복귀가 함께 있어야 채택한다. 차량 점유 여부는 검출 조건으로 사용하지 않는다.

중간 검수에서 분리봉 반사띠를 지면 paint로 투영한 오류를 발견했다. 해당 중간 결과는 채택하지 않았다. 최종은 주황색 직립 component의 몸체를 paint에서 제외하고 밑부분 접지점을 따로 사용한다. 한 프레임에 접지점 3개 이상의 공간 일관성과 여러 frame/station의 반복을 검사한다. 원본 확인 `runs/pocket-source-review-001/base-evidence.jpg`; 반사띠 오류 이력 `outer-evidence.jpg`. 접지점을 다시 보이는 front와 left에서 모아야 폭의 근거가 되는 것이며, 높은 반사띠의 픽셀을 ground point로 쓰지 않는다.

가려진 외곽은 안쪽 분리선이 지속되고, 뒤쪽에서 외곽 근거와 복귀가 다시 확인될 때 연결한다. 최대 80m spatial support gap에도 매 10m station에서 안쪽 분리선 근거가 필요하며, 중간에 정상 폭의 노란 경계가 관측되면 이어붙이지 않는다. 이 부분은 명시적인 추정이다.

최종 raw 폭 근거는 약 1.99m로, 미측정 rig의 편향이 포함된다. 기하로 존재를 먼저 확인한 뒤 폭만 관측 75%와 nominal lane width prior 25%로 정규화했다. 렌더 폭은 약 2.37m이며 실측 폭이라는 뜻이 아니다. 이 prior만으로 포켓을 생성하지 않는다.

## 도로·차량·상태 통합

- `road_geometry.py`는 local pocket 폭을 보간한다. 이번 원본에서 얻은 profile은 world X 110→120에서 넓어지고, 220→230에서 0으로 돌아간다. 이 숫자는 원본 관측에서 생성한 값이며 production 코드에 영상/좌표/ID 분기로 쓰지 않는다.
- 렌더는 테이퍼 asphalt strip, 바깥 경계, 본선/포켓 사이 백색 점선을 만든다. 기존 '빈 lane 0을 포켓으로 간주'하는 occupancy 임시 규칙을 제거했다.
- 본선 lane ID 0..N은 유지하고 추가 포켓만 -1로 둔다. 자차가 움직인다고 본선 ID를 재번호화하지 않는다. lane-state는 위치별 경계를 사용한다.
- 포켓 안에 지속적인 접지점 근거가 있는 차량은 world에 포함한다. 반대 방향 운동은 제외하며, 본선 밖에 있다는 이유로 모두 삭제하거나 모든 바깥 차를 새 차로로 만들지 않는다.
- 원본에서 확인한 대기 차량 4대 `front-0062`, `front-0070`, `front-0078`, `front-0082`를 표시한다. 검수 fixture ID일 뿐 production 분기가 아니다. `pocket-vehicles.jpg`에서 실제 같은 방향의 대기 차량인지 확인했다.

## 검사 결과

코드 **130개 테스트 통과**. 실제 결과는 다음과 같다.

- `runs/oct21-pocket-003`: 59.9초/1,198 frames. 한 포켓의 넓어짐·유지·좁아짐과 대기 차량을 원본 4채널에 대조했다. 자차 lane ID/횡좌표, 본선 경계, 기존 actor ID와 기존 actor 위치를 보존했다. 포켓 차량 중심은 local pocket 안에 있다. 관측/ghost 충돌 0.
- `runs/sep17-pocket-003`: 추가 포켓 0개. 기존 도로/pose의 일반 정확도를 새로 승인한 것이 아닌 topology 회귀 검사다.
- `runs/oct02-pocket-003`: 터널에 허위 추가 포켓 0개. 앞서 발견한 터널 pose-to-road 불일치는 이 작업으로 고치지 않았다.
- 빈 포켓 geometry, 곡선/전체 도로 이동과의 구분, 한쪽 경계만 있을 때 거부, 정상 폭 복귀가 없을 때 거부, 중복 pixel을 반복 frame처럼 세지 않음, lane ID 보존, 반대 방향 차량 거부를 unit case로 검사했다.

재검사 도구 `tools/review_pockets.py`, 결과 `runs/pocket-source-review-001/review.json`, source/output 연락 시트 `oct21-after.jpg`, `final-contact.jpg`. 전체 export decode와 마지막 frame도 확인했다.

포켓의 존재·형상을 원본 한 사례에서 구현/검수한 단계다. 좌회전/U턴 의미 분류, 모든 형태의 빈 포켓/가림/곡선/교차로 일반 검증까지 끝난 것은 아니다. OSM/GPS로 차로를 강제 지정하지 않았고, 빌드·릴리즈·푸시는 없다. 다크테마·1.3배 확대·하단 원본 4채널은 유지한다.

Drive: https://drive.google.com/file/d/19HEYjKKw1PhhxGkMrJbsi053rNn6NN_E/view?usp=drivesdk
