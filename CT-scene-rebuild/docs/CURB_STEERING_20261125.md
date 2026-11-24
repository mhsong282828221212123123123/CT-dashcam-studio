# 우측 갓길 오인과 조향각 보조 · 2026-11-25

사용자 지적: 우측 실선 밖 갓길을 차로로 늘리는 오류. 텔레매트리 조향각을 자차 회전 방향 참고에 활용.

## 원인과 변경

- 반복된 우측 실선이 있어도 좌측 끝선이 미확정이면 우측 끝선까지 버렸다. 이제 우측 경계 후보는 독립적으로 유지한다.
- 실선 밖의 추정 차량 위치가 도로 확장을 입증하고 끝선을 무효화했다. 연속 우측 경계 밖 차량 하나만으로 본선 차로를 추가하지 않는다. 이는 실선을 무조건 갓길로 분류한다는 뜻이 아니며, 반복 도색·자차와의 위치 관계를 이용하는 기존 끝선 후보 판정은 유지한다.
- 끝선 관측 누락 후 글로벌 도로 폭으로 자동 복원하며 허위 차로가 생겼다. 이제 도색 누락 자체로 폭을 늘리지 않는다. 실제 확장은 새 바깥 실선과 내부 점선의 반복 관측이 함께 필요하다.
- Nov01 고속의 후반은 카메라 실선과 자차 간격이 유지되는데 도로 기준선의 횡방향 위상이 달라진 경우다. 반복 실선으로 검증된 일정 간격 조건에서 도로 경계만 갱신하고, 차로 수·자차 위치는 유지한다. 실제 자차 차선 변경처럼 끝선 간격이 바뀌는 경우에는 이 보정을 적용하지 않는다. 미관측 사이의 간격은 양방향 보간 사전값이며 실측이 아니다.

## 조향각

테슬라 공식 dashcam.proto의 steering_wheel_angle 필드는 존재하지만 단위·부호가 명시되어 있지 않다: https://github.com/teslamotors/dashcam/blob/master/dashcam.proto

원본 wire 값만 사용하고, 같은 영상의 자차 SEI 진행방향 변화/속도로 부호와 상대 영점을 학습한다. 서로 다른 초의 표본으로 방향 일치와 오차를 확인하며, 회전 다양성 부족·속도 부족·누락·불일치는 보류한다. 같은 영상의 SEI를 이용한 내부 확인이므로 독립 실측 검증은 아니다.

적용 범위는 자차 대회전의 두 카메라 배경 회전 합의가 안정적인 조향 방향과 반대일 때 해당 구간을 SEI 증가량으로 되돌리는 보조 검사뿐이다. 조향각으로 차체 각도를 대체하거나 누적 적분하지 않고 주변 차 방향에도 적용하지 않는다.

실제 고속·램프·교차로·도심 네 편의 기존 전체 자차 시각에서 확인했다. 회전 세 편의 방향 보조 교차 확인은 통과했고 직선 고속은 회전 다양성 부족으로 보류했다. 이번 네 편에서 반대 방향 충돌은 0회이며 기존 방향은 변경되지 않았다. 따라서 조향각 추가로 이번 영상의 방향 정확도가 개선됐다고 주장하지 않는다.

## 검증 기록

- 코드 검사 420 PASS. 고스트 연결·실제 차로 확장·실선 밖 차량·관측 누락·상대 단위와 부호 반전·잘못된 시각 회전의 조향 방향 보류를 포함.
- 실제 출력 비교: runs/curb-steering-highway-compare-001.jpg. 원본: runs/curb-steering-source-front-001.jpg.
- Nov01 고속 2/30/55초 원본 기준 3차로 PASS. 수정 후 허위 자차 차선 변경 이벤트 없음. 자차 시각·좌표·방향은 전체 불변.
- Oct02 기존 측방 여섯 차량과 B필러 미래 관측 연결 PASS. 실제 차선 변경 이벤트는 유지.
- Nov01 B필러 연결·연속성·원래 차로·순간 이동 검사는 PASS지만 엄격한 직접 관측 렌더 상태 바이트 불변 검사는 FAIL. 도로 기준이 충돌 제약 후처리에 영향을 주어 18 actor-frame의 차이가 있으며 최대 약 1.07mm다. 원시 actors.json은 동일하다. 검사 기준을 완화하거나 실패를 삭제하지 않았다.
- 초기 전체 출력 검사에서 Nov01/Oct02의 잔여 footprint 겹침은 각각 3/2개 구간으로 숫자 종합 판정은 FAIL이다. 이는 무결점 전체 완성을 의미하지 않는다.
- 최종 영상과 회귀 결과, 업로드·Git 기록은 아래 추가한다. 이전 07시 종료 기록은 변경하지 않는다.

## 최종 결과

엔진 체크포인트 cd7a393, 소스 사본 runs/curb-steering-engine-002. 최종 네 편 4773프레임 전체 디코드. 고속 도로 3차로 원본 anchor 3개, 교차로 2차로 anchor 3개, 램프 3→1→3→2 anchor 6개 PASS. Oct02 측방6대와 B필러 연결 PASS. Qt 실제 재생·저장5검사 PASS. 자차 시각·좌표는 전체 불변. 교차로/램프 raw actors와 렌더 상태는 직전 버전과 동일하다.

최종 footprint 겹침 구간은 고속3, Oct02 2, 교차로1, 램프0이다. 고속·Oct02·교차로의 숫자 종합 결과는 FAIL을 보존한다. Nov01 고속의 엄격한 직접 관측 렌더 상태 불변 FAIL도 유지한다. 이는 새 갓길 기준·사각지대 연결 통과와 별개의 검사다.

- [갓길 수정 고속 전체](https://drive.google.com/file/d/1a-GXPYfVZ-swnk5k5dAygyaSH9-HhPCo/view?usp=drivesdk): runs/curb-steering-highway-002
- [기존 차선 변경 회귀](https://drive.google.com/file/d/1qqnrbkIEKTI-hK1DjE4KTdreOlCcWUtL/view?usp=drivesdk): runs/curb-steering-oct02-002
- [교차로 회귀](https://drive.google.com/file/d/1b6A9QCjSnKqo9bJcm8kVDQg8OMPB6z-e/view?usp=drivesdk): runs/curb-steering-junction-001
- [램프 합류 회귀](https://drive.google.com/file/d/1J5GaEx2ALc1HHHwAEuKp1xEIM87GZiSs/view?usp=drivesdk): runs/curb-steering-ramp-001

모두 테스트26 업로드 후 파일 크기와 부모 폴더 확인 완료. 각 경로의 surrounding-world.mp4는 하단 원본4채널을 포함한다. 해시·Drive ID·실패 포함 품질 기록은 runs/curb-steering-delivery-20261125.json, 실제 조향각 확인은 runs/steering-direction-review-002/review.json. 빌드·릴리즈하지 않는다.
