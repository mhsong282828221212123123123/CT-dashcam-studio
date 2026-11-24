# 자차 차로 변경 때만 주변 횡좌표와 시점을 보정

사용자 후속 지시: 전체 자기운동 추정을 다시 개발하지 않고, **자차가 차로를 옮기는 구간에만** 문제를 고려한다.

## 확인된 원인

기존 선행차의 31–33초 도로 횡좌표가 약 2.1m 흔들리는데도, 이벤트가 없고 기존 좌표를 보존했다는 이유로 충분히 검증했다고 판단한 것이 잘못이었다. 또한 renderer는 자차를 항상 화면 중앙·회전 0으로 그리고 카메라가 자차 회전/횡이동을 즉시 따라가서, 자차가 움직일 때 주변 차와 도로만 옆으로 움직여 보이는 구성이었다.

## 제한된 수정

`ego_maneuver.py`를 추가했다. 기존 자차 lane-state에서 완료되고 인접 차로이며 manoeuvre 지속이 10초 이하인 사건만 사용한다. 시작 전 2.5초의 회전 시작부와 완료 후 1.5초의 정착부를 포함한다. 전후 heading이 6도 이상 달라지는 사건은 제외한다. 긴 곡선 구간을 lane change로 오인한 기존 터널 사건에는 적용되지 않는다. 새 detector/VO/GPS fitting은 추가하지 않았다.

- 주변 차량이 전후에 같은 도로 차로에 있으면 그 양쪽 횡좌표로 중간만 재구성한다. 한쪽만 보이는 짧은 경우는 최대 5초의 lane-hold 추정으로 구분한다. 전후 차로가 다르거나 독립적인 차량 lane-change 근거가 있으면 평탄화하지 않는다. 근거가 없으면 보정하지 않는다.
- 차량의 종방향·높이·ID·visibility는 바꾸지 않는다. 기존 frozen input base도 수정하지 않는다. 기존 anchor의 오류를 그대로 고정하는 대신 output의 해당 구간 횡좌표만 명시적으로 보정한다. `position_before_ego_maneuver`, `yaw_before_ego_maneuver`, 지원 종류를 남긴다.
- 차로 변경 중에만 카메라의 횡방향/heading 기준을 변경 전의 도로 기준으로 안정화한다. 이때 자차는 실제 SEI pose 그대로 렌더되어 화면에서 옆으로 이동하고 회전한다. 완료 후 1.5초 동안 원래 ego-follow 시점으로 부드럽게 돌아간다. 카메라의 전진은 계속 자차를 따른다.
- 보정 구간 밖에서는 원래 actor position/yaw로 보간하여, 경계에 걸친 샘플 때문에 일반 주행 render state까지 바뀌지 않게 했다. 자차 trajectory·road·pocket은 입력 그대로다.
- 원본에 보이는 차는 보정된 geometry여도 solid clay를 유지한다. 새 geometry가 실제 관측값인 것처럼 표시하지 않고 temporal reconstruction으로 기록한다.

## 실제 검수

- October 21 `runs/oct21-ego-maneuver-001`: 보정 window 약 29.49–35.91초, 실제 자차 좌측 변경은 기존 상태의 31.99–34.41초. 선행차 도로 횡좌표 범위가 **2.13m→0.16m**로 줄었다. 이는 해당 영상의 출력 흔들림 범위이며, 실제 거리 정확도를 측정한 수치가 아니다. `oct21-contact.jpg`와 원본 하단 4채널에서 자차 이동/회전과 주변 차로 유지를 비교했다.
- September 17 `runs/sep17-ego-maneuver-001`: window 약 10.31–21.31초, 자차 우측 변경 구간을 같은 방식으로 검수했다. 원본 연락 시트 `sep17-contact.jpg`.
- October 2 `runs/oct02-ego-maneuver-001`: 기존 긴 곡선/pose-to-road 불일치 사건은 보정 window 0개. 이 한계를 해결했다고 보고하지 않는다.

코드 **135개 테스트 통과**. `tools/review_ego_maneuver.py` / `runs/ego-maneuver-review-001/review.json`에서 자차 lane-state, road, identity edges, actor IDs, 모든 종방향/높이 좌표를 보존함을 확인했다. 보정 window 밖의 sample과 20Hz render actor state는 직전 pocket 결과와 동일하다. 카메라도 window 밖에서는 ego-follow이다. 전체 export decode 완료, 관측/ghost 충돌 0.

이는 검수한 자차 차로 변경 두 사례에 대한 제한된 보정이다. 모든 주변 차량의 실제 차로 변경, 모든 곡선/교차로와 camera calibration 문제까지 해결한 것은 아니다. 판정 근거가 부족한 짧은 차량 track은 미보정으로 남긴다. 다크테마·1.3배 확대·포켓·하단 4채널과 원본/기존 코드 보존을 유지한다. CUDA/빌드/릴리즈/푸시는 없다.

전달은 두 영상 모두 대화 내 절대 경로 media embed와 Drive 링크를 병행한다.

- October 21: https://drive.google.com/file/d/1xexx9NDY465weBOpyvtXjtJchZ30Ckd2/view?usp=drivesdk
- September 17: https://drive.google.com/file/d/1objrpY0uSSuKGC__ec1t7Lwp16S24gmg/view?usp=drivesdk
