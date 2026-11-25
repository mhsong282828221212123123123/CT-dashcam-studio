# 전달 영상 초입 비행 검수 누락 · 2026-11-25

사용자가 지적한 것은 전달한30초004 영상의 시작0~3초부터 오른쪽 차가 도로 밖에서 회전하며 미끄러지는 문제였다. 제가 이를 후반/일반 위치 문제로 잘못 짚었고, 실제 초입을 보지 않은 채 통과 수치를 앞세워 보고한 것은 검수 실패다. 004는 이 문제 해결본으로 취급하지 않는다.

## 실제 확인

전달 preview0~3초는 oct22 원본24~27초다. right_repeater-0058은 시작0초부터 보이고, right_repeater-0061도 약0.8초에 등장한다. 0058의 yaw는 원본24~27초 사이 -2.01rad→-0.11rad, 약109도 돌아간다. 실제 초입은 runs/opening-flight-audit-001/opening-0-3s.jpg에 추출했다.

기존 필터는 도로 지원이 없는 구간, 가려진/작은 상자, 충분하지 않은 반복 관측을 unknown으로 보류하면서도 그 불확실한3D 위치를 계속 그렸다. 초입의0058·0061은 이 경로로 통과했다. 원본에는 도로 밖 측면 영역 차량으로 보인다.

## 수정 범위

전방 관측으로 정상 주행을 확인한 보호 대상과 교차로 중앙을 제외하고, 측면의3D 위치가 자차 도로 범위에서 한 차로 폭 이상 크게 벗어나는데 위치를 확정할 수 없는 경우 표시를 보류한다. 실제 객체를 지운다는 판정이 아니라 위치 미확정이다. 원본4채널·분석 관측은 유지한다. 도로 안에 있는 unknown 대상은 유지한다.

출력005는 기존 차량 궤적을 고친 것이 아니라 이런 불명확한 측면 표시를 차단한 결과다. 원래의 추적·yaw 추정 문제를 전부 고쳤다고 보고하지 않는다.

## 검증

코드471검사 PASS. 전체3편3574프레임 검사 및 원본 대조13개 PASS. 기존11개 기준에 **실제 전달 초입의0058@원본24초,0061@25초 제외**를 추가했다.

source24~27초의61개 렌더 상태 모두에서 두 차량이 없는지 검사했다. 그 다음 새30초005 MP4에서0·0.5·1·1.5·2·3초를 실제 디코드해 확인했다. 자료는 runs/opening-flight-audit-001/actual-opening-after-005.jpg와 after-review.json이다.

## 남아 있는 앞차 문제

같은 구간의 frozen front-0031도 추정 거리가 흔들린다. 원본의 실제 먼 차량을 추적하는 것은 확인했으나, 약40픽셀 높이의 상자에서 지면 접점 기반 거리와 base fitted 궤적이 크게 흔들리고 있다. 예를 들어 fitted world X는24초205m→26초175m→27초187m다. original/raw 좌표가 아니라 알고리즘 추정이다. 원본 검출 상자는 runs/opening-flight-audit-001/front-anchor-source.jpg에 확인했다.

이번 변경은 앞차를 임의로 감추거나 동작을 고친 것이 아니다. 앞차 거리 안정화는 아직 미해결이다. 따라서005도 차량 운동 전체의 완료본이 아니다.

## 전달

- [초입 수정30초005](https://drive.google.com/file/d/17BA95JsbsEPWxhHEXSeHMmfVPnbCWq-2/view?usp=drivesdk): G:/3d/CT-scene-road-scope/runs/road-scope-preview-30s-005.mp4
- [oct22전체005](https://drive.google.com/file/d/1LZIXnwjEq6stnDX0SoSE_Ip_0bq_rBhb/view?usp=drivesdk)
- [교차로전체005](https://drive.google.com/file/d/1DpLN3U-gwfPHZqL1vgXR7WhLnPHhQuzD/view?usp=drivesdk)
- [고속전체005](https://drive.google.com/file/d/1LGmUUV586vcDy_cIl36n4V_MRZwMIlmL/view?usp=drivesdk)

모두 테스트26 업로드 후 크기·폴더를 확인했다. 이전004와 실패 기록을 삭제하지 않았다. codex/road-scope 브랜치이며 기존 Gemini 브랜치는 변경하지 않는다.
