# 실제 주변 차량 차선 변경 검증 · 2026-11-16

후속 수정: [변경 전 차로 배치 수정](./LOCAL_ROAD_EXTENT_FIX_20261116.md)에서 같은 원본 라벨·평가기의 전체 source PASS를 확인했다. 아래는 수정 전 실패를 보존한 기록이다.

## 결과 요약

실제 다른 차량이 차선을 바꾸는 source 사례를 추가했다. Aug20 검은 Kia SUV는 painted divider를 왼쪽 차로에서 오른쪽 차로로 넘는다. 같은 구간에 자차는 왼쪽으로 이동하므로 주변 차량의 실제 움직임을 고정하면 안 되는 사례다. 선택한 SUV의 이동 방향·시점·연속성·body 방향은 PASS다. 다만 변경 전 차로 배치는 source와 달라 전체 source 판정은 FAIL로 남겼다. 이번에는 검증 사례/평가기를 추가하고 현 엔진을 재실행했으며, 차량 추론 알고리즘을 새로 고쳤다고 주장하지 않는다.

## 원본과 라벨

- 원본: 2026-08-20_07-34-02, 전방 카메라. source hash를 control에 고정했다.
- 42초: 검은 SUV는 왼쪽 차로, 자차는 오른쪽 차로. 44초: SUV가 divider를 넘는 중. 47초: SUV는 오른쪽, 자차는 왼쪽.
- crossing 허용 구간 43.5~45초, completion 46~48초. 원본 간격을 고려한 대략적인 구간 라벨이며 정밀 시간 계측이 아니다.
- 이 source의 예측은 이미 본 적 있는 개발 자료다. 독립 holdout 또는 prediction-unseen 라벨로 보고하지 않는다.
- 다른 Aug26/Aug28 및 Oct02 주변 11개 clip의 raw 후보를 훑었다. 이것은 전체 연속 source 평가가 아니다. 터널 차로 축소/교차로 통과를 실제 타 차량 합류 성공으로 세지 않았으며 합류 양성 사례는 아직 미확정이다.

## 검증

- 같은 보존 엔진으로 Aug20 1198프레임, Oct02 1194프레임 전체를 생성하고 decode/전체 시계/마지막 5초를 검사했다. 합계 2392프레임이다. raw 검출/track/base를 재사용했으며 신규 모델 추론을 반복하지 않았다.
- Aug20 SUV: 43.5~48초 world presence/solid 유지, 오른쪽 변경, crossing/completion 라벨 구간 일치, 순간 이동/반전 없음, body 진행방향 일치 PASS. 해당 manoeuvre 구간의 SUV 차체 겹침 interval도 없다.
- **실패 유지**: 42초 SUV는 원본 왼쪽 차로인데 렌더 lane unknown, 자차는 원본 오른쪽 차로인데 렌더 왼쪽 차로로 판정된다. 47초의 차로는 맞다. 이동 PASS가 이 배치 실패를 면제하지 않는다.
- 평가기는 motion_gate와 source_lane_gate를 분리하고 overall_source_gate는 둘 중 하나라도 실패하면 FAIL로 반환한다. CLI도 전체 source FAIL을 성공 종료로 취급하지 않도록 했다.
- Oct02의 기존 raw-reviewed 측면 차량 6개 차로/존재/solid 검사는 모두 PASS다. 이 중 22.3~27.9초 차량은 자차 변경 구간을 포함한다. 모든 차/모든 시각의 성공을 의미하지 않는다.
- strict 전체 겹침 FAIL: Aug20 153쌍·프레임, Oct02 10쌍·프레임. 선택 SUV의 이동 PASS를 전체 영상 품질 합격으로 쓰지 않는다.
- yellow paint 마스크 범위를 바꾸거나 semantic mask를 빼는 read-only 실험을 했지만 38~44초의 독립 지면 도색 근거가 추가되지 않았다. 이 실험은 정식 기하 코드에 채택하지 않았다.
- 코드 검사 365 PASS. wrong direction/누락 객체/부분 시간창의 거부 및 움직임이 통과해도 차로 실패를 숨기지 않는 검사를 추가했다.

## 전달과 재현

- control: runs/other-change-source-controls-001.json
- motion/source 평가: runs/other-change-motion-review-002/review.json
- 음성 차로 평가: runs/other-change-negative-review-001/review.json
- 전체 시계: runs/other-change-full-validation-001.json
- 엔진: runs/other-change-engine-001, 별도 평가기: runs/other-change-reviewer-002
- 원본 대조: runs/other-change-aug20-detailed-001.jpg
- 전체: runs/other-change-aug20-full-001/surrounding-world.mp4
- 30초 검토본: runs/other-change-review-30s-002.mp4 (원본 28~58초). 대상 SUV를 3D와 전방 strip에서 녹색으로 표시했다. 도로/차량 상태는 바꾸지 않았다.
- 001 검토본은 추가 bar가 일부 대상 표시와 겹쳐 기각하고 보존했다. 002는 bar를 하단으로 옮겨 화면 확인 및 600프레임 decode를 마쳤다.
- Drive 테스트26 업로드와 바이트/부모 대조 기록: runs/other-change-delivery-001.json. Oct02 검토 업로드는 이전 고속 영상과 영상 바이트가 같은 중복본이다.

남은 우선순위는 변경 전 차로 기준과 차량 배치의 불일치 원인 확인, 실제 합류 양성 source 확보다. 차량을 임의로 lane centre에 옮기거나 source ID별 offset을 넣어 실패를 감추지 않았다. 빌드/릴리즈/푸시 및 원본 변경은 하지 않았다.
