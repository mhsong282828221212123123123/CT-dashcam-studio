# 우측 리피터 차량 누락 수정

최종 결과: `runs/oct21-right-fixed-006/surrounding-world.mp4`, **59.9초/1,198 frame**, 하단 4채널/클레이/고스트 유지. 검수 `runs/right-repeater-review-003`, 원본 확대 `right-repeater-review-001`. 사용자는 기존 Drive 풀버전에서 우측 차가 안 보임을 지적했다. 이전의 12개 시각 비교만으로 측면 차량 포함을 승인한 검수는 충분하지 않았다.

## 확인된 원인과 수정

- 우측 가까운 차의 bbox 하단은 바퀴 접지점이 아니라 잘린 차체/범퍼/실루엣이다. 이를 평지 ray 교차로 해석한 좌표가 도로 밖이 돼 valid detection/ID까지 제거됐다. 우측은 왜곡 보정한 top/bottom ray의 apparent height와 GLB 차체 높이 prior로 중심을 추정한다. 이 방법도 실제 치수/카메라 실측을 대신하지는 않는다. 좌측은 이번 변경에서 기존 방식을 유지한다.
- 큰 근접 side 차가 경계 밖 1m 안쪽 오차에 있으면 foreground 조건하에 기존 바깥 차로 후보로 유지한다. 작은 배경/멀리 벗어난 차는 이 예외를 받지 않는다. 이를 이유로 새 차로를 생성하지 않는다.
- 같은 차량이 car/truck 두 클래스로 검출돼 중복 ID가 생겼다. 카메라 추적 전에 같은 vehicle family의 IoU>0.75 중복을 억제하고, 심한 외관 불일치는 매칭에서 거부한다. 서로 겹치던 box가 이후 동시에 분리되면 전체 track들을 union하지 않는다.
- 화면이 전방에 치우쳐 뒤쪽 유효 차량이 잘렸다. 고정 rear elevated 시점을 유지하면서 eye [-22,-16,28], target [0,0,0]으로 후측 표시 범위를 넓혔다. 차량 월드 좌표를 화면에 맞춰 옮기지는 않는다.

## 잘못된 연결 시도 폐기

중간 001/002/004는 비슷한 색 차를 front/side 동일 차량으로 가정해 큰 contact offset을 학습하는 실험이었다. raw 확대에서 우측 K5 DL3와 전방 선행 K5 JF가 다른 차량임을 확인해 해당 결과를 폐기했다. 주석/실험 코드는 `right-repeater-review-001/rejected-offset-fusion.py.txt`에 보관하고 실제 pipeline에서는 제거했다. co-visible 색상/대략 위치로 ID를 합치지 않는다. 현재 실제 시간 순서의 handoff와 양끝 후보정만 사용한다. 단순 색상으로 identity를 증명하지 않는다.

우측 K5는 `right_repeater-0009`, 전방 K5는 `front-0001`로 분리한다. 가까운 우측 SUV/승합차/세단도 독립 근거로 복원한다. 확정할 수 없는 front/right 중복 identity는 남아 있으며, 억지 통합 완료로 보고하지 않는다.

## 검증

- 0/28/30/33/36/40/50/59.85초의 새 3D+4채널 비교, 우측 검출 box 확대, 다른 세대 K5 앞뒤 raw 대조.
- 우측 K5 28/30/32/34/36/38초 world 포함 확인. 30/36초 원본 bbox와 3D mesh를 같은 파란 ID로 표시한 `right-k5-source-world-proof.jpg` 생성·직접 확인. 두 시각의 K5 mesh 전체 vertex 100%가 출력 frustum 안에 있다.
- 자차 모든 좌표/heading이 이전 full-base와 정확 일치. 전체 1,198 frame decode/finite/최종 frame 확인. legacy 608개 SHA 일치.
- 코드 **86개 검사 통과**: family 중복 억제, 가까운 큰 side 경계 오차 허용과 작은 배경/멀리 벗어난 차 거부, apparent-height 투영의 bbox 하단 독립성, co-visible 유사 차 미통합, 서로 분리된 동시 box union 거부 등을 포함한다. 이는 실제 거리/전체 객체 정답 합격이 아니다.

Drive 새 파일: https://drive.google.com/file/d/1DEJtJoA0aljzkZCcdkTaGUpoVZMbmLbk/view?usp=drivesdk . 이전 Drive 결과는 덮어쓰지 않는다.
