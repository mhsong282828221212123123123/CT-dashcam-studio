# 좌측 중앙선 안쪽 차량과 구간별 도로

Sep07 front/left/right20s 원본을 `runs/sep07-left-adjacent-source-001`에 확대 검토했다. 좌측 SUV 바깥의 이중 황색선, 우측 가까운 Hyundai/중간 Tesla/바깥 SUV를 확인했다. 앞 구간 차로 하한5 source label은 유지한다. 축소 contact만으로 차로 수를 단정하지 않는다.

기존 fit은 뒤 구간2.25m 황색 경계를 앞 구간의 흰 선에 상속해 실제 좌측 차량3–4.7m prior를 지웠다. max_lanes4도5차로 이상을 표현하지 못했다.

`left_median_membership.py`는 raw left camera에서 차량 바깥의 서로 다른 평행 황색선 두 개를 찾는다. body/mirror/객체 bbox를 제외하고8프레임/1.2초 이상을 요구한다. metric paint 투영/side rig fitting 없음. Sep07 SUV55프레임/7.00–21.19초. 첫 slope 가설은 evidence0으로 거부했고 source의 실제 line pair를 보고 direct pixel-order로 바꿨다.

첫 방식은 이 증거로 main divider를 거부하고 main lane을 늘렸다. 하지만 기존 Aug28의 검증된 pocket을 main lane으로 바꾸는 회귀가 나와 **미채택/수정**했다. `new-date-suite-007`/`branch-orientation-suite-009`은 그 중간 엔진의 동결 기록이며 최종 통과본이 아니다.

현재 `inside_median_pockets.py`는 main IDs/기존 pocket을 그대로 두고, source가 확인한 접근부 좌측 공간을 auxiliary band(-1)로만 추가한다. 기존 검증 pocket과 겹치면 기존 것을 보존한다. 차로의 회전 목적은 unknown이고 폭/보이지 않은 시작은 명시적 prior다. junction 앞에서 끝나며 ego/actor 위치를 역보정하지 않는다.

우측 단일 side track의 깊이 prior도 Aug28에 허위5차로를 만들어 거부했다. 이제 마지막 stripe 밖의 안정된 전진 track 두 개가 같은 source frame에서 반복해 보여야 한 outer band 후보를 지원한다. 황색 curb 너머 확장 금지. OSM lanes/width 미사용.

상한8은8차로 기본값이 아니다. 최소5차로/좌측 차량 복구/기존 pocket 보존은 targeted preview와 전체 합성에서 재검증했다. 현재 `approach-band-source-review-full-014`의 해당 도로·차량 검사는 모두 통과한다. 같은 보고서의 예전 버전 선행차 전 행 방향 동일성은 새 방향 보완에 따라 실패했으며 보존했다. 현재 base 동결 보존은 `current-eight-source-witnesses-015`에서 별도 통과했다. 코드282개 통과를 실제 source lane 합격과 혼동하지 않는다.
