# 도로 대비와 정후방 시점

사용자 요청에 따라 `configs/surrounding-prototype.json` 기본 렌더 스타일을 변경했다.

- 도로: 최초 중성 회색 RGB `[0.58, 0.58, 0.58]`에서, 사용자 후속 요청으로 `[0.74, 0.74, 0.74]`로 밝기를 올렸다. 클레이와의 명도 차이를 확보하며 흰 차선과 밝은 배경은 유지한다.
- 시점: 자차 좌표계 eye `[-30, 0, 30]`, target `[0, 0, 0]`. 측면 오프셋 없는 정후방 상단 약 45도.
- 접지 그림자의 외곽을 도로색에 맞춰, 어두워진 도로에 기존 밝은 타원이 남지 않도록 했다.

결과 `runs/oct21-rear-view-001/surrounding-world.mp4`: 59.9초, 1,198 frames, 하단 4채널 포함. 전체 decode 완료. 직전 `oct21-local-lanes-003`과 actors, render-states, ego-lane-state가 byte 단위로 동일하다. 따라서 위치/차로 추정/ID/고스트 판정은 이 변경으로 바꾸지 않았다. 테스트 109개 통과. 0초·39초·59.8초 화면은 `runs/rear-view-review-001/contact.jpg`, 불변량 검사는 같은 폴더 `review.json`에 기록했다.

Drive: https://drive.google.com/file/d/1IA-Hw3Js9Lgz1DHnrdua6d1vseQYSiYi/view?usp=drivesdk

후속 밝기 수정본: `runs/oct21-road-lighter-001` (59.9초/1,198 frames). `oct21-rear-reference-004`와 actors/render-states/ego-lane-state가 byte 단위 동일함을 검사했고 전체 decode가 완료됐다. 검수 `runs/road-lighter-review-001`. 새 Drive: https://drive.google.com/file/d/1VLR4VJf9vr89k57qEVTmZV26lR34hLYS/view?usp=drivesdk
