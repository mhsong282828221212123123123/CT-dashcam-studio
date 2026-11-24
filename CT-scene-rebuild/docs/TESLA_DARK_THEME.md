# 첨부 사진을 참고한 다크 색상

사용자 첨부 테슬라 화면의 색감을 참고해 `render_style.theme=tesla_dark_reference`를 기본으로 적용했다. 공식 Tesla 색상값을 추출/재현한 것은 아니며 사진을 참고한 팔레트다.

- 배경: 짙은 차콜 `[0.13, 0.14, 0.16]`.
- 도로: 검은색에 가까운 `[0.035, 0.04, 0.05]`.
- 차선: 밝은 회백색 `[0.78, 0.80, 0.83]`, 중앙선은 기존 노란색 유지.
- 주변 차량: 밝은 회색 클레이 `[0.70, 0.71, 0.73]`.
- 설명 글자도 밝은 색으로 조정했다. 자차 재질, 정후방 시점, 원본 4채널, 기존 ghost 판정은 유지한다.

ground와 framebuffer 배경의 조명 밝기를 맞춰 원경에서 밝은 면이 드러나지 않도록 했다. 미리보기 `runs/tesla-dark-review-001/preview39.png`. 결과와 geometry 불변량 검사는 같은 폴더의 `review.json`에 기록한다. 변경은 렌더 색상이며 원본 영상에는 색 보정을 적용하지 않는다.

최종 `runs/oct21-tesla-dark-001/surrounding-world.mp4`: 59.9초/1,198 frames 전체 decode 완료. 직전 `oct21-handoff-002`와 actors/render-states/ego-lane-state/association-review가 byte 단위 동일하다. 검수 화면 `runs/tesla-dark-review-001/full39.jpg`에서 도로·차선·차량·글자 대비와 원본 4채널을 확인했다.

Drive: https://drive.google.com/file/d/1KBWg-NAPO9hpJ4xIRNtJvbn3NaaWqMDq/view?usp=drivesdk

후속 사용자 요청으로 기본 `camera_zoom=1.3`을 적용했다. 카메라 eye/target와 geometry는 유지하며 projection을 1.3배 확대한다(수직 FOV 52도→약 41.13도). 하단 4채널은 확대하지 않는다. `runs/oct21-zoom130-001`은 59.9초/1,198 frames이며, 직전 `oct21-actor-lanes-001`과 actor/ego/lane/identity 상태가 byte 단위 동일하다. 검수 `runs/zoom130-review-001`.
