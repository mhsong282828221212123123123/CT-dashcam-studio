# 지도 본선 중복 표시 수정 · 2026-11-15

사용자가 지도에서 가져온 주행 도로가 옆길처럼 보인다고 지적했다. context_surface가 OSM의 본선까지 주변 도로와 같은 방식으로 따로 그린 것이 원인이다. GPS 궤적의 강체 변환만으로 영상 본선과 지도 중심선이 맞는다고 가정했다.

## 변경

- 반복되는 route way 후보를 주행 본선으로 분류하고, 이미 영상에서 그린 camera corridor와 통합해 중복 asphalt를 그리지 않는다. 회전 후 다른 OSM way ID로 이어지는 본선도 포함한다.
- 지도 표시를 영상 도로 중심에 횡방향으로 맞춘다. 자차가 주행하는 특정 차로의 중심에 OSM 중심선을 붙이지 않는다. 직선 구간에서 관측되지 않는 종방향 이동이나 scale을 새로 추정하지 않는다.
- 같은 레벨의 도로만 사용한다. 실제 주변 도로는 별도로 남긴다. 전체 지도에 lane/width 태그를 넣어 차량 차로를 정하지 않는다.
- 정렬 근거가 없는 지도 context는 표시를 보류한다. 기존 camera road, car position, ego, heading, lane states는 유지한다. 교차로 연결 전체가 해결됐다는 뜻은 아니다.

## 검증

- 단위 검사 358 PASS. GPS 편향으로 옆에 놓인 지도 본선을 통합하고 실제 평행 도로는 남기는 사례, 자차가 우측 차로에 있어도 도로 전체 중심을 사용하는 사례, 회전 뒤 lane support 없는 본선 ID 연속 처리와 정차/불충분 근거 보류를 포함한다.
- Aug25 전체 1181프레임: 지도 본선 3개 ID를 중복 면에서 제외했다. 카메라/지도 도로 정렬 anchor 24개 중 23개가 지원되며 지도 표시의 횡방향 이동은 약 22.8m였다. 이것은 표시 보정량으로 실측 GPS 정확도가 아니다. 전후 화면 10/25/45/58초를 확인했고 58초에 다시 생기던 평행 본선도 수정했다.
- Oct22 전체 1188프레임: 지도 정렬 anchor의 일관성이 약 59%라 기준 70%에 미달했다. 지도 context를 보류하고 camera road를 유지한다. 이 사례를 지도 정렬 성공으로 보고하지 않는다.
- 두 결과 모두 actor 파일과 render-state 파일이 이전 결과와 바이트 단위로 동일하다. camera road도 context_surface를 제외하면 같다. 기존 strict 겹침 FAIL은 유지한다.
- 원본과 001/002 출력은 보존한다. 새 결과는 main-road-fused-aug25-full-002와 main-road-fused-oct22-full-002다. 내부 -state artifact의 MP4는 재렌더 strip 참조용 이전 영상이며 최종 표시 영상은 -full-002의 MP4다.

## 전달

- 최종 저속 전체: runs/main-road-fused-aug25-full-002/surrounding-world.mp4
- 앞부분 30초: runs/main-road-fused-preview-002.mp4 (600프레임 디코딩 확인, 하단 4채널 유지)
- 전후 화면: runs/main-road-fused-aug25-before-after-002.jpg
- 검증: runs/main-road-fused-validation-002.json
- 코드 보존: runs/main-road-display-engine-002
- 전체 Drive: https://drive.google.com/file/d/1ExCUC7NrYOrwEbXogglDaeot__qTyJxb/view
- 30초 Drive: https://drive.google.com/file/d/1R994sIkKrqb3V-LMZJ3AAXRpaLukjPcM/view

이번 수정은 지도 본선의 중복 표시와 화면 정렬을 대상으로 한다. OSM 모양으로 차량 궤적을 바꾸거나 차로를 강제하지 않았다. 남은 교차로/주변 도로 연결의 정밀 일치는 별도 범위다.
