# October 21 원본 전체 시간축

원본 `E:/tesla/a4/TeslaCam/RecentClips/2026-10-21_08-43-59-front.mp4`의 전체 길이는 59.8835초, 마지막 frame PTS 59.8582초, 총 2,156 frame이다. 새 결과는 `runs/oct21-full-clay-005/surrounding-world.mp4`: H.264/yuv420p/faststart, 1280×1000, 20fps, 1,198 frame, 59.9초. 주변 차량 클레이/고스트, 자차 재질, 하단 원본 4채널을 유지한다.

## 전체 처리 변경

- 기존 단일 선행차 prototype는 해당 ID 관측이 끝나면 전체 영상도 끝났다. `full_timeline.py` / `full-source-timeline` CLI를 추가해 영상 원본 duration과 마지막 decoded PTS로 독립 ego 시간축을 만들었다. 선행차가 없어지거나 바뀌어도 전체 출력은 잘리지 않는다. 자차는 전 구간 자신의 SEI 속도/방향만 사용한다.
- 선행차 anchor는 관측 구간에만 있고 미관측 부분을 무한 연장하지 않는다. 다른 camera와 연결되면 기존 anchor 좌표를 보존하면서 그 전후 관측을 유지하도록 수정했다. 5Hz 샘플 사이의 최근접 시간 범위를 적용해 마지막 0.05초에 차량이 일괄 소멸하는 경계 문제도 수정했다.
- 원본 전체 4채널 1,197 raw sample에 YOLO11x DirectML을 실행했다. CPU 기하/오프라인 fit, OpenGL 렌더이며 CUDA 없음. 각 camera 길이가 달라 뒤쪽 reference tile은 자신의 마지막 sample을 유지하며 해당 PTS를 표시한다.

## 후반 원본 비교에서 추가 수정

처음 고정한 road +X와 실제 도로가 약 0.92도 달라 수백 m 주행 후 자차가 그려진 도로 밖으로 나갔다. 자차를 다른 차량에 맞춰 옮기지 않았다. 전방/좌측의 동적 마스크 밖 노란 지면 픽셀을 20m station별로 모으고, 충분한 시간·거리 범위가 있을 때만 road heading을 추정했다. 전방 camera 근거리 paint 우선, station별 동일 가중치, robust median slope를 사용한다. 짧은 원본이나 큰 회전에는 이 단순 직선 corridor 보정을 승인하지 않는다. lane prior와 scope, 도로 메시가 같은 heading을 사용한다.

흰 벤츠(번호판 394버 3904)는 좌측 리피터→전방으로 옮겨 보인다. `left_repeater-0001`와 `front-0016`을 실제 원본 확대에서 대조했다. 기둥 공백 중 가속해 이전 camera의 잘린 bbox 속도로만 예측하면 연결을 놓쳤다. 다음 camera의 처음 3초 운동도 역방향으로 확인하도록 보완했다. 동일 차로·FOV 경계·색상·시간/공간 조건과 모호 후보 거부는 유지했다. 관측 공백은 고스트로 표시한다. 전방의 다른 SUV `front-0025`와 혼동하지 않았다.

후반 접지점 흔들림으로 1개 corridor 차량 방향이 약 90도 꺾이는 것도 발견했다. 실제 횡단 방향 증거가 없는 same-direction corridor에서는 종방향 운동이 거의 없거나 추정 heading이 도로와 45도 넘게 다르면 road tangent를 방향 prior로 유지한다. 일반적인 주행/차로 이동 각도는 보존한다. 교차로 회전 추정 완성으로 세지 않는다.

## 검수와 범위

`runs/oct21-full-review-001`에 원본/3D 12개 시각 비교와 전체 frame decode, 시간축/finite/마지막 frame 차량 유지, 흰 벤츠 두 camera ID와 고스트 공백, frozen lead 좌표 보존 검사를 남겼다. 코드 80개 검사 통과. source/전체 timebase/후속 camera 샘플 보존/미래 운동 연결/road station evidence/방향 흔들림 회귀를 포함한다. 숫자 검사는 정확한 거리나 전체 객체 identity 정답의 승인이 아니다.

도로는 여전히 단순화된 세 corridor다. 후반 실제 차로 수 변화/교차로 형상은 완전히 반영하지 않는다. 가려진 일부 차량과 먼 접지점 미확인 차량은 빠지며 후방은 참고 화면이다. 전체 시간 출력 완료와 전체 장면 정확도 완료를 구분한다. 실패/중간 결과와 기존 원본은 보존한다.

재실행은 기존 전체 source/tracks artifact를 사용해 새 output 경로로 진행한다:

```powershell
$env:PYTHONPATH="$PWD/src"
.venv/Scripts/python -m ct_scene full-source-timeline --package runs/oct21-full-source-001 --tracks runs/oct21-full-tracks-001 --telemetry runs/oct21-full-sei-001 --lens-config configs/multicam-optical-candidates.json --source E:/tesla/a4/TeslaCam/RecentClips/2026-10-21_08-43-59-front.mp4 --actor-model ../CT-dashcam-studio/assets/scene/tesla_fsd_sedan.glb --output runs/new-full-base
.venv/Scripts/python -m ct_scene surrounding-prototype --package runs/oct21-full-source-001 --tracks runs/oct21-full-tracks-001 --base runs/new-full-base --lens-config configs/multicam-optical-candidates.json --prototype-config configs/surrounding-prototype.json --assets-root ../CT-dashcam-studio/assets/scene --output runs/new-full-video
```

Drive 업로드 완료: https://drive.google.com/file/d/1DntuZszBIROUWzEOq4Azv9_IMW2JYmgl/view?usp=drivesdk . 파일 metadata readback에서 MP4 이름/종류/크기 9,275,589 byte 일치를 확인했다. 공개 공유로 변경하지 않았다.
