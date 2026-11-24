# 다른 날짜 전체 원본: September 17

원본: `E:/tesla/a2/TeslaCam/SavedClips/2026-09-17_14-49-52/2026-09-17_14-42-53-front.mp4`와 4채널. 9월 17일 후보 6개를 2/50초에서 비교해 정지 위주의 구간을 피하고 주행 원본을 골랐다. 약 24~51km/h 구간이며 10월 예시와 다른 날짜다. 새 blind/holdout 합격으로 분류하지 않는다.

최종 영상: `runs/sep17-full-clay-003/surrounding-world.mp4`, **59.1초, H.264/yuv420p/faststart, 1280×1000, 20fps, 1,182 frame**. 원본 front 59.0553초, 마지막 PTS 59.0274초를 전부 포함한다. 클레이/고스트/하단 4채널과 후측 포함 고정 시점을 유지한다. 오른쪽은 초반을 지나면 원본에서도 대부분 빈 구간이다.

## 같은 처리에서 추가한 범위

- 4채널 1,183 raw sample에 YOLO11x DirectML 실행. CUDA 없음. 이후 camera tracking과 전체 source ego 시간축을 계산했다.
- 탑차와 버스 asset을 renderer에 연결했다. anchor model은 raw detector의 주요 차종과 bbox 비율로 자동 선택할 수 있게 했다(`--actor-model auto --assets-root ...`). 이번 중앙 anchor는 `box_truck`, 초기 우측 객체는 `bus`다. source-ID별 차량 종류/위치 패치는 없다.
- 첫 렌더는 왼쪽 중앙선을 확인하지 못했는데도 좁은 두 corridor로 scope를 잘라 탑차·좌측 차들을 잃었다. 오른쪽 curb의 ground yellow paint를 40m station별로 모아 충분한 거리/프레임 지원이 있을 때 robust 직선 경계를 추정한다. 이 기준과 지속적으로 전진하는 front 차량의 위치로 최대 4개 corridor를 추정한다. unknown left extent는 노란 중앙선으로 확정하지 않고 흰 실선 경계로 구분한다. 교차로/차로 수 완성은 아니다.
- 오른쪽 경계 fit은 차량 actor로 자차를 이동시키지 않는다. 자신의 SEI ego는 그대로 두고 별도의 도로 방향/범위를 얻는다. 최종 road slope는 약 0.00854, right boundary intercept는 약 -5.23m인 추정값이다.
- 좌측 차량들의 bbox 하단 접지점 해석도 틀어져 가까운 여러 차가 겹쳤다. 우측에서 사용한 apparent vehicle height prior를 좌측에도 대칭 적용했다. 최종 55초에는 이전 중간 렌더의 16대가 아니라 8대가 표시되며, 측면의 접지점 오해로 뭉치던 표현이 줄었다. 물리 거리/모든 ID의 정답 승인은 아니다.

## 검수

`runs/sep17-full-review-001`에 최종 0/5/15/25/35/45/55/59.05초 3D/4채널 비교와 audit를 남겼다. 4채널 raw 0/20/40/58초도 별도 대조했다. 전체 1,182 frame decode, finite world state, 마지막 frame 차량 유지, 탑차 anchor 좌표 원본 base와 완전 일치, 초기 10초 bus world 상태 유지, 전 ego sample의 추정 도로 내부 유지, 기존 legacy 608개 SHA 일치를 검사했다.

코드 **89개 통과**. bus/box truck/car 자동 asset 선택, 긴 거리 right edge의 outlier 저항과 짧은 span 거부, 좌우 apparent-height 기하 대칭 검사를 추가했다. lane count/road shape는 추정이며 일부 가림 차량/카메라 간 identity와 후방 geometry는 남았다.

Drive: https://drive.google.com/file/d/1_gYNZ4PjUmId42hLQCUrCheaHkh_9XwP/view?usp=drivesdk . 새 파일 업로드이며 기존 결과는 보존한다.

재실행:

```powershell
$env:PYTHONPATH="$PWD/src"
.venv/Scripts/python -m ct_scene full-source-timeline --package runs/sep17-full-source-001 --tracks runs/sep17-full-tracks-001 --telemetry runs/sep17-full-sei-001 --lens-config configs/multicam-optical-candidates.json --source E:/tesla/a2/TeslaCam/SavedClips/2026-09-17_14-49-52/2026-09-17_14-42-53-front.mp4 --actor-model auto --assets-root ../CT-dashcam-studio/assets/scene --output runs/new-sep17-base
.venv/Scripts/python -m ct_scene surrounding-prototype --package runs/sep17-full-source-001 --tracks runs/sep17-full-tracks-001 --base runs/new-sep17-base --lens-config configs/multicam-optical-candidates.json --prototype-config configs/surrounding-prototype.json --assets-root ../CT-dashcam-studio/assets/scene --output runs/new-sep17-video
```
