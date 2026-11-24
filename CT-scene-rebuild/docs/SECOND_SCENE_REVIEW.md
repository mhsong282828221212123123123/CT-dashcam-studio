# 두 번째 날짜 주변 차량 3D

결과: `runs/oct21-traffic-clay-003/surrounding-world.mp4` — **33.0초, 1280×1000 H.264, 20fps, 660 frame**. 클레이 주변 차량/기존 재질 자차/하단 원본 4채널을 유지한다.

원본: `E:/tesla/a4/TeslaCam/RecentClips/2026-10-21_08-43-59-front.mp4`와 sibling. 다른 날짜 후보 8개와 같은 날짜 주변 구간을 raw preview로 검토했다. 주차/완전 정지 후보는 제외하고 도시 도로의 정차→출발/자차 횡이동 구간을 선택했다. 큰 곡선 구간(08-41-58)은 현재 직선 corridor prototype의 승인 범위 밖이며 처리한 척 하지 않았다. 새 source selection을 곡선 일반화나 독립 blind 평가로 세지 않는다.

35초 4채널 700 raw frame에 동일 YOLO11x DirectML을 실행했다. CUDA 없음. 동일 렌즈/장착/차폭/추적 설정이며 source-ID별 좌표 패치는 없다. 선행차 asset은 원본 세단에 맞게 지정하고 frozen base에 asset 종류를 기록하도록 수정했다. 출력 범위는 현재 base의 중앙 선행차 추적 종료에 따라 약 33초다. 분석 원본의 35초 전부를 출력한 것은 아니다.

## 원본 비교에서 수정한 점

1. 첫 실행은 전방 중앙선이 차량에 가려져 2차로 기본값을 사용하고 왼쪽 차량을 제외했다. 좌측 리피터의 동적 검출 마스크 밖 노란 지면 픽셀을 공통 W로 옮겨, 5개 이상 프레임에서 지속되는 조밀한 후보로 경계를 보완했다. 최종 31개 프레임이 5.17m 경계를 지지한다. 프레임 수만 우선하면 배경의 희박한 노란 픽셀 7.71m 후보가 잘못 선택되는 문제도 픽셀 밀도/지속성 조건으로 수정했다. 카메라 보정은 추정값이므로 이 거리의 실측 정확도를 주장하지 않는다.
2. 가려져 좁은 조각만 보이는 side 차량을 온전한 ground contact처럼 투영해 여러 차량이 겹쳤다. 영상 폭의 10% 미만이면서 폭/높이<0.7인 지속 side 조각은 신뢰할 접지점이 없는 것으로 제외한다. 모든 검출을 억지로 world 차량으로 만들지 않는다. 이 규칙은 이전 샘플의 부분 트럭도 제외한다.
3. 정지 선행차 바로 앞을 교차로로 가정해 잘못된 넓은 도로 패치를 만들었다. 정지 queue prior는 영상의 가로 표식 증거가 3개 이상 프레임에 있을 때만 허용한다. 이번 구간은 그 증거가 없어 교차로를 생성하지 않는다. 이는 교차로 인식 완성이 아니다.

## 검수

`runs/oct21-traffic-review-001`에 8개 시각의 3D/4채널 비교 이미지와 review.json을 남겼다. 0/25초 front/left 검출 확대도 대조했다. 660개 전체 frame decode, 전 actor 좌표/방향 finite, frozen 선행차 위치 완전 일치, 기존 보관 원본 608개 SHA 일치를 확인했다. 표시 차량은 시점별 6~9대, 전체 world 후보 ID는 14개다. ID 전체가 정답이라는 평가는 아니다.

코드 **75개 통과**. 새 검사는 희박한 지속 배경보다 조밀한 ground paint를 선택하는지, 한 프레임의 큰 노란 패치를 경계로 승인하지 않는지 확인한다. 이전 October 15 샘플도 `pilot-clay-regression-001`로 다시 렌더했다. frozen lead와 실제 front→right SUV 연결이 유지됨을 확인했다.

한계: 가려진 좌측 차량 일부/접지점이 숨은 먼 트럭의 world 배치는 제외됐다. 후방은 참고 화면만 사용한다. 이번 영상에서 cross-camera 연결은 0건이며 새 인계 검증 사례로 세지 않는다. 곡선/교차로 복원, 모든 차량 identity와 거리 정확도는 미승인이다. 기존 결과·소스·모델 원본은 보존했다.

재실행:

```powershell
$env:PYTHONPATH="$PWD/src"
.venv/Scripts/python -m ct_scene surrounding-prototype --package runs/oct21-traffic-source-001 --tracks runs/oct21-traffic-tracks-001 --base runs/oct21-traffic-base-001 --lens-config configs/multicam-optical-candidates.json --prototype-config configs/surrounding-prototype.json --assets-root ../CT-dashcam-studio/assets/scene --output runs/new-oct21-result
```
