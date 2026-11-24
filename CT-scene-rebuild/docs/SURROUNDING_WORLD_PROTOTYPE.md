# 주변 차량 월드 prototype 검수

최신 결과: `runs/pilot-extended-surrounding-004/surrounding-world.mp4` (40.8초, 1280×1000, H.264 20fps, 816 frame).
원본은 October pilot의 같은 4채널이며 분석 구간을 35→41초로 확장했다. 새 820 raw frame을 YOLO11x DirectML로 분석했다. CUDA 없음.

## 이번 범위

- 자차 외 주변 차량 5대: 선행 승합차, 우측 전방 SUV와 먼 차량, 우측 접근 제네시스, 후측 부분 트럭. 모두 같은 차로라는 이유만으로 합치지 않는다.
- 전방 SUV가 33초대 화면 우측으로 빠진 뒤 35초대 우측 리피터에 나타나는 실제 사례를 global ID로 연결했다. 리피터 SUV의 중복/분할 ID는 bbox 중첩·연속성으로 통합한다. 전방 rear view와 측면/front view의 색상 차이를 고려하되 FOV 경계·시간·공간 후보가 함께 맞아야 연결한다. 유사 후보가 경쟁하면 미연결이다.
- 이 SUV의 국소 ID: front-0003, front-0013, right_repeater-0010/0011/0013. 리피터 제네시스는 다른 차이며 별도 ID로 유지한다.
- 두 관측 사이 공백은 미래 관측을 포함한 전체 경로 fit으로 보완하고 고스트로 표시한다. 한쪽 끝밖에 없는 긴 미관측 구간을 무한 생성하지 않는다.
- 노란 중앙선/흰 선 영상 증거로 비균일 차로 경계를 추정한다. 반대편 차들은 scope 밖으로 제외한다. 접근부의 빈 좌회전 포켓과 교차로 이후 2차로 표현은 단순 topology prior다. 지도 기반 교차로 복원 완료가 아니다.
- 4개 원본 화면을 하단에 포함한다. 전방·리피터만 world geometry에 사용하며 후방은 참고 화면만 표시한다. 후방 identity 연결도 아직 구현하지 않았다.

## 감독 및 수정

원본 front/right 35/40/45/50/55/59초를 대조해 실제 SUV 인계 구간을 선택했다. right 35.2/36/38/40초 검출 box 확대에서 중복 ID와 front-view 변화에 따른 분열을 확인했다. 색상이 비슷한 제네시스를 SUV로 합치는 연결은 배제했다.

첫 주변 차량 렌더의 균일 차폭 때문에 SUV가 선행차 쪽으로 붙는 문제를 비균일 영상 경계로 수정했다. 반투명 상태의 numpy bool JSON 저장 실패를 수정했다. 최종 40초 비교에서 road mesh 끝을 차량이 넘어가는 렌더 결함을 발견해 mesh 범위를 연장했다. 실패/중간 artifact는 보관했다.

`runs/surrounding-review-002`에 10개 시각의 원본/3D 비교 contact와 수치 검사를 남겼다. 816 frame 전체 decode, 모든 actor 상태 finite, 실제 인계 공백 19 render frame의 고스트 유지, 41초 base 선행차 좌표 완전 일치를 확인했다. 이전 35초 ego prefix 최대 변경은 **0m**다. 자차 계산에는 주변 차량을 넣지 않는다. 기존 608개 원본 SHA도 일치한다.

코드 검사 **73개 통과**. 신규 검사는 비균일 차로/중앙선 제외, 실제 형태의 front-side 인계와 다른 동시관측 차량 분리, 모호 후보 거부, 근접 중복 ID/외관 변화 통합, 옆의 같은 색 차 미통합을 확인한다. 이 결과는 코드 회귀 검사이며 거리 정확도나 여러 날짜 영상의 품질 통과를 뜻하지 않는다.

## 남은 한계

추정 mount/평지/차폭 prior를 사용한다. 실제 치수 계측, 후방 연결, 전체 교차로와 곡선·자차 차선변경 재현, 등화, 여러 날짜 3D 품질 검수, 실험실 UI 통합은 남았다. 1개의 실제 front→side 사례를 통과했으며 전체 multi-camera fusion 완료로 세지 않는다. 건물 정밀 보정으로 돌아가지 않는다.

## 재실행

새 output 경로가 필요하다. 프로젝트 루트 PowerShell:

```powershell
$env:PYTHONPATH="$PWD/src"
.venv/Scripts/python -m ct_scene surrounding-prototype --package runs/pilot-extended-source-001 --tracks runs/pilot-extended-tracks-001 --base runs/pilot-extended-base-001 --lens-config configs/multicam-optical-candidates.json --prototype-config configs/surrounding-prototype.json --assets-root ../CT-dashcam-studio/assets/scene --output runs/new-surrounding
```

## 클레이 재질 후속

`runs/pilot-surrounding-clay-001/surrounding-world.mp4`: 동일한 40.8초 장면을 주변 차량은 무광 회청색 클레이, 자차는 기존 재질로 렌더했다. 고스트 구분은 유지한다. actors.json/render-states.jsonl/association-review.json은 이전 004와 byte 단위 일치한다. 추론 재실행 없이 기존 검출/추적 artifact를 사용했다. 816 frame decode와 73개 회귀 검사 통과, 첫 장면 클레이 외관을 직접 확인했다.
