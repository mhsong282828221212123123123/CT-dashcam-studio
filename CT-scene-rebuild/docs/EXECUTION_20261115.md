# 남은 계획 실행 기록

사용자 지시: 남은 WP06~WP11을 되묻지 않고 진행한다. 폴·사람 모델 표시는 낮은 우선순위다. 정확한 실측 계측 대신 원본과 대조해 납득 가능한 차량 운동을 목표로 한다. 빌드·릴리즈·푸시·CUDA는 사용하지 않는다.

## 이번 구현

- `road_frame.py`: 고정 W 횡좌표 대신 구간별 arc-length / 법선 좌표. 곡선 차로 판정은 local tangent를 사용한다. 기존 직선의 단위·값은 유지한다.
- `road_perception.py`: 로컬 YOLOPv2 ONNX를 DirectML/CPU로 실행하고 원본 픽셀 공간의 road/lane PNG를 보존한다. 실제 provider kernel 집계와 모델 hash를 기록한다. 터널 벽 무늬가 표식으로 들어오던 경로를 차단했다.
- `curved_road.py`: 반복 관측된 영상 표식에서 reference line을 만든다. 자차 경로는 검색 구간 인덱스일 뿐이고 표식 offset 근거가 필요하다. 차선 변경이 표식 phase alias로 사라지는 후보를 거부한다. 부족한 구간은 선을 확정하지 않는다. 차량으로 보완한 외곽 범위는 독립 표식 근거와 구분한다.
- `osm_context.py`: OSM XML/Overpass JSON을 읽고 공유 node와 layer별 연결 후보를 찾는다. 지도 lanes/width tag는 사용하지 않는다. 명시적 작은 bbox 조회와 캐시를 제공한다. 실제 OSM API로 October 21 / September 17 / August 30 세 캐시를 받았다. 각 context의 역할은 topology 참고이며 자차/차로 좌표를 역수정하지 않는다.
- `junctions.py`: 자체 회전 + 영상 표식 공백 + 반복 지도 후보 또는 영상 횡표식으로 opening을 만든다. 표면은 추정이며 전체 branch의 실측/지도 정합은 미평가다. 교차로 안의 횡단 차량에는 평행 차로 제약을 풀었다.
- `actor_heading.py`: 횡단 차량의 정지 후 첫 이동에 observer yaw가 갑자기 적용되던 현상을 미래 운동 근거로 교정한다. 정차 jitter만으로 방향을 새로 만들지 않는다.
- 같은 카메라에서 처음에는 박스가 겹치다가 나중에 서로 다른 차로 갈라지는 tracklet을 병합하던 결함을 고쳤다. 터널 마지막 2초의 21개 불연속 후보가 재검사에서 0개가 됐다.
- 전방의 지면 접점이 horizon/경사 때문에 미확정일 때, 잘리지 않은 현재 detector box의 차체 높이를 제한적으로 보조한다. 기존 frozen sample은 그대로 보존하고 누락된 geometry만 추가한다. 이는 크기 prior이며 실측 거리의 대체가 아니다.
- `pipeline.py` / `analyze`: 전체 원본→SEI→4채널 검출→추적→자차→도로 모델→OSM 참고→전체 출력→수치 감사. 완료 stage 검증 후 재사용하고 미완료 work는 보존해 이어간다. 입력/모델/config/코드가 달라지면 같은 작업으로 조용히 재사용하지 않는다.
- `review_app.py`: 독립 Qt 창의 전체 분석·취소·이어하기·결과 열기·재생·seek·MP4 저장. 분석은 격리 Python subprocess다. 기존 편집기에는 `실험실 / Lab → 주행 재현 v2 / 분석·검토·MP4` 실행 어댑터만 추가했다. 변경 전 UI는 `../legacy-archive/ui-adapter-20261115/main_window.before-v2.py`에 보존했다.

## 확인한 증거

- 코드 검사 160개 통과(후속 수정 시 다시 확인).
- `runs/review-app-smoke-001`: 실제 59.25초 MP4의 Qt decoder 47개 유효 frame / 재생 시각 진행 / MP4 byte 동일 내보내기, 한글과 영상이 나오는 창 PNG. offscreen 동작 검사이며 수동 사용성 합격과는 구분한다.
- `runs/oct02-whole-export-review-002`: 1,185 frame, 수치 불연속 후보 0. 원본 관계 전체 정확도 평가는 아니다.
- `runs/aug22-whole-export-review-004`: 실제 회전 59.3초 / 1,186 frame, 직전 4개 방향 급변 후보 → 0. 원본 contact 비교에서 추가 surface 수정도 수행했다.
- `runs/*-milestone-002`: October 21, September 17, August 30, October 2의 전체 약 59초 결과. 각 `*-milestone-review-002`에서 전체 decode·마지막 5초·수치 discontinuity 검사 통과. 이 중 August 30의 원거리 geometry 누락은 추가 수정 중이다.
- `runs/aug30-height-preview-001`: 0/15/30/40/50/59초 원본 4채널 비교. 미확정 접점 때문에 빠졌던 두 전방 ID가 40초 world에 존재한다. 먼 차량은 사용자가 정한 확대 시점 밖일 수 있다. preview는 전달용 전체 영상이 아니다.

## 남은 검증과 기능

전체 WP 완료나 일반화 합격은 아직 아니다. 계속 진행할 항목:

1. 높이 보조 후 전체 구간 재검사 및 기존 handoff/ego-change/pocket 회귀의 원본 대조. 실패 시 같은 원인 최소 사건으로 돌아간다.
2. 실제 주변 차량 lane-change positive, 추가 교차로·합류·직진 통과·야간/비/고속, 새 날짜 보류 평가. 현재 5개 개발 날짜만으로 독립 holdout 합격을 주지 않는다.
3. sparse 3D preview나 decoder 성공을 객체 recall로 사용하지 않는 독립 relation/visibility 평가. 원본에서 중요 대상과 사건을 먼저 작성한다.
4. 자차 SEI 등화와 모델 등화 그룹 연결, 주변 차 등화는 영상 근거가 없으면 unknown.
5. Qt 취소/이어하기와 편집기 adapter 실행의 추가 검사. manual correction layer는 자동 점수와 분리한다.
6. 결과 영상은 바로 재생할 로컬 MP4와 `테스트26` Drive 업로드를 함께 전달한다. `VIDEO_DELIVERY.md` 참조. 다섯 전체 개발 영상의 업로드와 metadata readback을 마쳤다. 목록은 `DELIVERY_20261115.md`다.

후속: 코드 161개 통과. Qt 실제 영상 표시/한글 font, 작업자 취소 후 컨트롤 복귀, 편집기의 독립 실행 인자/선택 원본 전달 검사를 수행했다. 전방 보조 크기로 기존 frozen 샘플을 보존하면서 누락 geometry를 보완했고, 잘린 세단을 truck/van으로 결정해 모델 크기가 잘못 적용되던 경우를 수정했다. 실제 source에서 차체가 잘린 경우의 class prior임을 따로 기록한다.

자차 등화는 후속 구현에서 `tesla_ego_model3.glb`의 별도 BrakeLight / TurnSignal / ReverseLight node를 SEI에 연결했다. 원래 차체 좌표 원점을 모든 부분 mesh에 유지한다. 브레이크는 brake_applied proxy이며 방향지시등 점멸 위상은 시각적 재현이다. 주변 차량의 등화는 미확정이다. `runs/ego-lamp-render-review-001`의 실제 렌더 픽셀 변화와 좌우 mesh 범위를 검사했다.

실행 경로: `Launch-Scene-v2.ps1`. 새 창은 prototype이며 metric 정확도는 미평가다. 원본·기존 엔진 알고리즘·기존 결과는 변경하지 않는다(기존 UI에는 실행 어댑터만 추가).

## 추가 날짜 원본 대조와 수정

`runs/new-date-suite-002`는 코드·모델·설정·asset을 고정한 8개 날짜 전체 분석이다. 도로 6개(주간 4, 야간 2), 주차 환경 음성 사례 2개다. 8개 모두 실행은 완료했으나, 그것을 시각 품질 합격으로 집계하지 않았다. 원본 5 Hz crop 545개를 먼저 직접 확인하고 네 핵심 객체에 두 시점 수동 bbox와 관계 라벨을 만들었다. `runs/dense-source-controls-001`에 저장했다. 자동 검출로 정답을 만들지 않았다.

평가기는 첫 수동 bbox에서 global ID를 한 번 고정하고 나중 bbox로 ID 유지 여부를 검사한다. 보이는 차를 다른 ID로 바꿔 점수를 얻을 수 없다. 원본 SHA, solid 유지율, 최장 누락, 진행 방향, 선행 관계, 자차/차량 차로와 명확한 단일 차로 수를 검사한다. 안정화된 화면 카메라 위치를 자차 위치로 쓰지 않고 원래 ego trajectory를 사용한다. full-rate 전체 객체 recall/실측 정확도 검사는 아니다.

동결 baseline 결과(`runs/source-controls-baseline-001`):

| 원본 핵심 대상 | Baseline 결함 | 수정 결과 |
|---|---|---|
| Sep 08 갈색 선행차 30–39.8초 | 검출 bbox는 있지만 우회전 뒤 직선 corridor 밖으로 제외, solid 0% | 곡선 reference를 막던 허위 자차 변경 보호를 제거. 해당 구간 ID·solid·동일 차로·전방 관계 100%, 단일 차로 확인 |
| Oct 26 흰 택시 20–58.5초 | solid 100%이나 자차와 택시를 C1에 배치 | 반복 yellow divider 너머 white paint를 우리 차로로 세지 않도록 수정. C0 관계 100% |
| Sep 20 녹색 선행차 10–50초 | 대상 유지됐지만 도로를 2차로로 생성 | 무조건 추가하던 외곽 stripe 제거. 해당 단일 차로 관계 100% |
| Sep 14 좌측 BMW 30–50초 | 선택한 대상의 ID·solid·진행 방향 baseline 통과 | 추가 원본 범위/다른 객체 정확도는 미평가 |

위 세 실패 사례는 수정에 사용했으므로 이제 개발 사례다. 독립 holdout 합격으로 주장하지 않는다. Baseline·실패 렌더·원본은 모두 보존했다.

추가 수정:

- 실제 위치는 바꾸지 않고, 정차 부근 접점 속도 부호가 바뀌어 차체를 180° 돌리던 orientation 축 모호성을 연속 시간으로 해소했다. 점진적인 U-turn과 긴 미관측 간격은 유지한다.
- side 영상의 매우 좁고 세로로 긴 가장자리 조각을 전체 차 높이로 해석하지 않는다. 같은 offline track에 나중의 온전한 차체 관측 3개가 있으면 유지한다. Sep 20 마지막 구간에서 잘못 자차 옆으로 끌어온 조각을 제거했다.
- Aug 17 / Oct 26의 작은 GPS bbox를 OSM API에서 각 1회 받고 캐시했다. 지도는 nearby topology의 느슨한 확인만 한다. 반복 횡표식의 카메라 W station으로 직진 교차로 opening을 만든다. map-only / paint-only 후보는 확정하지 않는다.
- 교차로 표면 안에서 횡으로 움직이는 차량은 중앙을 지나더라도 평행 차로로 먼저 분류하지 않는다. `runs/oct26-junction-repair-001`에서 횡단 track 8개가 평행 차로 제약을 벗어났다. 가려진 횡단 트럭 누락과 전체 branch 폭/연결은 아직 해결되지 않았다.
- pipeline 상태 저장의 Windows transient replace 실패에 유일 temp와 제한된 재시도를 추가했다. 같은 output의 동시 실행은 OS lock으로 막는다. 실제 중단·같은 동결 코드 재개 검사는 `tools/check_pipeline_resume_e2e.py`에서 별도 수행 중이다.

코드 검사 180개 통과. Sep 08 804 frame / Sep 20 1,184 frame / Oct 26 교차로 1,178 frame / Aug 17 교차로 1,193 frame을 전부 decode했고 최종 5초 포함 수치 불연속 후보는 0이다. 선택 객체 대조와 수치 검사는 구분한다. 주차장에 도로를 그리는 fallback, 새 holdout, 실제 주변 차 변경 positive, 가림 횡단 차량, 수동 보정 UI는 남았다. 폴·보행자 모델은 후순위다.

`runs/new-date-suite-003`은 위 8개 날짜의 개발 회귀 검사다. 초기 OSM 두 호출에서 runner가 잘못된 CLI flag를 사용한 기록을 보존하고 `tools/retry_suite_osm_cli.py`로 그 두 호출만 복구한다. OSM 자료나 모델/분석 설정을 바꾸는 재시도가 아니다.

## 후속 검수: 짧은 횡단 트럭과 정차 방향

- 8개 날짜 회귀가 완료됐다. 원래 CLI 거부 2건과 복구 로그는 따로 보존했고, 유효 결과 8개 모두 전체 수치 감사 후보 0이다. `runs/source-controls-development-regression-001`의 네 선택 객체 관계도 통과했다. 주차장 형태와 모든 미라벨 객체의 정확도를 이 네 객체로 보증하지 않는다.
- `runs/resume-e2e-001/review.json`: 실제 전체 분석을 source/telemetry 발행 후 강제 중단하고 동일 동결 코드로 재개했다. 두 발행 artifact ID가 동일하고 verified cache가 재사용됐으며 전체 MP4까지 완료했다. 모의 sleep 작업 취소와는 별개의 실제 engine 수명주기 검사다.
- Oct 26 49.0–50.2초의 덤프트럭은 detector score .81–.94로 7회 관측됐지만 최소 8표본 규칙 때문에 버려졌다. 짧은 전방 track은 반복 고신뢰 관측과 지원된 교차로 내부의 횡단 변위가 함께 있을 때만 통과시켰다. 추론 gap만으로 짧은 track을 승인하지 않는다.
- 트럭 두 온전한 측면 관측 + 반복 class 7 근거가 있으면 heavy-truck clay proxy를 쓴다. 측면 차체의 깊이 inset은 반 길이가 아니라 반 폭을 사용한다. 실제 dump body와 같은 mesh라는 뜻은 아니다.
- `runs/oct26-short-crossing-source-review-002`: 원본 49.5916초 수동 bbox와 IoU .978, 같은 고정 ID, 49.0–50.2초 출력 25표본 solid 유지, 원본과 같은 좌→우 진행, 차체 앞 방향과 운동 방향 일치. 원래 frozen 선행차 표본은 그대로다. 선택한 개발 횡단 사건만의 검사다.
- 정차 첫 관측의 차체 방향이 회전 전 W-X 축에 머물던 별도 결함을 고쳤다. 현재 구간 local tangent로 초기화한다. 옆 흰 차량의 50초 local tangent 오차는 약 39.9°→0°이며 모든 차량 위치 표본은 수정 전과 동일하다. 원본 확대 및 수정 렌더 49.6초를 직접 대조했다.
- 최종 `runs/oct26-short-crossing-full-002`는 58.9초 / 1,178 frame 전체 decode, 마지막 5초 포함 수치 후보 0이다. 테스트26 업로드와 metadata 크기 확인을 마쳤다. 코드 검사 182개 통과.

새 holdout 후보는 `new-date-selection-003`과 `005`의 6개 날짜다: Aug 20 / Aug 28 / Sep 09 / Aug 26 / Oct 18 / Oct 22. 주간 다차로, 자차 변경 후보, 선행 버스, 단일 차로 loop ramp, 좁은 교차로 우회전과 역광이다. 6개 raw 4채널 contact를 먼저 직접 보고 `runs/new-holdout-source-controls-001/source-controls.json`에 기록했다. 자차 변경 후보의 정확한 시각은 아직 확정 라벨이 아니다. 영상 내용으로 결과를 본 후 선택하지 않았다. 원본 hash와 지역 OSM 캐시를 확보하고 `runs/new-holdout-suite-001`에서 같은 동결 엔진을 실행 중이다. 상세 raw anchor/연속 가시성 라벨을 prediction 내용 열람 전에 고정해야 하며, 현재 새 holdout 합격 수는 0이다.

## 새 날짜 검증에서 추가로 드러난 실패

새 6개 원본의 10개 확대 anchor, 705개 5 Hz 대상 crop, 두 자차 변경 raw sheet를 먼저 확인했다. `new-holdout-dense-controls-001`에는 선택 차량 다섯 대의 source-only 라벨을 동결했다. Aug 20의 전체 차로 수를 전방의 가려진 부분만으로 3이라고 잘못 적은 것은 우측 원본 확대에서 발견해 4로 교정했다. 가까운 Sonata와 더 바깥 curb lane의 Prius가 구분된다. `new-holdout-dense-controls-002/annotation-correction.json`에 원래 hash와 교정 근거를 보존했다. 최초 count 평가는 무효이며, 최초 선행차 자체를 제외하던 실패는 count와 무관하게 유효하다. 이 자료를 독립 holdout 성공으로 집계하지 않는다.

`new-holdout-suite-001` 원래 실행은 3개 완료/3개 runtime 실패다. 실패 결과와 동결 코드는 유지한다.

- Aug 28: rear SIFT RANSAC이 finite H와 inlier 0개를 반환했는데, 빈 convex hull의 면적을 먼저 계산해 OpenCV 예외가 났다. 최소 inlier/ratio 검사 후 hull을 계산하도록 고쳤고 zero/2-inlier 회귀를 추가했다.
- Oct 18 / Oct 22: 한 MP4 packet 안의 여러 SEI가 같은 PTS를 가지며 속도만 극소량 달랐다. raw telemetry는 보존한다. downstream motion은 같은 PTS의 일관된 continuous field를 median으로 묶어 한 관측으로 취급하고, 유의미한 속도/heading/gear 충돌은 계속 거부한다. 중복을 새 시각이나 독립 관측으로 만들지 않는다. base에 합친 개수를 기록한다.
- `new-date-recovery-suite-001`은 위 수정 후 upstream source/검출/추적을 그대로 검증 재사용하고 base부터 새로 계산하는 개발 재검증이다. 이전 base의 code/dependency가 바뀌면 cache를 쓰지 않도록 runner를 고쳤다.

Aug 20에서는 보이는 먼 쪽 stripe 두 개만으로 도로 범위를 잡아 자차와 선행차를 도로 밖으로 제외했다. 반복 표식은 clip 전체 프레임 비율이 아니라 실제 반복 관측 수로 세고, 자차 경로 대부분을 제외하는 경계 후보는 거부한다. 표식이 차에 가린 경우 pre-road 좌표에서 지속적으로 전진하는 차량 corridor를 outer paint로 감싸고, 누락된 내부 stripe와 외곽 한 차로 폭은 prior임을 명시한다. 반대 방향·횡단 운동은 확장 근거에서 제외한다. 차량 기반 차로 추정은 독립 차로 실측으로 표시하지 않는다.

`aug20-traffic-road-repair-002`의 선택 pickup은 이전 solid 0%→100%, 교정된 C1/4차로 관계 100%다. 1,187 frame 전체 decode와 수치 후보 0을 확인하고 12시점 원본 strip/렌더 contact를 직접 봤다. 그러나 clip 초기 차선 변경의 정확한 crossing event는 아직 검수 미완료다. 초기 footprint가 이미 경계에 걸쳐 시작하는 자료를 완전한 사건으로 단정하지 않는다.

Aug 28에서는 raw에서 확인한 좌측 변경의 예상 시각과 실제 event crossing 25.10초 / completion 26.99초가 맞았다. 다만 접근부 4차로·출구 2차로를 하나의 전역 고정 경계로 그려 초기 lane index와 뒤쪽 도로 폭이 어긋난다. Sep 09 / Aug 26도 선택 대상의 ID·solid·진행 방향은 유지되지만 차로 수/배치 gate가 남았다. **수치 안정성을 이유로 이 결과를 품질 PASS로 바꾸지 않는다.** 다음 핵심 수정은 구간별 road section 경계와 junction 사이 연결이며, 단순히 전역 차로 수를 더 늘리는 방식이 아니다.

노란색 mask 허용 범위 확대 실험은 Aug 20 실패에 효과가 없었으므로 채택하지 않고 원래 범위로 복귀했다. 해당 probe 결과는 보존한다. 코드 검사 188개 통과. 수동 correction layer, 주변 등화, 범용 road section, 주차 환경 fallback, 나머지 원본 사건 검수와 이전 날짜 회귀는 여전히 미완료다.

후속 구간별 road section / 횡단보도 결합 / 실선 경계와 갓길 구분 구현 및 실패 검수는 `ROAD_SECTIONS_REVIEW.md`에 기록했다. Aug28 출구 2차로라는 source 라벨은 확대 원본에서 3차로로 교정했으며 이전 2차로 판정은 무효다. 구간별 ID 전환을 차선 변경으로 세지 않는 처리를 추가했다. 193개 코드 검사와 별개로 실제 도로 정합은 아직 부분 미통과다. 측방 노란 선의 추정 rig 투영과 front pitch 후보는 도로/차량 수정에 채택하지 않았다.

## 램프 reference 지원과 수동 교정 분리

램프의 8–42초 unknown 68표본은 실제 ego가 차로 밖으로 움직여서가 아니라 reference phase 공백 때문이었다. `road_continuation.py`가 양쪽 실제 실선의 공간별 반복 프레임을 요구하여 지원한다. 결과 `oct18-camera-edge-continuation-001`: 170표본 lane0 100%, 좌표·yaw·렌더 actor/시점 표본은 이전과 동일, 850프레임 수치 후보0. 원본 4채널 contact를 직접 대조했고 전체 영상을 테스트26에 올렸다. 이 변경은 좌표 보정이나 모든 도로 완성을 뜻하지 않는다.

수동 correction layer와 별도 전체 MP4 renderer, Qt의 차량 수동 교정본 버튼을 구현했다. 자동 결과 artifact ID·base ID로 결합하고 원본은 덮어쓰지 않는다. 자동 성능 evaluator가 수동 결과를 거부한다. `MANUAL_REVIEW.md`에 사용법/제약을 기록했다. 합성 교정 E2E `manual-review-integration-check-002`는 지정한 39프레임만 바뀌며 자차·시점·기타 시간은 보존됨을 확인했다. 실제 사용자 조작/자동 품질 향상 증거와 구분한다. 코드 검사201개 통과, 기존 Qt 자동 MP4 내보내기도 동일 바이트 유지.

6개 runtime recovery는 모두 완료됐다. `new-recovery-source-evaluation-003`에서 선택한5차량의 고정ID·solid가 전 구간 유지됐다. black Kia는 0–10초의 1차로 선행 관계도 통과했다. pickup/Sep09 SUV/bus의 도로 관계는 구 recovery 결과에서 실패하며, Aug28은 pocket을 포함하는 절대 index 라벨 범위 자체의 추가 검수가 필요하다. 기존 실패 평가를 숨기지 않는다. 새 날짜 전체 holdout 합격이 아니고, 개발 수정 결과와 구 recovery 결과도 섞지 않는다. 다음 핵심 작업은 bus의 2차로를4차로로 확장한 근거 분리, 곡선 pocket, 다른 교차로/합류 연결이다. 주변 등화와 더 넓은 원본 재검수도 남았다. 폴·보행자는 여전히 후순위다.

후속 원인 분리에서 bus의 clip-wide2차로 source 라벨도 잘못됐음을 raw0/2/4/6초 확대 연속 화면으로 확인했다. 세 병렬 차로가 보이는0–6초만3차로/C2로 교정하고 나머지 count는 보류했다. 원래 라벨/평가/hash는 보존한다. `aug26-observed-paint-phase-003`은 실제 stripe 위상을 적합하고 독립 우측 실선 경계를 유지하여4→3차로를 만든다. ego 카메라/선행 bus pose 전체는 그대로다. 전체1,199프레임 수치 감사와 원본 strip 대조 후 테스트26에 업로드했다. Aug28은 차도 밖 진입로 경계의 lane-mask 오탐을 road-mask 교차로 제거하고, local divider 의미 소실을 보완했다. 후속 전체1,187프레임 검수에서 초기3차로와 실제 자차 좌측 변경을 유지했다. 곡선 pocket과 branch/merge 전체 완성은 미해결이다. 상세 근거와 미통과 범위는 `ROAD_SECTIONS_REVIEW.md`에 기록했다. 코드 검사206개 통과.

곡선 pocket 경로를 추가했다. 첫 median-top 투영 실패와 일반 차량의 side-anchor 회귀를 원본/불변식 검수에서 발견하고 각각 거부·수정했다. 최종 `aug28-curved-pocket-005`는 선택 source13.4초의 두 queue 차량을 같은 ID/solid/auxiliary lane에서 확인했고 ego와 기존 global 선행차의 전 구간 pose가 수정 전과 정확히 같다. 원래 자차 변경 완료26.9949초도 유지했다. 전체1,187프레임 수치 감사와13.4/27/45초 source-strip 대조 후 테스트26에 업로드했다. pure paint가 가려지면 반복 rear-appearance cue와 ground separators로 auxiliary lane을 **추정**한다. 새 lights 상태를 만들어 표시하지 않는다. 버스·램프 회귀에는 새 pocket0개다. 코드 검사219개, 해당 엔진 snapshot105파일을 보존했다. 상세 실패/검증 범위는 `ROAD_SECTIONS_REVIEW.md`. 다른 날짜의 branch/merge와 주변 등화, 새 동결 검증은 계속 필요하며 전체 목표 완료가 아니다.

교차로 연결의 후속 개발/실패/원본 대조는 `BRANCH_ORIENTATION_REVIEW.md`에 정리했다. 최종 `branch-orientation-suite-005` 네 구간4,752프레임을 같은 동결 코드로 출력하고 수치 감사와 선택 원본 회귀를 통과했다. 관측 회전의 도로 표시, T자 미래 연장, 중앙선 반대편 택시 scope, image-supported rear red appearance를 추가했다. 정확한 branch lane 복원이나 모든 주변 등화 control 추론으로 보고하지 않는다. 기존 공통 actor sample과 ego camera view는 전 구간 동일하다. 실패001/002/median probes는 보존한다. 코드 검사240개, exact engine/config/tests111파일 snapshot을 남겼다.4개 전체 영상은 로컬+테스트26 Drive에 병행 전달하고 metadata bytes/parent를 대조했다.4개 새 날짜 source-only controls와 동결 재추론 suite006도 시작했다. 이는 아직 independent source-quality PASS가 아니며 원래 목표는 active다.
