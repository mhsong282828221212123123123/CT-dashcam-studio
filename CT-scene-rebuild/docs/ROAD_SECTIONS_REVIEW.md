# 구간별 차로 구현과 남은 원인

`road_sections.py`는 영상 표식이 확인된 교차로 opening 사이를 독립 구간으로 나누고 각 구간의 카메라 표식/명시적 차량 폭 prior를 별도로 적합한다. `road_geometry.py`의 lane/footprint/renderer 조회가 같은 구간 경계를 사용한다. `lane_state.py`는 구간 ID가 달라져 차로 번호가 바뀐 것을 차선 변경 사건으로 세지 않는다. 구간 안의 실제 좌측 변경은 유지한다. association도 서로 다른 구간의 숫자만 비교해 ID 연결을 막지 않는다. ego/world 좌표는 도로 구간으로 수정하지 않는다.

193개 코드 검사 통과. 합성 검사는 4→2차로 구간 변경 중 차량이 움직이지 않는 경우와, 뒤 구간에서 실제 1→0 좌측 변경을 구분한다. 이는 합성 검사이며 실제 Aug28 출구가 2차로라는 증거가 아니다.

## 원본 검증 라벨 교정

원본 Aug28의 출구를 축소 contact에서 2차로라고 잘못 기록했다. 확대 `runs/aug28-road-section-raw-review-001/front-30.png`에는 자차 차로/중간 QM6 차로/맨 오른쪽 차로가 보인다. 우측 repeater30s에도 자차 오른쪽의 두 구분선이 보인다. 실제는 **4→3차로**로 교정했다. 이전 라벨과 hash/교정 근거는 `runs/new-holdout-source-controls-002/annotation-correction.json`에 보존했다. 기존 2차로 평가는 무효다. 예측에 맞추어 정답을 고친 것이 아니라 확대 원본에서 잘못된 라벨을 발견한 것이며, 해당 자료의 독립 holdout 합격을 주장하지 않는다.

## 구현 원인 분리

- 단일 Hough 직선만 요구해 연결되지 않은 횡단보도 띠를 놓쳤다. road mask와 actor mask를 유지하며 가로로 가까운 흰 띠를 결합했다. Aug28 camera crossing 후보가 기존 41초/45초 두 곳뿐이던 데서 16–18초의 큰 교차로 앞 표식도 검출된다. `aug28-section-paint-probe-001/002`에 변경 전후 원본 근거와 지도 잔차를 보존했다.
- 램프의 흰 실선/갓길/사선 지대를 모두 차로 경계로 세어 1차로를 3차로로 만들었다. 같은 실제 프레임의 색상 연속성과 지면 선분 길이를 따로 저장하고, 반복된 가까운 실선 두 개로 carriageway를 묶었다. 실선 너머 지속적으로 전진하는 차량이 있으면 bus separator일 수 있으므로 이 단일 carriageway 가설을 거부한다.
- `oct18-solid-edge-ramp-001`에서 차로 수는 3→1, 원본에서 비어 있는 8–42초의 25m 내 허위 actor state는 0이다. 그러나 ego lane0 판정은 59.8%뿐이며 나머지는 unknown/outside다. 850 frame 수치 감사 후보 0과 별개로 **전체 원본 정합은 미통과**다. paint/reference gap과 평면 투영의 관계를 더 확인해야 한다.
- `aug28-road-sections-003`은 구간별 4/3/2/3 차로 후보를 만든다. 초기 차로의 원점과 일부 구간 폭은 여전히 미확정이다. 형태가 달라졌다는 이유로 합격시키지 않는다.

## 채택하지 않은 측방 경계 실험

추정 side rig로 repeater의 노란 지면 선을 직접 W에 투영하자 접근부 경계가 2차로로 과도하게 좁아졌다. `aug28-side-curb-preview-001`을 실패 실험으로 보존한다. 그 투영은 도로 적합에 사용하지 않으며, 후속 산출물 `side-ground-candidates.json`은 diagnostic-only다. side vehicle 관측/ID와 rear identity 참고는 계속 별개로 사용한다.

현재 front ground projection은 pitch=0을 가정한다. 실제 원본의 원거리 지면 접점/차로 소실점과 이 가정의 불일치가 남아 있다. 다음 원인 분리는 정적 lane vanishing-point로 front ground-plane pitch만 추정해 보고, 차량/ego로 역보정하지 않은 재투영과 원본 관계를 검사하는 것이다. 광학 전체 SfM이나 건물 디테일 실험을 다시 시작하는 것은 아니다. 승인되지 않은 side metric rig를 정답처럼 사용하지 않는다.

후속 정적 소실점 실험(`aug28-front-pitch-probe-001`)은 차량을 적합에 쓰지 않고 69개 source frame에서 median pitch 약 0.054°, 80–20% spread 0.521°를 냈다. 첫 evidence의 원본 재투영 horizon도 직접 봤다. 이 결과로 일정한 큰 pitch 오차를 원인으로 확정할 수 없다. 후보는 **미채택**이며 rig/ego/actor를 바꾸지 않았다. 불일치를 맞추려고 특정 pitch를 강제로 넣지 않는다. 국소 경사·선분 선택·곡선/포켓 구간의 관측 범위를 분리해야 한다.

추가 원본 확대에서 접근부의 좌회전 pocket과 통과 차로를 구분할 필요도 확인했다. 통과 차로 번호와 pocket을 포함한 물리적 왼쪽 순번은 같은 의미가 아니다. 현재 source 라벨의 절대 index 판정은 이 구분을 검수한 뒤 사용해야 한다. 잘못된 단일 차로 수/번호를 맞추도록 solver를 조정하지 않는다.

## 램프 reference 공백의 원인과 보완

`oct18-solid-edge-ramp-001`의 unknown은 실제 ego 좌표가 경계를 벗어나서 생긴 것이 아니다. 260.50–351.57m, 361.70–452.64m의 reference phase 공백에서 lane query가 unknown을 반환했다. 양쪽 실제 흰 경계선은 그 공백에서도 관측된다. `road_continuation.py`는 이미 1차로로 확정된 구간에만, 20m 공간 bin마다 양쪽 경계에서 서로 다른 원본 프레임 2개 이상이 반복된 경우 reference 보간을 지원한다. 교차로 opening·다차로·한쪽만 보이는 구간은 이 규칙으로 연장하지 않는다. 원래 공백과 채택 프레임 ID를 보존하고 `two_sided_camera_edges_inferred_reference`로 구분한다. ego·actor 좌표·도로 폭은 바꾸지 않는다.

`runs/oct18-camera-edge-source-review-001/review.json`: 8–42초 170표본에서 lane0 판정 60%→100%, unknown 68→0이다. ego lateral 전체 표본과 `render-states.jsonl` 전체 바이트가 이전 결과와 같다. 이는 도로 지원 상태 보완이며 차량 위치 보정 성공으로 계산하지 않는다. 8/18/20/26/30/40초의 렌더+원본 4채널 contact를 직접 확인했다. 전체 42.5초 850프레임 decode, 최종 5초 포함 수치 후보 0. 다른 다차로 포켓·교차로 정합의 합격을 뜻하지 않는다.

## Aug26 버스 구간 — 전역 차로 수와 source 라벨 재검수

최신 전체 `aug26-road-sections-001`에서도 4차로가 유지됐다. fallback은 앞서 관측한 흰 선 lateral 8.33/4.75/-5.63m와 차량 lateral 5.92/2.90/-1.10m 등을 전체 구간에 합친다. 내부 경계 2개가 실제로 검출된 것이 아니라 폭 prior로 채워졌다. `aug26-road-sections-source-review-001/bus_11.json`에 실패 결과를 보존했다.

중요한 source 해석도 수정 검토가 필요하다. `aug26-outer-traffic-raw-review-001/tracks.jpg`를 확대 확인했으며 front-0002/0100의 흰 Sonata는 **후면이 보이는 같은 방향 차량**이다. 반대편 차량이라 가정하여 지워서는 안 된다. front-0003 흰 SUV, front-0010 파란 트럭도 같은 방향이다. 시작 부근에는 이들 차량과 버스가 서로 다른 횡위치를 점유하고, 교차로 전후 좌회전 pocket의 유무가 바뀐다. 따라서 frozen `expected_lane_count=2`를 0–50초 전체의 pocket 포함 절대 차로 수로 사용하는 평가는 아직 유효하지 않다. 원래 라벨과 FAIL 기록은 보존하며, 4차로 예측이 옳다는 뜻으로 고치지 않는다. 통과 2차로와 국소 pocket, 버스정류장 가장자리를 원본의 구간별 범위로 먼저 다시 검수해야 한다. 해당 count/index는 **SOURCE_SCOPE_REVIEW_REQUIRED**, 선택 버스의 ID/solid/선행 검사는 별도로 유효하다.

노란 선의 lane-mask gate를 road-mask로 바꾼 `aug26-yellow-road-mask-probe-001`은 지원이 늘지 않았다(기존30→12선분, 후보 bounds 그대로). 미채택이며 기본 pipeline은 변경하지 않았다. raw 노란 선 부족을 없던 도로 경계로 채우거나 같은 방향 Sonata를 반대편으로 지우는 해결책은 쓰지 않는다.

## 카메라 표식 위상과 차도 바깥 후보 수정

원인을 추가로 분리했다. 기존 boundary fitter는 reference line에 대해 표식이 항상 `width/2 + k*width`에 있을 것으로 가정했다. 실제 bus 구간의 반복 stripe/curb는 약 0.7/-2.9/4.4m에 있는데, 이 고정 위상 때문에 실제 경계를 버리고 -5.6m 바깥 후보를 채택했다. `observed_paint_phase`가 실제 흰 선의 주기적 위치를 측정하도록 바꿨다. ego/차량으로 위상을 적합하지 않는다. 차로 폭3.5m는 명시적 prior이며 공간20m bin마다 투표를 제한해 긴 정차 중 같은 문자/패치가 계속 보이는 것에 끌리지 않게 했다.

한쪽 실선이 내부 차로 구분선으로 판정되어도 반대쪽 독립적인 경계 증거까지 버리지 않는다. 왼쪽에 같은 방향 차량이 있어 단일 carriageway 가설을 거부할 때에도, 우측 실선 너머에 반복 전진 차량이 없으면 우측 경계 후보를 보존한다. 우측 경계는 추정 가설로 기록하며 실제 측량 경계라고 부르지 않는다. 차량으로 하나의 외측 사용 차로를 보조하는 규칙은 좌우 대칭으로 만들었으며, 관측된 노란 divider 너머는 차량만으로 확장하지 않는다.

`aug26-observed-paint-phase-003`: 59.95초 / 1,199프레임. 4→3차로, 자차 카메라 표본과 선행 bus pose 전체가 수정 전과 정확히 같다. 전체 decode/최종5초 수치 후보0. 0/20/40초의 전후 렌더+원본 strip을 직접 대조했다. `aug26-outer-traffic-raw-review-001/first-six-seconds.jpg`의 원본0/2/4/6초에는 Sonata/흰 SUV/bus가 세 병렬 차로에 있다. frozen2차로 라벨을 물리적 전체 차로 수로 쓰던 것이 잘못됐으며 `new-holdout-dense-controls-003`에 hash/이전 라벨/사유를 보존하고 **0–6초만**3차로/C2로 라벨했다. 나머지 시간의 count를 예측에 맞춰 확정하지 않는다. bus ID/solid/선행 관계는0–50초, count/C2는0–6초만 통과했다. 개발 자료 검수이며 독립 holdout이 아니다.

Aug28의 추가 오른쪽 후보를 실제 원본에 재투영하자 도로 옆 진입로/시설물 경계였다(`aug28-joint-road-lane-gate-probe-001/outer-candidate-source.jpg`,2.3847초). lane mask만으로 지면 선을 허용하던 처리를 road mask와 교차시켰다. 경계 픽셀은15px dilation으로 보존한다. raw actor mask도 유지한다. probe에서 초기 main boundary는4→3으로 줄고 bus/ramp의 필요한 경계는 유지됐다. 밤 Sep20에는 원래 active curved reference가 없으므로 이 boundary branch의 합격으로 계산하지 않는다. 첫 probe의 KeyError 로그는 보존하고 두 번째는 paint-only/기하 미평가로 기록했다.

`aug28-joint-road-lane-gate-001`은 초기 오른쪽 허위 차로를 제거했으나, 교차로 뒤 local section에 반대편 흰 선을 다시 추가한 실패를 발견했다. global camera yellow divider 의미가 local section에서 사라진 원인이다. 실제 local white separator가 같은 lateral에서 계속 관측될 때만 그 divider 의미를 이어받도록 했다. 해당 구간에서 실제 yellow가 바깥으로 벌어지면 새 관측을 우선한다. 단순히 지도/GPS나 고정 lane count로 잘라내지 않는다. 후속 전체 `aug28-joint-road-lane-gate-002`를 재검수한다. 곡선 pocket 검출은 아직 미구현이며, 이것을 pocket 완성이라고 계산하지 않는다.

장시간 direct 렌더가 실행되는 중 source가 편집되어 종료 시점 hash와 실제 실행 코드가 달라지는 것을 방지하도록, 실행 시작 revision을 저장하고 종료 시 다르면 publication을 거부하게 했다. 초기 observed-phase 중간 실험은 최종 동결 검증본으로 쓰지 않는다. 전달용 bus003 및 후속 비교본은 source가 고정된 실행 결과를 사용한다.

후속 `aug28-joint-road-lane-gate-002`: 1,187프레임 전체/최종5초 수치 후보0, source strip 포함10/27/40초 전후 이미지를 직접 대조했다. local section은3/3/2/3으로 초기 오른쪽 허위 차로와 교차로 뒤 반대편 흰 선으로 인한 추가 차로가 제거됐다. 해당 source의 전체 구간별 실제 count 합격을 뜻하지 않는다. 자차 실제 좌측 변경은 crossing25.052초/완료26.9949초로 유지됐다. 선택 QM6는 같은 ID와 solid1.0을 유지했다. 0–20초401표본 중 lane이 둘 다 정해지는306개는 모두 자차와 같은 차로다. 나머지95개는15.3초 이후 선행차·자차가 서로 다른 시각에 intersection opening에 들어가며 lane unknown이 된다. 이 unknown을 수치 pass로 바꾸지 않으며 원래 절대index/count FAIL 평가도 보존했다. 곡선 pocket은 여전히 따로 구현해야 한다. 코드 검사206개 통과.

## 곡선 포켓 구현과 source 검수에서 잡은 실패

`pocket_lane.paint_observations`에 곡선 RoadFrame station/lateral 추출을 추가했다. 이 경로는 기존에 거부한 repeater metric 투영을 섞지 않고 front 전체5Hz만 사용한다. 직선 legacy 추출은 보존했다. `fit_curved_pockets`는 반복 yellow/white 바깥 경계, 안쪽 구분선, 옆의 기존 through 구분선을 함께 요구하고, pocket이 지원된 교차로 opening으로 이어지면 존재하지 않는 닫힌 return을 요구하지 않는다. 모든 후보의 폭은 추정치이며 자차 pose는 바꾸지 않는다.

첫 `aug28-curved-pocket-001`은 실패다. `aug28-pocket-source-review-001/yellow-anchor-frames.jpg`를 실제 원본에 대조하자 중앙분리대 윗면의 색선을 지면으로 투영한 것이었다. 숫자 검사/합성 fixture가 통과해도 이를 채택하지 않았다. raw road-mask 지면 영역과3px margin으로 제한한 `aug28-curved-pocket-probe-002`에서는 해당 잘못된 paint-only pocket이0개다. 윗면 근거를 좋은 표식으로 재해석하거나 실패본을 지우지 않았다.

실제 포켓은 주로 차량에 가려져 있다. `vehicle_supported_curved_pockets`는 실제 front rear-appearance 단서(서로 떨어진 두 red corner lamp, 여러 원본 프레임), 반복된 ground separator, 기존 through separator, 지원된 junction approach가 함께 있을 때만 **추정 보조 차로**를 만든다. 차량이 있다는 사실만으로 만들지 않으며 velocity sign도 방향 정답으로 삼지 않는다. 정차 차량의 ground contact depth 잡음이 음의 속도로 적합될 수 있기 때문이다. rear cue는 브레이크/방향지시등 상태를 뜻하지 않는다. outer edge가 가려지면3.5m prior를 명시하고, 다른 경우도 부분 white paint와 폭 prior로 기록한다. `pocket_support`와 `geometry_status`로 pure-paint와 vehicle-assisted 추정을 구분한다.

`aug28-pocket-source-review-001/candidate-source-crops.jpg`에서 front0020 흰 SUV와 front0033 검은 세단의 실제 후면을 확인했다. front0043은 반대편에서 오는 차량이며 cue0이었다. front0082는 다음 junction의 흰 SUV이고, front0102의 전체 원본45.6초는 왼쪽 queue의 검은 경차임을 확인했다. 흰 SUV로 이름을 추측하지 않는다. raw39.997/42.004/43.986/45.994초 contact도 직접 봤으며 두 교차로 사이에 실제 중앙 분리 시설과 도로 구간이 있으므로 단순히 하나의 큰 교차로로 합치지 않는다.

차로 입회·필터·soft prior가 곡선에서도 world X를 station으로 쓰던 오류도 고쳤다. 포켓과 main 사이 이동 중 관측을 잘라내지 않으며, 가상 포켓 경계를 붙여도 main lane ID는 바꾸지 않는다. opening 내부에서는 평행 lane-change 사건과 prior를 만들지 않는다. `pocket_actor_motion.py`에 원본 표본 보존/고정 main ID/실제 main→pocket/unknown junction 검사를 추가했다.

독립 regression에서 추가 실패를 잡았다. 포켓 가용성을 기준으로 모든 actor의 timeline을 나누면 우측 repeater의 기존 clear anchor가 끊겨, global front0002의 후속 side 추정이 최대1.44m 바뀌었다. `aug28-pocket-source-review-002/review.json`은 그 실패를 그대로 기록한다. 포켓을 실제로 드나드는 차량에만 새 prior를 적용하도록 고쳤고 기존 side anchor 보존 검사를 추가했다. `aug28-curved-pocket-005`에서 전 구간을 다시 대조한다.

front0043은 일부 noisy contact만 main에 들어와도 인정하는20% 규칙으로 통과하던 반대편 차량이다. 관측된 median 너머의 front track은 지원된 pocket membership이 없는 한 거부하도록 했다. 첫 두 차량의 실제 pocket을 이 규칙으로 지우지 않는 negative/positive 검사를 함께 추가했다. bus/ramp 재렌더에는 새 pocket이0개이며 각각1,199/850프레임 수치 후보0이다. 이 두 regression은 후속 main-anchor 수정 전 revision이며 최신 전체 scope 동결 검사를 대신하지 않는다. 현재 코드 검사219개 통과. branch/merge 전체, 주변 등화 표시, 독립 holdout 검수는 여전히 미완료다.

최종 개발 영상 `aug28-curved-pocket-005`는 전체59.35초/1,187프레임을 검사했고 최종5초 포함 수치 후보0이다. 원본13.4115초의 수동 bbox 두 개와 같은 fixed ID를 대조했다: 흰 SUV front0020 IoU.876, 검은 세단 front0033 IoU.879, 둘 다 solid이며 auxiliary lane -1 안이다. 이는 선택한 source anchor의 개발 검수이며 전체 차량 recall이나 정밀 위치 합격이 아니다. `aug28-pocket-source-review-003/review.json`에서 자차 camera sample 전체와 global front0002의 위치/yaw **전 구간이 수정 전과 정확히 동일**함을 확인했다. 이전1.44m 회귀는 해소됐다. 반대편 front0043은 제외됐다. 실제 자차 좌측 변경은 crossing25.052365/완료26.9949초로 동일하다. 13.4/27/45초의 전후 렌더+원본strip도 직접 봤다.

3개 auxiliary corridor는 모두 vehicle/ground-separator 기반 추정이며 외측 표식의 실측 복원이 아니다. 초기중앙분리대-top 후보로 전체 구간에 pocket을 넓히던 실패001은 전달하지 않는다. 최종005만 테스트26에 업로드했다. 실행 code revision과 동일함을 확인하고 `aug28-curved-pocket-005-engine`에 source/config/tests105파일 snapshot을 남겼다. 이후 작업트리의 header 설명만 고쳤으며 기능 변화는 없다. 남은 우선 작업은 다른 날짜·branch/merge·주변 등화와 새로운 동결 회귀/holdout이다.
