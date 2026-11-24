# 원본 영상 대조 기반 재검증 작업 기록

현재 판정: **불합격. 작업 진행 중. 상용 수준 또는 개선 완료로 보고하지 않는다.**

최신 코드 검사: 전체 160개 테스트 통과(77.304초), compileall 및 diff 검사 통과.
이 결과는 영상 품질 합격을 뜻하지 않는다. `tests-latest.log`에 실행 결과를 보존했다.

## 자료와 분리

- `tools/catalog_scene_validation.py`로 E:/tesla/a2~a6를 읽기 전용 조사.
- 전방 파일 4,601개, 타임스탬프 중복을 묶은 클립 4,477개, 촬영 날짜 62개.
  바이트 단위 동일 파일 검증은 아니다. 날짜 단위로 개발/별도 평가를 분리.
- 저장소 옆 `scene-results/quality-rebuild/catalog/catalog.json`에 전체 목록과
  최초 60개 장면 검토 후보, contact-00~04.jpg에 원본 접촉 시트가 있다.
- 이 중 주행/교차로/야간/대형차/이륜차/보행자/주차 21개를 1차 조사.
  `baseline-cases.json`, `baseline-report.json` 및 `baseline/<clip>/`에 결과.
- 개발 14개, 별도 평가 7개. 총 2,000개 분석 시점(10 Hz), 원본 4채널.
  대부분 10초 구간이며 2개는 원본 길이가 짧다. 30초 연속 검수 통과가 아니다.
- SEI로 정차, 저속 0~31 km/h, 고속 87~118 km/h 등을 확인했다.
  정지 이미지의 외관만 보고 속도/차선 변경을 확정하지 않는다.
- 독립 수동 정답의 전체 차량/차선 표기는 아직 미완료다.

## 적용한 결함 수정

1. 기본 리피터 수평선: 좌 585→504, 우 545→486 px.
   카메라 높이 .9m/대표 승용차 높이 1.65m 가정의 개발 클립별 중앙값이다.
   계측 캘리브레이션은 아니다. 기존 값은 실제 접점이 수평선보다 높다고
   해석해 접근 차량을 배치에서 버렸다. 실제 2026-09-05 고속 13초 박스 회귀 검사 추가.
2. 3D 모델 실제 GLB 크기가 배치 계산의 크기와 달랐다. 예: 세단 GLB 높이
   약 1.447m, 배치 가정 1.65m. 융합 Actor에 높이를 보존하고 opaque/ghost 모두
   배치의 폭/길이/높이와 같은 크기로 렌더링한다. 이 수정은 실제 차량 크기를
   측정했다는 뜻이 아니다. 그 대표 크기 자체의 오차는 별도 문제다.
3. 긴 관측 공백에서 부모 보간기가 지운 도로/신호를 융합 보간기가 이전
   프레임에서 복구하던 오류를 막았다.

실험 프로필(반올림 전)의 측면 확정 관측→표시 연결률:

| 분리 | 기존 | 수정 후보 | 확정 관측 수 |
|---|---:|---:|---:|
| 개발 | 571 (14.70%) | 2,608 (67.15%) | 3,884 |
| 별도 평가 | 64 (6.39%) | 739 (73.75%) | 1,002 |

이는 검출된 관측의 연결률이며 실제 차량 검출률/미터 거리 정확도가 아니다.
오검출 물체가 더 많이 표시될 수도 있다. 영상 재투영 검토에서 오검출,
차종 크기 차이, 접점 오차가 여전히 확인됐다. 합격 지표로 쓰지 않는다.
baseline 스크립트는 이제 이전 수평선 585/545를 명시적으로 고정한다.

## 차선/도로 실험 (기본 분석 메뉴에는 자동 적용하지 않음)

- UFLDv2 TuSimple ONNX: 전체 4:3 입력과 하단 16:9 viewport 비교.
  전체 입력에서는 심한 굴곡 오류, viewport 보정 후에도 교차로/가림에 잘못된 선.
  따라서 결과를 기본 차선으로 채택하지 않았다.
- YOLOP ONNX: 주행 가능 영역과 차선 픽셀을 별도 분할. 주간에서는 추가 도로
  범위를 확인했지만 야간 횡단보도/정지선 혼입과 경계 오류가 남는다.
- `scene_road_evidence.py`: 종방향 차선 조각을 추출. 2개 선 제한 없음.
  물체 영역 배제, 횡방향 성분 제외. 도로 단면은 차선 수와 독립적으로 추출.
- `scene_road.py`: 새 road_evidence가 있는 결과에만 별도 경로 사용.
  도로 단면을 투영하고 과거/미래 각 2초 카메라 관측을 자차 이동을 보정한
  좌표계에서 모은다. 공백/위치 초기화를 넘어서 연결하지 않는다.
  `scene_road_fit.py`는 여러 관측을 robust quadratic 곡선으로 맞추되 큰 미관측
  구간을 가로질러 차선을 만들지 않는다. 차량 위치를 차선으로 옮기지 않는다.
- 추정 도로 외곽 실선/내부 표시 점선. 실제 표식 유형 분류는 미완료.
- `road-sequences-v2/<clip>/`는 개발 14개 구간의 보강 결과. 원본 관측은
  `baseline/`에 보존. `tools/enrich_scene_roads.py`로 생성.
- `reconstruction-review-*` 폴더의 단계별 3D+원본 비교 이미지를 확인했다.
  도로 폭/뒤쪽 및 먼 쪽 차선 연결은 개선됐지만 전체 차선 수, 교차로,
  가림, 시야 외 도로 윤곽에는 실패가 남는다. 전달용 새 영상은 만들지 않았다.

## 시간축 방향/위치 최적화 실험 (자동 적용하지 않음)

- `scene_trajectory_fit.py`: 0.7~4초 구간의 월드 위치/속도/차체 축을 함께 최적화.
  짝수 시점으로 맞추고 홀수 시점의 박스/접점으로 검사한다.
- 합성 횡방향 주행 및 회전 중인 자차의 독립 시점 검사 추가.
- 실제 개발 영상에서는 박스 점수가 높아도 정차 차량을 12도 정도 잘못
  회전시키는 사례가 확인됐다. 대표 차체 크기와 방향의 모호성이 원인 후보.
  따라서 `accepted`는 이미지 게이트만 의미하며 씬에 적용하지 않는다.
- `trajectory-review/`, `trajectory-reprojection/`의 기존 보고서는 이후
  자차 회전 부호 수정 이전 실행 결과이다. 다음 비교 전에 재생성해야 한다.

## 실행 및 남은 검수

- 원본 전체 재검출 전에 저장된 관측으로 기하/후처리를 비교한다.
- `tools/review_scene_reconstruction.py <quality-rebuild> road-sequences-v2 <tag>`
  는 내부 비교 이미지이며 사용자 전달/완성 판정이 아니다.
- `tools/scene_acceptance_gate.py`는 독립 수동 30초 이상 연속 검수가 없는
  장면을 자동 통과시키지 않는다. `acceptance.json`은 현재 FAIL이다.
- 다음 핵심: 수동 정답과 사건별 누락/오배치 평가, 모든 필수 장면별 독립
  30초 이상 3개 이벤트, 카메라 기하/차체 형상 검증, 측면↔전방 연결,
  차선 변경 및 교차로 연속 검수. 평균 수치로 실패 사건을 숨기지 않는다.
- CUDA 사용 없음. DirectML/CPU만 사용. 원본 수정/빌드/릴리즈/푸시 없음.

## 측면→전방 사각 구간 연결 실험

- `continuous/2026-09-22_08-59-36` 원본 1~36초, 350시점 분석 완료.
- 원본 0.5초 간격 검수: 흰 탑차가 7초 측면 끝에 보이고, 7.5/8초에는
  양쪽 영상에 없으며 8.5초 전방 왼쪽 끝에서 다시 보인다. 정확한 시야
  진출입 시각은 프레임 단위 추가 검수 필요.
- 기존 결과는 8.5/9초에 `fused:1` 고스트와 `fused:7` 관측을 중복 표시.
- `scene_handoff.py` 실험: 측면 끝 진출/전방 끝 진입, 같은 종류,
  2초 이내 공백, 기존 유효 예측, 양방향 유일 후보, odometry 연속성,
  보간 경로의 다른 차량 충돌 여부를 확인해 연결한다. 추정 구간은
  관측으로 승격하지 않고 interpolated로 표시한다.
- 최초 단방향 실험은 1건 연결. 이후 양방향으로 확장하여 3개 35초
  연속 영상에서 각 1건, 기존 21개 짧은 구간에서 추가 2건 연결.
- 앞뒤 영상 키프레임을 직접 확인했다. 주간 맞은편 흰 승용차,
  야간 왼쪽 추월 승용차, 지하차도 옆 미니밴의 연결이 영상과 부합한다.
  정확한 미터 단위 위치/방향 및 전체 연속 구간 합격을 뜻하지 않는다.
- 2초를 넘는 공백은 최대 4초까지만, 양 끝 0.8초 관측의 속도와
  이동량이 일치할 때 허용한다. 곡선의 속도 방향 변화를 고려하여
  두 끝 속도의 사다리꼴 적분을 비교한다. 온라인 TTL은 늘리지 않았다.
- `FusedSceneTimeline`의 실제 postprocess 경로에 적용했다.
  `enable_handoffs=False`로 A/B 비교 가능. 원본 관측은 수정하지 않는다.
- 통합 후 실제 세 연속 영상에서 `stitched_handoffs=1`씩 확인.
  첫 통합 시 인자명이 내부 지역변수와 겹쳐 호출이 누락된 문제를
  발견하고 수정했으며 통합 호출 검사를 추가했다.
- `2026-10-31`은 이 규칙 개발에 사용했으므로 split-overrides.json에서
  개발 날짜로 전환했다. 새 독립 검증 날짜 확보가 필요하다.
- 다음: 맞은편 차량 방향, 차선/도로 전체 형태 및 새 독립 연속 영상.

## 공개 모델 출처

## 이동 방향 증거 실험

- `scene_motion_heading.py`는 자차 회전/이동을 제거한 관측 위치에서
  앞뒤 0.8초 이동 방향을 추정한다. 최소 관측 수/시간, 잔차, 양분한
  관측의 속도 방향 일치로 제한하며 정차/저속 차량은 판정하지 않는다.
- `motion-heading-experiment.json`은 3개 연속 영상과 8월 30일 사례의
  진단 결과이며 렌더 방향에는 아직 적용하지 않았다.
- 이동 방향은 차체가 향하는 방향과 같다는 보장이 없다. 후진/정차는
  영상의 차체 방향 증거가 별도로 필요하다. world 속도가 작다는 이유만으로
  8월 30일 차량 위치를 잘못됐다고 단정했던 기록을 수정했다.
- 자차가 정차 물체를 지나감, 자차 회전 중 맞은편 이동, 카메라 위치
  불연속에 대한 3개 검사 통과.

- [UFLDv2 원 프로젝트](https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2)
- [시험한 ONNX 미러](https://huggingface.co/shadow-cann/hispark-modelzoo-ufldv2)
  SHA256: `376aaeaf419379c9cfefd023398609eb2e0ec4d5cbf287c94b19c304db335eab`
- [YOLOP 공식 저장소](https://github.com/hustvl/YOLOP)
  `weights/yolop-640-640.onnx`
  SHA256: `cd66a3e0087a7258ae07768cc02cb742eed93865727ae4c9baf969b8fa190696`
- 모델은 저장소 옆 `scene-models/`에 검증용으로 보관. 영상 외부 전송 없음.

## 트럭 형상 결함과 크기 비교

- 길이만으로 승용차 지붕을 적용하던 corners 조건을 수정했다. 명시적 truck은 전체 높이의 직육면체를 사용하며 기존 기본 car 호출은 유지한다. 관련 fusion 검사 18개 통과.
- 모든 truck의 기존 사전 크기는 2.0×5.2×2.0m였다. 높이가 큰 적재함의 박스를 거부한다.
- 세 연속 영상의 truck 관측을 0.5초 간격으로 네 크기 후보와 비교했다. 결과는 truck-prior-experiment.json. 9월 22일 front:1 62개 관측 중 기존 2m 형상 45개, 2.7m 소형 박스 62개, 3.3m 중형 61개, 3.8m 대형 57개 적합. 크기 선택은 아직 자동 적용하지 않았다.
- 박스 적합 성공은 실제 거리/차종 정답이 아니다. 단일 프레임 최적 후보로 형상이 출렁이지 않도록 추적 단위 선택과 독립 영상 검증이 필요하다.

## 추적 단위 트럭 형상 선택 통합

- scene_truck_shapes.py: 동일 camera-local ID의 비가림/비잘림 관측 최소 8개, 0.7초 이상에서 최대 16개를 골라 네 형상의 박스 적합을 비교한다. 실패도 비용에 포함하고 기본 대비 개선이 작은 경우 변경하지 않는다.
- FusedSceneTimeline은 저장된 관측의 예비 계산으로 시점별 카메라 기하를 확보하고 한 추적의 형상을 고정한다. 영상 추론은 반복하지 않지만 기하 계산 비용은 증가한다. select_truck_shapes=False로 비교 가능.
- 문제 구간 18.5초 front:1은 truck_medium으로 배치되고 렌더에서도 다시 표시됨. 실제 거리/차체 치수는 미검증. camera-local ID 사이의 크기 일관성은 추가 검수가 필요하다.
- 세 연속 영상의 actor-frame 수 664→722, 1098→1103, 1017→1017. 이 수치는 정답 차량 표시율이 아니다. truck-shape-integrated.json에 기록.
- 합성 다중 거리의 중형 트럭 선택, 낮은 화물차 기본 유지, 희소 관측 선택 거부 검사 통과.

## 추가 독립 영상 검증 실행

- fresh-validation/run-configuration.json에 4개 신규 날짜의 35초 영상과 scene 코드 해시를 고정했다: 8/18 혼잡 시내, 8/19 횡단 교차로, 8/31 비 오는 좁은 곡선, 10/16 야간 주차 음성 사례. 기존 원본 회귀 날짜 9/4는 제외했다.
- run_scene_fresh_validation.py 실행 중(PTY session 76951). 8/18 완료, 8/19 분석 진행을 확인했다. 다음 턴에 기존 핸들을 먼저 확인하며 중복 실행하지 않는다.
- source-keyframe-annotations.json은 추론 결과 확인 전 원본에서 표기한 주요 차량 박스/신호/도로 기대사항이다. 전체 영상 정답이나 합격 기록이 아니다.
- evaluate_scene_keyframes.py로 검출 박스와 최종 배치 ID를 독립 비교한다. 8/18 15초의 근접 차량 3개는 모두 검출 및 1개씩 배치됨. 거리/방향/연속성은 별도 미검증이다. 8/19는 분석 후 평가 필요.

## 독립 교차로 실패와 이륜차 증거

- 신규 4개 35초 분석 완료(각 350시점). PTY 76951 종료 성공.
- 8/19 15초 수동 표기 5개 대상: 검출 4, 최종 배치 2. 빨간 가로 신호등 3개는 검출되지만 모두 unknown. keyframe-evaluation.json 및 independent-failure-summary.json 참조.
- 이륜차는 14~16.5초 원본 확대 연속 이미지에서 확인. 기존 표준 검출은 14~17.5초의 0.5초 간격 표본에서 전부 누락, 18초에 motorcycle로 검출. 기존 tiled CPU 모드는 15초 라이더를 person으로 검출하지만 motorcycle은 누락. 가짜 수동 관측을 넣지 않고 이후 자동 검출에서 역방향 영상 추적을 검토한다.
- 8/19 분석 결과를 이용해 알고리즘을 변경하면 이 날짜를 개발용으로 전환해야 한다. 아직 core 변경 없이 실패 평가와 읽기 전용 tiled 실험만 수행했다.

## 자동 검출에서 역방향 영상 추적 실험

- scene_image_track.py: 특징점의 왕복 LK 오차, RANSAC 부분 affine, 공간 분포, 축척/회전/이동 제한으로 박스 후보를 만든다. 검출로 승격하거나 씬에 자동 주입하지 않는다.
- 8/19 18초 자동 motorcycle 검출을 시작점으로 14초까지 40시점 역추적 후보 확보. 15초 원본에서 이륜차 위에 후보 박스가 놓이는 것을 확인했다. 라이더 전체가 아닌 바이크 부분 박스이므로 기존 1.65m 높이 가정에 바로 넣지 않는다.
- 매 단계 회전한 사각형을 축 정렬 경계로 다시 감싸며 박스가 커지는 오류를 발견했다. 중심/축척 갱신으로 수정 후 18→15→18초 왕복 anchor IoU가 0.599에서 0.901로 개선됐다. 과거 proposal과 새 결과 모두 fresh-validation/reverse-image-track/proposals.json에 보존. 예전 JPG는 수정 이전 결과다.
- 이동 왕복 및 완전 가림 거부 검사 2개 통과. 실제 다른 물체/가림에서의 드리프트와 바이크/라이더 박스 범위 검증은 남는다.
- 8/19는 알고리즘 개발에 사용했으므로 split-overrides.json에 개발용 전환 기록.

## 이륜차 검출 형상/표시 형상 분리

- motorcycle_body/bicycle_body 검출용 사전 높이 1.1m와 라이더 포함 표시 높이 1.65m를 분리했다. 원본에서 검출 박스가 바이크 본체를 주로 포함하는 사례를 확인한 데 따른 대표 형상이며 실제 측정 치수는 아니다.
- 8/19 18초 front:51은 기존 높이 1.65m 적합에서 탈락했으나 본체 형상 적용 후 실제 전체 35초 타임라인에서 fused:56으로 배치됨. 표시 모델 높이는 1.65m 유지. cycle-body-regression.json에 17.5~20초 결과 기록.
- 역추적 15초 박스는 본체 높이를 적용해도 적합되지 않아 아직 씬에 넣지 않았다. 검출 복구와 3D 위치 복구를 혼동하지 않는다.
- fusion 19개, actor geometry 1개, postprocess 11개 검사 통과. 다른 이륜차 영상 및 라이더/바이크 중복 문제 검수는 남는다.

## 빨간 램프의 노란 포화 중심부 처리

- scene_features.signal_appearance에서 한 램프의 작은 빨간 테두리가 노란 중심부를 둘러싸는 경우를 구분했다. 분리된 red/amber 램프를 잇지 않도록 크기/종횡비/내부 노란 픽셀 비율/빨간 지지 비율/녹색 혼입 조건을 둔다. 흰 번짐 거부는 유지한다.
- 8/19 15초 원본의 세 가로 신호등은 단일 프레임 색 분류에서 red. 그러나 기존 3연속 지지 조건을 포함한 SignalStateTracker에서는 그 시점에 1개만 red, 나머지는 아직 unknown이다. 세 신호가 모두 안정적으로 표시된다고 주장하지 않는다.
- 해당 35초에서 unknown→red 565개 head-observation 변경. 다른 3개 주야간 개발 클립에서는 상태 변경 없음. signal-rim-review/report.json과 changed-heads.jpg에서 실제 램프를 확인했다. 원본 분석 파일은 변경하지 않았다.
- 빨간 테두리/노란 중심부, 분리된 적황 동시등, 순수 황색, 흰 번짐 검사 4개와 기존 feature 검사 5개 통과. 신호 변경 연속성/불확실 상태의 표시 검수는 남는다.

## 신호 시간축 지지 보완

- 최대 0.5초/최근 5개 관측 중 같은 색 3개 지지를 요구한다. 현재 unknown은 과거 색으로 표시하지 않는다. 다른 색 후보는 지지를 초기화하고, 시간 공백/중복 시각은 지지를 쌓지 않는다.
- enrich_scene_signals.py가 원본 픽셀과 저장된 검출 head만 다시 읽어 새 결과를 만든다. 원본 분석 관측은 보존하며 새 신호등 검출을 추가하지 않는다.
- fresh-validation/signal-enriched/2026-08-19_17-16-37 결과에서 수동 표기 15초 빨간 head 3개 모두 red, 지지 수 5/3/4. keyframe-verification.json에 원본 표기 대조 기록. 전체 신호 시퀀스/차선별 연결은 미검증.
- 다른 주야간 3개 개발 영상의 기존 신호 상태는 변경 없음. 새 시간축 검사 3개, rim 4개, 기존 feature 5개 통과.

### Crossing motion-model comparison (2026-11-12)
- Compared constant velocity and optional constant turn rate against the same withheld timestamps for development intersection 2026-08-19_17-16-37, sources front:25/26/35, 13–16 s.
- All six proposals failed existing image/contact gates. Turn-rate median IoU was 0.6133/0.3663/0.3449 versus straight-motion 0.6118/0.3652/0.3456. No proposals were applied. Merely allowing turning does not resolve these reconstruction failures.
- Artifact: ../scene-results/quality-rebuild/fresh-validation/crossing-motion-model-comparison.json. These are image-consistency results, not metric accuracy.
- Added a synthetic curved-motion withheld-frame regression; trajectory suite: 4 tests passed. This validates solver mechanics only, not real footage quality. Full suite has not been rerun for this experiment.
- Diversity includes four additional 35-second cases (dense city, crossing intersection, rainy curved road, night parked truck), in addition to the existing 21-case baseline. The intersection date is now development, not independent acceptance data. Commercial acceptance remains FAIL.

### Road surface delivery revision (2026-11-12)
- User requested the road revision, a video result, then stopping work. No commercial-quality completion claim is authorized by current evidence.
- Replaced temporal union of overlapping segmented-road quads with per-cross-section temporal median consensus. One source time supplies one interval vote. Missing longitudinal bands and disconnected lateral components remain open; this does not infer lane count or create walls from nonvisibility.
- Surface observations are transformed through the same ego poses before consensus. Existing bounded memory is retained when no continuous-window surface observation is available; odometry-reset clearing remains tested.
- Four new tests cover outlier edge rejection, unsupported longitudinal gaps, disconnected road areas, and curve retention. Updated the extent test to verify footprint bounds independent of polygon tessellation. Road suite: 25 passed. Full-suite/export outcomes to be recorded after completion.
- Current internal lane paint/estimated road-edge styling remains dashed/solid respectively. Physical walls, independently confirmed road boundaries, complete road topology and commercial acceptance remain unresolved. A cleaner surface alone is not reconstruction accuracy.
- Motion-fitting experiment: uniform axis starts and absolute central numerical differences replace duplicate symmetric yaw starts / float32-sensitive tiny differences. Four synthetic trajectory tests passed. The three real crossing tracks still fail acceptance; proposals remain unapplied. Comparison artifacts and source overlay are under fresh-validation/crossing-*.json and crossing-fit-overlay.jpg.

- Full verification for the delivery revision: `python -m unittest discover -s tests` passed 191 tests in 82.492 seconds. `git diff --check` and compilation of changed road/trajectory/export modules passed. These checks do not establish visual/metric acceptance.

- Visual delivery review: low-speed video decoded all 350 frames at 10 fps (35.0 s, 1280x960). Inspected original-camera comparison at 0/5/10/15/20/25/30/34 seconds. Road edges still show substantial jaggedness and the road/vehicle reconstruction remains inconsistent. Temporal median surface consensus did NOT resolve the requested road completeness. Do not describe this as a completed road model or accepted improvement. Physical boundary recognition remains missing. Export is an honest current-state review artifact, not an acceptance sample.
- User explicitly requested delivery followed by stopping. Preserve source/results and pause further goal work after delivering both videos; do not automatically iterate, build, publish, release, or push.
- High-speed export also decoded all 350 frames at 10 fps (35.0 s, 1280x960); original 4-channel comparison included. Export subprocess completed successfully. Both delivery videos and decoding reports are in ../scene-results/quality-rebuild/delivery-road-consensus/. No build, release, push, or original-video writes were performed.

### Vehicle-supported road reset (user-scoped revision)
- User clarified the original defect: host lane change was attributed to leading traffic and added a false right lane. The requested starting point is vehicle occupancy -> lanes -> continuous road. This supersedes the earlier camera-only prohibition; old camera segmentation code remains available only via add_camera_corridors and its legacy tests.
- New default scene_vehicle_road forms persistent lane indices from sustained observed vehicle/host residence. Width is explicitly assumed 3.5m. One-frame outliers and predicted ghosts cannot establish lanes. Surface and internal/outer markings are continuous; vehicle positions/detections are not changed.
- Rejected first attempt: holding a straight world road across the full original high-speed clip accumulated curved-route motion into 12 host lane widths. `vehicle-road-reset/high-original-37s.mp4` is a failed intermediate, NOT the delivered revision.
- Corrected approach separates local road orientation and unwrapped camera lane phase from persistent lane identity. Ego and every object receive the same rigid display transform. Missing paint retains phase with multi-track common motion support. Pure odometry fallback is used only for segments with no paint evidence.
- Original 2026-09-05 high-speed regression: corrected `high-phase-37s.mp4` is 370 decoded frames / 10 fps / 37 s, includes original four cameras. Fixed two-lane inferred topology throughout; host phase moves from 0 to -1.04 lanes by 36 s, rather than spawning an extra lane. Inspected 0/5/10/15/20/25/30/36 s source-comparison frames. Vehicle misses and absolute-position errors remain outside this road-only correction.
- Seven targeted tests include left lane change with host yaw and stationary road-reference lead vehicle, phase unwrapping, curved odometry without false lanes, vehicle-only road creation, occlusion persistence, ghost/outlier rejection, and odometry reset.
- Final scoped verification: 198 tests passed in 84.908 s, including 7 new road/host-motion regressions; git diff --check passed. Separate 2026-10-31 35 s export decoded all 350 frames, held two inferred lanes throughout, and host lane phase remained approximately -0.15..+0.05 at sampled timestamps rather than growing an extra lane. Its mid-frame source comparison was inspected. This is not a claim of full-scene commercial accuracy. User cleared the persistent goal; no automatic continuation is requested.

### Ego display stabilization (scoped user request)
- Added symmetric robust local quadratic temporal fitting for ego lateral position and angle within each continuous segment. Edge fits use a linear model to reduce endpoint noise; actual lane-change timing remains centered rather than causally delayed.
- Stabilization applies only to ego_display_override. Lane occupancy, original road frame, detections and surrounding actors are unchanged. Playback interpolates the corrected ego pose between analysis frames, without crossing long gaps.
- Original high-speed 37 s trace: per-0.1s lateral step P95 0.17319 -> 0.03884 m; >2cm consecutive direction reversals 56 -> 0. Final lateral position -3.6208 -> -3.5914 m. These are jitter measures, not absolute position accuracy. Comparison saved beside high-stable-37s.mp4.
- Targeted verification: stabilization 2, vehicle-road 7, fusion 19, driving-scene 6 tests passed. Synthetic tests cover noise rejection, heading jitter, preserved lane-change midpoint/travel and no constant-velocity lag. git diff --check passed. Export decoded 370 frames at 10 fps (37 s). Full commercial-quality acceptance is unchanged.

### Remove surrounding-vehicle ego-position reference
- Removed all actor reads from _road_frames. Missing paint now uses host odometry projected onto world road orientation interpolated from surrounding paint observations; later road phase corrects accumulated drift through the gap. Consecutive paint anchors unwrap against preceding road phase, preventing an erroneous odometry jump from inventing a lane crossing.
- New regression changes two neighbouring vehicles arbitrarily during a paint gap and asserts identical host poses, plus recovery of a host lane change through the gap. Vehicle-road 8, ego-stabilization 2, fusion 19 tests passed; diff check passed.
- Actual high-speed 37 s export fully decoded. Host lane displacement at 23/24/25 s: -0.972/-1.031/-1.058 lane (formerly ~-0.69/-0.70/-0.74). At 21/22 s the estimate remains -0.624/-0.832; exact real completion timing is not established. Inspected source-comparison frame at 24 s. Delivered high-host-road-only-37s.mp4. No commercial completion claim.

### OSM topology connected to actual road surfaces
- New scene_map_surface.attach_map_surface is invoked after OSM matching for the active vehicle-supported corridor. OSM is no longer solely a HUD prior: locally tangent-aligned connected ways now form road quads; matched-route boundaries follow the polyline and stop near shared junction nodes.
- Width/number come from inferred vehicle/camera corridor, not OSM tags. Connected branch widths currently assume the same display width and remain unverified. Registration is local tangent/corridor alignment, not calibrated global map registration. Only direct shared-node same-level roads are selected; full route-network topology is not finished.
- Three new tests cover cross-road surface/open lane lines, curved merge geometry, and rejection of disconnected/grade-separated roads. Full suite passed 204 tests in 109.613 s before the subsequent short-quad rendering correction and degree-2-node opening guard; latest map-focused suite 10 passed after both corrections. Diff check passed.
- Actual intersection OSM download failed with server HTTP 502 and then 40 s timeout. No intersection-original acceptance claim. Cached high-speed OSM was used for a 37 s rendering smoke check instead. First export exposed long polygons being dropped at near plane; subdivided footprints into <=3m strips for revised export. Vehicle body orientation still unresolved.

### Camera-only junction prototype (no OSM)
- Added core/scene_junction.py: repeated transverse paint components plus lateral road-appearance support, object masking, >=0.8 s temporal confirmation. It emits an explicit junction hypothesis; crosswalks alone, missing lanes alone, and map tags do not activate it. This is a rule-based near-field prototype, not a general learned intersection/topology detector.
- Integrated into new analyses in scene_pipeline; cached analyses require tools/enrich_scene_junctions.py to read original front frames and write a new result. This tool explicitly removes road_map for no-OSM validation. scene_fusion applies observed opening quads and interrupts lane lines within their projected band. Two observed transverse-moving tracks may support an additional cross-road band after compensating host odometry; they never estimate the host pose.
- Four 35 s clips were reprocessed (350 frames each). Final analysis-v3 active hypothesis counts: 8/19 intersection 331, 8/18 city 0, 8/31 rainy curve 0, 10/16 night parked 0. Earlier versions falsely detected road arrows/fragmented lines in city and were rejected. All four dates are now development via split-overrides.json; these are not independent accuracy claims.
- Corrected the first geometric attempt: near crosswalk bottoms were too close to the host; look for the road appearance band beyond the near crosswalk. Early videos intersection-no-osm*, intersection-camera-road* are intermediate reviews, not final accepted results.
- Tests: 208 full-suite tests passed before the latest component/geometry and transverse-motion refinements. Latest targeted junction suite 5 passed (markings+road jointly required, object exclusion, temporal gap reset, no-map opening, transverse motion with host compensation); fusion 19 passed after transverse support addition. Compilation and diff check passed.
- Limits: unmarked intersections, night intersection recognition, signal-free intersections, exact left/right branch topology, turn geometry, cross-road widths (assumed), metric extent and vehicle body heading are not solved. Current road-arm appearance and perspective projection still need broader original-video validation. Do not present this as complete intersection reconstruction.

### Orient moving vehicle models along travel
- Connected existing two-sided world-motion evidence to the actual Actor.yaw after position recovery and handoff stitching. Vehicles/cycles with consistent observed motion now face their ego-compensated direction of travel. Position coordinates are unchanged.
- Retain the established direction as a world vector through stops/prediction, converting it into each current ego frame. Clear memory at track disappearance, time gaps, or odometry reset. Actors without reliable movement evidence remain unknown in vehicle_headings audit; stationary body front/back and reversing are not inferred from motion alone.
- Verified sedan source mesh front is +Z (front nose/headlight coordinates in supplied generator), matching renderer yaw convention. Added transverse travel/no-position-change and stopped-heading/turning-host tests. Motion-heading 5 and fusion 19 passed; diff check passed.
- Re-exported no-OSM intersection, 35 seconds. Midpoint 18.5 s visibly rotates the crossing motorcycle and ghost vehicle compared with previous sideways-slide rendering. Do not claim all vehicles have verified body orientation: world-motion gates require >=3m/s and stable positions, and some tracks remain missing/unknown.

### Faster continuous heading response
- User observed coarse heading ticks. Parent timeline sample interpolated x/z but retained yaw; now yaw interpolates over the shortest circular arc at every playback sample (179 -> -179 goes through 180, not 0).
- Direction evidence previously fitted a straight line over +/-0.8s and could reject turning motion, retaining stale yaw. Replaced its position fit with a locally weighted quadratic and uses the centre derivative; independent temporal halves still validate consistency and position jumps remain rejected.
- Added a curved 5m/s trajectory test asserting heading at each analysis timestamp and a wrap-around/render-subframe interpolation test. Heading 6, driving-scene 7, fusion 19 tests passed; diff check passed.
- Comparison exporter now supports --fps 30, uses timeline.sample on every output frame, and verifies the full expected frame count. Object inference remains at the cached 10Hz; 30fps rendering is interpolation, not additional detections. Motion confidence gates and unknown/stopped heading policy remain.

### Rear lens rectification / video estimation attempt
- User requested correcting rear distortion before inference and confirmed no calibration is available; video-based estimation needed.
- Added scene_rectification.CameraRectifier for calibrated OpenCV pinhole/fisheye and experimental one-parameter division models. Remap is before detector/tracker/diagnostic output. Per-camera metadata records coordinate space, effective rectified matrix, source matrix, profile and validity fraction. Inference excludes detections centred outside valid remapped pixels.
- analyze_scene.py accepts --rear-calibration JSON. Fusion cameras_for uses the rectified focal/principal point, preserving the existing estimated extrinsic pitch; lens correction does not calibrate physical yaw/height/scale. Raw review overrides are rejected with rectified analysis to avoid mixing coordinate spaces. With no calibration, no remap is performed and metadata explicitly reports not_applied_no_calibration. Existing caches are not silently reinterpreted.
- Five rectification tests (identity/no mutation, fisheye ray, division inverse roundtrip, invalid profile rejection, geometry matrix consistency), analysis 9 and fusion 19 passed. Compilation and diff check passed. These are transform mechanics, not real calibration accuracy.
- tools/estimate_rear_lens.py attempted object-masked near-vertical plumb-line hypotheses on multiple existing dev and validation clips. Final run: 282 training / 391 validation edge hypotheses; clip-balanced scale-normalized error 1.3308 -> 1.1537 train, 0.6690 -> 0.6008 validation. Optimum +0.24999 lies on search boundary; independent improvement only ~10%. Hypotheses are not verified physical straight lines. Visual review of a candidate at 8/19 10s showed irregular nonvehicle boundary support. Result REJECTED, NOT APPLIED; no corrected real-world detection accuracy claim.
- Artifacts ../scene-results/rear-rectification/: rear-original.jpg, edges-*.jpg, edge-hypotheses.json, lens-estimate-report.json. No production calibration file was generated and no empirical coefficients were auto-enabled. This user request remains incomplete at actual usable coefficient estimation; next work requires trustworthy straight-line correspondences/model selection or a real calibration dataset, not arbitrary coefficient tuning.

### Retry actual intersection OSM acquisition
- User requested executing the OSM intersection check. overpass-api.de POST and GET both returned HTTP 502 on this turn. No cache was downloaded.
- Added cache reuse (validated existing payload), GET transport, and explicit alternate endpoint option to prepare_scene_map.py. Alternate overpass.kumi.systems request was REJECTED BEFORE EXECUTION by automatic approval review: prior authorization covered overpass-api.de, not filming-area transmission to a different destination. Do not bypass. Asked user explicitly whether the alternative destination is authorized; pending response at this point.
- Added tools/attach_scene_map.py to attach a cached payload to a NEW result without altering observations or originals. Ready to attach actual intersection data and render when acquisition is possible. Map-related tests 11 passed, including source preservation/byte-identical observation copy/no overwrite; compile/diff checks passed.

### Authorized alternate OSM destination / cached-map fallback check
- User explicitly authorized filming-area transmission to overpass.kumi.systems. Attempted the bounded GET query: timed out at 40s. Its GPS-free /api/status check also timed out after 15s with zero bytes. Original overpass-api.de status via curl showed TLS handshake failure; earlier Python POST/GET returned 502. No new intersection payload was downloaded, and no claim that rate limiting caused these errors.
- Checked existing OSM cache extents against target 2026-08-19 intersection: nearest cached points were ~4,386m (low) and ~10,577m (high), neither bounding box contains target. Do not attach these maps to the new intersection.
- Independently checked the existing clips' own caches: low 12 roads / 11 topology junction nodes / nearest node to route 9.21m; high 66 roads / 57 nodes / nearest 28.19m. These are graph junction candidates, not verified semantic intersection classifications. Report cached-map-junction-check.json.
- Attached low's own cached map to a NEW cached-low-analysis result with byte-identical observations, then rendered original-camera comparison. This is a different older source, not new-area verification. Export road-audit now records OSM and camera-junction status alongside corridor status for review.
- Cached-low render completed OSM attachment on all 370 analysis timestamps (surface_connected), junction_openings >0 on all 370, 3 local openings in first-frame audit. Inspected 18.5s source-comparison image: inferred 5-lane width and adjacent-road footprint visibly disagree with original. This verifies graph lookup/renderer connectivity only, NOT correct intersection reconstruction. New target 8/19 still has no downloaded map. Do not report success on target or road-registration accuracy.

### OSM context only; remove forced map registration (2026-11-13)
- User clarified OSM is a contextual topology reference; GPS cannot identify lane occupancy. Removed attach_map_surface invocation and deleted the experimental scene_map_surface module. OSM cannot replace road surfaces/lanes or change road_view/ego pose/actors.
- apply_map_roads now emits road_context (nearby junction, connected branch, curved-way candidates; camera confirmation required) and keeps the separate approximate map schematic. Camera/vehicle observations remain geometry authority. Fused playback retains the context metadata.
- Replaced superseded surface-generation tests with context/invariance tests: intersections and curves supply hints without changing complete scene geometry; GPS offsets 0/12/1000m and absurd map width/lanes do not change geometry. Map 11, junction 5, fusion 19 tests passed; diff check passed. No remaining runtime references to attach_map_surface.
- Actual cached-low 370-frame check: compared all actors x/z/yaw, road lanes/surfaces, road_view and ego display overrides before/after attaching its own OSM cache. Geometry changed on ZERO frames; map/junction context available on 370 frames. Report ../scene-results/map-topology-review/map-context-independence.json. This proves isolation, not full lane or intersection reconstruction accuracy. The older cached-low-osm-junction-37s video contains the rejected forced-map geometry and is superseded.

### Scene Lab MP4 export UI (2026-11-13)
- Added visible header button `3D 영상 저장 · MP4` for a loaded real-analysis timeline. It exports the currently selected camera composition's complete 3D timeline at 960x640/30fps without rerunning detection. Save dialog chooses destination; demo/observation-only/no-result states disable the button.
- core/scene_export.py streams renderer frames to local FFmpeg in a background SceneExportWorker, interpolating timeline.sample per frame. UI playback/control changes stop during export. Progress/status and cancellation use existing controls; closing requests interruption and waits for the worker. Output is staged in a temporary directory and only replaces the selected target after successful encoding. Cancellation preserves an existing file and removes partial output.
- Scope: 3D video only, no original-camera comparison/audio. Existing CLI comparison exporter remains separate. No build/release/push performed; Python source restart required.
- Verification: dialog 5 tests (including save-dialog -> worker -> cancel -> controls restored), real encoder/export 2 tests (6 decoded frames at 30fps, no overwrite unless authorized, cancellation preserves previous bytes and removes staging). Compilation/diff checks passed. Full suite not rerun for this UI-scoped addition.

### Perception model trials / experimental UI options (2026-11-13)
- User authorized testing improved models for a nominal >=4GB target. Downloaded official YOLO26m-seg and YOLOPv2 weights (GitHub URLs/hashes in ../scene-models/model-review/provenance.json). Python HTTPS trust store failed; Windows curl downloaded with normal certificate verification, no insecure bypass. Exported ONNX opset17 on CPU; no CUDA inference. Installed torch has CUDA build, but exports explicitly device=cpu / map_location=cpu and inference is ORT DirectML.
- YOLO26m-seg 640 and 960 raw exports have 80 class scores +32 mask coefficients and prototype outputs. Added multi-output InferenceEngine.run_all, InstanceSceneDetector with NMS-aware box/mask matching, original-pixel masks and JSON-safe polygons. Mask polygons are retained in raw and tracked observations. Existing box detector remains unchanged. Named -seg ONNX files select mask adapter via create_scene_detector used in UI/CLI.
- YOLOPv2 TorchScript road heads exported separately through scripted wrapper. RoadEvidenceDetector auto-detects YOLOP vs v2 schemas, uses v2's RGB/255 (not YOLOP ImageNet normalization), two-channel road argmax and single-channel lane probability threshold. New optional road_detector path in analysis stores segmentation curves/evidence and preserves Hough pitch observations. Front only; rear distortion still uncalibrated.
- UI analysis presets expose GPU vehicle masks YOLO26m 640/960 (experimental), optional YOLOP/YOLOPv2 road observer; CLI --road-model. Default detector/road settings remain unchanged. New analyses required; old caches are not silently upgraded. Latest diagnostics draw stored masks and segmentation curves.
- Same 12 original front images across 4 dates: YOLO11x 35.4ms, YOLO26m-seg640 55.3ms, seg960 95.0ms, YOLOP 78.6ms, YOLOPv2 84.6ms mean inference+decode. RTX4070Ti 12GB; sampled whole-GPU post-inference memory ~1244..1935MiB, NOT transient peak or model allocation; no 4GB-device guarantee. Detailed comparison.json + MODEL_REVIEW.md under ../scene-results/perception-model-review.
- Manual 8-target /2-keyframe comparison: all three vehicle configurations 7/8, same missed motorcycle. Masks add information but no demonstrated recall improvement. Road masks still classify crosswalk stripes as paint; do not promote directly to lane topology.
- Real combined pipeline: 3s smoke /4channels then 35s /4channels /350timestamps with YOLO26m-seg640 +YOLOPv2, DirectML no detector fallback. Saved combined-intersection-35s, 8,733 tracked mask observations and 350 road observations. Existing 15s target review: detected4/5, placed1/5. Thus model swap did NOT solve existing 3D position-fit failures. Depth model was deferred, not tested/adopted.
- Verification: full suite 224 passed in85.095s before final diagnostic-only refinements/one new road preprocessing test; latest targeted UI5 +mask2 +road1 passed; analysis9/inference4 passed earlier. Tests cover mask inverse letterbox coordinates, NMS mask association, RGB normalization/probability decode. Compile/diff checks passed. No build/release/push.

### Fix reproduced user-export lateral explosion
- Located matching original cache for user Downloads/driving-scene.mp4: ../scene-results/interactive/20261113-004658-23eff7, front source 2026-10-25_14-11-15, 500 samples /50s. Reproduced with current code: x=14.4m at30s,36.9 at35s,68.0 at40s,107.8 at45s; last road observations disappear and phase_source remains host_odometry_between_road_observations.
- Root cause: np.interp retains last world-road heading beyond final paint anchor; integrating curved-route host motion against that frozen tangent accumulates invented lateral displacement. Changed _road_frames to hold last visual lane phase/orientation outside the two-sided observation interval. Segments with no road observations use an explicit unobserved reference instead of unbounded fixed-tangent odometry. No lane snapping or hidden fabricated position accuracy.
- Scene ego_position_state=held_uncertain propagates through playback and HUD. Rendering displays EGO POSITION HELD / ROAD UNOBSERVED. Existing bounded intervals with road anchors on both sides still use compensated odometry and endpoint correction.
- Updated synthetic lane-change fixture to provide actual road paint evidence (otherwise bend vs lane-change ambiguity is unobservable). Added unsupported-curve/end-gap and entirely-unobserved-reference regressions. Vehicle-road10, fusion19 passed; final full suite 227 passed in84.312s.
- Actual corrected 500-sample trace: x range -3.418..+0.404m, last -3.411m; 311 timestamps held_uncertain. This prevents explosion but does not recover exact lane position over the missing road interval.
- Export ../scene-results/user-export-review/driving-scene-road-drift-fixed-50s.mp4: 1500 frames /30fps /50s, fully decoded. Inspected 28/40/49.9s frames: host remains visible. Old detector/positions from the user's original cache were used, isolating the coordinate fix from model upgrades. Other detection/placement failures remain. No original files modified, no build/release/push.

### Comparison export default and practical acceptance (2026-11-13)
- User clarified the aim is a plausible reconstruction, not perfection, and requested four original camera videos at the bottom of exports. This is now Scene Lab's default export: top 960x640 reconstruction + bottom 4x240x180 aspect-preserved originals, total960x820 at30fps. Order front/back/left_repeater/right_repeater. Same nominal source timestamp; missing source cameras display MISSING rather than invented footage.
- Core export supports manifest+source_directory, closes readers on error/cancel, and rejects overwriting one of the original source videos. New analyses store source_front_path. Existing analyses prompt for their matching original front file once in the save workflow. Header button now 3D + 4채널 영상 저장. Inference isn't rerun for saving.
- Tests: export4 (including real encoder composite dimensions/colour-coded 4-camera placement and same timestamp), dialog5 (save/cancel workflow with original path), analysis9 passed. Diff check passed. No full-suite rerun for this narrowly scoped export/layout change.
- OSM status: remains contextual topology only. No GPS/OSM centreline registration of scene geometry. Cached graph lookup works; new 8/19 target map acquisition still failed on both permitted servers. No additional server query in this turn.
- Revised same-source comparison exported as ../scene-results/user-export-review/driving-scene-fixed-4camera-50s.mp4. Decoded all1500 frames at30fps,960x820. Inspected49.9s frame: host remains visible and all four originals appear below. Existing frozen/uncertain road-reference limitation remains visible; no claim of exact road curvature/pose restoration. SCENE_ACCEPTANCE.md now records user practical-plausibility target and treats former stringent metrics as historical reference, not a new acceptance claim.

### False third lane / false host lane change in the same user clip
- User correctly identified the road as two lanes with the host remaining in lane2 throughout. Inspected original front frames0/10/15/20/30/49.9s: no host lane change. Previous 'drift fixed' video retained a false completed lateral transition and an extra lane; it only fixed explosion, not lane correctness.
- Old support audit: occupied[-1,0,1], fake -1 supported solely by ego drift. Requiring coherent paired road boundaries (similar slopes and width-compatible separation) before lateral phase updates removes false host crossing from isolated curved fragments. Single fragments/missing road cannot establish a new host lane phase.
- A subsequent pass exposed another fake lane from fused:17, a forward-moving lead car at a bend (not oncoming): held straight road reference placed it laterally7m away. If visual lateral references exist in the segment, only frames with valid camera lane phase can establish additional lane occupancy; unsupported distant-curve observations cannot grow lane count. No two-lane hard cap: regression explicitly preserves a supported three-lane scene.
- Current actual500-frame trace: lane_count2 at EVERY frame; host lateral -0.00695..+0.18036m, last+0.10775m, support0 ego/1 fused:3. Host display remains in its initial right lane rather than changing to phantom lane3. Absolute centring/curvature are still assumptions; this is no claim of all vehicle accuracy.
- Added regressions for single-fragment false host change, distant-car/unknown-bend false third lane, and retaining a truly supported third lane. Vehicle-road13 and fusion19 passed before full rerun. New composite output driving-scene-two-lanes-4camera-50s.mp4; previous fixed-4camera video superseded for lane correctness.
- Final verification: full232 tests passed in86.609s; revised two-lane composite decoded1500 frames/30fps and inspected49.9s original-comparison frame. Corrected display remains two lanes with host in its initial right lane; unsupported intervals still explicitly held_uncertain. Diff check passed.

### Repeater-to-front lateral pull audit / pixel identity switch correction
- User reported vehicles shift left on entering front camera. Audited observed handoffs in user's50s clip and two existing continuous sequences. User50 transitions: side/front camera-local x 4.566->5.242 at1.8->2.6s,3.884->4.140 at17.4->18.3s,3.975->3.765 at22.3->23.0s. These are not a demonstrated uniform front-camera zero offset. Full ego-compensated/world motion must be considered; no global camera principal-point offset was applied.
- Source review showed a distinct confirmed bug: original front:6 was a small far vehicle across the median at1.5..2.5s, then became the tall clipped overtaking truck at2.6s. Boxes108x53 ->37x254 had similar area, so MotionTracker's area-only shape gate accepted a different physical object. This contaminated shape/identity at handoff.
- Added per-axis width/height ratio consistency in addition to area/displacement gates. A severe aspect/height switch now starts a tentative new local track rather than continuing the old confirmed identity. Added targeted regression; existing fast-motion/two-stage tracking tests retained.
- tools/retrack_scene_observations.py replays cached raw detections without model inference into a new result, rejecting reviewed overrides and preserving originals. Saved ../scene-results/user-export-review/retracked-size-gate (500frames). Old cache is unchanged; newly analysed clips use new tracker, old clips need retracking/reanalysis.
- Corrected first actual handoff: side x4.566 at1.8s -> front x4.425 at2.9s, new front:11, vs original erroneous early front:6 x5.242 at2.6s. Other two handoffs still present with small opposite-sign lateral differences. This corrects one identity-driven pull, not calibrated front/repeater intrinsics or all cross-camera position bias. Road still2 lanes throughout.
- Targeted tests: analysis10,fusion19,handoff7 passed; diff check passed. Exported revised 50s four-camera comparison for visual inspection. Do not claim global camera calibration solved.
- Revised handoff export decoded1500frames/30fps. Inspected2.6/2.9/3.5/5s original-comparison frames: confirmed ID switch removal reduces initial transition, BUT the truck still hugs the left outer display boundary after entering front view. Thus user's lateral-alignment complaint remains partially unresolved. Need validate front projection AND camera-relative road curvature/reference rather than asserting a uniform zero offset or lane-snapping vehicles. Do not label this video as fully corrected front calibration.

### General left-lane outward bias: contact-priority fitting (partial improvement)
- User correctly reports that the bias affects multiple normally travelling left-lane vehicles, beyond the earlier ID switch. No camera principal-point offset, arbitrary global x shift, or lane-centre snapping was introduced.
- Confirmed one contributor: equal relative box-edge weights let representative body dimensions override the detected bottom. Actual user truck at5s: fitted bottom was14.9px above detector box bottom; raw fit x5.255m. Prioritizing the detected bottom reduces that to approximately0.04px and raw x4.486m. This is image agreement, NOT measured ground-truth vehicle position. Source inspection also shows that the detector bottom can truncate tyres, so it is explicitly only a contact proxy.
- Production fitting now ramps forward-view contact precision with apparent box height40..100px; distant small boxes and repeaters retain the prior weighting. Single-camera and joint fitting use the same objective; normalized box-error/IoU gates are retained. If contact-priority fitting fails its gates, retain the previously supported ordinary box fit rather than losing that vehicle. The initial no-fallback experiment lost some detections in other dates and was rejected.
- Camera-yaw/corners/ground-clearance experiments were insufficient to explain/remove the full bias; no speculative calibration or shape edits from those experiments were adopted. Road corridors still assume straight fixed-width geometry beyond reliable paint, leaving a separate curve/reference problem.
- Reproducible tool: tools/review_scene_ground_contact.py. Actual per-frame pitch,40sampled times per result; compares original/new objectives without modifying observations. Final report ../scene-results/user-export-review/contact-priority-fallback-review.json. Effective results: user10/25 accepted42->42, contact residual median4.89->1.34px; city8/18 accepted144->146,8.94->0.029px; night10/16 accepted39->39,5.98->0.004px; intersection8/19 accepted115->115,3.87->0.061px. Rain8/31 yielded no accepted comparison pairs and supplies NO evidence of improved placement. Box IoU can decrease slightly; this does not establish world-position accuracy.
- Final full user timeline: two lanes at all500frames. Truck displayed x5.21->4.535m at5s and x4.86->3.674m at8s. First sedan0.5s x4.89->4.561m; some remaining boundary bias is visible. Distant vehicle at30s still has unreliable placement on the held straight road; NOT solved, NOT acceptance-complete.
- Full237tests passed108.893s; targeted regressions cover contact agreement, exact synthetic reprojection, distant/repeater precision, and preservation of a previously supported fit. Export driving-scene-contact-reviewed-4camera-50s.mp4 is a progress comparison with four originals below, not an accepted final reconstruction. Earlier contact-priority experiment exports are superseded.

### 27s false ghost and opposite-road display scope
- Inspected25/26/26.5/27/27.5/28s of the actual exported50s comparison. At27s an erroneous ghost descended beside ego. Trace: WorldTracks accepted front:48 into a stranger whose existing source was front:39, after the known front:48 track's association gate failed. This manufactured a large negative velocity: fused:66 moved from about50m at26s to a predicted2.84m at27s. This was NOT a real approaching vehicle.
- WorldTracks now forbids a known live local identity from falling back to an unrelated live track, and forbids merging distinct live IDs from the same camera. Genuine handoffs between different cameras remain supported. Added two regressions for the steal and same-camera merge.
- User requests ignoring vehicles on the opposite/outside road. Added conservative post-road display scoping, with per-frame audit road_scope_excluded. Clearly outside actors with no established membership are omitted, including their ghosts. A visual road reference is required; same-road members survive noisy curve projections; active camera junctions/branch surfaces are exempt; longitudinal rear distance alone does not exclude a car. This uses the existing inferred envelope, NOT a new semantic median/barrier recognizer or surveyed road geometry. Added five boundary/memory/junction/unknown-road/rear regressions.
- Actual500-frame output remains two lanes. At27s only the legitimate front:48 lead remains at about32.62m; no stranger ghost beside ego. Excluded232 off-road actor-frame instances (not232 unique vehicles). Other cached cases exercised: city8/18,350frames/696excluded instances; night10/16,350frames/0excluded; intersection8/19,350frames/0excluded,331active-junction frames. Counts are diagnostic, not ground-truth recall/precision.
- Full244tests passed113.321s. New driving-scene-road-scoped-4camera-50s.mp4 decoded all1500frames/30fps/960x820 with four source views below; inspected exported27s frame. Source observations/videos unchanged. Residual left-lane position bias and held straight-road curvature are still unresolved; no overall acceptance claim.
- Subsequent SOURCE visual regression rejected the initial tight road-scope gate: in8/18 at6s, real overtaking white sedan front:7 (x3.76m,z3.45m) was wrongly removed because the inferred road omitted its lane. See road-scope-city-source-review.jpg. The initial road-scoped video/filter is superseded; its exclusion counts above describe that rejected experiment, not final validation.
- Final scope therefore allows a two-assumed-lane lateral uncertainty margin before excluding unrelated traffic, in addition to established-membership and junction exemptions. This deliberately cannot classify every nearby opposite-lane vehicle; without a verified separator, a narrow display boundary is insufficient to erase neighbouring traffic. Added a regression preserving actual-like near3.76m and further7m adjacent vehicles when the inferred corridor contains only ego lane.
- Final source visual review road-scope-city-final-6s.jpg confirms front:7 is retained; city excluded165 instances rather than696. Final user timeline excludes218 instances rather than232, remains two lanes, and has zero near predicted ghosts in26.4..27.3s. Final output is driving-scene-road-scope-reviewed-4camera-50s.mp4. 27s identity theft is independently fixed by WorldTracks; the scope filter is not a substitute for fixing that velocity/identity error.
- Final245tests passed119.139s; final video decoded1500frames/30fps/50s and exported26.5/27s frames visually checked. Overall quality_accepted remains false because remaining road/position limitations were not solved by this scoped fix.

### Ego turn display included, independent of lateral stabilization
- Root confirmed in the new8/22 source: vehicle heading305->199deg during5..20s (about106deg left turn), while missing paired paint made _road_frames return identity and the old ego display yaw stayed zero. Holding uncertain lateral phase must not hold body yaw.
- Added core/scene_ego_turn.py after lane-phase inference. Vehicle-heading odometry drives body/camera yaw even with missing paint; the rear-upper camera follows with0.8s time constant. Steering-wheel angle is not used as vehicle yaw. Paint-derived road angle cannot reverse actual turn direction. Actors and road use the same display rotation; original road scope reference is retained separately.
- Sustained turns add an estimated travel corridor sampled from the ego odometry trace, with known lateral lane phase removed. This does NOT identify junction topology, branch count, road edges, or fix the existing five-lane error. Endpoint continuation is a tangent assumption where the clip supplies no future path. Unknown heading does not invent a turn; stationary compass noise is ignored by existing odometry.
- Road, view and ego yaw interpolate between10Hz observations at video frame rate. HUD now distinguishes EGO LATERAL HELD from heading and labels estimated turn corridors. Existing saved observations can be reopened without rerunning model inference after restarting the app.
- Actual source8/22:15s world heading change-60.108deg, displayed yaw+17.779deg relative to following camera;18s world heading change-108.402deg.104frames use estimated turn geometry. Source9/4:34.5s actual clockwise change+33.241deg, displayed yaw-3.040deg;45turn-geometry frames. No lateral phase changes in either source.
- Original user50s regression: max lateral phase difference0, still2lanes in all500frames,27s only legitimate lead at32.623m and no near ghost. Raw observations and source videos unchanged; no model inference repeated.
- Eight regressions cover missing paint, heading wrap, stopped noise, missing heading vs steering, common transforms/scope, bad paint reversing a right turn, lane-change phase preservation, and render interpolation. Full253tests passed129.497s; diff check passed.
- Outputs ../scene-results/ego-turn-review/{2026-08-22_09-53-09,2026-09-04_10-50-27}-turn-4camera-35s.mp4. Each decoded1050frames/30fps/960x820, four originals below. Inspected exported15s left-turn and34.5s right-turn frames. configuration.json freezes implementation hashes; regression-review.json records numeric checks. Existing vehicle-placement, lane count and intersection-topology failures remain; full_scene_quality_accepted=false.
