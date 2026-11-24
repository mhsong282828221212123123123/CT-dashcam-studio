# 수정 계획 실행 기록

최신 사용자 지적에 따른 [도로 밖 차량 삽입 수정 기록](./ROAD_MEMBERSHIP_CORRECTION_20261115.md)을 우선 참조한다. 아래 방향 수정본의 검사는 해당 방향에 대한 이력이며, 해당 제공 영상은 이후 도로 membership 실패로 기각했다.

사용자의 계획 진행 지시로 작업을 재개했다. 기존 중지 manifest와 산출물은 보존한다.

- 평가 guard를 추가해 빈 결과·누락/중복 case·빈/불연속 검사 구간·원본보다 짧은 출력이 거부되도록 했다.
- source evaluator와 기존 전체 집계 도구가 FAIL을 정상 종료로 내보내던 경로를 수정했다. 기존 실제 source FAIL 보고서를 재평가한 `runs/correction-gate-negative-001`은 종료 코드 2를 반환한다.
- `resume_selected_scenes.py`가 동결 엔진을 사용해 미완료 `aug26`, `oct22` 두 개만 생성했다. 기존 12개는 artifact identity/무결성을 확인하고 참조로 재사용했다. 입력 목록은 `runs/correction-baseline-001/cases.json`이다.
- 원본 시계와 대조하는 새 manifest 평가 결과 `runs/correction-baseline-review-001/review.json`: 14개 중 13개 수치 PASS, Aug20 한 개 FAIL. 전체 합격으로 표시하지 않았다.
- `runs/correction-baseline-source-witnesses-001`의 선택 핵심 사건 16개는 PASS다. 전체 객체 정확도의 결과는 아니다.
- Aug20 FAIL은 `front-0017`의 52.65/52.70초 body 방향 불연속 두 건이다. 처음에는 다른 카메라 표본 연결의 π 위상 문제로 추정했지만, 아래 원본 대조에서 기준 궤적 자체의 최초 앞뒤 선택 오류로 확인했다.
- `merge_anchor_samples`에서 인접한 외부 카메라 표본만 anchor body 축에 연결하도록 수정했다. 고정 관측·위치·ID는 변경하지 않는다. 긴 실제 가시 공백의 재등장은 강제로 같은 위상으로 지정하지 않는다.
- 후보의 코드 검사 289개는 통과했지만 전체 영상 검사에서는 불연속이 2→3개로 늘었다. 이 수정안은 미채택하고 production 코드를 기준 상태로 복구했다. 후보 코드·테스트·실패 이유는 `runs/correction-anchor-phase-rejected-001`에 보존했다. source 50/59초 검사는 통과했어도 전체 수치 FAIL을 합격으로 처리하지 않는다.
- 다음 원인 분리는 52.587초까지의 base body 방향, 그 직후 생성된 handoff gap의 속도 기반 방향, 실제 원본의 차량 identity/방향을 각각 대조하는 것이다. 합성 단계에서 동결 base 방향을 덮어쓰거나 수치 임계값을 올려 숨기지 않는다.

전체 목표는 진행 중이다. 최신 후보의 전체 원본 대조, 영향 회귀 및 계획의 후속 단계는 남아 있다. CUDA·빌드·릴리즈·푸시는 수행하지 않았다.

## 기준 차량의 최초 방향 보정과 비교 검사

- 원본 확대에서 대상은 회색 SUV이며, 전방 `front-0017`에서 우측 `right_repeater-0068`로 연결되는 같은 차량임을 확인했다. 가까이 있던 흰색 해치백과 구분했다. 증거 이미지: `runs/anchor-body-source-001.jpg`.
- 최초 기준 body 방향은 실제 뒷면과 반대로 선택되어 있었다. 반복된 후면 외관 근거 29/29개가 반대 위상을 지지했다. `full_timeline`에서 새 기준을 발행하기 전에만 π 선택을 보정한다. 합성 단계의 동결 기준 덮어쓰기나 좌표 이동은 하지 않는다.
- `actor_rear_axis.orient_unpublished`는 최소 5개 관측과 90% 이상 일치가 있을 때 일정한 π 보정만 적용한다. 근거가 부족하거나 이미 맞으면 보류한다. 코드 검사 289개 통과.
- `runs/correction-body-phase-full-001`: 전체 59.8907초, 1,198프레임. `correction-body-phase-numerical-002`에서 불연속 0건, 원본 시계 및 마지막 5초 검사 통과. `correction-body-phase-source-001`에서 50/59초 두 차로와 전방 두 차량의 우측 차로 보존 통과. 원본 4채널을 포함한 20–54초 선택 화면을 대조했다.
- 기존 정상 Audi 구간도 다시 발행·합성했다. `correction-body-phase-negative-001/invariance.json`에서 **기준 궤적의 모든 필드가 기존과 동일**하다. 전체 1,190프레임 수치 및 원본 시계 검사는 `correction-body-phase-negative-review-001`에서 통과했다. 선택 원본 사건 16개는 `correction-body-phase-negative-witnesses-001`에서 통과했다. 이 중 해당 Audi만 새 합성이고 나머지는 기존 비교 기준임을 명시한다.
- 14개 입력 모두의 최초 기준 방향에 대한 추가 회귀를 `tools/review_initial_body_phase_suite.py`로 실행한다. 방향 외 필드의 완전 동일성을 검사하고, 방향이 달라지는 사례는 원본 재검토 대상으로 남긴다. 재합성하지 않은 사례를 새 엔진 전체 영상 합격으로 보고하지 않는다.
- 개발 검수 MP4를 테스트26에 업로드했다: https://drive.google.com/file/d/1Z576pijcfudHTApAESxiMAnUEO9DsS8u/view . 파일 크기 12,059,018바이트와 목적 폴더 ID를 확인했다. 전체 객체 일치나 전체 계획 완료를 의미하지 않는다.

## 14개 영향 회귀 완료와 다음 실패 분리

- `runs/correction-body-phase-regression-001/review.json`: 14/14 기준을 같은 동결 엔진으로 재계산했다. 모든 구간에서 방향 외 궤적 필드가 완전히 동일하다. 12개는 방향까지 전부 동일하고 Aug20 및 `2026-08-26_08-33-44`만 body π 위상이 바뀌었다. 이것은 기준 궤적 회귀이며, 14개 전체 영상을 새 엔진으로 다시 합성했다는 의미가 아니다.
- Aug26의 `front-0005`는 원본 11·15·20·25·30·35초에서 같은 제네시스의 뒷면임을 확인했다. 근거 64/64개가 기존 위상의 반대를 지지한다. `runs/correction-body-phase-aug26-source-001.jpg`에 원본과 crop을 보존했다.
- Aug26도 전체 58.8353초, 1,177프레임을 다시 합성했다. `runs/correction-body-phase-aug26-review-001`에서 수치 이상 0건, 전체 원본 시계 및 마지막 5초 검사 통과. 11·20·30·40초 원본 4채널 포함 화면을 확인했다. 주변 차량/교차로의 전체 일치 합격은 아니다.
- Aug26 MP4도 테스트26에 올렸다: https://drive.google.com/file/d/1vkT5fGx8hCaVfXcTIdQrkTodZqGlUjeM/view . 11,898,322바이트와 목적 폴더를 확인했다. 최신 코드 검사 289개 통과.
- 다음 단계의 원본 검토 대상을 얻기 위해 `tools/review_visible_exclusions.py`를 추가했다. 기존 14개 결과에서 큰 차체가 반복 관측됐지만 도로 범위로 제외된 175개 track을 모았다. 상위 12개 원본 화면은 `runs/correction-visible-exclusion-queue-001`에 있다. 반대편·주차 차량이 포함되므로 **175개 누락/오류 또는 recall 점수로 세지 않는다**.
- Aug26 버스 `front-0001`(11.7906초까지) → `left_repeater-0016`(12.8008초부터) 후보를 우선 검토했다. 원본은 같은 11번 노선의 파랑/초록 버스의 후면→우측면→전면을 보여준다. 리피터 track은 `outside_same_direction_corridor`로 제외된다. 하지만 임시 진단에서 lane 조건만 같게 두어도 기존 연결 후보가 생성되지 않았다. 두 추정 끝점 차이는 약 12m다. scope 하나만 풀어 해결됐다고 보고하지 않는다.
- 버스 진단은 `runs/correction-bus-scope-diagnostic-001.json`, 원본 비교는 `runs/correction-bus-handoff-source-001.jpg`다. 이 진단에서 임시 lane을 준 결과는 최종 ID/월드/합성에 반영하지 않았다. 카메라 전환 시의 잘린 전방 box 지면 접점과 차체 중심 offset, 리피터의 선명한 후속 관측을 다음 보정 대상으로 삼는다. 외관만으로 연결하거나 거리 문턱을 넓히지 않는다.
- 추가 분리 결과: 기존 양방향 선명 관측 fit을 사용하면 버스의 연결 거리 2.3846m, 속도 차이 1.8305m/s, 각 window의 잔차 p90 0.2475/0.0814m다. 따라서 약 12m는 **잘린 끝 프레임 자체의 차이**이며 이 차량의 선명 관측까지 모두 12m 틀렸다는 뜻이 아니다. 현 연결의 추가 거부 원인은 HSV 외관 거리 0.6867이 기준 0.65를 넘는 것이다. 밝기를 제외한 색상 성분 거리도 진단했지만 이는 확인된 새 identity 근거가 아니므로 적용하지 않았다. 복구 순서는 (1) 범위 밖 후보도 ID 진단에 보존, (2) 선명 관측 fit으로 잘린 위치만 복원, (3) 원본 ID와 대조한 카메라 간 외관 근거 개선이며, 거리/외관 문턱을 단순 완화하지 않는다. 재현용 후보 쌍은 `runs/correction-bus-handoff-pair-001.json`에 있다.
