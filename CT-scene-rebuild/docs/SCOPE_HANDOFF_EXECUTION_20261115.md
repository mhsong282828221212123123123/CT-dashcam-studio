# 수정 계획 단계 3 — 범위 판정과 카메라 전환

최신 검수본은 **engine005 / `runs/scope-handoff-fourteen-005/cases.json`**이다. 아래 engine001–003 기록은 원인 분리 이력이다. 전체 목표는 아직 완료가 아니다.

사용자의 계속 지시에 따라 도로 밖 삽입 수정 이후 계획을 진행했다. 전체 계획 완료 문서가 아니다.

## 변경

- 전방/측방에서 추적되고 3D 후보까지 만들어진 객체를 road scope 실패만으로 identity 검사 전에 삭제하지 않는다. `scope_pending`으로 후보에 보존한다. 명확한 영상 중앙선/분리대 제외는 이 후보 경로 전에 유지한다.
- 미확정 범위 후보는 이미 채택된 객체와의 독점 연결이 있어야 world에 들어간다. 미확정 후보끼리의 연결만으로 world 차량을 만들지 않는다. 채택된 연결이 없으면 기존 범위 제외를 유지하며 원본 관측은 보존한다.
- 이 연결은 FOV 경계 전환, 양쪽 선명 관측 motion fit 거리 3m 미만·속도 차이 2m/s 미만, 반복 identity 근거 및 기존 배타성 검사를 요구한다. lane unknown을 같은 차의 연결을 검사하지 못하는 이유로 쓰지 않는다. 거리 기준을 느슨하게 하지 않았다.
- 세 장의 평균 외관이 거부될 때만, 위 독립 motion 조건을 만족하는 쌍에서 양쪽 최대 12개 선명 원본 관측을 비교한다. 최소 여섯 개의 서로 다른 frame과 1초 span이 필요하다. 모든 쌍의 거리 중앙값을 사용해 우연히 한 장만 닮은 것을 배제한다. 기존 appearance 한계 0.65는 유지한다.
- `report.json`의 `scope_identity_review`가 미확정 후보의 원래 사유, 연결 global ID 또는 world 제외, 원본 관측 보존을 기록한다. global actor에는 identity로 보존된 scope member 목록을 남긴다.
- bus 첫 후보는 전체 검사에서 12.8초 차체 회전 한 건으로 기각했다. 연결 공백의 Hermite 위치 궤적 접선을 차체 방향으로 쓰는 것이 원인이었다. 위치 궤적은 유지하고 차체 방향만 양 끝의 실제 body 방향 사이에서 잇는다. 고정 anchor는 변경하지 않는다. 실제 양 끝 회전은 보존하는 paired test를 추가했다.

## Aug26 원본 사건

`front-0001` → `left_repeater-0016`은 원본의 같은 11번 노선 버스다. 증거: `runs/correction-bus-handoff-source-001.jpg`.

- 양쪽 선명 window 연결 거리 2.3846m, 속도 차이 1.8305m/s.
- 기존 세 장 평균 appearance 0.6867에서 여러 선명 관측의 pairwise 중앙값 0.6383으로 판정 근거가 바뀌었다. 실제 frame ID 24개와 144개 비교를 edge에 남겼다.
- 최신 `runs/scope-handoff-aug26-full-002`에서 같은 ID로 연결됐다. 12.85–24.10초 world 존재와 solid 유지, 다른 동시 관측 버스 미연결을 `scope-handoff-bus-source-002`에서 확인했다.
- 기존 제공 영상은 동일 검사에서 실패했다: `scope-handoff-bus-negative-control-001`.
- 12.8/14/16/19초 원본 4채널 포함 변경 전후 화면을 확인했다: `scope-handoff-bus-before-after-002.jpg`. 버스를 우리 차로로 옮기거나 새 차로를 만들지 않고 왼쪽 공간에 유지한다. 도로 면/교차로 표현 개선은 별도 단계다.

## 현재 검사

- 코드 검사 300개 통과. 중복 frame, 한 장만 닮은 경우, 두 후보가 동등한 경우, 연결 거리 불일치, 미확정 후보끼리의 연결, 실제 차체 회전 유지도 포함한다.
- Aug26 및 Aug20 최신 전체 영상의 수치/원본 시계/마지막 5초 검사 통과. 도로 밖 주유소 차량·건너편 차량의 강제 삽입 방지와 정상 우측 차량 유지도 통과했다.
- 같은 동결 엔진 `runs/scope-handoff-engine-002`로 14개 전체 회귀 진행 중. 입력/완료 manifest는 `runs/scope-handoff-fourteen-002/cases.json`이다. 현재 엔진으로 이미 완성된 네 구간은 artifact/code revision/base 부모를 확인해 재사용하고, 나머지 열 구간만 재합성한다.
- world 존재와 화면 범위를 구분하는 `tools/audit_scene_frustum.py`를 추가했다. 실제 renderer MVP와 모델 bounds로 검사한다. `scope-handoff-frustum-002`에서 버스는 world 486프레임 중 보수적인 frustum 후보 386, 화면 밖 100이다. 이는 raster pixel 가시성/가림 검사나 recall 점수가 아니다. 실제 pixel 가시성/가림은 미평가로 명시한다.

이전 도로 삽입 수정은 유지한다. 원본·기존 코드·이전 실패 영상은 보존하며 CUDA·빌드·릴리즈·푸시는 하지 않았다. 전체 14개 결과와 선택 원본 사건 검사가 끝나기 전에는 단계 전체 완료로 표시하지 않는다.

## 추가 회귀와 원본 라벨 감사

- engine002 회귀는 13개 생성/수치 PASS였고 aug28 한 개가 존재하지 않는 local lane 번호 접근으로 실행 실패했다. 실패 manifest는 보존했다. engine003은 해당 station에 이전 lane이 없으면 보정을 보류하도록 수정했다. 섹션 축소 paired test를 추가했고 코드 검사 301개가 통과했다.
- aug28의 engine003 전체 영상은 수치 PASS, 원본 자차 차로 변경 완료 시점도 유지된다. 코드가 섞인 13+1을 동일 엔진 14개 합격으로 쓰지 않고 `runs/scope-handoff-fourteen-003`에서 같은 engine003으로 다시 실행 중이다.
- 원래 source annotation과 gate를 그대로 재평가한 `runs/scope-handoff-original-label-audit-003`은 Audi PASS, Hyundai/Carnival 상대 차로 FAIL로 이전과 같다. 세 객체의 solid coverage는 모두 1.0이며, 이 결과는 차로 unknown 실패를 면제하지 않는다.
- 원본 재감사 `runs/source-lane-label-audit-001.jpg`에서 Carnival의 50–52초는 표식이 있는 신호대기 접근부임을 확인했다. 과거의 ‘주차 구간’ 설명은 잘못됐다. 원본 라벨을 완화하거나 FAIL을 지우지 않았다. 이 구간은 전역 paint reference가 성립하지 않아 전체 clip을 unknown으로 두는 경로의 문제이며, 이후 국소 도로 근거 처리에 남아 있는 실제 한계다.
- 실제 타 차량 차로 변경/합류 양성 사례를 찾기 위해 Oct02·Oct18 원본을 확인했지만 이 접촉 화면만으로 새 positive label을 확정하지 않았다. 자차 변경이나 거의 빈 톨게이트 접근을 실제 타 차량 변경/합류 합격으로 세지 않는다.

## 최종 회귀와 추가로 발견한 반대편 차량

- engine003 14개가 모두 수치 PASS였지만, 새로 보존된 scope member를 전부 대조하다가 Oct22의 `front-0016` → `left_repeater-0003`이 반대편 벤츠임을 확인했다. 같은 차량의 ID 연결은 맞아도 표시 대상은 아니었다. 원본: `runs/scope-handoff-oct22-additional-source-003.jpg`. engine003은 새 paired source 검사에서 FAIL이다: `scope-handoff-opposing-negative-control-003`.
- 전방 box가 화면 끝에서 잘려 작아지는 것을 후퇴로 오해해 기존 중앙선 제외가 보류됐다. 다섯 개 이상·0.8초 이상의 선명한 whole-body 관측으로 접근을 입증할 수 있을 때 그 근거를 유지한다. 기존 whole-body/own-side/crossing/ego-turn 보호는 유지한다. 새 unit test는 이전 engine003에서 실제 실패하며 수정 코드에서는 통과한다.
- 초기 후보에서 다른 기존 제외가 해제되는 변화도 발견해 채택하지 않았다. 최종 engine005의 모든 14개 cached median 판정을 다시 계산한 `scope-handoff-median-decision-audit-005.json`에서 바뀐 제외는 문제의 벤츠 한 대뿐이다. 원본/평가기 hash를 남겼다.
- 혼합 코드 13+1을 합격으로 쓰지 않고, **engine005 하나로 14개 전체를 다시 생성했다**. `scope-handoff-final-review-005`: **16,344프레임, 14개 모두 전체 시계/마지막 5초/수치 PASS**. source는 별도이므로 전체 gate는 `REVIEW_REQUIRED`다.
- `scope-handoff-final-scope-pair-005`: 같은 버스 보존·solid·다른 버스 미연결 + 반대편 벤츠/리피터 제외·원본 황색선 근거 통과.
- `scope-handoff-final-outside-source-005`: 주유소/반대편 차량의 차로 삽입 방지, 정상 우측 차량의 차로 유지 통과.
- `scope-handoff-final-visible-source-005`: Aug20 후반 두 차로와 59초 전방 두 차량 보존 통과.
- `scope-handoff-final-witnesses-005`: 16개 선택 사건을 **전부 최신 engine005 영상으로** 재평가해 통과했다. 횡단 차/버스·트럭, 선택 반대편 차량 제외, 카메라 연결, 정차 body, 자차 변경 완료, 빈 도로를 포함한다.
- `scope-handoff-final-invariants-005.json`: 모든 동결 전방 anchor 보존, 의도한 반대편 벤츠 외 기존 local ID 소실 없음, 도로는 각 입력 장면과 동일. 버스를 우리 도로로 옮기거나 road lane을 추가하지 않았다.
- 코드 검사 **302개 통과**. source 원래 annotation/gate로 재검사한 `scope-handoff-original-label-audit-005`에서 Audi PASS, Hyundai/Carnival 상대 차로 FAIL은 유지된다. full source recall, 실측 3D, 실제 타 차량 차로 변경 양성 검증, 전체 raster occlusion 검수 완료로 확대하지 않는다.
- 최종 버스 MP4는 앞서 화면으로 대조한 engine002 영상과 SHA256이 같음을 확인했다. 최신 artifact의 입력과 provenance는 engine005다. 58.8353초 원본 전체를 테스트26에 제공했다: https://drive.google.com/file/d/1AFb7b2iEodolRWL-IyxxaL9wGyh2ZeJT/view . 11,989,305 bytes와 부모 폴더를 검증했다.

## 이어갈 계획 범위

이번 결과는 단계 3의 핵심 누락·잘못된 scope 연결 수정과 14개 회귀 milestone이다. 전체 계획 달성을 선언하지 않는다. 단계 4의 immutable scene state/후보정 revision 분리, 독립 도로 근거와 실제 타 차량 변경 양성 검증, 단계 5의 camera-local 도로 support 및 교차로/합류 면 개선, 단계 6의 최종 UI/다양성 원본 검수가 남아 있다. 폴/사람은 계속 낮은 우선순위다.
