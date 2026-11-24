# Pilot 원본 검수와 시간 계약 보강

## 실제 브라우저 검수

computer-use의 browser API로 원본 정답 도구를 열어 실제 canvas drag, ID 입력, 박스 확정, 프레임 전체 확인, JSON 다운로드와 다음 프레임 이동을 확인했다.
다운로드 파일은 backend validator로 검사했고 1개 라벨/1개 확인 프레임이 기록되었다. ID는 `__ui_test__`이며 `runs/annotation-ui-review-001`에 test-only로 보관했다. 실제 GT나 평가 점수에 포함하지 않는다.

발견한 결함:

- 박스를 추가해도 화면의 labels 개수가 다음 프레임 이동 전까지 갱신되지 않았다.
- 이미지 로딩 중 이전 canvas에 새 프레임 ID로 박스를 작성할 가능성이 있었다.
- 이미 확인한 프레임에 라벨을 추가/취소해도 확인 완료가 해제되지 않았다.

소스에서 개수/확인 상태를 즉시 갱신하고, 로딩 중 입력 금지 및 실패 이미지 확인 금지, 라벨 변경 시 재확인 요구를 추가했다.
원본 artifact는 변경하지 않았다. 수정 UI는 `runs/pilot-queue-source-002`에 새 artifact로 게시했다. 수정 전의 실제 조작 검수와 수정 후 코드/계약 검증을 구별하며, 수정 후 브라우저 재검수와 JSON 재가져오기는 아직 미완료다.

## 선택 차량의 원본 reference

front 0/15/33초 원본을 보며 선행 흰색 van과 우측 SUV의 raw bbox, 상대 차로 관계, 우측 truncation 여부를 작성했다.
`runs/pilot-reference-review-001/sparse-references.json`의 6개 항목이다. source frame hash와 실제 PTS를 참조한다.
정확한 metric 위치/yaw는 작성하지 않았다. 이는 선택 차량의 sparse reference이며, 700프레임의 모든 차량·도로·사건 정답이 아니다. frame 전체 검수 또는 전체 recall 계산에 사용할 수 없다.

## 새 SEI 파서의 실제 실패 수정

pilot 원본 정지 부분의 speed는 -2.4835267709022446e-07 m/s 같은 미세 음수다.
초기 새 파서는 speed>=0 조건 때문에 이 값을 invalid/null로 만들었다. 실제 35초 1,261개 메시지 중 874개가 해당했다.

schema는 float 속도이며 부호 자체가 손상 증거는 아니다. finite raw signed speed를 그대로 보존하도록 수정했다. 음수에는 해석 필요 flag만 추가하고 0으로 clamp하지 않는다. nonfinite만 null/invalid로 남긴다. 정지 판정·gear를 통한 운동 해석은 후속 ego 단계의 책임이다.

수정 결과: `runs/pilot-queue-sei-002`, 1,261개 메시지, 속도 nonfinite 0, 미세 음수 값 보존. 구 `pilot-queue-sei-001`은 실패 이력으로 남겼다.
이 동작은 테스트로 고정했다. 센서 속도의 실제 정확도를 검증했다는 뜻은 아니다.

## typed 계약

- FrameRecord: camera ID, source SHA, 정수 PTS/timebase, 무효 timebase 거부. unverified clock에 확정 session time을 넣지 못한다.
- Clock: 유효 구간·불확실성 없이 estimated/verified 선언 불가. 유효 구간 밖 extrapolation 거부.
- ActorEvidenceState: visible-but-unlocalized와 occluded를 분리. unresolved 위치의 solid/ghost 생성, 현재 관측 없는 solid 표시를 거부한다.
- 실제 PTS audit의 frame 생성 경로에도 FrameRecord 검증을 연결했다.

## 남은 작업

브라우저 수정본 검수, 전 구간 원본 GT와 사건/차선 reference, 독립 split 선정, rig/clock 실험이 남아 있다. 인식·공통 3D 궤적·도로 품질은 미평가다. 기존 소스와 원본 영상은 수정하지 않는다.
