# 자차 회전 시점과 정적 배경 교차 검증

8월 25일 전방→우측 리피터의 같은 어두운 세단은 GS25 앞에 정차해 있다. 후반에는 보도/나무가 차체 하단을 가린다. 상자 전체 높이를 차체 전체 높이로 간주하는 깊이 추정은 이 구간에서 오차가 커진다. 확대 원본은 `runs/aug25-rp-late-source-001/raw.jpg`, `crops.jpg`에 보존했다. 세단을 SUV로 적은 이전 기록은 잘못이다.

## 거부한 보정

실제 sedan GLB의 모든 정점을 고정된 카메라 광학/장착 prior와 SEI 궤적으로 원본에 재투영했다. 차량 위치·방향만 학습하고 49초 이후는 제외했다. `tools/probe_static_reprojection.py`를 사용했다. SEI만 사용한 궤적에서는 제외한 후반의 정규화 bbox 잔차 중앙값이 약 0.224였다. 정차라고 무조건 위치/방향을 고정하지 않았다. 영상과 맞지 않는 static fit은 채택하지 않았다. 이 진단의 bbox는 detector 출력이므로 수동 GT가 아니다.

## 독립적인 자차 회전 증거

`background_rotation.py`는 전방·좌우 리피터에서 추적된 동적 객체의 모든 bbox와 자체 차체/미러 영역을 제외한다. 왕복 optical flow → 왜곡 보정 좌표 → essential matrix/relative rotation 순서로 상대 회전 후보를 구한다. 건물의 정밀 형상이나 주변 차의 3D 위치를 계산하지 않는다.

`ego_turn_rotation.py`는 자차 SEI가 큰 회전을 보이는 시간 창에만 적용한다. 최소 두 카메라의 배경 각속도가 일치하고, 시간 coverage와 시작/끝 각도 일관성을 통과해야 한다. 창 양끝의 SEI 방위각은 그대로 두고 내부 회전 시점만 보정한다. 거리 적분은 원래 signed SEI speed다. 일반 차선 변경 정도의 작은 회전에는 적용하지 않는다. 한 카메라뿐이거나 불일치/증거 누락/과도한 누적 오차이면 원래 SEI를 유지한다. OSM/GPS 위치/다른 차의 운동은 입력에 없다.

8월 25일 두 회전 창의 카메라 합의 coverage는 각각 100%, 최대 시점 보정은 13.09°/17.23°였다. 10월 22일은 coverage 97.98%, 최대 6.78°다. 8월 16일 초반은 coverage 97.03%, 최대 7.92°이며 누적 오차가 허용 한계에 가까워 특히 원본 대조가 필요하다. 이는 실제 정답 각도 오차를 측정한 수치가 아니다.

8월 25일 두 번째 회전만 보정한 독립 실험에서 위 static GLB의 제외한 후반 bbox 잔차 중앙값은 약 0.094로 줄었다. 자차 회전 시점이 원인 중 하나임을 뒷받침한다. 수동 GT나 전체 정밀 3D 통과를 뜻하지 않는다.

## 실행·채택 상태

`runs/{aug25,oct22,aug16}-own-background-rotation-001`은 원본/추적/텔레메트리 해시 및 광학 해시를 가진 독립 evidence artifact다. `full-source-timeline --ego-rotation`이 입력 일치 확인 후 새 궤적을 만든다. 이전 base/영상은 수정하지 않는다.

세 원본 풀버전001~005를 각각 동결·생성·대조했다. 001의 연석 차량 삭제는002에서 복구했다. 003의 큰 search polygon을 asphalt로 그리던 문제는004에서 짧은 camera-anchored opening으로 분리했다. 005는 중앙선과 bbox 모서리가 겹치는 접근 차량도 반복된 centre/looming/미래 own-side veto로 구분한다. Aug25 oncoming SUV와 Oct22 Lexus 확대 원본을 확인했다. 세 영상3,456프레임/마지막5초 수치 후보0은 전체 source recall 합격과는 다르다.

Aug25 59.05초005 검수본: local `runs/own-turn-rotation-suite-005/aug25/scene/surrounding-world.mp4`, Drive 테스트26 `1-pMMerYXIw5aetzV-7DydM-MsmkKkwHT`. 크기13,402,296와 부모를 readback 대조했다. `pipeline.py`에도 own-rotation→새 base 경로를 연결했고, `new-date-suite-007` 네 날짜와 `branch-orientation-suite-009` 기존 네 날짜를 같은 새 경로로 회귀 중이다. 모두 개발 자료이며 새 holdout이 아니다.

`static_actor_refine.py`는 선택된 stationary clear-window handoff와 반복된 front body angle이 있는 자유 차량만 검사한다. 실제 GLB의 bbox 재투영으로 차량XY/yaw만 적합한다. 카메라/ego/scale/ID/도로/frozen lead는 바꾸지 않는다. 뒤27% 시간은 적합에서 제외한다. 완전한 bbox 등식은 가려진 하단 때문에 실패했다. 후속은 roof/좌우 extents는 등식, visible bottom은 foreground에 가릴 수 있는 하한으로 다룬다. 문턱을 올리지 않았고 uncensored 오차도 보존한다. 실제로 움직이는 제외 시간의 합성 음성은 거부한다. 실측 합격이 아니라 조건부 정차 가설이다.

`runs/aug25-static-source-reprojection-003/source.jpg`의 detector빨강/mesh녹색을 직접 검토했다. 제외 median0.0529/p900.2273, uncensored full-box p900.2692. 해당35.19–53.81초의 같은 세단을 고정 위치/방향으로 재현한다. 초기29–31초의 작은 bbox는 충분히 맞지 않아 억지 고정하지 않았다. 유한 후보정 경계는 smooth blend로 위치 점프를 막는다. 클립명별 위치/각도 상수는 없다.
