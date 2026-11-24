# 수동 교정본

실험실 → 주행 재현 v2 → 자동 결과 열기 → 차량이 있는 시점으로 이동 → **차량 수동 교정본**.

차량 ID, 시작/교정/종료 시점, 월드 X/Y 이동량, 방향 보정, 원본 대조 근거를 입력한다. 시작과 종료에서는 자동 위치로 돌아오고 그 사이를 보간한다. 좌표는 고정 월드 좌표이며 자차 기준 좌우가 아니다. 자차·도로·ID·가시성은 바꾸지 않으며 관측되지 않은 차량을 새로 만들지 않는다. 같은 차량의 겹치는 교정 구간은 거부한다.

JSON layer와 새 MP4는 `runs/manual-review-<timestamp>` 및 옆 `.corrections.json`으로 저장한다. 원본 자동 결과를 덮어쓰지 않는다. 결과에 노란 `MANUAL REVIEW COPY` 표시를 넣고 내보내기 metadata에도 수동 교정임을 기록한다. 여러 교정은 같은 원본에 묶인 JSON의 `edits` 배열로 입력할 수 있다. 현재 UI는 한 번에 한 구간을 만들며 다중 구간 timeline 편집기는 아니다. 수동 결과를 다시 자동 결과로 간주하여 교정을 누적하지 않는다.

CLI는 `python -m ct_scene manual-render --scene <자동 scene> --base <연결된 base> --corrections <JSON> --assets-root <GLB 폴더> --prototype-config configs/surrounding-prototype.json --output <새 결과>`다. layer는 원본 artifact ID에 묶이며 다른 영상이나 다른 자동 결과에 적용하지 않는다. base ID도 검증한다. 모델·코드 hash와 원래 자동 artifact를 기록한다.

수동 결과는 자동 성능 평가에서 제외한다. source-control evaluator가 이를 명시적으로 거부한다. 원래 road/lane audit은 자동 결과에만 해당하며 수동 위치의 차로 정합은 다시 원본과 대조해야 한다. 자동 추정 개선을 대신하는 기능이 아니다.

실행 검증: `runs/manual-review-integration-002`, `runs/manual-review-integration-check-002/review.json`. 합성 교정 0–2초만 사용했다. 850프레임 중 39개만 변하고 자동 원본/자차/카메라/다른 시점은 유지됐다. peak 이동량 0.5m/0.25m, yaw 5°를 확인했고, 하단 4채널 재인코딩 PSNR 41.90dB, UI 수동 표시/MP4 바이트 동일 내보내기/수동 metadata를 검사했다. 합성 통합 검사이며 원본 정확도 보정의 실증은 아니다. offscreen Qt 검사이며 실제 사용자 조작 검수와 구분한다.
