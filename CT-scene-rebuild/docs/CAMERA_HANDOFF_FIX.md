# WP06: 카메라 전환과 양쪽 관측을 이용한 공백 복원

## 원본 검수에서 바로잡은 사항

이전 메모는 `right_repeater-0011`을 Sorento SUV로 잘못 분류했다. 확대 원본은 세단이며 전방 재등장 `front-0060`의 번호판은 56더1869로 최초 선행차와 같다. 기존의 서로 다른 차라는 검수 판정이 잘못됐다. `right_repeater-0010`과 `front-0071`도 번호판 43너4300으로 같은 흰 세단임을 확인했다. 반면 `right_repeater-0009`는 다른 어두운 차량이다.

원본 비교: `runs/handoff-review-001/actual-pairs.jpg`, 전체 맥락 `candidate-source.jpg`. 실제 판정에는 번호판·차체·시간 순서를 함께 확인했다. 번호판이나 위 ID는 검수 fixture일 뿐 production 분기/강제 연결 값으로 쓰지 않는다.

## 구현

- 잘린 전방 진입 box의 하단은 접지점으로 불안정하다. side→front 후보에서 처음 온전한 전방 detector 관측부터 1.1초 내 관측(5개 이상, 0.8초 이상)을 fit해 과거 endpoint와 대조한다. 첫 명확 관측까지의 지연은 최대 3.2초, 과거 endpoint까지 역추정은 최대 4.5초, fit residual p90은 0.75m 이하, 전진 속도는 45m/s 미만이다. 명시적인 미래 운동 추정이며 측정된 거리/보정값이 아니다.
- 선택된 연결에서만 잘린 전방 prefix의 종방향 geometry를 미래 속도로 복원한다. 명확한 전방 관측·자차·횡방향 prior는 유지한다. 지원 상태는 `offline_future_clear_geometry`; 원본에 실제 차가 있으면 solid clay다. 큰 camera offset을 학습하거나 전체 영상을 이동하지 않는다.
- 측면에서 차체가 넓게 8프레임 이상/1.4초 이상 관측되고 track이 3초 이상이면, 잘린 과거 관측도 명확한 다음 전방 관측으로 검토한다. 한 번 잘린 glimpse만으로는 연결하지 않는다.
- 앞선 전방→잘린 측면의 연결은 단독으로 완화하지 않는다. 독립적으로 선택된 측면→명확한 전방 successor, 양쪽 FOV 전환, 지속적인 넓은 측면 차체 관측, 두 전방 외관의 일치가 함께 있어야 중간 구간을 해석한다. mutual-best·모호성·동시 관측 충돌 규칙은 유지한다.
- 연결 후 모든 관측을 다시 통째로 smoothing하던 부분을 제거했다. 관측은 유지하고, 선택된 카메라 전환의 공백(최대 4초)에만 `complete_gap()`의 양 끝 위치·속도를 사용한 bounded Hermite 경로를 추가한다. endpoint 범위를 벗어나지 않도록 tangent를 제한한다. 공백만 `offline_two_endpoint_gap`이며 원본 관측이 없을 때 ghost다.

## 결과와 검사

- `runs/oct21-handoff-002`: 59.9초/1,198 frames. 최초 선행차의 front→right→front chain, 흰 차의 right→front, 기존 Benz left→front를 유지/연결하고 공백 네 곳 복원.
- `runs/sep17-handoff-002`: 59.1초/1,182 frames. 새 연결을 강제하지 않음.
- `runs/pilot-handoff-002`: 40.8초/816 frames. 기존 Palisade front→right 연결과 한 공백 유지/복원.

테스트 120개 통과. `tools/review_handoffs.py` / `runs/handoff-review-001/review.json`에서 세 영상의 자차 lane-state·road 불변, 기존 anchor 관측 좌표 보존, 알려진 동일 차량 연결과 다른 차량 분리, 관측/ghost 충돌 0, 전체 export decode를 확인했다. 복원 경로가 endpoint 범위 내에 있으며 October 공백 최대 속도 11.68m/s, pilot 4.40m/s다. 이 값은 전체 영상 최대 속도나 일반적인 위치 정확도가 아니다. `render-contact.jpg`의 전환 사건과 원본 4채널을 비교했다.

기존 결과는 보존했다. CUDA/새 detector 추론/OSM/빌드/릴리즈/푸시는 추가하지 않았다. 도로 밝기와 정후방 시점은 유지한다. 모든 차량 ID/가림/교차로 처리가 완료됐다는 판정은 아니다. WP06은 계속 IN_PROGRESS다.

Drive: https://drive.google.com/file/d/12EWZbwABnAQuU8c-kfoMwBIdBnn04MSB/view?usp=drivesdk
