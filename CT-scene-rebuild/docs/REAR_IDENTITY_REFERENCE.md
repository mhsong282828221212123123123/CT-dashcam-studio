# WP06: 후방 식별 보조 연결 1차

`rear_reference.py`를 `surrounding-prototype`에 연결했다. 후방은 좌표 계산 없이 식별 보조로만 사용한다. CPU SIFT의 양방향 대응, RANSAC, 특징 분포, 반복 관측, 동시 후보의 배타성을 검사한다. 기존 YOLO11x/DirectML 검출 캐시를 사용하며 CUDA·새 신경망 추론은 추가하지 않았다.

후방 camera-local ID가 다른 차로 넘어간 실제 사례가 있다. October `back-0010`은 45초 흰 세단에서 50초 어두운 차로 바뀐다. 따라서 후방 ID 전체를 world ID에 붙이지 않는다. 최소 3개 프레임·0.4초 지속의 근거가 있는 시간 구간만 연결한다. 모호하거나 상충하는 대응은 구간을 끊는다. rear ID가 같다는 이유로 world ID들을 union하거나 구간 밖으로 외삽하지 않는다.

초기 검수의 5대/6구간은 최종 채택 수치가 아니다. 동일 디자인의 휠은 다른 차량/앞뒤 바퀴에도 반복되므로, 휠에만 모인 특징과 배경 outlier에 의존한 후보를 추가 검사에서 제외했다. 이 구현은 일반적인 차량 ReID 모델의 대체가 아니며, 동일 차종 여러 대의 일반 식별 정확도는 아직 미평가다.

최종 검수:

| 원본 | 후방 식별 보조 연결 |
|---|---|
| October 21 | 흰 세단 `back-0002` ↔ `right_repeater-0010`, 32.99–33.40초 |
| September 17 | 흰 세단 `back-0004` ↔ `left_repeater-0003`, 6.59–9.59초 |
| October 15 pilot | 근거 부족으로 0개 채택 |

위 ID는 검수 fixture이며 production 분기가 아니다. 채택 구간의 시작/끝 원본과 특징 분포를 시각 검수했다. 검수: `runs/rear-reference-review-003/{review.json,oct21-endpoints.jpg,sep17-endpoints.jpg}`, pilot: `runs/rear-reference-review-001/pilot-final.json`. 재검사 도구 `tools/review_rear_reference.py`, 테스트 **116개 통과**.

파이프라인 결과:

- `runs/oct21-rear-reference-004`: 59.9초, 1,198 frames.
- `runs/sep17-rear-reference-003`: 59.1초, 1,182 frames.

`rear-identity-reference.json`에 후보·채택 구간·대응 좌표·정책·clock assumption을 기록한다. `observation-world-audit.json`은 후방 식별 연결/미확정/geometry 미확정을 구분한다. 연결된 후방 detector 관측은 source presence에 반영하되 geometry 유효 구간을 새로 늘리지 않는다. 두 full 결과의 관측/ghost 충돌은 0개다.

자차·주변 차량 좌표, 기존 handoff, road는 직전과 같다. October 영상 SHA-256은 정후방/짙은 도로 영상과 동일한 `b89533cafe5454fdff5c2dfe355996b00a5edc181d8f7a370ca7d8f44af7cb18`이다. **이번 단계는 후방 연결 근거를 추가한 것으로, 실제 사각 구간 복원 완료가 아니다.** 동일 영상의 Drive 중복 업로드는 하지 않았다.

다음은 명확한 side→front 사건의 단편/앞뒤 순서를 원본으로 정리하고, 독립적인 동일 차량 근거와 양쪽 world endpoint가 있는 공백만 후보정하는 일이다. rear ID 단독·색상 단독·큰 강제 좌표 offset은 근거로 삼지 않는다. 연결 수보다 중복·사라짐·앞뒤 역전·순간이동을 검사한다. WP06은 IN_PROGRESS이며, 원본/legacy 보존 및 빌드·릴리즈·푸시 금지는 유지한다.
