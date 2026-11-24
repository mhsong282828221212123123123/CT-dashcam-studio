# 변경 전 차로 배치 수정 · 2026-11-16

이전 검증에서 실패했던 Aug20 42초의 도로/자차/검은 SUV 차로 배치를 수정했다. 같은 control·평가기에서 source_lane_gate와 overall_source_gate가 FAIL에서 PASS로 바뀌었다. 전체 프로젝트 완료나 모든 차량의 정확도를 뜻하지 않는다.

## 원인과 수정

327.65~799.95m의 긴 road section에 후반의 2차로 경계를 고정 적용해, 앞부분의 더 넓은 포장 구간과 차량 위치가 맞지 않았다. 전역 차량 좌표를 옮기는 대신 section 내부의 도로 경계를 거리별로 보간하도록 했다.

- 20m 간격 ±30m 도색 창에서 독립적으로 바깥 경계를 찾는다. 최소 세 연속 창, 40m 이상에서 왼쪽 추가 폭을 반복 관측하고 이후 황색 분리선으로 기존 폭 복귀가 지지될 때만 적용한다.
- 실제 적용 도색 창은 400/420/440m, 복귀 지지는 520m다. 차로 폭과 테이퍼 위치는 추정값이다.
- 도로 경계 좌표는 도색으로 계산한다. 반대편 포장면을 추가 차로로 오인하지 않도록, 원본 전방 차량의 전진 관측을 범위 제한 조건으로 쓴다. 해당 조건에는 대상 SUV의 29표본, 5.6081초도 포함된다. 따라서 차량과 완전히 독립된 정확도 검증으로 주장하지 않는다.
- profile의 폭 0에 가까운 추가 slot은 차로로 세지 않는다. 왼쪽 차로가 열려 인덱스가 바뀌거나 바깥 차로가 닫혀도 정지한 차가 차선을 바꾼 사건으로 만들지 않는다.
- 소스 이름·ID별 offset이나 고정 시간창 보정은 넣지 않았다. 기존 원본·엔진·실패 결과는 보존했다.

## 원본 대조 결과

| 시각 | 원본 SUV / 자차 차로 | 수정본 SUV / 자차 차로 |
|---|---|---|
| 42초 | 0 / 1 | 0 / 1 |
| 47초 | 1 / 0 | 1 / 0 |

검은 SUV의 실제 오른쪽 변경, 43.5~45초 선 통과, 46~48초 완료, 관측 중 solid 유지, 진행방향·연속성 검사도 통과했다. 자차가 왼쪽으로 이동하는 동안 SUV의 실제 오른쪽 이동을 유지했다. 시간 라벨은 대략적인 원본 검토 범위이며 정밀 계측은 아니다.

control SHA256: `427a7330c0e95863baea297f7f4d6c356cb92c013838d5d7842e9cce6d4173e6`

평가기 SHA256: `e46bcb10bb93ec1cf554c4c7b237e222f9b97a7cecf35316ce148c128b9a2921`

평가기와 원본 라벨은 이전 실패 판정과 같다. Aug20은 이미 검토한 개발 사례이며 독립 holdout이 아니다.

## 보존·회귀 검사

- raw actors.json과 전 프레임 자차 위치/방향은 부모 결과와 정확히 같다. 대상 SUV 렌더 좌표 변화도 0이다. 다른 차량 front-0062에 기존 충돌 처리의 최대 0.0000138m 차이만 있다.
- Aug20 1198, Oct02 고속 1194, Oct22 교차로 1188: 새 엔진으로 전체 3580프레임 생성, decode·전체 시계·마지막 5초 검사를 마쳤다. 고속 및 교차로의 차량 상태 파일은 부모와 바이트 동일하다.
- 고속의 기존 원본 대조 측면 차량 6개 차로/존재/solid 검사 PASS. 고속 MP4는 기존 파일과 SHA256까지 동일하여 Drive 링크를 재사용한다.
- 14개 기존 캐시 장면에서 적용 조건만 읽기 검사했다. 대상 Aug20만 적용되고 나머지 13개는 유지된다. 14개를 모두 새로 렌더하거나 원본 전체 대조한 것은 아니다.
- 코드 검사 369 PASS. Qt 실험실에서 새 결과 열기, 실제 MP4 디코드, 재생 길이, 저장 시 영상 바이트·원본 manifest 보존 검사 PASS.
- 고정 상태 다시 렌더 1198프레임 전체 디코드 PASS. 도로·차량 상태는 정확히 같고 추론 호출은 0이다. 도로 profile 2100개 거리 표본에서 경계 역전이나 음수 차로 폭이 없었다.
- 확인본의 PASS 문구는 같은 scene artifact 평가에만 허용한다. 다른 장면의 PASS 파일을 넣으면 출력 전 거부하는 검사도 통과했다.
- 30초 확인본은 원본 28~58초다. SUV에 녹색 표시만 추가하고 하단 4채널 원본을 유지했다. 600프레임 전체 디코드 및 42/44/46/48초 화면을 확인했다.

## 남은 한계

기존 strict 겹침 FAIL은 Aug20 153, Oct02 10, Oct22 14 쌍·프레임으로 남긴다. 이 수정의 선택 구간 PASS가 전체 겹침을 해결하지 않는다. 교차로 도로 윤곽의 시각적 부정확성과 종방향 지도 정합, 실제 합류 양성 검증도 남는다. 반복 도색+복귀 근거가 없는 넓은 도로에는 이 규칙을 억지 적용하지 않는다. 건물·폴·사람 모델은 여전히 낮은 우선순위다.

## 결과와 재현 자료

- 엔진: runs/local-extents-engine-001
- 전체 결과: runs/local-extents-aug20-full-001, runs/local-extents-oct02-full-001, runs/local-extents-oct22-full-001
- 원본 평가: runs/local-extents-aug20-source-001/review.json
- 고속 평가: runs/local-extents-oct02-source-001
- 전체 검사: runs/local-extents-full-validation-001.json
- 좌표 보존: runs/local-extents-geometry-invariants-001.json
- 14개 조건 검사: runs/local-extents-fourteen-screen-001.json
- 전후 화면: runs/local-extents-aug20-before-after-001.jpg
- 재렌더 보존: runs/local-extents-replay-validation-001.json
- 최종 무결성·도로 폭: runs/local-extents-final-integrity-001.json
- 확인본 평가 출처: runs/local-extents-review-provenance-001.json
- Qt 열기/저장: runs/local-extents-qt-review-001/review.json
- Drive 테스트26 업로드 크기·부모 확인: runs/local-extents-delivery-001.json

[전체 약 60초](https://drive.google.com/file/d/1oeX3SK0NJhK1MAHedk1cYj6IyLAdz8bM/view) · [30초 SUV 원본 비교](https://drive.google.com/file/d/1JAvwRjPEFqjdS7X15xrYQXYbiTMbdHpQ/view)

빌드·릴리즈·푸시는 하지 않았다. 다음 우선순위는 새 실제 합류 source 양성 검증이며 이 차로 수정의 일반화 성공을 미리 주장하지 않는다.
