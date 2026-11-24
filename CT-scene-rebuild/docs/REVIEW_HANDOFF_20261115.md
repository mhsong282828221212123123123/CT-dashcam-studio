# 검토용 완료·미완료 정리 — 사용자 요청으로 중지

**전체 목표는 미완료이며, 현재 상태를 보존하고 작업을 중지했다.** 실행 중이던 재합성·검사 프로세스 8개를 종료했고, 같은 작업 경로를 사용하는 Python 프로세스가 남지 않았음을 확인했다. 자동 재개하지 않는다. 기존 코드·원본·완료 영상·실패 결과·중단된 작업 파일을 삭제하거나 덮어쓰지 않았다.

## 1. 현재 구현된 것

| 계획 | 구현 내용 | 검토 판정 |
|---|---|---|
| WP06 차량 추적·연결 | 카메라별 검출/추적, 전역 ID 후보와 배타성 검사, front↔repeater 연결, 앞뒤 관측을 이용한 공백 보완, 관측 없는 구간의 반투명 고스트 | 선택 사건은 확인했으나 모든 차량·유사 차량·긴 가림의 합격은 미확인 |
| WP07 도로·차로 변경 | 구간별 차로, 외곽 실선/내부 점선, 좌측 포켓, 자차 변경과 주변 운동 분리, 황색선의 좌우 판정 수정 | 일부 source 사건 통과. 최신 수정본 전체 회귀는 중단 시 미완료 |
| WP08 곡선·교차로·OSM | 곡선 reference, 카메라 표식/회전/횡단 객체를 이용한 opening, OSM 캐시와 연결 방향 참고 | 넓은 교차로·톨게이트 광장·합류의 완전한 윤곽 재현은 미완료 |
| WP09 화면·영상 | 테슬라 다크, 정후방 1.3배 시점, 자차 GLB·주변 클레이, 하단 원본 4채널, 전체 H.264 MP4, 자차 SEI 등화 | 출력 기능 구현. 영상 생성 성공을 추적 정확도 합격으로 간주하지 않음 |
| WP10 검증 | 코드 검사, 전체 decode/마지막 5초/연속성 검사, 기준 관측 보존, 선택 원본 사건과 실패 사례 검토 | 원본 일치 전체 합격 아님. 독립 새 날짜 holdout도 미달 |
| WP11 실험실 연결 | 독립 Qt 검토 UI, 기존 편집기 메뉴, 분석·취소·이어하기·재생·MP4 저장, 자동 원본을 보존하는 별도 수동 교정 | UI/재생/내보내기 실행 기록 있음. 모든 입력 조건의 사용성 검수 완료는 아님 |

현재 처리 대상은 같은 시각의 **4채널**이다. 후방은 식별·접근 참고이며 3D 위치 융합에는 쓰지 않는다. 6채널 확장, 사람·폴·신호등 시설 모델의 통합, 주변 차량 등화 상태의 완전한 판단은 미완료다. 카메라 파라미터는 추정값이며 실측 거리 정확도는 미평가다.

인식 모델은 YOLO11x 검출, YOLOPv2 도로/차선, 선택 front 차량의 MobileNetV3 body 방향 ONNX를 사용한다. 실행은 DirectML GPU 또는 명시적 CPU 선택이며 CUDA는 사용하지 않는다. VRAM 4GB 장치에서의 실측 검증은 하지 않았다.

## 2. 증거와 그 범위

- **코드 검사 283개 통과:** 최신 황색선 수정의 기준선 평행 이동 검사 포함. 이것만으로 원본 일치 합격을 선언하지 않는다.
- **수정 전 동결본 14개 / 16,344프레임 수치 검사 통과:** `runs/final-fourteen-export-review-001/review.json`. 그 후 원본 대조에서 도로·차량 누락이 발견됐으므로 전체 품질 합격본이 아니다.
- **수정 전 선택 핵심 검사 16개 통과:** `runs/current-eight-source-witnesses-015/review.json`. 선행차 기준 관측 보존, 횡단 트럭 방향, 횡단 차·버스 유지, 선택 반대편 차량 제외, 카메라 연결, 정차 body 유지, 자차 변경 완료, 빈 2차로 등이다. 모든 객체 recall이나 모든 차로 정확도의 결과가 아니다.
- **최신 도로 좌우 수정 후:** 아래 12개 전체 영상이 생성·발행됐으며 artifact 무결성을 확인했다. 나머지 2개와 전체 최신 회귀 검사는 사용자 요청으로 중단했다. 새 전체 수치 검사 통과 수를 14개로 보고하지 않는다.
- **최신 실패 구간의 선택 원본 검사 통과:** `runs/yellow-gauge-aug20-full-source-review-001/review.json`. 50/59초의 2차로와 59초 우측 전방 Soul·트럭의 solid/전방/우측 차로 관계를 전체 MP4의 상태에서 확인했다. 기존 실패 영상은 같은 검사에서 거부됨도 확인했다. 실패를 보고 추가한 개발 검사이며 독립 holdout은 아니다.
- **최신 four 구간의 사전 검사:** `runs/yellow-gauge-four-precheck-001/review.json`은 새 four 결과와 이전 branch 결과를 혼합한 검사다. 최신 branch 전체까지 재검증한 것으로 사용하면 안 된다.
- **실제 UI 환경 검사:** `runs/qt-current-engine-review-005/review.json`에서 재생 프레임 수신·영상 길이·내보낸 MP4 바이트 동일성·원본 manifest 보존을 확인했다. 최신 도로 좌우 수정 이전 시점의 검사다.

독립 원본 라벨 평가인 `runs/new-date-source-evaluation-014/review.json`은 **Audi PASS, Hyundai/Carnival 상대 차로 FAIL**을 그대로 보존한다. Hyundai는 총 201표본 중 차로가 확인된 128표본에서 잘못된 상대 차로 0, 나머지 73표본은 교차로 내부 unknown이었다. Carnival은 주차 구간 41표본 모두 차로 unknown이다. 이 실패를 통과로 바꾸지 않았다.

추가 6개 영상은 이미 노출된 날짜의 새 clip이다. 독립 6개 새 날짜 검증으로 보고하지 않는다. 최초 Aug20 후보는 34.8858초 이후 SEI 주행 데이터가 없어 실패했고, 전 구간 주행 데이터가 있는 다른 clip으로 대체했다. 원본과 실패 산출물은 보존했다.

## 3. 최근에 고친 원인

1. **선행차 방향 동결 순서:** 합성 단계에서 동결된 방향을 180도 변경하던 경로를 제거하고, body 축 연속성을 base 발행 전에 결정했다. 지면 투영만 실패하고 영상에는 계속 보이는 경우와 실제 가시 공백을 구분했다. 양쪽에 확정 body 관측이 있는 보완 구간에서는 크기 prior의 흔들리는 속도 방향 대신 관측 방향을 보간한다. 자세한 기록은 `docs/FROZEN_LEAD_HEADING_REVIEW.md`다.
2. **도로 좌우 원점 오류:** 도로 reference의 횡좌표 0을 자차 중심으로 가정해, 실제 왼쪽 황색선을 오른쪽 curb로 오인했다. 그 결과 Aug20 후반에 도로 구간과 차량이 제외됐다. `curved_road.py`는 이제 고정된 자차 궤적의 동일 station 위치를 기준으로 황색선 좌우를 판단한다. 자차 위치를 주변 차량에 맞춰 바꾸는 방식이 아니다.
3. **검증 누락 방지:** 원본 라벨 평가가 아직 생성되지 않은 영상을 조용히 건너뛰지 못하도록 전체 라벨 대상의 결과 존재 여부를 먼저 검사한다. 이전 부분 평가와 실패 보고서는 무효 사유와 함께 보존했다.

최신 수정은 `runs/yellow-gauge-full-suite-001/frozen`에 코드·설정·모델 사본을 동결했다. 중지 시 작업 코드와 해당 엔진 코드 해시가 같음을 확인했다. 원본 인식·자차/base 입력은 재사용하고 도로 판정·합성을 다시 실행한 결과다.

## 4. 중단 지점과 남은 일

- 재합성 **12/14개 생성 완료**, `branch/aug26`, `branch/oct22`는 중단. 해당 `*-work` 파일은 완성 영상으로 취급하지 않는다.
- `runs/yellow-gauge-fourteen-export-review-001/review.json`은 생성되지 않았다. 즉, 최신 14개 전체 회귀 결과는 없다.
- 최신 결과에 대한 전체 원본 대조, 누락·잘못된 병합·방향·차로·고스트의 최종 판정이 남았다.
- 넓은 교차로/광장/합류, 회전 전 다차로 접근부의 과소 표시, 일부 잘린 측방 차량의 위치 미확정이 남아 있다.
- 계획서의 독립 도로 보정 단계·leave-one-track-out·전체 회귀 및 독립 날짜 평가 요구는 최종 완료로 확인하지 못했다.
- 최신 황색선 수정본의 구글드라이브 업로드는 하지 않았다. 폴·사람 모델은 낮은 우선순위로 미구현 상태다.

작업 기록의 `running` 표시가 남아 자동 재개로 오해되지 않도록, 외부 suite 상태는 `stopped_by_user`로 기록했다. 완료된 개별 artifact는 수정하지 않았다. 중지 기록은 `runs/yellow-gauge-full-suite-001/STOPPED_BY_USER.json`이다.

## 5. 검토 방법

편집기의 **주행 재현 v2 / 분석·검토·MP4** 메뉴에서 **v2 결과 열기**로 아래 `scene` 결과를 열 수 있다. 재합성 검증 폴더는 아래 표의 정확한 scene 경로를 사용한다. **MP4 내보내기**는 같은 합성 영상을 저장한다. 새 분석은 사용자가 재개를 지시하기 전까지 실행하지 않는다.

[구글드라이브 테스트26](https://drive.google.com/drive/folders/19Q8hPBlHfhKbCJdIuGColfhl_SCECX30)에는 최신 황색선 수정 **이전**의 검수 영상 13개가 있다. `20261115-v2-검수-`와 `20261115-v2-추가-` 이름으로 구분된다. 부모 폴더와 파일 크기를 로컬과 대조한 기록은 `runs/final-fourteen-export-review-001/delivery-pre-yellow-fix.json`이다. 공유 권한은 변경하지 않았다.

CUDA·빌드·릴리즈·푸시는 수행하지 않았다. 원본과 기존 코드/결과를 보존했다. 전체 목표 완료나 상용 품질로 판정하지 않는다.

## 6. 중지 시 생성 완료된 최신 로컬 영상

아래 표는 생성·무결성 확인 목록이며 전체 품질 합격 목록은 아니다.

| 구분 | 구간 | 길이 | 전체 MP4 | 결과 폴더 |
|---|---|---:|---|---|
| four | 2026-08-16_14-59-58 | 54.35초 | [영상](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/four/2026-08-16_14-59-58/scene/surrounding-world.mp4) | [scene](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/four/2026-08-16_14-59-58/scene) |
| four | 2026-08-25_08-20-56 | 59.05초 | [영상](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/four/2026-08-25_08-20-56/scene/surrounding-world.mp4) | [scene](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/four/2026-08-25_08-20-56/scene) |
| four | 2026-09-02_12-07-46 | 59.80초 | [영상](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/four/2026-09-02_12-07-46/scene/surrounding-world.mp4) | [scene](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/four/2026-09-02_12-07-46/scene) |
| four | 2026-09-07_18-09-51 | 59.50초 | [영상](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/four/2026-09-07_18-09-51/scene/surrounding-world.mp4) | [scene](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/four/2026-09-07_18-09-51/scene) |
| six | 2026-08-26_08-33-44 | 58.85초 | [영상](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/six/2026-08-26_08-33-44/scene/surrounding-world.mp4) | [scene](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/six/2026-08-26_08-33-44/scene) |
| six | 2026-09-14_22-14-31 | 49.20초 | [영상](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/six/2026-09-14_22-14-31/scene/surrounding-world.mp4) | [scene](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/six/2026-09-14_22-14-31/scene) |
| six | 2026-10-18_11-23-27 | 59.75초 | [영상](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/six/2026-10-18_11-23-27/scene/surrounding-world.mp4) | [scene](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/six/2026-10-18_11-23-27/scene) |
| six | 2026-10-02_14-58-03 | 59.70초 | [영상](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/six/2026-10-02_14-58-03/scene/surrounding-world.mp4) | [scene](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/six/2026-10-02_14-58-03/scene) |
| six | 2026-11-10_22-07-20 | 59.50초 | [영상](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/six/2026-11-10_22-07-20/scene/surrounding-world.mp4) | [scene](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/six/2026-11-10_22-07-20/scene) |
| six | 2026-08-20_07-34-02 | 59.90초 | [영상](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/six/2026-08-20_07-34-02/scene/surrounding-world.mp4) | [scene](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/six/2026-08-20_07-34-02/scene) |
| branch | oct26 | 58.90초 | [영상](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/branch/oct26/surrounding-world.mp4) | [scene](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/branch/oct26) |
| branch | aug28 | 59.35초 | [영상](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/branch/aug28/surrounding-world.mp4) | [scene](C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/yellow-gauge-full-suite-001/branch/aug28) |

중지 시각: 2026-11-15T10:44:13.402677+09:00

최신 동결 엔진 SHA-256: 444b2a784ec1f50c6940c46bb5f3d2f1ad113b267e17b9a748017fe33e55457b
