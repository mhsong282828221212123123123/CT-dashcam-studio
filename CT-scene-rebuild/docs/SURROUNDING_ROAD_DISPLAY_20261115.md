# 주변 도로 표시 · 2026-11-15

사용자 지적에 따라 자차 통로 밖의 노면도 표시하도록 수정했다. 도로 면의 존재와 상세 차로 판정을 분리한다. 차량이 있다는 이유만으로 새 차로나 노면을 생성하지 않는다.

## 구현

- `observed_surface.py`: 기존 전방 road-mask를 고정 월드에 투영한다. 반복 관측, 동적 차량 영역 제외, 작은 폐쇄 공백 보완 및 제한된 경계 평활화로 노면 mesh를 만든다. 차량 양옆의 직접 노면 근거가 있을 때만 가림 아래를 보완한다. 카메라 기반 지면 투영 자체는 근사다.
- `context_surface.py`: 검증된 기존 OSM 캐시에서 주변 도로 중심선·연결을 읽어 대략적인 노면을 추가한다. 새 네트워크 요청은 없다. GPS 궤적에서 고정 월드로 지도만 강체 정렬하며 자차·차량 좌표나 scale을 조정하지 않는다. 도로 등급별 표시 폭을 사용하며 OSM lanes/width 태그를 상세 차로로 사용하지 않는다.
- `surrounding.py`: 두 노면 mesh를 차량 추적과 차로 계산이 끝난 뒤 렌더링에만 추가한다. 상세 차로·선은 기존 영상 근거를 유지한다. 지도 등급이 불명확하면 주변 지도 면을 생략한다. Tesla 다크, 후방 1.3배 시점, 클레이, 하단 원본 4채널을 유지하고 OSM 출처를 표시했다.

## 검증과 결과

동결 코드 `runs/observed-surface-engine-004`로 저속 Aug26 58.8353초, 주간 고속 Oct02 59.6728초, 신호 접근 Aug16 54.3352초를 전체 재합성했다. 총 3,458프레임의 디코딩·시계·마지막 5초·수치 불연속 검사를 통과했다. 세 구간 모두 scope-handoff005 기준의 actors.json, render-states.jsonl과 바이트가 같다. 엔진003에서 검수한 노면·차선 표시 데이터도 동일하며 004는 표시 출처 문구를 추가했다. 상세 검사: `runs/observed-surface-validation-004.json`, 이전 분석 도로 불변 검사: `runs/observed-surface-validation-003.json`.

원본/렌더 대조: `runs/observed-surface-final-before-after-003.jpg`, `runs/observed-surface-other-source-contact-003.jpg`, 최종 영상 16초 프레임 `runs/observed-surface-final-frame-004.png`.

주변 도로는 이전보다 넓게 표시되지만 **Aug26 16초 버스 부근에서 지도 면과 카메라 면 사이의 틈이 남는다**. 지도 폭은 근사이며 GPS 잔차만으로 실제 노면 정렬을 합격 처리하지 않는다. Aug26/Aug16 지도-궤적 정렬 p90은 약 2.12m/2.09m다. Oct02는 지도 고도 등급이 모호하여 지도 노면을 생략하고 카메라 면만 표시했다. 이번 변경이 차량 위치 정확도·상대 차로 FAIL을 해결한 것은 아니다. 기존 Hyundai/Carnival 판정과 annotation을 유지한다.

## 계획과 다음 수정

사용자 지시로 계획 단계5의 노면 표시 부분을 먼저 수행했다. 14개 전체의 최신 추적 기준은 scope-handoff005이며 이번 새 노면 결과는 3개 개발 구간이다. 14개 모두 새 노면 검수가 끝났다는 뜻이 아니다. 다음은 원본의 도로 경계와 지도 면의 잔여 정렬 검토, 동결 scene-state 분리 및 기존 상대 차로 실패 수정이다. 차량 아래에 임의로 도로를 깔거나 차를 지도에 끌어당겨 정렬하지 않는다. 전체 계획 완료 판정은 아니다.

## 영상

- 저속: `runs/observed-surface-aug26-full-004/surrounding-world.mp4` / https://drive.google.com/file/d/1pAOn0EMZF8lZCP8J1nahW4zdu2_RCuEA/view
- 주간 고속: `runs/observed-surface-oct02-full-004/surrounding-world.mp4` / https://drive.google.com/file/d/1UsvbtuNiKCtBNaPOhp4vd_l848fR_Sl6/view
- 신호 접근: `runs/observed-surface-aug16-full-004/surrounding-world.mp4`

두 Drive 영상은 테스트26 폴더에서 기존 ID의 파일 바이트를 최종004로 교체하고 크기와 부모 폴더를 재조회했다. 기존 결과와 실패 후보를 보존했다. CUDA·빌드·릴리즈·푸시는 하지 않았다.
