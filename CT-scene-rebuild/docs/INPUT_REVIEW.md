# 입력 검증과 pilot 선정

## 원본 목록

새 reader로 a2~a6 목록을 다시 생성했다. 실제 front 파일 4,809개, 파일명 기준 묶음 4,685개, 날짜 65개다. 기존 catalog의 과거 숫자를 재사용하지 않았다.
같은 이름의 파일을 byte 동일 복사본으로 간주하지 않는다. 전체 목록은 metadata-only이며 선정 원본만 full SHA-256을 수행했다.
기존 결과 manifest에서 source 참조가 확인된 날짜 32개는 보수적으로 legacy_regression 처리했다. 참조가 없다고 미사용 holdout으로 승인하지 않으며 나머지는 unassigned다.

결과: runs/inventory-001. 독립 holdout은 아직 미선정이다.

## SEI 실제 검사

August 15 원본의 35초에서 1,260개 Tesla version 1 메시지를 추출했고 parsing error는 없었다. 이는 schema parsing 검사이며 물리적 센서 정확도 검증이 아니다.

- packet PTS와 payload를 함께 보존한다. sensor exposure sync로 사용하지 않는다.
- float 속도를 반올림하지 않는다.
- 정상 version 1의 protobuf scalar 기본값과 payload 없음/미지원/손상을 구별한다.
- 알 수 없는 버전에는 유효 false/0을 만들어 넣지 않는다.
- steering angle은 공식 schema만으로 단위를 확인하지 못해 raw 값을 보존하고 rad 값은 null로 남겼다.
- GPS의 schema-default 0은 실제 촬영 위치로 사용하면 안 된다. presence가 wire인지 확인해야 한다.
- H.265/비표준 format은 미지원으로 명시한다.

근거: [Tesla 공식 schema](https://github.com/teslamotors/dashcam/blob/master/dashcam.proto), [공식 SEI parser의 vendor envelope](https://github.com/teslamotors/dashcam/blob/master/dashcam-mp4.js).

## 첫 pilot

source: `E:\tesla\a4\TeslaCam\SentryClips\2026-10-15_17-39-27\2026-10-15_17-33-13-front.mp4`와 3개 sibling.
각 카메라의 첫 PTS부터 35초, 5Hz 원본 700프레임을 준비했다. 카메라간 물리 동기는 미검증이다.

2/15/33초 원본 contact sheet를 확인했다. 선행 흰색 van/SUV급 차량, 우측 SUV/세단, 후방 접근 트럭, 정차 후 출발 및 뚜렷한 가까운 차선이 있어 single-actor 영점과 측면/후방 continuity를 단계적으로 검사하기 적절하다. source 폴더의 Sentry 이름만으로 주차 영상이라고 분류하지 않는다.

이 source는 과거 개발에 쓴 legacy regression이다. 새 일반화/독립 평가 자료로 취급하지 않는다. 개발 pilot의 첫 구간은 이 목록으로 동결하되, 아직 전 구간 GT와 사건 검토가 끝난 것은 아니다.

August 15 구간은 곡선·교차로와 빈 전방이 많아 calibration/static/timing 참고로 유지하고 첫 쉬운 차량 pilot에서는 제외했다. 초기 annotation 후보의 floating point 누적 때문에 704장(카메라당 1장 추가)이 나온 결함을 발견해 integer grid로 수정했다. 기존 artifact는 수정하지 않았고 새 pilot package는 700장/700 unique frame으로 검사했다. 같은 결함은 35초 synthetic video test로 고정했다.

## 상태

- isolated environment 설치와 hash lock 완료, pip check 통과.
- synthetic pixel distortion/crop/letterbox roundtrip 검사 통과. 실제 camera calibration 통과 아님.
- renderer context와 GLB geometry는 실제 GPU에서 출력됨. 고스트 alpha에 내부면 draw-order 결함을 시각적으로 확인하여 투명도 검수는 FAIL이다. 최종 renderer 합격 아님.
- 단위/통합 테스트 22개 통과. annotation 작성·관계/사건 정답·camera clock/rig 검증은 미완료다.
- 원본 정답 도구의 HTML은 생성했지만 브라우저 조작 end-to-end 검수는 미완료다. 사용 가능성까지 완성으로 보고하지 않는다.

다음 순서: annotation 도구 실제 조작 검수 → pilot 전체 정답/사건 작성 → typed state/clock 유효 구간 계약 보강 → static rig/ego 실험. 렌더러의 외형 개선으로 기하 단계의 실패를 가리지 않는다.
