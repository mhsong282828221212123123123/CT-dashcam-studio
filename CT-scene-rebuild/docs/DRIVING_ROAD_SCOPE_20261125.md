# 주행도로 밖 차량 표시 제외 · 2026-11-25

사용자 요청은 자차가 주행하는 도로 밖 차량의 표시 제외다. Gemini 엔진 cf08ea6 / 편집기560ee61을 기준으로 codex/road-scope 별도 작업 공간에서 구현·검증했다. 원본 영상과 기존 결과는 보존한다.

## 최종 구현

src/ct_scene/driving_road_scope.py는 원시 카메라 위치 후보, 지원되는 도로 경계, 포켓차로, 교차로 영역을 검사한다. 제외는 **모든 추정·충돌 처리가 끝난 최종 렌더 계획에서만** 수행한다. 기존 Kalman 추정기와 충돌 계산은 그대로다. 분석용 관측이나 actor 파일을 삭제하지 않는다.

- 연속된 직접 관측 최소3개/0.3초가 바깥에 있고 근처의 안쪽 관측이 없을 때 외부 후보로 본다. 작은 상자·화면 끝·가려진 불명확한 접점은 근거에서 제외한다. 도로 지원이 없으면 보류한다.
- 경계 밖이라는 최종 좌표 하나만으로 정상 차량을 숨기지 않는다. 전방에서 같은 방향으로 지속적인 전진(6–55m/s, 제한된 횡방향 변화)을 보이는 차량은 횡방향 추정 오류일 수 있어 보호한다.
- 포켓차로 폭을 포함한다. 교차로는 중앙 교차 영역을 보호하지만, 도로 표시용 긴 옆가지 폴리곤 전체에 있는 차량까지 보호하지 않는다. 중앙 범위는 자차 차로 폭에 좌우 한 차로 여유를 둔 근사값이다.
- frozen lead도 표시 범위 검토 대상이다. 기본적인 위치 검사에서 면제하지 않는다.
- 반대편/별도 도로를 그리는 display 정보는 자차 도로 범위를 늘리지 않는다.

이 판단은 정확한 실측 도로 소속 판정이 아니다. 평행 별도 도로의 같은 방향 차가 남거나 불확실한 대상이 남을 수 있다. 실제 도로 차량의 위치 오류를 숨겨 외부0건을 만드는 목적이 아니다.

## 원본 대조와 실패 보존

첫 좌표 필터는 정상 고속도로 front-0026, front-0006도 제외했다. 원본을 직접 대조한 뒤 같은 방향 전방 주행 보호와 반복 관측 조건을 보완했다. 첫 후보는 runs/source-scope-preflight-001/excluded-source-candidates.jpg에 보존했다.

001에서 Kalman prior까지 바꾸자 oct22 front-0083/0087에 겹침2프레임이 새로 생겼다. Kalman 변경을 되돌린002에서도 충돌 계산 전에 차량을 빼면 남은 차량191프레임의 좌표가 최대2.01m 바뀌고 같은 겹침이 생겼다. 001·002 FAIL은 그대로 보존한다.

최종적으로 **최종 표시 필터**만 남겼다. 저장된 기존3D 상태를 재사용해 남겨둔 차량·자차·도로가 정확히 같음을 검사하고 재출력한다. 옆도로까지 넓게 포함하는 교차로 표시 폴리곤 때문에 일부 바깥 차량이 남던003은 중앙 교차 영역으로 제한한004로 보완했다.

## 도구와 결과

- tools/render_driving_road_scope.py: 기존 상태에 표시 필터를 적용하고 원본4채널을 유지해 재생성한다. 추론 호출0회. 남은 actor 상태와 자차 상태가 바이트 내용 수준으로 같은지 assert한다.
- src/ct_scene/scene_replay.py: 자동 driving_road_scope_plan 입력을 받아 출력하고 필터 검증 파일을 보존한다. 중간 scope-plan의 MP4는 원본4채널을 재사용하기 위한 필터 전 증거 영상이며 최종 결과가 아니다.
- surrounding.py에도 같은 필터를 최종 충돌 처리 뒤에 연결했다. 새 분석에서도 같은 표시 범위가 적용된다. 기존 Kalman 파일은 cf08ea6과 동일하다.
- tools/render_rebased_case.py에 읽기 전용 --cache-root와 기준 report의 opt-in 접점 adapter 유지 기능을 추가했다. v7 모델이 빠진 비교를 같은 조건이라고 취급하지 않는다.
- configs/driving-road-scope-source-controls.json: 원본을 직접 확인한 정상 유지2건과 외부 제외3건. 개발용 기준이며 독립 holdout이 아니다.

최종 결과는 G:/3d/CT-scene-road-scope/runs/road-scope-{highway,junction,oct22}-004 아래 surrounding-world.mp4다. 최종 수치·업로드는 아래 추가한다.

## 범위

기존 위치 추정과 충돌 계산은 유지했다. 따라서 이전 분석에서 외부 차량 때문에 이미 바뀐 다른 차량의 궤적이나 고정 앞 차량의 불안정성은 이번에 해결한 것이 아니다. 이 필터를 위치 정확도 개선이나4단계 융합 완성으로 보고하지 않는다. 수치 PASS는 실제3D 정확도 PASS가 아니다.

## 최종 검증과 전달

코드469검사 PASS. 세 편 총3574프레임 전체 디코드와 수치 검사 PASS, 겹침0. 실제3D 정확도 측정 성공을 뜻하지 않는다. 원본을 직접 확인한 정상 유지2건/외부 제외9건, 총11개 개발 기준 PASS. 추가 원본 대조는 runs/road-scope-additional-source-004.jpg다.

| 씬 | 전체 프레임 | 제외 actor-frame | 남은 actor·자차·도로 상태 |
|---|---:|---:|---|
| highway |1190|0|기준과 정확히 동일|
| junction |1196|60|남은 상태 정확히 동일|
| oct22 |1188|628|남은 상태 정확히 동일|

제외는 차량 전체 삭제가 아니라 조건이 충족된 표시 구간에만 적용한다. actors.json 원시 분석 파일은 기준과 바이트 동일하다. 실제 원본 관측 커버리지는 재계산했고, 카메라4채널은 원본 영상 스트립을 유지해 재인코딩했다. 손실 재인코딩이므로 영상 RGB 바이트 동일을 주장하지 않는다.

- [30초 검토](https://drive.google.com/file/d/1jTCQOuYtospG4hWdeZSUuPIvQXaWdKJJ/view?usp=drivesdk): runs/road-scope-preview-30s-004.mp4
- [고속 전체](https://drive.google.com/file/d/1uD2PmyzFLUr85Hi1QHkalmZyv4bt5Xqo/view?usp=drivesdk)
- [교차로 전체](https://drive.google.com/file/d/18FBJhWXcy_DPbdKqJ9f--LBSz_U4wpV-/view?usp=drivesdk)
- [oct22 전체](https://drive.google.com/file/d/14HQ-czR6QMS4MjG4PzvJUbDapTwjtiRR/view?usp=drivesdk)

전부 테스트26에 업로드 후 부모 폴더와 바이트 크기 확인 완료. 로컬 영상·코드 작업 공간은 G:/3d/CT-scene-road-scope, 편집기 사본은 G:/3d/CT-dashcam-road-scope다. 원래 G:/3d/gemini 작업본과 브랜치는 그대로 두었다. 변경은 별도 codex/road-scope 브랜치로 전달한다. 빌드·릴리즈·기존 브랜치 병합은 하지 않는다.

전체 검증: runs/road-scope-final-source-review-004.json. 전달 해시: runs/road-scope-delivery-004.json. 재검증 도구: tools/review_driving_road_scope.py. 새 전체 분석은 surrounding.py 마지막 표시 필터를 사용하고, 저장된 결과는 tools/render_driving_road_scope.py로 추론 없이 같은 표시 변경을 재현할 수 있다.
