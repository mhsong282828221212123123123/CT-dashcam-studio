# 새4날짜 검수와 되돌림 — 2026-11-15

`new-date-selection-006`의4날짜는 이전 source-only 후보/선정 기록을 검색한 뒤 선택했다. raw4채널 contact를 먼저 직접 확인하고 sparse source-controls를 저장했다. `new-date-suite-006`은 코드/model/config를 동결하여 YOLO11x/YOLOPv2 DirectML로 처음부터 실행했다.4개 모두 runtime 완료, engine unchanged, OSM cache 공급/네트워크 조회0이다. source geometry 점수와 runtime/numerical 성공을 구분한다. 아래 내용을 보고 수정했으므로 이제 개발 자료다.6날짜 독립 holdout 통과로 보고하지 않는다.

## 원본 라벨과 최초 실패

`new-date-dense-source-controls-006`의 source5Hz crop을 직접 확인했다: Audi 선행51프레임(9/7,10–20초), 우측 Hyundai sedan51프레임(20–30초), Carnival 선행11프레임(8/16,50–52초). bbox는 full raw에서 지정했고 동일 ID를 최초 anchor로 고정했다. 후속 시각마다 가장 유리한 차량으로 바꾸지 않는다. 최초 thumbnail의 Tucson/Genesis 명칭 추정은 full raw에서 Audi/Hyundai로 교정했다. 이전 명칭과 hash는 `source-controls-002.json`에 보존했다. 원본으로 라벨했으며 prediction geometry를 라벨 근거로 사용하지 않았다. pipeline progress의 actor count 로그는 보았다.

`new-date-source-evaluation-006`:

- Audi: 같은 fixed ID, solid·진행방향·같은 차로/선행 관계 모두100%.
- Hyundai: 같은 fixed ID, solid·진행방향·자차 한 차로 오른쪽 관계 모두100%.
- Carnival: detector bbox IoU.973인데 world ID 없음, 전 구간 삭제. **FAIL**.
- 별도 sparse 도로 라벨: 9/2 원본0–10초2차로/C1인데 출력1차로. 9/7 원본은 최소5차로가 관찰되는데 global road4차로. 선택 두 차량의 PASS를 전체 scene PASS로 쓰지 않는다. 9/7의 lane 축소 전후/교차로와 좌측 가까운 차량까지 추가 검수해야 한다.
- 8/25 실제 우회전은 camera-only junction2개가 생성되어 경로를 따라간다. 그러나 근처 검은 SUV의 body axis가 원본과 어긋나는 시점이 있어 geometry 합격으로 전달하지 않는다.

## Carnival 삭제와 차체 반전 수정

카메라 white-paint fit이 실패하면 기존 road가 world Y=0 직선으로 남았다. 원본 주거지 회전 뒤의 실제 선행차 Y≈136m를 그 직선의 도로 밖으로 보고 삭제한 원인이다. `unresolved_road.py`는 의미 있는 자차 회전/이동이 있을 때 관측 자차 경로를 display/search reference로만 유지한다. 모든 lane은 unknown으로 두고 camera-fit·section·pocket·lane prior를 적용하지 않는다. route 폭은 명시적 표시 prior이며 차로 수가 아니다. 자차/지도/다른 차로 lane centre를 맞추지 않는다. 실제 detector 차량을 이 허위 전역 직선으로 삭제하지 않는다.

`aug16-unresolved-route-full-001`은 차량을 되살렸지만 정차 뒤 motion noise의 음의 속도를 body front로 따라 180° 반전됐다. 이를 PASS로 전달하지 않았다. `actor_rear_axis.py`는 반복된 실제 front rear-appearance의 anchor에서 theta/theta+pi를 선택하고, 짧은 관측 간격의 anchor 사이를 후보정한다. 단순 majority가 혼재한 앞뒤 반전을 해결하지 못한 실험도 수정했다. 넓은 source 공백(>4초)을 가로지르는 보간은 하지 않는다. XYZ/ego/road/ID는 바꾸지 않고 frozen lead는 보존한다.

최종 후보 `aug16-unresolved-route-full-002`:54.35초/1,087프레임, 수치 후보0. 선택 Carnival은 같은 ID, solid100%, 실제 같은 진행방향100%, ahead100%. XYZ는001과 전 표본 동일하다. **relative lane gate는 계속 FAIL**이다. 도로의 차로가 unknown이기 때문이다. 이를 완성 영상이나 lane 복원 합격으로 부르지 않는다.43/46/50초 source strip 전후를 직접 봤다.

기존4개 branch005의 appearance cue에 새 body-axis 함수를 적용한 진단에서 Aug26 front0112만 변경됐다. `aug26-rear-axis-source-review-001/raw-target.jpg`를 직접 확인했다:54.7817초 흰색 fastback의 실제 후면이다. 별도 전체 `aug26-rear-axis-full-001`은1,199프레임 수치0, 모든 XYZ와 기존 ego/view 유지, 해당 actor의 body polarity만 소스 anchor로 보정했다. 나머지3개 scene의 함수 replay는 변경0이다. 이 body-axis 방식도 일반 oblique yaw를 측정하는 모델은 아니다.

## 빈2차로 도로의 외곽 수정

9/2에서는 내부 dash의 일부가 긴 선분으로 투영되어, 더 많이 반복 관측된 바깥 stripe를 무시하고 nearest solid를 왼쪽 도로 끝으로 삼았다. 바깥 stripe215프레임 대 내부 solid 후보30프레임이다. 연속 선분 후보 바깥 한 차로 거리에서 훨씬 강한 ground stripe가 반복되면 그 후보를 단독 외곽으로 확정하지 않는다. 독립 우측 curb는 보존한다. 반대편 yellow divider 밖 후보는 기존 semantic scope로 먼저 걸러진다. 차량이 없어도 camera paint로 두 차로를 남길 수 있다. global nominal width prior3.5m는 유지했다.

`inner-solid-edge-probe-001`: Oct26/Aug28/Aug26/Oct22 기존 global bounds 모두 동일, 9/2만1→2차로. 최종 `sep02-inner-stripe-full-001`59.8초/1,196프레임, 전체/최종5초 수치0. 원본 라벨0–10초51표본 모두2차로/C1, 빈 source 구간 phantom car0.0/10/20/30/40/50초 full source-strip contact를 직접 확인했다.30초의 camera-only straight junction은 아직 복원되지 않는다. 전체 intersection 성공이 아니다. exact engine/config/tests115파일 snapshot은 `sep02-inner-stripe-full-001-engine`. 코드 검사248개.

## 다음 핵심

빈2차로와 Carnival의 선택 실패는 수정됐지만, camera-only 직진 교차로의 전후 lane section과 혼잡 구간의 좌측 가까운 차량, 비스듬한 body yaw는 남았다.9/7의 global4차로를 단순히 설정6으로 늘려 해결했다고 하지 않는다. before/after topology를 섞은 문제인지 먼저 검수한다. source6개를 다시 무작정 교체하기보다 이번4날짜의 구체적인 실패를 수정/재검수한다.

영상 appearance 자체에서 body angle을 추정하는 모델도 별도 비교 중이다. 원래 skhadem VGG 배포는 connector와 직접 HTTPS 모두404였다. 대체 공식 [ruhyadi/yolo3d-lightning v0.6](https://github.com/ruhyadi/yolo3d-lightning/releases/tag/v0.6)의 `regressor_mobilenetv3.pt`를 획득했다(121,048,486 bytes; SHA256 `a1e4a12870566967d248c28e9dcc6f40945142d1b0ecc3d11636668becbdae80`). source commit `a9698015adb157250d09e1e3ae1e4b480aaff5f1`의 전처리/학습 bin/축 정의를 먼저 읽는다. 이 모델은 아직 기본 pipeline에 채택하지 않았다. learned XYZ/dimensions를 진실로 사용하지 않고 angle 후보를 비교할 예정이다. 별도 `.orientation-export-venv`의 torch2.5.1+cpu / torchvision0.20.1+cpu, `torch.version.cuda is None`을 확인했다. global CUDA wheel은 사용하지 않는다. export/runtime 모두 CUDA 경로를 쓰지 않는다. Tensor state는 weights_only/CPU로 읽었다. dtype/device·ONNX parity·actual source 검수 전 adoption하지 않는다. source/config는 `../scene-models/orientation-review`에 보존한다.
