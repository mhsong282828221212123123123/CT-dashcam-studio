# 라벨 전파 검수와 정적 배경 운동 후보

## 이번 단계의 범위

기존 코드를 import하지 않고 raw annotation assist, ONNX 차량 마스크, 정적 특징점 후보, 조건부 두 시점 카메라 운동 실험을 구현했다. 3D 차량 위치·도로·metric ego 결과가 완성된 단계는 아니다.

## 원본 라벨 전파 검수

`label_assist.py`는 수동 keyframe bbox에서 KLT/forward-backward 검사/robust similarity로 라벨 후보를 만든다. production tracker가 아니며 자동 GT 승격이 없다.

검수 대상: 선행 흰 van 175시점과 우측 SUV 175시점. source crop/fallback 전체 350칸, 각 차량 5페이지를 확인했다.

발견·처리:

- 축 정렬 bbox를 매번 회전한 enclosure로 바꾸면 박스가 누적 팽창했다. 중심/scale 기반 제안으로 수정했다.
- 잘린 SUV의 visible bbox 폭을 전체 차체 폭처럼 전파하면 0.2~3초에서 후방 가시 부분을 잘라낸다. 이 구간의 bbox는 GT 승인하지 않았다.
- SUV가 보이는 30.6~31.2초에서 전파 bbox가 끊겼다. 실패로 기록했다.
- 33.6~34.8초 SUV의 미세 가시성/퇴장은 더 촘촘한 원본 확인이 필요해 점수화하지 않았다.
- 단일 anchor만 있는 곳의 spread=0은 정확도 근거가 아니다. 새 코드에서는 다중 anchor가 없으면 disagreement=null과 support IDs를 출력한다. 기존 artifact는 수정하지 않았다.

`runs/pilot-presence-review-001`: 선행차 175개, SUV 168개, 총 343개의 선택 객체 존재/카메라 내 동일성 reference를 원본 검수로 작성했다. 전체 프레임의 모든 객체 GT가 아니다. bbox/contact/yaw/미터 위치/카메라 간 handoff 정확도는 승인하지 않았다. 사람이 아닌 assistant의 원본 검수라는 한계도 기록했다.

선행 van에는 루프 박스가 있다. detector bbox에 포함된 상단 부착물을 표준 bare vehicle 높이로 맞추면 depth가 틀어질 수 있으므로 이후 geometry fitter에서 분리해야 한다.

## 새 DirectML raw mask prepass

두 날짜 source를 각각 35초·4채널·5Hz, 700프레임씩 새 원본 공간에서 처리했다.

- 정체/출발: `2026-10-15_17-33-13`, source package `pilot-queue-source-002`.
- 주행/곡선: `2026-08-15_17-39-32`, source package `calibration-drive-source-001`.
- 모델: 기존 로컬 자산 YOLO11x 640 ONNX. 가중치만 재사용하고 기존 detector 코드는 사용하지 않았다.
- 새 환경 `onnxruntime-directml==1.23.0`, `requirements/windows-directml.lock`과 wheel hashes로 고정.
- 각 session의 첫 실행 profile에서 DmlExecutionProvider kernel을 확인했다. 두 run 모두 CPU fallback 없음. CUDA provider는 사용하지 않았다.
- 700프레임 run의 wall time은 각각 약 18.4초, 18.1초였다. JPEG 입력 기반 mask 단계 수치이며 전체 원본 decode/재현 시간이나 4GB VRAM 보장이 아니다.

12개 원본 시점 overlay를 확인했다. front·side의 주요 차량은 가려지지만 distorted rear의 2초 원본에서 전체 영상을 차량으로 잘못 검출한 사례가 있다. rear를 공통 기하 계산에 넣지 않았다. 마스크는 static feature 제외 후보일 뿐 정답 segmentation이 아니다.

## 정적 배경 후보

front/left/right만 사용한다. detector mask와 provisional ROI로 차량·자차 차체·mirror·sky edge를 줄이고, 양방향 optical flow로 대응점을 만든다. 후보가 정적 세계점이라고 확정하지 않는다. raw lens distortion은 이 단계에서 풀었다고 주장하지 않는다.

검토할 최신 artifact:

- `runs/pilot-static-features-002`
- `runs/calibration-drive-static-features-002`

각 522쌍(3카메라×174구간)이다. 정체 source에서는 camera별 119~122쌍이 low image motion이다. 움직임이 식별되지 않는 구간을 이동 포즈로 강제 변환하지 않는다. input 해상도와 pixel space가 artifact에 기록된다.

## 조건부 두 시점 운동 실험

렌즈 후보는 `configs/front-lens-candidate.json`이다. **기존 8월 front optical 실험의 inferred f/k1을 초기 가정으로 명시해 사용**했다. 새 재개발에서 보정 완료한 값도, 공장 실측값도 아니다. 8월 원본은 해당 prior의 과거 fitting source이므로 새 독립 검증으로 계산할 수 없다.

새 코드는 front raw points를 이 후보로 normalized ray에 옮긴 뒤, point 일부를 fitting에서 제외하고 essential matrix/회전/단위 이동 방향을 검사한다. source/lens 해상도 미확인은 오류로 거부한다. 테스트 잔차는 undistorted equivalent pixel이며 original raw pixel error나 metric error가 아니다.

테스트가 드러낸 두 결함:

1. OpenCV recoverPose의 기본 50-baseline triangulation cutoff가 실제 정답인 작은 이동/먼 점도 버렸다. 해당 단위가 m가 아님을 명시하고 충분히 큰 수치 한계를 사용하며, 실제 퇴화는 parallax/별도 모델로 검사했다.
2. 위 한계를 풀면 순수 회전에서도 임의 translation 해가 나올 수 있었다. training bearing으로만 rotation-only 모델을 구하고 제외 point에서 확인하여 translation 불관측을 기각하도록 수정했다.

알려진 synthetic R/t 회복과 pure rotation 기각을 테스트했다. 실패했던 `*-motion-candidates-001/002`와 중간 `003`은 이력으로 남겼고 채택하지 않는다.

최신 실험:

- `runs/pilot-motion-candidates-004`
- `runs/calibration-drive-motion-candidates-004`

여기의 conditional_candidate는 가정한 렌즈 아래의 2-view 후보라는 뜻이다. translation은 단위 벡터뿐이고 scale은 없다. 기각 구간 사이를 임의로 이어 world ego를 만들지 않았다. cross-camera rig·카메라 body 원점·절대 거리·ground truth ego는 아직 미확립이다.

## 검증과 다음 단계

격리 환경의 tests 35개와 pip check 통과. 테스트 개수와 candidate 채택률은 실제 재현 정확도가 아니다.

다음 필수 작업:

1. 잘림/가림을 고려한 bbox·contact reference 정밀화와 나머지 중요 객체·사건 GT.
2. 미완료된 수정 UI 실제 재검수/JSON import, ghost alpha 결함 처리.
3. 다중 시간의 static landmark ID를 연결하고 광학·clock·rig를 공동 검증. 현재 pair별 point holdout은 장면 전체의 독립 calibration holdout을 대신하지 않는다.
4. fixed ego/surface를 확립한 뒤에만 actor metric solve와 road 재구성으로 진행.
