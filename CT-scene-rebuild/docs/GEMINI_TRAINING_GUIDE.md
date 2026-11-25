# 제미나이 인계: 실행·정답 작성·모델 학습

실제 프로젝트를 이어받기 위한 안내다. 현재 목표는 대시캠 영상의 납득 가능한 차량3D 재현이다. 이번 학습 실험은 이미지 접점·대략적인 차체 방향만 다룬다. 현재 학습 모델은 새 날짜 검증에 실패해 기본3D에 미적용이다.

## 1. 경로와 현재 상태

| 대상 | 경로 |
|---|---|
| 실제 개발 엔진 | G:/3d/CT-scene-rebuild |
| 원격 푸시하는 편집기 저장소 | G:/3d/CT-dashcam-studio |
| 편집기에 포함된 엔진 사본 | G:/3d/CT-dashcam-studio/CT-scene-rebuild |
| 3D 모델 원본 | G:/3d/fsd-sedan-3d/models |
| 기존 차량 모델 | G:/3d/scene-models |
| 원본 영상 | E:/tesla/a2 … a6 |
| CPU 학습 Python | G:/3d/CT-scene-rebuild/.orientation-export-venv/Scripts/python.exe |
| 분석·ONNX 실행 Python | G:/3d/CT-scene-rebuild/.venv/Scripts/python.exe |

두 Git 저장소는 별개다. 실제 엔진에서 고친 뒤 변경한 파일만 편집기 하위 사본에 동기화한다. runs, 원본 영상, 가상환경은 Git에 없다. 다른 PC에서는 Git 복제만으로 크롭·원본·환경이 복구되지 않는다. 이 안내는 현재 PC의 G:·E: 경로 기준이다.

먼저 [직접 정답과 v2 결과](CONTACT_LABELS_V2_20261125.md), [첫 학습](CONTACT_LEARNING_20261125.md), [갓길·조향각 수정](CURB_STEERING_20261125.md), [영상 목록](LATEST_RESULTS.md)을 읽는다.

v2는 총106개(접점62·방향102), 학습63 / 조정32 / 최종 평가11개다. 새 Oct26의 접점 평가6개에서 평균 오차가 상자 바닥점0.1406→모델0.2294로 악화했다. 방향은7/10이다. 오차는 검출 상자 높이로 정규화한 이미지2D 거리이며 미터가 아니다. 학습 방향61/61은 일반화 성능이 아니다. 실패 가중치를 기본값으로 켜지 않는다.

## 2. 사용자 화면과 단일 모델 실행

대시캠 편집기: **실험실 / Lab → 주행3D 재현 / 분석·재생·MP4 → 최근 완료 결과 → 선택 결과 열기**. 저장은 **MP4 내보내기**.

독립 검토 창:

```powershell
Set-Location 'G:/3d/CT-scene-rebuild'
./Launch-Scene-v2.ps1
```

최근3D 영상: G:/3d/CT-scene-rebuild/runs/curb-steering-highway-002/surrounding-world.mp4. 학습 모델을 켜는 메뉴는 아직 없다. 학습 검토 영상은 정지 프레임 모음이며 새3D 결과가 아니다.

```powershell
Set-Location 'G:/3d/CT-scene-rebuild'
$env:PYTHONPATH = 'src;tools'
./.venv/Scripts/python.exe -m unittest discover -s tests -q
./.venv/Scripts/python.exe tools/predict_contact_adapter.py --model models/contact-adapter-v2 --annotations configs/contact-learning-annotations-v2.json --id oct26-034
```

이 인계 시점 코드 검사428개. applied=false, metric_position_available=false는 의도한 결과다. 이 필드만 true로 바꾼다고 3D 연결이 구현되지는 않는다.

## 3. 실제 학습 구조와 파일

YOLO 검출기 자체를 재학습한 것은 아니다. 기존 KITTI 차량 MobileNetV3 특징 추출부를 고정하고, 이미지 특징과 카메라/상자 메타데이터로 작은 ridge 출력 가중치를 학습한다. backbone 전체 미세 조정도 아니다. CPU만 사용한다.

- contact: 검출 상자 기준 정규화 접점(u,v). 보이는 바퀴 접촉점 두 곳의 중간이며 차량3D 중심이 아니다.
- direction: 이미지상 차체 전방의 away / toward / left / right. 월드 yaw나 실제 운동 방향이 아니다.
- 기존 XYZ·차로 배정·OSM·추정 차량 운동은 정답으로 사용하지 않았다.
- 카메라 외부 파라미터나 절대 미터 위치를 학습한 모델은 아직 없다.

| 파일 | 역할 |
|---|---|
| src/ct_scene/contact_learning.py | 라벨 변환·분할 검증·ridge 계산·오차 |
| src/ct_scene/learned_contact.py | CPU ONNX+학습 head 실험 예측 |
| tools/prepare_contact_learning.py | 원본 크롭·빈 정답·시트 준비 |
| tools/extract_contact_features.py | CPU 특징 추출과 ONNX 내보내기 |
| tools/train_contact_adapter.py | 학습·validation 선택·test 평가 |
| tools/package_contact_adapter.py | 정답/특징/가중치 일치 검증·실행 묶음 |
| tools/review_contact_adapter.py | CPU 예측 일치와 원본 비교 영상 |
| tools/predict_contact_adapter.py | 한 표본 예측 확인 |

기존 가중치는 models/contact-adapter-v1 및 v2에 보존했다. adapter.npz가 새 학습 head, contact-encoder.onnx가 고정 특징 추출부, manifest.json이 체크섬·파라미터·미승인 상태다.

## 4. 정답 작성

기존 정답은 configs/contact-learning-annotations-v2.json이다. 새 실험은 v3 등 새 파일로 만들고 원본 정답·실패 결과를 덮어쓰지 않는다.

준비용 cases JSON 형식:

```json
[
  ["새촬영식별자", "train", "runs/실제로존재하는-scene"],
  ["다른촬영식별자", "test", "runs/다른실제-scene"]
]
```

기존 예시는 configs/contact-learning-new-cases-v2.json. 같은 예시를 재실행하면 이미 본 같은 영상이 나오므로 새로운 최종 평가가 아니다. scene artifact와 연결된 원본/추적 캐시가 필요하다.

```powershell
./.venv/Scripts/python.exe tools/prepare_contact_learning.py --cases configs/새-cases.json --output runs/새-라벨-폴더
```

카메라별 최대4개 표본의 크롭, *-sheet.jpg, manifest.json을 만든다. annotation_status=unreviewed이며 아직 정답이 아니다. 검출 점수가 높아도 가림을 직접 확인해야 한다.

시트 각 타일은360×270이다. 위35픽셀은 제목이며 아래 **360×230 이미지 영역 내부 좌표**로 표시한다. 전체 시트 좌표나 원본1280×960 좌표가 아니다. 전체 좌표에서 타일 좌상단과 제목35픽셀을 빼야 한다. tile_transform이 원본으로 변환한다.

```json
{
  "wheel_contacts_tile_xy": [[80, 199], [243, 197]],
  "contact_tile_xy": [161.5, 198.0],
  "direction": "toward",
  "contact_visibility": "visible_tire_pair",
  "contact_uncertainty_tile_px": 8,
  "annotation_status": "visual_reviewed",
  "reviewer": "Gemini"
}
```

그림자·범퍼·차로 중앙을 바퀴 접점으로 찍지 않는다. 두 점이 독립적으로 안 보이면 wheel_contacts_tile_xy와 contact_tile_xy는 null, contact_visibility는 occluded_or_unresolved로 기록한다. 차체 앞뒤도 모르면 direction=null. 접점 불확실성8픽셀은 기존 표시 기준이며 실측 정확도가 아니다.

source_image, source_image_sha256, frame_id, crop, crop_rect_xyxy, bbox_xyxy, tile_transform, camera는 근거 없이 바꾸지 않는다. reviewer는 실제 검수자 이름을 쓴다. 새 라벨은 visual_reviewed 상태를 사용하면 되고 Codex라고 허위 기록할 필요가 없다. visual_review_by_codex는 기존 자료 호환용으로 허용한다.

표시점을 원본 크롭에 겹쳐 다시 확인한 뒤 records를 새 정답 JSON으로 병합한다. 같은 날짜/세션과 같은 원본 이미지는 분할을 넘기지 않는다. session은 같은 날짜의 여러 클립에 동일한 값을 쓰고, 다른 문자열로 검사만 우회하지 않는다. guard_splits는 바퀴 두 점·중간값 불일치와 가린 접점 정답도 거부한다. 눈으로 틀린 라벨인지를 자동 판별하는 검사는 아니다.

현재 Oct26은 v2 결과를 이미 봤다. 이를 보고 조정한다면 개발용으로 전환하고, 새로운 미사용 촬영 날짜를 최종 평가로 남긴다. Nov01도 v2에서 이미 개발용으로 전환했다.

## 5. CPU 재학습 전체 명령

아래는 **현재 v2 재현** 명령이다. 새 실험에서는 annotations를 새 JSON으로 바꾼다. 모든 output은 존재하지 않아야 하므로 매번 runTag를 새로 만들고 이전 폴더를 삭제해 재사용하지 않는다.

```powershell
Set-Location 'G:/3d/CT-scene-rebuild'
$env:PYTHONPATH = 'src;tools'
$runTag = 'gemini-' + (Get-Date -Format 'yyyyMMdd-HHmmss')
$annotations = 'configs/contact-learning-annotations-v2.json'
$features = "runs/$runTag-features"
$trained = "runs/$runTag-trained"
$model = "runs/$runTag-model"
$review = "runs/$runTag-review"

./.orientation-export-venv/Scripts/python.exe tools/extract_contact_features.py --annotations $annotations --weights G:/3d/scene-models/orientation-review/regressor_mobilenetv3.pt --output $features
if ($LASTEXITCODE -ne 0) { throw '특징 추출 실패' }

./.venv/Scripts/python.exe tools/train_contact_adapter.py --annotations $annotations --features $features --output $trained
if ($LASTEXITCODE -ne 0) { throw '학습 실패' }

./.venv/Scripts/python.exe tools/package_contact_adapter.py --annotations $annotations --features $features --trained $trained --output $model
if ($LASTEXITCODE -ne 0) { throw '패키징 실패' }

./.venv/Scripts/python.exe tools/review_contact_adapter.py --annotations $annotations --features $features --model $model --reference-predictions "$trained/predictions.json" --snapshot-seconds 3 --output $review
if ($LASTEXITCODE -ne 0) { throw '검토 출력 실패' }
```

특징 추출은 기존 CPU PyTorch/torchvision 환경, 실행·평가는 분석용 ONNX Runtime 환경을 사용한다. CUDA 설치, upstream model.cuda() 예제 실행, 가상환경 삭제/덮어쓰기를 하지 않는다. 초기 source .pt는 약121MB이며 실행 묶음에는 필요한 encoder 약3.7MB만 저장한다.

정답·순서를 바꾸면 새 output으로 다시 실행한다. feature NPZ ids와 records 순서가 달라지면 안 된다. package 도구는 학습 보고서의 정답/encoder 해시와 특징/예측 ID 순서를 확인한다. 실험 패키지를 만들었다는 사실은 적용 승인이 아니다.

## 6. 결과 해석과 다음 구현

features: features.npz / contact-encoder.onnx / provenance.json.
trained: adapter.npz / report.json / predictions.json.
model: adapter.npz / contact-encoder.onnx / manifest.json / training-report.json / encoder-provenance.json.
review: heldout-contact-review.mp4 / heldout-sheet.jpg / review.json / predictions.json.

최초 재현은 기존 v2 selected·results와 CPU 예측 일치를 확인한다. reviewer 기록 구조를 개선했으므로 과거 report 전체가 바이트 단위로 같을 필요는 없다. 변경한 라벨·코드·장치를 기록한다.

코드 검사·ONNX 일치와 정확도는 별개다. 평균뿐 아니라 p90·카메라별 사례·가림·차종·다른 촬영 날짜를 확인한다. report의 방향70% candidate나 접점 candidate만으로 적용 승인하지 않는다. validation 악화·표본 부족·3D 미검증도 함께 봐야 한다. test를 본 뒤 기준을 낮추거나 실패 표본을 뺀 점수를 새 최종 평가라 부르지 않는다.

다음 후보는 바퀴별 랜드마크·가시성·어느 바퀴인지의 분리 학습이다. 데이터가 작은 상태에서 backbone 전체 미세 조정만 먼저 한다고 일반화가 좋아진다고 단정하지 않는다.

3D 연결은 src/ct_scene/surrounding.py의 project_contact / project_front / project_vehicle_height와 카메라 설정을 별도로 검토해야 한다. uv를 원본 픽셀로 바꾸는 식은 pixel=bbox[:2]+uv*(bbox[2:]-bbox[:2])다. 이를 지면에 투영해도 차량3D 중심이나 정밀 yaw 정답은 아니다. 카메라별 재투영·동일 차량 연속성·자차와 상대 차량 분리를 검증한 뒤 별도 opt-in 경로를 만든다.

## 7. Git과 사용자 요구

실제 엔진은 로컬 Git, 원격 푸시는 편집기 저장소에서 한다. 각각 git status부터 확인하고 다른 AI 변경을 덮어쓰지 않는다. 브랜치 이름은 codex/ 접두사. 엔진에서 변경한 파일만 편집기 하위 사본에 동기화한다. 코드·정답·작은 가중치·검증 문서를 커밋하고 runs, .venv, .orientation-export-venv, 원본 영상, .git은 통째로 복사/스테이징하지 않는다. 필요하면 해당 경로에 대한 명령별 -c safe.directory 설정을 사용한다.

사용자 요구: 정중한 한국어. CUDA 제외. Git으로 되돌릴 수 있게 보존하고 푸시까지만. 빌드·릴리즈·배포하지 않는다. 도로 형태는 OSM 참고, 세부 차로는 카메라 근거이며 GPS로 차로를 결정하지 않는다. 왜곡된 후방은 참고이고 전방/리피터를 우선한다.

영상은 로컬 재생 MP4와 Google Drive **테스트26**(19Q8hPBlHfhKbCJdIuGColfhl_SCECX30)를 함께 제공한다. 장면 합성은 하단 원본4채널을 유지한다. Drive 도구 권한이 없으면 업로드했다고 쓰지 않는다. 학습 정지 프레임 영상과 실제3D 결과를 구분한다.

첫 작업: 현재 v2를 먼저 재현하고 원본 정답·분할을 검토한다. 새 방법은 기존 XYZ를 정답으로 삼지 말고 새 날짜의 접점·방향 개선을 입증한 뒤3D 연결을 진행한다. 실패 가중치는 실패 상태로 보존하고 기존 기본 장면을 훼손하지 않는다.

## 인계 시 실제 재현 확인

이 안내의 특징 추출→학습→패키징→CPU 영상 검사 전체를 runs/gemini-handoff-repro-001-* 경로로 실행했다. 기존 v2 selected·results와 학습 가중치 배열이 정확히 같았고, 원본11장/660프레임 검토 영상을 디코드했다. 다른 정답 해시를 연결한 패키징은 거부했으며 잘못된 모델 폴더를 만들지 않았다. 기록은 runs/gemini-handoff-verification-001.json이다. 추가로 특징 provenance와 정답 해시가 다르면 학습·패키징 양쪽에서 거부한다.
