# 전방 차체 방향 보완

공개 ruhyadi/yolo3d-lightning v0.6 MobileNetV3 weights SHA256 `a1e4a12870566967d248c28e9dcc6f40945142d1b0ecc3d11636668becbdae80`, 고정 ONNX `60d8cb92e59daebb41231f37a95099fe931a20bf71d1e04303f4ca924045327c`를 로컬 평가했다. upstream main `a9698015adb157250d09e1e3ae1e4b480aaff5f1`를 기록했다. 배포/릴리즈는 하지 않았다.

별도 `.orientation-export-venv`의 torch2.5.1+cpu/vision0.20.1+cpu로 변환했고 CUDA 없음 확인. 엔진 `.venv`에는 torch를 추가하지 않았다. CPU tensor vs DirectML ONNX maxabs: orientation2.38e-7/logits2.15e-6/dimensions6.33e-8. 런타임은 DirectML/명시적 CPU다.

`image_vehicle_heading.py`: raw front car crop, 반전 일관성,80×60 최소, 잘림/가림/rear-red 충돌 검사. 학습 치수/XYZ는 미사용. 리피터54초의 잘못된 반대 방향을 발견해 side 모델은 미채택. `image_heading_refine.py`는 자차 SEI yaw와 반복된 image angle만 합치며 frozen lead/확정 lane/대형차를 보존한다.

`clear_handoff.py`: 과거/미래 clear1.2초 창으로 clipped endpoints를 대체해 연결 후보를 평가한다. 외관/FOV/배타성/시간 gates는 유지한다. Aug25 끝 프레임7.6m에 비해 clear-window 차이 약0.16m. 선택 전 좌표 수정 없음. 선택 후 frozen/clear anchors 보존, clipped suffix/prefix만 복원. 전체 정차 mesh 후보정은 별도 검증한다.

`pipeline.py`는 detector 옆 `orientation-review/regressor_mobilenetv3.onnx`가 있을 때 독립 image-headings stage를 사용한다. 모델 hash를 resume key에 넣고 source/track/optics 일치를 확인한다. 없으면 기존 경로다.

풀버전 own-turn005/branch008에서 평가했다. Aug28/Aug26의 확정 lane은 image yaw 변경0. 기존003과 전체 equality를 요구한007 회귀는 FAIL로 보존했다. 변화는 Oct26 roadside1대, Aug28 clear handoff2개, Aug26 rear-axis였다. Aug28 frozen lead 변화는 리피터 clipped prefix5표본뿐이며 trusted front anchor는 같다. 새 연결/새 roadside는 원본 대조 전 최종 source 합격으로 보지 않는다.
