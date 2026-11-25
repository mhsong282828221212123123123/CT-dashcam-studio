# CT Dashcam Studio 3D 재현 프로젝트 인계

**이미 작동하는 다중 카메라 3D 재현 모듈이 있습니다.** 실제 본체는 G:/3d/CT-scene-rebuild입니다. 최근 모델 학습은 이 본체를 개선하기 위한 보조 실험입니다. 3D 모듈을 새로 만드는 것으로 시작하지 마세요.

1. 먼저 G:/3d/CT-scene-rebuild/docs/GEMINI_3D_MODULE_GUIDE.md를 읽으세요. 편집기 연결·실행 파일·입력부터3D MP4까지의 흐름·기존 결과·소스 지도가 있습니다.
2. G:/3d/CT-scene-rebuild/runs/curb-steering-highway-002/surrounding-world.mp4를 보세요. 이것이 현재3D 본체의 결과입니다.
3. 학습이 필요하면 G:/3d/CT-scene-rebuild/docs/GEMINI_TRAINING_GUIDE.md를 이어서 읽으세요. contact-adapter-v2는 기본3D에 아직 미적용입니다.

편집기 연결은 G:/3d/CT-dashcam-studio/ui/driving_scene_panel.py → ct_scene.review_app → pipeline.py → surrounding.py입니다. 인계 시에는 편집기 하위 사본에 .venv가 없어서 형제 G:/3d/CT-scene-rebuild가 실제 선택됩니다. 구형 core/driving_scene.py만 보고 현재 모듈을 판단하지 마세요.

정중한 한국어, CUDA 제외, 기존 결과 보존, Git으로 되돌릴 수 있게 작업하고 푸시까지만 하세요. 빌드·릴리즈하지 마세요. 영상은 로컬과 Google Drive 테스트26을 함께 제공합니다.
