# 제미나이 작업 시작 안내

실제 개발 엔진은 G:/3d/CT-scene-rebuild, 편집기 저장소는 G:/3d/CT-dashcam-studio입니다.

먼저 G:/3d/CT-scene-rebuild/docs/GEMINI_TRAINING_GUIDE.md를 읽으세요. 실행·정답 표시·분할·CPU 학습·패키징·검증·현재 실패 결과·Git 반영 경로가 들어 있습니다.

현재 모델은 models/contact-adapter-v2, 정답은 configs/contact-learning-annotations-v2.json(106개)입니다. 현재 모델은 새 날짜 접점 검증 실패로 기본3D에 미적용입니다. 기존 v2를 먼저 재현하고, 같은 test를 반복 조정에 쓰지 말고 새로운 날짜에서 검증하세요. 기존 추정 XYZ를 정답으로 재사용하지 마세요.

정중한 한국어, CUDA 제외, 기존 결과 보존, Git으로 되돌릴 수 있게 작업하고 푸시까지만 하세요. 빌드·릴리즈하지 마세요. 영상은 로컬 재생과 Google Drive 테스트26을 함께 제공합니다.
