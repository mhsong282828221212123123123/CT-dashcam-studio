# 테스트 영상 전달 규칙

사용자 지정 Google Drive 폴더: **테스트26**.

- 폴더 ID: `19Q8hPBlHfhKbCJdIuGColfhl_SCECX30`
- 폴더 URL: https://drive.google.com/drive/folders/19Q8hPBlHfhKbCJdIuGColfhl_SCECX30
- 앞으로 테스트 MP4를 업로드할 때 `upload_file`의 `parent_folder_id`에 위 ID를 지정한다. 내 드라이브 루트에 새 테스트 영상을 만들지 않는다.
- 전달할 때 로컬 절대 경로의 바로 재생 가능한 영상과 Google Drive 파일 링크를 함께 제공한다.
- 이전 버전은 보존하고 새 결과는 구분 가능한 이름으로 업로드한다. 공유 권한은 임의로 변경하지 않는다.
- 업로드 후 파일 메타데이터 또는 폴더 목록으로 저장 위치를 검증한다.

2026-11-15 정리: 내 드라이브 루트의 CT 프로젝트 테스트 영상 39개를 이 폴더로 이동했다. 기존 파일 ID와 링크를 유지했고, 폴더 목록 재조회로 39개 모두 확인했다. 관련 없는 원본 영상은 이동 대상에서 제외했다.
