# UI 및 클립 탐색기 검증

프로젝트 루트에서 실행합니다. 앱 실행에 필요한 기존 패키지 외에 추가 테스트 패키지는 필요하지 않습니다.

```powershell
python -m unittest discover -s tests -v
```

- `test_clip_groups.py`: 10개 묶음과 마지막 1~2개 병합, 179/180/181초 경계, 정렬·자정·경로 정체성.
- `test_desktop_ui.py`: 실제 PyQt 위젯을 만들어 6개 테마 크기, 구간·재생 표시·스크롤 유지, 언어 전환, 이벤트 폴더 보존, 팝업·설정을 검사합니다. 설정은 임시 INI 파일에 기록합니다. 업데이트 확인 요청은 보내지 않습니다.
- UI 검사는 `offscreen` 방식으로 실행합니다. Windows에서는 설치된 시스템 글꼴을 명시적으로 등록합니다.

배율 검사:

```powershell
$env:QT_SCALE_FACTOR = '1.25'
python -m unittest discover -s tests -v
$env:QT_SCALE_FACTOR = '1.5'
python -m unittest discover -s tests -v
Remove-Item Env:QT_SCALE_FACTOR
```

이 테스트는 실제 테슬라 원본 영상, 얼굴·번호판 인식 정확도, 실행 파일 패키징을 검증하지 않습니다. 실제 미디어를 이용한 재생·내보내기 확인은 별도로 수행해야 합니다.
