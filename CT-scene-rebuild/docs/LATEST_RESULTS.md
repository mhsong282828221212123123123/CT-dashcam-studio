# 최신 검토 영상 · 2026-11-16

2026-11-25 최신 진행본: [40초 바로 확인](https://drive.google.com/file/d/1Im_Z_0QZy8CLWfDStWJv0Zg1eSk3uupY/view) · [신규 교차로 전체](https://drive.google.com/file/d/1GpY1mkfRgO80iboD9-REQuVYAL_bSOmK/view) · [신규 곡선·터널 전체](https://drive.google.com/file/d/1cuKapIhuqtLCxyOiaxzyzKACMOuqA1MK/view). 로컬 runs/blindspot-junction-progress-40s-005.mp4 및 runs/fresh-nov01-city-road-002, runs/fresh-nov01-curve-001/scene.

2026-11-25 이전 진행본: [긴 사각지대 고속 전체](https://drive.google.com/file/d/1Xr01OcGNoGL0Qu7d2xo1m8KKLNm0OCZP/view) · [교차로 1차 전체](https://drive.google.com/file/d/1DQ8DUSn4HEQlmwDFhbep1LzDDiywIEo3/view) · [1차 30초 요약](https://drive.google.com/file/d/1SyygVBMC8cWV0Wz7d8CQYHtMdjd7qNWF/view) · [진행/검사](./BLINDSPOT_JUNCTION_20261125.md). 오전 7시(KST)까지 작업 중. 고속은 5초 사각 구간의 미래 관측 연결 검사 PASS, 전체 겹침 문제는 남는다.

최신 수정본: [전체 약 60초](https://drive.google.com/file/d/1oeX3SK0NJhK1MAHedk1cYj6IyLAdz8bM/view) · [30초 원본 비교](https://drive.google.com/file/d/1JAvwRjPEFqjdS7X15xrYQXYbiTMbdHpQ/view) · [수정 범위/검사](./LOCAL_ROAD_EXTENT_FIX_20261116.md). 이전 실패했던 42초 차로 배치를 수정했고 실제 SUV 오른쪽 변경은 유지했다. 선택 source PASS이며 전체 겹침 합격은 아니다.

최신 검토용 · 차로 배치 미해결: [SUV 강조 30초](https://drive.google.com/file/d/1lU0HgVUzTs8SCN93f8iQmeoUoJblyBoR/view) · [전체](https://drive.google.com/file/d/1DnGyv_7ifIjIaxeceDwc6867W4gKV3jJ/view) · [판정/남은 결함](./OTHER_VEHICLE_CHANGE_REVIEW_20261116.md). 차선 변경 운동은 통과했지만 변경 전 차로 배치가 달라 전체 source FAIL이다.

최신 초반 지도 보완: [전체 59초](https://drive.google.com/file/d/1AyWDGZsNNof4hlEGwH4KolQ1Z6JzV3yj/view) · [앞부분 30초](https://drive.google.com/file/d/1LrJ3c6Ypll6A8Wuf196dFSHxISkWgpFG/view) · [검증과 남은 범위](./EARLY_MAP_SUPPORT_20261116.md).

최신 교차로 지역 정렬: [전체 59초](https://drive.google.com/file/d/1rxx_XldVf-WK5bF_CMR681awIeW5CSTl/view) · [20~50초 확인본](https://drive.google.com/file/d/1N4yTR_B2HSR92VOjJn1KaCYb_1qrNDNq/view) · [범위와 한계](./LOCAL_MAP_ALIGNMENT_20261115.md). 초반 지도 미지원 지역은 아직 보류한다.

최신 지도 본선 수정: [전체 59초](https://drive.google.com/file/d/1ExCUC7NrYOrwEbXogglDaeot__qTyJxb/view) · [30초](https://drive.google.com/file/d/1R994sIkKrqb3V-LMZJ3AAXRpaLukjPcM/view) · [수정/한계](./MAIN_ROAD_DISPLAY_FIX_20261115.md). 아래 기존 묶음은 이전 표시를 보존한 기록이다.

정밀 계측이 아닌 실용 수준의 검토본이다. 기존 겹침/차로 한계는 유지한다. [작업 내용](./WORKFLOW_EXECUTION_20261115.md).

| 장면 | 길이 | 로컬 전체 영상 | Google Drive 테스트26 |
|---|---:|---|---|
| 저속·회전 · 새 전체 분석 | 59.05초 | [재생](<C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/practical-fresh-aug25-pipeline-001/scene/surrounding-world.mp4>) | [Drive](https://drive.google.com/file/d/1Le9QJjQ-4-FSVv_9mqaLz8EyHl0ULE4X/view) |
| 빈 도로 · 새 전체 분석 | 59.80초 | [재생](<C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/practical-fresh-sep02-pipeline-001/scene/surrounding-world.mp4>) | [Drive](https://drive.google.com/file/d/1xb1AYmagAzEleQ3nYM12AtOdyEPb8xbF/view) |
| 주간 고속 · 기존 수정본 유지 | 59.70초 | [재생](<C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/practical-display-oct02-full-001/surrounding-world.mp4>) | [Drive](https://drive.google.com/file/d/1DOvaJJeJbxx2oMty86ScqF3vTfUzfSKO/view) |
| 교차로 · 기존 수정본 유지 | 59.40초 | [재생](<C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/practical-display-oct22-full-001/surrounding-world.mp4>) | [Drive](https://drive.google.com/file/d/1UwbdjdS890K5FoE70T5rsvW8KxmEuntj/view) |
| 세 장면 발췌 요약 | 30초 | [재생](<C:/Users/ddno2/Documents/ChatGPT/테슬라/CT-scene-rebuild/runs/practical-progress-30s-001.mp4>) | [Drive](https://drive.google.com/file/d/1XOHiFhZQjvvNd7oSxX-xqcndG_4E3bn9/view) |

고속 영상은 이전 업로드와 SHA256이 같아 기존 링크를 재사용했다. 새 업로드 3개의 부모 폴더·바이트를 metadata로 대조했다. Drive 원격 재생 처리 완료는 별도로 확인하지 않았다.

요약은 저속·고속·교차로를 각 10초씩 연결한 영상이다. 모든 영상 하단에 원본 4채널을 포함한다.
