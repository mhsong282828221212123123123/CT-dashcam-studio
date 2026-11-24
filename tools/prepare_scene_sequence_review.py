"""Prepare unannotated source evidence; sparse sheets are not sequence acceptance."""
import json
import sys
from pathlib import Path
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import CameraReader


def prepare(root):
    root = Path(root)
    out = root / 'source-sequence-review'
    out.mkdir(exist_ok=True)
    cases = json.loads((root / 'baseline-cases.json').read_text(encoding='utf-8'))
    records = []
    for case in cases:
        duration = min(35., case['duration_s'] - 1.)
        if duration < 30:
            records.append(dict(id=case['id'], status='too_short', duration_s=duration))
            continue
        readers = {name: CameraReader(Path(case['cameras'][name]))
                   for name in ('left_repeater', 'front', 'right_repeater')}
        tiles = []
        times = list(np.arange(1., 1. + duration, 5.))
        try:
            for t in times:
                row = []
                for name, reader in readers.items():
                    _, frame = reader.read_at(float(t))
                    if frame is None:
                        raise RuntimeError(f"Missing source frame: {case['id']} {name} {t}")
                    tile = cv2.resize(frame, (480, 360))
                    cv2.putText(tile, f'{name} {t:.1f}s', (8, 24), 0, .6, (0, 255, 255), 2)
                    row.append(tile)
                tiles.append(np.hstack(row))
        finally:
            for reader in readers.values():
                reader.close()
        path = out / (case['id'] + '.jpg')
        cv2.imencode('.jpg', np.vstack(tiles))[1].tofile(str(path))
        records.append(dict(id=case['id'], split=case['split'], source=case['source'],
                            start_s=1., duration_s=duration, sheet=str(path),
                            sampled_times_s=times, status='awaiting_source_review',
                            continuous_review_complete=False, acceptance=False))
        print(case['id'], flush=True)
    (out / 'index.json').write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    prepare(sys.argv[1])
