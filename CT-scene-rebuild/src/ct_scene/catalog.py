"""Source inventory, without treating filename copies as byte-identical."""
from collections import defaultdict
from pathlib import Path
import re
from .artifacts import canonical, digest, publish
from .ingest import CAMERAS


def inventory(roots, legacy_results, destination):
    groups = defaultdict(list)
    for root in roots:
        root = Path(root).resolve()
        if not root.is_dir():
            raise ValueError('Missing source root: '+str(root))
        for front in sorted(root.rglob('*-front.mp4')):
            prefix = front.name[:-len('-front.mp4')]
            cameras = {}
            for camera in CAMERAS:
                p = front.with_name(prefix+'-'+camera+'.mp4')
                if p.is_file():
                    cameras[camera] = dict(path=str(p), bytes=p.stat().st_size,
                                         sha256=None, content_identity='NOT_HASHED')
            groups[prefix].append(cameras)
    references = defaultdict(list)
    # Deliberately exclude old catalogs, which list unseen sources as well.
    for path in Path(legacy_results).rglob('manifest.json'):
        text = path.read_text(encoding='utf-8', errors='replace')
        for prefix in set(re.findall(r'20\d{2}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}', text)):
            references[prefix].append(str(path))
    exposed_dates = {p[:10] for p in references}
    rows = []
    for prefix, copies in sorted(groups.items()):
        date = prefix[:10]
        rows.append(dict(source_id=prefix, recording_date=date, candidate_copies=copies,
                         duplicate_status='filename_group_only_requires_byte_hash',
                         split='legacy_regression' if date in exposed_dates else 'unassigned',
                         exposure_history=references.get(prefix, []),
                         unseen_status='NOT_ESTABLISHED', split_group=date))
    report = dict(unique_filename_groups=len(rows), front_files=sum(len(x) for x in groups.values()),
                  recording_dates=len({r['recording_date'] for r in rows}),
                  legacy_dates=len(exposed_dates), independent_holdout='NOT_SELECTED',
                  warning='Absence from old manifests does not establish unseen data. Selected files require SHA-256 and exposure review.')
    return publish(destination, stage='inventory', payloads={
        'sources.jsonl':b''.join(canonical(r)+b'\n' for r in rows), 'inventory-report.json':canonical(report)},
        inputs=[], config={'roots':[str(Path(r).resolve()) for r in roots]},
        code_revision=digest(Path(__file__).read_bytes()), runtime={})
