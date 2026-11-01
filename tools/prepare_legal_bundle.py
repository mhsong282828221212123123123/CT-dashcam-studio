"""Stage offline legal resources and honest source/build identification.

SPDX-License-Identifier: GPL-3.0-only
This stages notices, not missing corresponding third-party source or a release.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
DOCUMENTS = ('LICENSE', 'PRIVACY.md', 'THIRD_PARTY_NOTICES.md', 'LICENSE_AUDIT.md',
             'REDISTRIBUTION_REPORT.md', 'SECURITY_FINDINGS.md', 'COMPLIANCE_DISCOVERY.md',
             'COMPLIANCE_VERIFICATION.md', 'DEPENDENCY_AUDIT.json', 'BINARY_AUDIT.json')


def digest(file):
    return hashlib.sha256(file.read_bytes()).hexdigest()


def prepare(output):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    documents = {}
    for relative in DOCUMENTS:
        source = ROOT / relative
        shutil.copy2(source, output / relative)
        documents[relative] = digest(source)
    for source in sorted((ROOT / 'licenses').rglob('*')):
        if source.is_file():
            relative = source.relative_to(ROOT)
            target = output / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            documents[relative.as_posix()] = digest(source)
    def git(*args):
        return subprocess.check_output(['git', '-c', 'safe.directory=*', *args], cwd=ROOT,
                                       text=True, stderr=subprocess.DEVNULL).strip()
    try:
        revision, dirty = git('rev-parse', 'HEAD'), bool(git('status', '--porcelain'))
    except (OSError, subprocess.CalledProcessError):
        revision, dirty = None, None
    source_files = {name: documents[name] for name in documents}
    for directory in ('core', 'ui', 'network', 'tools', 'tests', 'assets', 'models'):
        for file in sorted((ROOT / directory).rglob('*')):
            if file.is_file() and '__pycache__' not in file.parts and file.suffix not in ('.pyc', '.log', '.mp4'):
                source_files[file.relative_to(ROOT).as_posix()] = digest(file)
    for file in [ROOT/'CT_Dashcam_Studio.py', ROOT/'icon.ico', *sorted(ROOT.glob('README*.md'))]:
        source_files[file.name] = digest(file)
    sys.path.insert(0, str(ROOT))
    from core.constants import APP_VERSION, APP_LICENSE
    metadata = {'applicationVersion': APP_VERSION, 'originalCodeLicense': APP_LICENSE,
                'sourceRevision': revision, 'workingTreeDirty': dirty,
                'sourceTreeSha256': hashlib.sha256(json.dumps(source_files, sort_keys=True).encode()).hexdigest(),
                'python': sys.version.split()[0], 'documents': documents, 'sourceFiles': source_files,
                'releaseClearance': False,
                'unresolved': ['AGPL-tagged YOLO model/composition/source', 'exact dependency corresponding source and native notices',
                               'artwork provenance', 'OSM policy conflicts', 'QR security findings']}
    (output/'build-info.json').write_text(json.dumps(metadata, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    return metadata


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    result = prepare(parser.parse_args().output)
    print(json.dumps({key: result[key] for key in ('applicationVersion', 'originalCodeLicense', 'sourceRevision', 'workingTreeDirty', 'sourceTreeSha256', 'releaseClearance')}, indent=2))
