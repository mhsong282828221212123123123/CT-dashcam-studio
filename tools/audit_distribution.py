"""Read-only dependency/binary audit; copy original notices, never upload or install.

SPDX-License-Identifier: GPL-3.0-only
Run from the repo with its actual build interpreter. Output is a dated snapshot,
not evidence of complete source-code/redistribution compliance.
"""
import argparse
import hashlib
import importlib.metadata as metadata
import json
from pathlib import Path
import re
import subprocess
import sys
import zipfile

from packaging.requirements import Requirement

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ['PyQt6', 'opencv-python', 'numpy', 'Pillow', 'qrcode', 'imageio-ffmpeg']
BUILD = ['Nuitka', 'ordered-set', 'zstandard']


def sha(path):
    return hashlib.file_digest(path.open('rb'), 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else hashlib.sha256(path.read_bytes()).hexdigest()


def normalized(name):
    return re.sub(r'[-_.]+', '-', name).lower()


def package_inventory():
    pending = [(name, 'runtime') for name in RUNTIME] + [(name, 'build') for name in BUILD]
    rows, seen = [], set()
    notice_root = ROOT / 'licenses' / 'packages'
    while pending:
        name, scope = pending.pop(0)
        if normalized(name) in seen:
            continue
        seen.add(normalized(name))
        dist = metadata.distribution(name)
        requires = []
        for raw in dist.requires or []:
            requirement = Requirement(raw)
            active = not requirement.marker or requirement.marker.evaluate({'extra': ''})
            requires.append({'requirement': raw, 'activeWithoutExtras': active})
            if active:
                pending.append((requirement.name, scope))
        notices = []
        for relative in dist.files or []:
            file = Path(dist.locate_file(relative))
            if not file.is_file() or not any(token in file.name.lower() for token in ('license', 'licence', 'copying', 'notice')):
                continue
            # Each original path is retained to distinguish vendor notices.
            target = notice_root / normalized(name) / str(relative)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(file.read_bytes())
            notices.append({'file': target.relative_to(ROOT).as_posix(), 'sha256': sha(target)})
        license_value = dist.metadata.get('License-Expression') or dist.metadata.get('License') or ''
        rows.append({'name': dist.metadata['Name'], 'version': dist.version, 'scope': scope,
                     'licenseDeclaration': license_value.splitlines()[0][:250] if license_value else None,
                     'licenseClassifiers': [c for c in dist.metadata.get_all('Classifier', []) if c.startswith('License ::')],
                     'projectUrls': dist.metadata.get_all('Project-URL', []), 'requires': requires, 'notices': notices})
    return rows


def model_metadata(file):
    # Top-level ONNX protobuf fields only: graph is skipped, no ONNX dependency.
    def varint(data, pos):
        value, shift = 0, 0
        while True:
            byte = data[pos]; pos += 1
            value |= (byte & 127) << shift
            if byte < 128:
                return value, pos
            shift += 7
    def fields(data):
        pos = 0
        while pos < len(data):
            tag, pos = varint(data, pos)
            number, wire = tag >> 3, tag & 7
            if wire == 2:
                size, pos = varint(data, pos); value = data[pos:pos+size]; pos += size
                yield number, value
            elif wire == 0:
                _, pos = varint(data, pos)
            elif wire == 1:
                pos += 8
            elif wire == 5:
                pos += 4
            else:
                break
    result = {}
    for number, value in fields(file.read_bytes()):
        if number in (2, 3):
            result['producerName' if number == 2 else 'producerVersion'] = value.decode(errors='replace')
        elif number == 14:
            pair = {key: text.decode(errors='replace') for key, text in fields(value)}
            if pair.get(1) not in ('names',):
                result[pair.get(1, 'unknown')] = pair.get(2, '')
    return result


def binary_inventory(directory):
    import pefile  # Already in this build environment; audit only, not app code.
    rows = []
    if not directory.exists():
        return rows
    for file in sorted(directory.rglob('*')):
        if not file.is_file() or file.suffix.lower() not in ('.dll', '.pyd', '.exe'):
            continue
        row = {'path': file.relative_to(directory).as_posix(), 'bytes': file.stat().st_size, 'sha256': sha(file)}
        try:
            pe = pefile.PE(str(file), fast_load=True)
            pe.parse_data_directories(directories=[pefile.DIRECTORY_ENTRY['IMAGE_DIRECTORY_ENTRY_IMPORT'], pefile.DIRECTORY_ENTRY['IMAGE_DIRECTORY_ENTRY_RESOURCE']])
            row['imports'] = sorted({entry.dll.decode(errors='replace') for entry in getattr(pe, 'DIRECTORY_ENTRY_IMPORT', [])})
            if getattr(pe, 'VS_FIXEDFILEINFO', None):
                info = pe.VS_FIXEDFILEINFO[0]
                row['fileVersion'] = '.'.join(str(v) for v in (info.FileVersionMS >> 16, info.FileVersionMS & 65535, info.FileVersionLS >> 16, info.FileVersionLS & 65535))
            pe.close()
        except Exception as error:
            row['inspectionError'] = str(error)
        rows.append(row)
    return rows


def run(bundle, archive):
    packages = package_inventory()
    import cv2
    import imageio_ffmpeg
    from PyQt6.QtCore import qVersion, PYQT_VERSION_STR
    ffmpeg = Path(imageio_ffmpeg.get_ffmpeg_exe())
    def command(*args):
        completed = subprocess.run([str(ffmpeg), *args], capture_output=True, text=True, timeout=30,
                                   creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        return completed.stdout + completed.stderr
    assets = []
    for directory in ['models', 'assets/fonts']:
        for file in sorted((ROOT / directory).rglob('*')):
            if file.is_file():
                row = {'path': file.relative_to(ROOT).as_posix(), 'bytes': file.stat().st_size, 'sha256': sha(file)}
                if file.suffix == '.onnx':
                    row['onnxMetadata'] = model_metadata(file)
                assets.append(row)
    for file in [ROOT/'icon.ico', *sorted(ROOT.glob('screenshot*.png'))]:
        assets.append({'path': file.relative_to(ROOT).as_posix(), 'bytes': file.stat().st_size, 'sha256': sha(file)})
    result = {'reviewed': '2026-11-01', 'python': sys.version.split()[0], 'pyqt': PYQT_VERSION_STR, 'qtRuntime': qVersion(),
              'packages': packages, 'assets': assets, 'ffmpeg': {'filename': ffmpeg.name, 'sha256': sha(ffmpeg), 'versionAndConfiguration': command('-version'), 'licenseOutput': command('-L')},
              'opencvBuildInformation': cv2.getBuildInformation()}
    if archive:
        with zipfile.ZipFile(archive) as zipped:
            result['existingArchive'] = {'filename': archive.name, 'sha256': sha(archive), 'files': zipped.namelist(),
                                         'noticeText': {name: zipped.read(name).decode(errors='replace') for name in zipped.namelist() if name.lower().endswith(('.txt', '.md'))}}
    (ROOT/'DEPENDENCY_AUDIT.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    binaries = binary_inventory(bundle)
    (ROOT/'BINARY_AUDIT.json').write_text(json.dumps({'reviewed':'2026-11-01', 'scope':'Existing Nuitka standalone payload retained by the pre-change 1.3.1 onefile build; paths are relative to that payload.', 'files':binaries},indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'packages':len(packages),'copiedNotices':sum(len(p['notices']) for p in packages),'nativeBinaries':len(binaries),'assets':len(assets)},indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bundle',type=Path,default=ROOT/'build/nuitka/CT_Dashcam_Studio.dist')
    parser.add_argument('--archive',type=Path)
    args = parser.parse_args()
    run(args.bundle,args.archive)
