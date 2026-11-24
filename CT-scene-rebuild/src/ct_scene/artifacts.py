"""Content-addressed, verified, immutable-by-contract stage outputs."""
from pathlib import Path, PurePosixPath
from datetime import datetime, timezone
import hashlib
import json
import os
import shutil
import tempfile


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=False, allow_nan=False).encode('utf-8')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def safe_member(name):
    p = PurePosixPath(name)
    if not name or '\\' in name or ':' in name or p.is_absolute() or '..' in p.parts or str(p) != name:
        raise ValueError('Invalid payload path: ' + name)
    if name == 'artifact.json':
        raise ValueError('Reserved manifest path')
    return p


def identity(manifest):
    return digest(canonical({k: v for k, v in manifest.items()
                             if k not in {'artifact_id', 'created_utc', 'run_id'}}))


def verify(directory):
    root = Path(directory).resolve()
    manifest = json.loads((root / 'artifact.json').read_text(encoding='utf-8'))
    from jsonschema import Draft202012Validator, FormatChecker
    schema = json.loads((Path(__file__).parent/'schemas/artifact.json').read_text(encoding='utf-8'))
    errors = list(Draft202012Validator(schema, format_checker=FormatChecker()).iter_errors(manifest))
    if errors:
        raise ValueError('Artifact schema: '+errors[0].message)
    required = {'schema', 'schema_version', 'stage', 'inputs', 'config_sha256',
                'code_revision', 'model_sha256', 'runtime', 'payloads', 'status', 'quality_gate'}
    if not required <= manifest.keys() or manifest['status'] != 'complete':
        raise ValueError('Incomplete artifact manifest')
    if identity(manifest) != manifest['artifact_id']:
        raise ValueError('Manifest identity mismatch')
    names = set()
    for item in manifest['payloads']:
        member = safe_member(item['path'])
        path = root.joinpath(*member.parts)
        if item['path'] in names or not path.resolve().is_relative_to(root):
            raise ValueError('Duplicate or escaped payload')
        names.add(item['path'])
        if path.is_symlink():
            raise ValueError('Symlink payload forbidden')
        data = path.read_bytes()
        if len(data) != item['bytes'] or digest(data) != item['sha256']:
            raise ValueError('Payload integrity failure: ' + item['path'])
    return manifest


def publish(destination, *, stage, payloads, inputs, config, code_revision,
            runtime, model_hashes=(), cancelled=lambda: False):
    dest = Path(destination).absolute()
    if dest.exists():
        raise FileExistsError(dest)
    dest.parent.mkdir(parents=True, exist_ok=True)
    temp = Path(tempfile.mkdtemp(prefix='.partial-', dir=dest.parent))
    try:
        records = []
        for name, data in sorted(payloads.items()):
            if cancelled():
                raise InterruptedError('Artifact publication cancelled')
            member = safe_member(name)
            target = temp.joinpath(*member.parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
            records.append({'path': name, 'sha256': digest(data), 'bytes': len(data)})
        manifest = dict(schema='ct_scene.' + stage, schema_version='0.1.0',
                        run_id=dest.name, stage=stage, status='complete', inputs=inputs,
                        code_revision=code_revision, config_sha256=digest(canonical(config)),
                        model_sha256=list(model_hashes), runtime=runtime, random_seed=20261114,
                        payloads=records, warnings=[], quality_gate='NOT_EVALUATED',
                        created_utc=datetime.now(timezone.utc).isoformat())
        manifest['artifact_id'] = identity(manifest)
        (temp / 'artifact.json').write_bytes(canonical(manifest))
        verify(temp)
        if cancelled():
            raise InterruptedError('Artifact publication cancelled')
        # Windows rename refuses an existing target; do not replace completed runs.
        os.rename(temp, dest)
        return manifest
    finally:
        if temp.exists():
            shutil.rmtree(temp)
