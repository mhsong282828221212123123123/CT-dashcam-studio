$env:PYTHONPATH = Join-Path $PSScriptRoot 'src'
& (Join-Path $PSScriptRoot '.venv/Scripts/python.exe') -m ct_scene.review_app @args
