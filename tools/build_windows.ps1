param(
    [string]$OutputDir = "build/nuitka",
    [ValidateSet("standalone", "onefile")][string]$Mode = "standalone"
)
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
Push-Location $repoRoot
try {
    New-Item -ItemType Directory -Force $OutputDir | Out-Null
    $legalStage = Join-Path $OutputDir 'legal-bundle'
    & python tools/prepare_legal_bundle.py --output $legalStage
    if ($LASTEXITCODE -ne 0) { throw 'Could not stage legal documents.' }
    $appVersion = (& python -c 'from core.constants import APP_VERSION; print(APP_VERSION)').Trim()
    if ($LASTEXITCODE -ne 0 -or $appVersion -notmatch '^\d+\.\d+\.\d+(\.\d+)?$') {
        throw 'Could not read the app version from core/constants.py.'
    }
    $arguments = @(
        '-m', 'nuitka', "--mode=$Mode", '--enable-plugin=pyqt6',
        '--mingw64', '--assume-yes-for-downloads', '--windows-console-mode=disable',
        "--product-version=$appVersion", "--file-version=$appVersion",
        '--file-description=CT Dashcam Studio - GPL-3.0-only original code; third-party terms apply',
        '--windows-icon-from-ico=icon.ico' , '--output-filename=CT_Dashcam_Studio.exe',
        "--output-dir=$OutputDir", "--report=$OutputDir/compilation-report.xml",
        '--include-data-dir=assets=assets', '--include-data-dir=models=models',
        '--include-data-files=icon.ico=icon.ico', '--include-data-files=LICENSE=LICENSE',
        "--include-data-dir=$legalStage=legal",
        '--include-package-data=imageio_ffmpeg', '--nofollow-import-to=pytest,tkinter,IPython',
        '--jobs=4', 'CT_Dashcam_Studio.py'
    )
    & python @arguments
    if ($LASTEXITCODE -ne 0) { throw "Nuitka build failed ($LASTEXITCODE)." }
    $legalCompanion = Join-Path $OutputDir 'portable-legal'
    $legalDestination = Join-Path $legalCompanion 'legal'
    New-Item -ItemType Directory -Force $legalDestination | Out-Null
    Get-ChildItem -LiteralPath $legalStage | Copy-Item -Destination $legalDestination -Recurse -Force
} finally { Pop-Location }
