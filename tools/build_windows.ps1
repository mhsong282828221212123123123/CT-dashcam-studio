param(
    [string]$OutputDir = "build/nuitka",
    [ValidateSet("standalone", "onefile")][string]$Mode = "standalone"
)
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
Push-Location $repoRoot
try {
    New-Item -ItemType Directory -Force $OutputDir | Out-Null
    $appVersion = (& python -c 'from core.constants import APP_VERSION; print(APP_VERSION)').Trim()
    if ($LASTEXITCODE -ne 0 -or $appVersion -notmatch '^\d+\.\d+\.\d+(\.\d+)?$') {
        throw 'Could not read the app version from core/constants.py.'
    }
    $arguments = @(
        '-m', 'nuitka', "--mode=$Mode", '--enable-plugin=pyqt6',
        '--mingw64', '--assume-yes-for-downloads', '--windows-console-mode=disable',
        "--product-version=$appVersion", "--file-version=$appVersion",
        '--windows-icon-from-ico=icon.ico' , '--output-filename=CT_Dashcam_Studio.exe',
        "--output-dir=$OutputDir", "--report=$OutputDir/compilation-report.xml",
        '--include-data-dir=assets=assets', '--include-data-dir=models=models',
        '--include-data-files=icon.ico=icon.ico', '--include-data-files=LICENSE=LICENSE',
        '--include-package-data=imageio_ffmpeg', '--nofollow-import-to=pytest,tkinter,IPython',
        '--jobs=4', 'CT_Dashcam_Studio.py'
    )
    & python @arguments
    if ($LASTEXITCODE -ne 0) { throw "Nuitka build failed ($LASTEXITCODE)." }
} finally { Pop-Location }
