param([string]$OutputDir = "build/nuitka")
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
Push-Location $repoRoot
try {
    New-Item -ItemType Directory -Force $OutputDir | Out-Null
    $arguments = @(
        '-m', 'nuitka', '--mode=standalone', '--enable-plugin=pyqt6',
        '--mingw64', '--assume-yes-for-downloads', '--windows-console-mode=disable',
        '--windows-icon-from-ico=icon.ico', '--output-filename=CT_Dashcam_Studio.exe',
        "--output-dir=$OutputDir", "--report=$OutputDir/compilation-report.xml",
        '--include-data-dir=assets=assets', '--include-data-dir=models=models',
        '--include-data-files=icon.ico=icon.ico', '--include-data-files=LICENSE=LICENSE',
        '--include-package-data=imageio_ffmpeg', '--nofollow-import-to=pytest,tkinter,IPython',
        '--jobs=4', 'CT_Dashcam_Studio.py'
    )
    & python @arguments
    if ($LASTEXITCODE -ne 0) { throw "Nuitka build failed ($LASTEXITCODE)." }
} finally { Pop-Location }
