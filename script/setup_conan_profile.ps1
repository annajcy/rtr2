param(
    [string]$ProfileName = "rtr2-local",
    [string]$VsPath
)

$profilesDir = Join-Path $HOME ".conan2\\profiles"
if (-not (Test-Path $profilesDir)) {
    New-Item -ItemType Directory -Force -Path $profilesDir | Out-Null
}

if (-not $VsPath) {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\\Installer\\vswhere.exe"
    if (-not (Test-Path $vswhere)) {
        Write-Error "vswhere not found. Install Visual Studio Build Tools or pass -VsPath."
        exit 1
    }

    $VsPath = & $vswhere -latest -products * `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
        -property installationPath
}

if (-not $VsPath) {
    Write-Error "Could not detect Visual Studio. Pass -VsPath to set it manually."
    exit 1
}

$vcvars = Join-Path $VsPath "VC\\Auxiliary\\Build\\vcvarsall.bat"
if (-not (Test-Path $vcvars)) {
    Write-Error "Missing VC toolchain. Install 'Desktop development with C++' or pass a valid -VsPath."
    exit 1
}

$profilePath = Join-Path $profilesDir $ProfileName
$content = "[conf]`n" +
    "tools.microsoft.msbuild:installation_path=$VsPath`n"

Set-Content -Path $profilePath -Value $content -Encoding ASCII
Write-Output "Wrote $profilePath"
