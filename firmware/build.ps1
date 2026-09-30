$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
if (-not $env:PLATFORMIO_CORE_DIR) {
  $env:PLATFORMIO_CORE_DIR = Join-Path $repoRoot ".platformio"
}

$repoPio = Join-Path $env:PLATFORMIO_CORE_DIR "penv\Scripts\pio.exe"
$pio = if ($env:PLATFORMIO_EXE) {
  $env:PLATFORMIO_EXE
} elseif (Test-Path $repoPio) {
  $repoPio
} else {
  (Get-Command pio -ErrorAction Stop).Source
}

& $pio run -d $PSScriptRoot
