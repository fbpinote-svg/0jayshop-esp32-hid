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

function Get-Esp32UploadPort {
  $devices = & $pio device list --json-output | ConvertFrom-Json
  foreach ($device in $devices) {
    if ($device.hwid -match "VID:PID=303A" -or
        $device.hwid -match "VID_303A" -or
        $device.description -match "ESP32|USB JTAG") {
      return $device.port
    }
  }
  return $null
}

$port = if ($args.Count) { $args[0] } else { Get-Esp32UploadPort }
if (-not $port) {
  throw "ESP32 upload port not found. Pass a COM port, e.g. .\upload.ps1 COM20"
}

& $pio run -d $PSScriptRoot -t upload --upload-port $port
