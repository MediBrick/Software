# Install / update the Arduino libraries needed by the MediBrick firmware.
# Libraries are cloned with git into ~\Arduino\libraries (override with $env:ARDUINO_LIBS).
#
# For each library:
#   not installed          -> clone it
#   installed, up to date  -> nothing to do
#   installed, behind      -> fast-forward update
#   local changes/diverged -> skipped with a warning
#   not a git checkout     -> skipped (use -Force to replace it)
#
# Usage: powershell -ExecutionPolicy Bypass -File install_libraries.ps1 [-Force] [-Check]
#   -Force  replace non-git installs (old copy is moved to <name>.bak)
#   -Check  only report status, change nothing

param(
    [switch]$Force,
    [switch]$Check
)

if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    Write-Error "git is required but not installed."
    exit 1
}

$LibDir = if ($env:ARDUINO_LIBS) { $env:ARDUINO_LIBS } else { Join-Path $HOME "Arduino\libraries" }

# Keep in sync with README.md (duplicates removed).
# Entries are "url" (folder = repository name) or "url|folder".
# Libraries developed by Urs Utzinger are installed as UUtzinger_<name>.
$Repos = @(
    "https://github.com/h2zero/NimBLE-Arduino",
    "https://github.com/uutzinger/Arduino_BLESerial|UUtzinger_BLESerial",
    "https://github.com/uutzinger/Arduino_RingBuffer|UUtzinger_RingBuffer",
    "https://github.com/uutzinger/SavitzkyGolayFilter|UUtzinger_SavitzkyGolayFilter",
    "https://github.com/uutzinger/logger|UUtzinger_logger",
    "https://github.com/adafruit/Adafruit_SSD1306",
    "https://github.com/sparkfun/SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library",
    "https://github.com/adafruit/Adafruit_LC709203F",
    "https://github.com/uutzinger/Arduino_MAX30001G|UUtzinger_MAX30001G",
    "https://github.com/Sensirion/arduino-i2c-sfm3505",
    "https://github.com/TronixLab/SFM3300",
    "https://github.com/uutzinger/Arduino_AFE44XX|UUtzinger_AFE44XX",
    "https://github.com/pschatzmann/arduino-audio-tools",
    "https://github.com/sparkfun/SparkFun_MicroPressure_Arduino_Library",
    "https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary",
    "https://github.com/sparkfun/SparkFun_BMP581_Arduino_Library",
    "https://github.com/7semi-solutions/7Semi-BMI323",
    "https://github.com/CancianMatteo/BMM350-Magnetometer",
    "https://github.com/sparkfun/SparkFun_u-blox_MAX-M10S",
    "https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library",
    "https://github.com/adafruit/Adafruit_NeoPixel",
    "https://github.com/Sensirion/arduino-core",
    "https://github.com/Sensirion/arduino-i2c-sen5x",
    "https://github.com/Sensirion/arduino-i2c-scd4x",
    "https://github.com/sensirion/arduino-sht",
    "https://github.com/Sensirion/arduino-i2c-sgp41",
    "https://github.com/Sensirion/gas-index-algorithm",
    "https://github.com/eNBeWe/MiCS6814-I2C-Library",
    "https://github.com/uutzinger/Arduino_DRV8704|UUtzinger_DRV8704"
)

# Old folder names -> current folder names. A git checkout under an old name is renamed;
# anything else (e.g. Library Manager copies) is only reported as a duplicate, never deleted.
$Legacy = @(
    "Arduino_AFE44XX:UUtzinger_AFE44XX",
    "Arduino_BLESerial:UUtzinger_BLESerial",
    "Arduino_DRV8704:UUtzinger_DRV8704",
    "Arduino_MAX30001G:UUtzinger_MAX30001G",
    "Arduino_RingBuffer:UUtzinger_RingBuffer",
    "SavitzkyGolayFilter:UUtzinger_SavitzkyGolayFilter",
    "logger:UUtzinger_logger",
    "BLE_Serial_-_NUS:UUtzinger_BLESerial",
    "MAX30001:UUtzinger_MAX30001G"
)

$new = 0; $ok = 0; $upd = 0; $skip = 0; $fail = 0

# Run git quietly, return stdout (trimmed) and set $script:GitOk
function Invoke-Git {
    $out = & git @args 2>$null
    $script:GitOk = ($LASTEXITCODE -eq 0)
    if ($out) { return ($out -join "`n").Trim() } else { return "" }
}

function Install-Clone($url, $dir) {
    Invoke-Git clone --quiet --depth 1 $url $dir | Out-Null
    if ($script:GitOk) { $script:new++ } else { Write-Host "           clone failed"; $script:fail++ }
}

New-Item -ItemType Directory -Force -Path $LibDir | Out-Null
Write-Host "Arduino libraries folder: $LibDir"
if ($Check) { Write-Host "(check only, no changes will be made)" }
Write-Host ""

if (-not $Check) {
    foreach ($pair in $Legacy) {
        $old, $newName = $pair.Split(':')
        if ((Test-Path (Join-Path $LibDir "$old\.git")) -and -not (Test-Path (Join-Path $LibDir $newName))) {
            Write-Host "[RENAME]   $old -> $newName"
            Move-Item (Join-Path $LibDir $old) (Join-Path $LibDir $newName)
        }
    }
}

foreach ($entry in $Repos) {
    $url  = $entry.Split('|')[0]
    $name = if ($entry.Contains('|')) { $entry.Split('|')[1] } else { $url.TrimEnd('/').Split('/')[-1] }
    $dir  = Join-Path $LibDir $name

    if (-not (Test-Path $dir)) {
        if ($Check) { Write-Host "[MISSING]  $name"; $new++; continue }
        Write-Host "[INSTALL]  $name"
        Install-Clone $url $dir
        continue
    }

    if (-not (Test-Path (Join-Path $dir ".git"))) {
        if ($Force -and -not $Check) {
            Write-Host "[REPLACE]  $name (not a git checkout, old copy moved to $name.bak)"
            if (Test-Path "$dir.bak") { Remove-Item -Recurse -Force "$dir.bak" }
            Move-Item $dir "$dir.bak"
            Install-Clone $url $dir
        } else {
            Write-Host "[SKIP]     $name installed but not a git checkout (use -Force to replace)"
            $skip++
        }
        continue
    }

    # Existing git checkout: compare with the remote's default branch.
    Invoke-Git -C $dir fetch --quiet origin | Out-Null
    if (-not $script:GitOk) { Write-Host "[FAIL]     $name could not fetch from origin"; $fail++; continue }
    Invoke-Git -C $dir remote set-head origin --auto | Out-Null
    $upstream = Invoke-Git -C $dir rev-parse --verify --quiet '@{u}'
    if (-not $upstream) { $upstream = Invoke-Git -C $dir rev-parse --verify --quiet origin/HEAD }
    $localRev = Invoke-Git -C $dir rev-parse HEAD

    if (-not $upstream -or -not $localRev) {
        Write-Host "[SKIP]     $name cannot determine remote revision"; $skip++; continue
    }

    if ($localRev -eq $upstream) {
        Write-Host "[OK]       $name is current"; $ok++
        continue
    }

    Invoke-Git -C $dir merge-base --is-ancestor $localRev $upstream | Out-Null
    if ($script:GitOk) {
        if ($Check) { Write-Host "[OUTDATED] $name"; $upd++ }
        elseif (Invoke-Git -C $dir status --porcelain) {
            Write-Host "[SKIP]     $name is outdated but has local changes"; $skip++
        } else {
            Write-Host "[UPDATE]   $name"
            Invoke-Git -C $dir merge --quiet --ff-only $upstream | Out-Null
            if ($script:GitOk) { $upd++ } else { Write-Host "           update failed"; $fail++ }
        }
    } else {
        Write-Host "[SKIP]     $name has local commits not in remote (diverged)"; $skip++
    }
}

foreach ($pair in $Legacy) {
    $old, $newName = $pair.Split(':')
    if ((Test-Path (Join-Path $LibDir $old)) -and (Test-Path (Join-Path $LibDir $newName))) {
        Write-Host "[DUPLICATE] $old duplicates $newName, remove one of them (Arduino finds both)"
        $skip++
    }
}

Write-Host ""
Write-Host "Installed/missing: $new  Current: $ok  Updated/outdated: $upd  Skipped: $skip  Failed: $fail"
Write-Host ""
Write-Host "Not installable automatically:"
Write-Host "IMU Fusion (see https://github.com/xioTechnologies/Fusion), Amphenol ELVH pressure sensor (custom driver)."
if ($fail -gt 0) { exit 1 }
