#!/usr/bin/env bash
# Install / update the Arduino libraries needed by the MediBrick firmware.
# Libraries are cloned with git into ~/Arduino/libraries (override with ARDUINO_LIBS).
#
# For each library:
#   not installed          -> clone it
#   installed, up to date  -> nothing to do
#   installed, behind      -> fast-forward update
#   local changes/diverged -> skipped with a warning
#   not a git checkout     -> skipped (use --force to replace it)
#
# Usage: ./install_libraries.sh [--force] [--check]
#   --force  replace non-git installs (old copy is moved to <name>.bak)
#   --check  only report status, change nothing

set -u

LIBDIR="${ARDUINO_LIBS:-$HOME/Arduino/libraries}"
FORCE=0
CHECK=0
for arg in "$@"; do
    case "$arg" in
        --force) FORCE=1 ;;
        --check) CHECK=1 ;;
        -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
        *) echo "Unknown option: $arg" >&2; exit 2 ;;
    esac
done

command -v git >/dev/null 2>&1 || { echo "git is required but not installed." >&2; exit 1; }

# Keep in sync with README.md (duplicates removed).
# Entries are "url" (folder = repository name) or "url|folder".
# Libraries developed by Urs Utzinger are installed as UUtzinger_<name>.
REPOS=(
    https://github.com/h2zero/NimBLE-Arduino
    "https://github.com/uutzinger/Arduino_BLESerial|UUtzinger_BLESerial"
    "https://github.com/uutzinger/Arduino_RingBuffer|UUtzinger_RingBuffer"
    "https://github.com/uutzinger/SavitzkyGolayFilter|UUtzinger_SavitzkyGolayFilter"
    "https://github.com/uutzinger/logger|UUtzinger_logger"
    https://github.com/adafruit/Adafruit_SSD1306
    https://github.com/sparkfun/SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library
    https://github.com/adafruit/Adafruit_LC709203F
    "https://github.com/uutzinger/Arduino_MAX30001G|UUtzinger_MAX30001G"
    https://github.com/Sensirion/arduino-i2c-sfm3505
    https://github.com/TronixLab/SFM3300
    "https://github.com/uutzinger/Arduino_AFE44XX|UUtzinger_AFE44XX"
    https://github.com/pschatzmann/arduino-audio-tools
    https://github.com/sparkfun/SparkFun_MicroPressure_Arduino_Library
    https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary
    https://github.com/sparkfun/SparkFun_BMP581_Arduino_Library
    https://github.com/7semi-solutions/7Semi-BMI323
    https://github.com/CancianMatteo/BMM350-Magnetometer
    https://github.com/sparkfun/SparkFun_u-blox_MAX-M10S
    https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library
    https://github.com/adafruit/Adafruit_NeoPixel
    https://github.com/Sensirion/arduino-core
    https://github.com/Sensirion/arduino-i2c-sen5x
    https://github.com/Sensirion/arduino-i2c-scd4x
    https://github.com/sensirion/arduino-sht
    https://github.com/Sensirion/arduino-i2c-sgp41
    https://github.com/Sensirion/gas-index-algorithm
    https://github.com/eNBeWe/MiCS6814-I2C-Library
    "https://github.com/uutzinger/Arduino_DRV8704|UUtzinger_DRV8704"
)

# Old folder names -> current folder names. A git checkout under an old name is renamed;
# anything else (e.g. Library Manager copies) is only reported as a duplicate, never deleted.
LEGACY=(
    Arduino_AFE44XX:UUtzinger_AFE44XX
    Arduino_BLESerial:UUtzinger_BLESerial
    Arduino_DRV8704:UUtzinger_DRV8704
    Arduino_MAX30001G:UUtzinger_MAX30001G
    Arduino_RingBuffer:UUtzinger_RingBuffer
    SavitzkyGolayFilter:UUtzinger_SavitzkyGolayFilter
    logger:UUtzinger_logger
    BLE_Serial_-_NUS:UUtzinger_BLESerial
    MAX30001:UUtzinger_MAX30001G
)

n_new=0; n_ok=0; n_upd=0; n_skip=0; n_fail=0

mkdir -p "$LIBDIR" || { echo "Cannot create $LIBDIR" >&2; exit 1; }
echo "Arduino libraries folder: $LIBDIR"
[ "$CHECK" -eq 1 ] && echo "(check only, no changes will be made)"
echo

if [ "$CHECK" -eq 0 ]; then
    for pair in "${LEGACY[@]}"; do
        old="${pair%%:*}"; new="${pair##*:}"
        if [ -d "$LIBDIR/$old/.git" ] && [ ! -e "$LIBDIR/$new" ]; then
            echo "[RENAME]   $old -> $new"
            mv "$LIBDIR/$old" "$LIBDIR/$new"
        fi
    done
fi

for entry in "${REPOS[@]}"; do
    url="${entry%%|*}"
    if [ "$entry" = "$url" ]; then name="$(basename "$url")"; else name="${entry##*|}"; fi
    dir="$LIBDIR/$name"

    if [ ! -e "$dir" ]; then
        if [ "$CHECK" -eq 1 ]; then
            echo "[MISSING]  $name"; n_new=$((n_new+1)); continue
        fi
        echo "[INSTALL]  $name"
        if git clone --quiet --depth 1 "$url" "$dir"; then n_new=$((n_new+1)); else echo "           clone failed"; n_fail=$((n_fail+1)); fi
        continue
    fi

    if [ ! -d "$dir/.git" ]; then
        if [ "$FORCE" -eq 1 ] && [ "$CHECK" -eq 0 ]; then
            echo "[REPLACE]  $name (not a git checkout, old copy moved to $name.bak)"
            rm -rf "$dir.bak"
            mv "$dir" "$dir.bak" || { n_fail=$((n_fail+1)); continue; }
            if git clone --quiet --depth 1 "$url" "$dir"; then n_new=$((n_new+1)); else echo "           clone failed"; n_fail=$((n_fail+1)); fi
        else
            echo "[SKIP]     $name installed but not a git checkout (use --force to replace)"
            n_skip=$((n_skip+1))
        fi
        continue
    fi

    # Existing git checkout: compare with the remote's default branch.
    if ! git -C "$dir" fetch --quiet origin 2>/dev/null; then
        echo "[FAIL]     $name could not fetch from origin"; n_fail=$((n_fail+1)); continue
    fi
    git -C "$dir" remote set-head origin --auto >/dev/null 2>&1
    upstream="$(git -C "$dir" rev-parse --verify --quiet '@{u}' 2>/dev/null || git -C "$dir" rev-parse --verify --quiet origin/HEAD 2>/dev/null)"
    local_rev="$(git -C "$dir" rev-parse HEAD 2>/dev/null)"

    if [ -z "$upstream" ] || [ -z "$local_rev" ]; then
        echo "[SKIP]     $name cannot determine remote revision"; n_skip=$((n_skip+1)); continue
    fi

    if [ "$local_rev" = "$upstream" ]; then
        echo "[OK]       $name is current"; n_ok=$((n_ok+1))
    elif git -C "$dir" merge-base --is-ancestor "$local_rev" "$upstream" 2>/dev/null; then
        if [ "$CHECK" -eq 1 ]; then
            echo "[OUTDATED] $name"; n_upd=$((n_upd+1))
        elif [ -n "$(git -C "$dir" status --porcelain)" ]; then
            echo "[SKIP]     $name is outdated but has local changes"; n_skip=$((n_skip+1))
        else
            echo "[UPDATE]   $name"
            if git -C "$dir" merge --quiet --ff-only "$upstream"; then n_upd=$((n_upd+1)); else echo "           update failed"; n_fail=$((n_fail+1)); fi
        fi
    else
        echo "[SKIP]     $name has local commits not in remote (diverged)"; n_skip=$((n_skip+1))
    fi
done

for pair in "${LEGACY[@]}"; do
    old="${pair%%:*}"; new="${pair##*:}"
    if [ -e "$LIBDIR/$old" ] && [ -e "$LIBDIR/$new" ]; then
        echo "[DUPLICATE] $old duplicates $new, remove one of them (Arduino finds both)"
        n_skip=$((n_skip+1))
    fi
done

echo
echo "Installed/missing: $n_new  Current: $n_ok  Updated/outdated: $n_upd  Skipped: $n_skip  Failed: $n_fail"
echo
echo "Not installable automatically:"
echo "IMU Fusion (see https://github.com/xioTechnologies/Fusion), Amphenol ELVH pressure sensor (custom driver)."
[ "$n_fail" -eq 0 ]
