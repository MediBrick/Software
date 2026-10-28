#include <drv8704.h>
#include <logger.h>

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr BridgeId kBridge = BridgeId::A;
constexpr float kDefaultCurrentAmps = 0.1f;
constexpr float kDefaultCurrentLimitAmps = 5.0f;
constexpr float kDefaultShuntOhms = 0.0005f;
constexpr float kDefaultDutyPercent = 25.0f;
constexpr uint32_t kDefaultPwmFrequencyHz = DRV8704_DEFAULT_INPUT_PWM_HZ;
constexpr uint8_t kDefaultResolutionBits = DRV8704_DEFAULT_PWM_RES_BITS;
constexpr size_t kCommandBufferSize = 80;

enum class DriveMode : uint8_t {
  CurrentLimited = 0,
  PwmLimited,
  FullBridge
};

struct SweepStep {
  const char* label;
  uint8_t offTime;
  uint8_t blankTime;
  DecayMode decayMode;
};

char commandBuffer[kCommandBufferSize];
size_t commandLength = 0;
float requestedCurrentAmps = kDefaultCurrentAmps;
float requestedCurrentLimitAmps = kDefaultCurrentLimitAmps;
float configuredShuntOhms = kDefaultShuntOhms;
float requestedDutyPercent = kDefaultDutyPercent;
uint32_t requestedPwmFrequencyHz = kDefaultPwmFrequencyHz;
uint8_t requestedResolutionBits = kDefaultResolutionBits;
Direction requestedDirection = Direction::Forward;
CurrentModePreset currentPreset = CurrentModePreset::Heater;
DriveMode driveMode = DriveMode::CurrentLimited;
bool offTimeOverrideEnabled = false;
uint8_t offTimeOverride = 0U;
bool blankTimeOverrideEnabled = false;
uint8_t blankTimeOverride = 0U;
bool decayModeOverrideEnabled = false;
DecayMode decayModeOverride = DecayMode::Slow;
bool driveEnabled = false;
bool driverReady = false;

bool applyDriveSettings();
bool beginOrRebeginPwmMode();

DRV8704Pins makePins() {
  DRV8704Pins pins;
  pins.csPin = 5;
  pins.sleepPin = 11;
  pins.resetPin = A0;
  pins.faultPin = A1;
  pins.ain1Pin = A2;
  pins.ain2Pin = A3;
  pins.bin1Pin = A4;
  pins.bin2Pin = A5;
  return pins;
}

DRV8704& motorDriver() {
  static DRV8704Pins pins = makePins();
  static DRV8704 driver(pins);
  return driver;
}

const char* directionName(Direction direction) {
  return direction == Direction::Forward ? "forward" : "reverse";
}

const char* runtimeStateName(BridgeRuntimeState runtimeState) {
  switch (runtimeState) {
    case BridgeRuntimeState::Coast:
      return "coast";
    case BridgeRuntimeState::Brake:
      return "brake";
    case BridgeRuntimeState::CurrentDrive:
      return "current-drive";
    case BridgeRuntimeState::PwmDriveWithCurrentLimit:
      return "pwm";
  }

  return "unknown";
}

const char* senseGainName(SenseGain gain) {
  switch (gain) {
    case SenseGain::Gain5VperV:
      return "5 V/V";
    case SenseGain::Gain10VperV:
      return "10 V/V";
    case SenseGain::Gain20VperV:
      return "20 V/V";
    case SenseGain::Gain40VperV:
      return "40 V/V";
  }

  return "unknown";
}

const char* driveModeName(DriveMode mode) {
  switch (mode) {
    case DriveMode::CurrentLimited:
      return "current-limited";
    case DriveMode::PwmLimited:
      return "pwm-limited";
    case DriveMode::FullBridge:
      return "full-bridge";
  }

  return "unknown";
}

const char* presetName(CurrentModePreset preset) {
  switch (preset) {
    case CurrentModePreset::Heater:
      return "heater";
    case CurrentModePreset::ThermoelectricCooler:
      return "tec";
    case CurrentModePreset::MotorSmallInductance:
      return "motor-small";
    case CurrentModePreset::MotorMediumInductance:
      return "motor-medium";
    case CurrentModePreset::MotorLargeInductance:
      return "motor-large";
  }

  return "unknown";
}

const char* decayModeName(DecayMode mode) {
  switch (mode) {
    case DecayMode::Slow:
      return "slow";
    case DecayMode::Fast:
      return "fast";
    case DecayMode::Mixed:
      return "mixed";
    case DecayMode::AutoMixed:
      return "auto";
  }

  return "unknown";
}

const char* pwmBackendName(PwmBackendType backendType) {
  switch (backendType) {
    case PwmBackendType::None:
      return "none";
    case PwmBackendType::GeneralAnalogWrite:
      return "analogWrite";
    case PwmBackendType::Esp32Ledc:
      return "esp32-ledc";
    case PwmBackendType::TeensyHardware:
      return "teensy-hw";
    case PwmBackendType::StmHardware:
      return "stm-hw";
    case PwmBackendType::Unsupported:
      return "unsupported";
  }

  return "unknown";
}

bool isMaximumSenseGain(SenseGain gain) {
  return gain == SenseGain::Gain40VperV;
}

uint8_t effectiveOffTime(const DRV8704CurrentPresetConfig& presetConfig) {
  return offTimeOverrideEnabled ? offTimeOverride : presetConfig.offTime;
}

uint8_t effectiveBlankTime(const DRV8704CurrentPresetConfig& presetConfig) {
  return blankTimeOverrideEnabled ? blankTimeOverride : presetConfig.blankTime;
}

DecayMode effectiveDecayMode(const DRV8704CurrentPresetConfig& presetConfig) {
  return decayModeOverrideEnabled ? decayModeOverride : presetConfig.decayMode;
}

bool applyTimingOverrides() {
  DRV8704& driver = motorDriver();

  if (offTimeOverrideEnabled && !driver.setOffTime(offTimeOverride)) {
    Serial.println("Failed to reapply TOFF override");
    return false;
  }

  if (blankTimeOverrideEnabled && !driver.setBlankTime(blankTimeOverride)) {
    Serial.println("Failed to reapply TBLANK override");
    return false;
  }

  if (decayModeOverrideEnabled && !driver.setDecayMode(decayModeOverride)) {
    Serial.println("Failed to reapply decay override");
    return false;
  }

  return true;
}

void clearTimingOverrides() {
  offTimeOverrideEnabled = false;
  blankTimeOverrideEnabled = false;
  decayModeOverrideEnabled = false;
}

void configureTimingOverrides(uint8_t offTime, uint8_t blankTime, DecayMode decayMode) {
  offTimeOverrideEnabled = true;
  offTimeOverride = offTime;
  blankTimeOverrideEnabled = true;
  blankTimeOverride = blankTime;
  decayModeOverrideEnabled = true;
  decayModeOverride = decayMode;
}

void printActiveTimingSummary() {
  const DRV8704CurrentPresetConfig presetConfig = motorDriver().currentModePresetConfig();
  Serial.print("Active timing: TOFF=0x");
  Serial.print(effectiveOffTime(presetConfig), HEX);
  Serial.print(" TBLANK=0x");
  Serial.print(effectiveBlankTime(presetConfig), HEX);
  Serial.print(" DECAY=");
  Serial.println(decayModeName(effectiveDecayMode(presetConfig)));
}

void printCurrentLimit(Stream& out, const DRV8704CurrentLimitResult& result) {
  out.print("Requested current limit (A): ");
  out.println(result.requestedCurrentLimitAmps, 4);
  out.print("Effective current limit on bridge A (A): ");
  out.println(result.appliedCurrentLimitAmpsA, 4);
  out.print("Effective current limit on bridge B (A): ");
  out.println(result.appliedCurrentLimitAmpsB, 4);
  out.print("Reference shunt (ohms): ");
  out.println(result.referenceShuntOhms, 6);
  out.print("Selected gain: ");
  out.println(senseGainName(result.selectedGain));
  out.print("Using maximum gain: ");
  out.println(isMaximumSenseGain(result.selectedGain) ? "yes" : "no");
  out.print("Torque DAC: ");
  out.println(result.torqueDac);
  out.print("Max current (A): ");
  out.println(result.achievableMaxCurrentAmps, 4);
  out.print("Smallest current step (A/count): ");
  out.println(result.ampsPerTorqueCount, 6);
}

void printPwmCapability(Stream& out, const DRV8704PwmCapability& capability) {
  out.print("PWM backend: ");
  out.println(pwmBackendName(capability.backendType));
  out.print("Requested frequency (Hz): ");
  out.println(capability.requestedFrequencyHz);
  out.print("Achieved frequency (Hz): ");
  out.println(capability.achievedFrequencyHz);
  out.print("Resolution bits: ");
  out.println(capability.resolutionBits);
  out.print("Duty steps: ");
  out.println(capability.dutySteps);
  out.print("Smallest increment (%): ");
  out.println(capability.smallestDutyIncrementPercent, 6);
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  Common:");
  Serial.println("    help                         - show this help");
  Serial.println("    status                       - print current, PWM, and fault state");
  Serial.println("    mode <current|pwm|full>      - select current drive, PWM drive, or full bridge");
  Serial.println("    preset <heater|tec|ms|mm|ml> - set the current-limit preset");
  Serial.println("    shunt <ohms>                 - set current-sense shunt resistance");
  Serial.println("    direction <f|r>              - set direction to forward or reverse");
  Serial.println("    full [f|r]                   - energize bridge A with no current limit");
  Serial.println("    coast                        - disable bridge drive");
  Serial.println("    brake                        - brake bridge A");
  Serial.println("    clearfaults                  - clear DRV8704 latched faults");
  Serial.println("  Current Mode:");
  Serial.println("    current <amps>               - set requested current in amps");
  Serial.println("    drive [amps] [f|r]           - apply current-limited drive");
  Serial.println("    off <0..255>                 - set current-chop off-time register");
  Serial.println("    blank <0..255>               - set current-sense blanking register");
  Serial.println("    decay <slow|fast|mixed|auto> - set decay mode");
  Serial.println("    sweep heater                 - run step-by-step heater timing sweep");
  Serial.println("    sweep decay                  - vary decay mode only");
  Serial.println("    sweep blank                  - vary TBLANK only");
  Serial.println("    sweep off                    - vary TOFF only");
  Serial.println("  PWM Mode:");
  Serial.println("    limit <amps>                 - set PWM current limit in amps");
  Serial.println("    duty <0..100>                - set PWM duty percent");
  Serial.println("    freq <hz>                    - set PWM frequency");
  Serial.println("    res <bits>                   - reinitialize PWM mode with new resolution");
  Serial.println("    pwm [duty] [f|r]             - apply PWM using current limit");
  Serial.println();
}

bool parseDirectionToken(const char* token, Direction& direction) {
  if (token == nullptr) {
    return false;
  }

  if (strcmp(token, "f") == 0 || strcmp(token, "forward") == 0) {
    direction = Direction::Forward;
    return true;
  }

  if (strcmp(token, "r") == 0 || strcmp(token, "reverse") == 0) {
    direction = Direction::Reverse;
    return true;
  }

  return false;
}

bool parsePositiveFloat(const char* token, float& value) {
  if (token == nullptr || *token == '\0') {
    return false;
  }

  char* end = nullptr;
  const float parsed = strtof(token, &end);
  if (end == token || *end != '\0' || parsed <= 0.0f) {
    return false;
  }

  value = parsed;
  return true;
}

bool parseDutyPercent(const char* token, float& value) {
  if (token == nullptr || *token == '\0') {
    return false;
  }

  char* end = nullptr;
  const float parsed = strtof(token, &end);
  if (end == token || *end != '\0' || parsed < 0.0f || parsed > 100.0f) {
    return false;
  }

  value = parsed;
  return true;
}

bool parsePositiveUint32(const char* token, uint32_t& value) {
  if (token == nullptr || *token == '\0') {
    return false;
  }

  char* end = nullptr;
  const unsigned long parsed = strtoul(token, &end, 0);
  if (end == token || *end != '\0' || parsed == 0UL) {
    return false;
  }

  value = static_cast<uint32_t>(parsed);
  return true;
}

bool parseResolutionBits(const char* token, uint8_t& value) {
  uint32_t parsed = 0UL;
  if (!parsePositiveUint32(token, parsed) || parsed > 16UL) {
    return false;
  }

  value = static_cast<uint8_t>(parsed);
  return true;
}

bool parsePresetToken(const char* token, CurrentModePreset& preset) {
  if (token == nullptr) {
    return false;
  }

  if (strcmp(token, "heater") == 0) {
    preset = CurrentModePreset::Heater;
    return true;
  }

  if (strcmp(token, "tec") == 0 || strcmp(token, "thermoelectric") == 0) {
    preset = CurrentModePreset::ThermoelectricCooler;
    return true;
  }

  if (strcmp(token, "ms") == 0 || strcmp(token, "motor-small") == 0) {
    preset = CurrentModePreset::MotorSmallInductance;
    return true;
  }

  if (strcmp(token, "mm") == 0 || strcmp(token, "motor-medium") == 0) {
    preset = CurrentModePreset::MotorMediumInductance;
    return true;
  }

  if (strcmp(token, "ml") == 0 || strcmp(token, "motor-large") == 0) {
    preset = CurrentModePreset::MotorLargeInductance;
    return true;
  }

  return false;
}

bool parseByteValue(const char* token, uint8_t& value) {
  if (token == nullptr || *token == '\0') {
    return false;
  }

  char* end = nullptr;
  const long parsed = strtol(token, &end, 0);
  if (end == token || *end != '\0' || parsed < 0L || parsed > 255L) {
    return false;
  }

  value = static_cast<uint8_t>(parsed);
  return true;
}

bool parseDecayModeToken(const char* token, DecayMode& mode) {
  if (token == nullptr) {
    return false;
  }

  if (strcmp(token, "slow") == 0) {
    mode = DecayMode::Slow;
    return true;
  }

  if (strcmp(token, "fast") == 0) {
    mode = DecayMode::Fast;
    return true;
  }

  if (strcmp(token, "mixed") == 0) {
    mode = DecayMode::Mixed;
    return true;
  }

  if (strcmp(token, "auto") == 0 || strcmp(token, "automixed") == 0) {
    mode = DecayMode::AutoMixed;
    return true;
  }

  return false;
}

bool beginOrRebeginPwmMode() {
  DRV8704& driver = motorDriver();
  driver.endPwmMode();

  DRV8704PwmConfig pwmConfig;
  pwmConfig.frequencyHz = requestedPwmFrequencyHz;
  pwmConfig.preferredResolutionBits = requestedResolutionBits;

  if (!driver.beginPwmMode(pwmConfig)) {
    Serial.println("PWM mode unavailable on this target");
    return false;
  }

  return true;
}

bool waitForSweepAdvance() {
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("Press Enter for next step, or q then Enter to stop.");

  char firstChar = '\0';
  while (true) {
    while (Serial.available() <= 0) {
      delay(10);
    }

    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      return !(firstChar == 'q' || firstChar == 'Q');
    }

    if (firstChar == '\0' && !isspace(static_cast<unsigned char>(incoming))) {
      firstChar = incoming;
    }
  }
}

bool runSweepSteps(const char* name, const SweepStep* steps, size_t stepCount) {
  DRV8704& driver = motorDriver();

  currentPreset = CurrentModePreset::Heater;
  if (!driver.setCurrentModePreset(currentPreset)) {
    Serial.println("Failed to apply heater preset");
    return false;
  }

  driveMode = DriveMode::CurrentLimited;
  Serial.println();
  Serial.print("Starting ");
  Serial.print(name);
  Serial.println(" sweep");
  Serial.print("Requested current for sweep: ");
  Serial.print(requestedCurrentAmps, 4);
  Serial.println(" A");

  for (size_t i = 0; i < stepCount; ++i) {
    configureTimingOverrides(steps[i].offTime, steps[i].blankTime, steps[i].decayMode);
    Serial.println();
    Serial.print("Sweep step ");
    Serial.print(i + 1U);
    Serial.print(" of ");
    Serial.print(stepCount);
    Serial.print(": ");
    Serial.println(steps[i].label);

    if (!applyDriveSettings()) {
      Serial.println("Sweep aborted");
      return false;
    }

    printActiveTimingSummary();
    if ((i + 1U) < stepCount && !waitForSweepAdvance()) {
      Serial.println("Sweep stopped by user");
      return false;
    }
  }

  Serial.println();
  Serial.print(name);
  Serial.println(" sweep complete");
  Serial.println("Use 'status' to inspect the final step or 'preset heater' to restore defaults.");
  return true;
}

void runHeaterSweep() {
  static const SweepStep kSteps[] = {
      {"heater-base", 0x40U, 0x00U, DecayMode::AutoMixed},
      {"blank-0x20", 0x40U, 0x20U, DecayMode::AutoMixed},
      {"short-off", 0x01U, 0x00U, DecayMode::AutoMixed},
      {"slow-decay", 0x01U, 0x00U, DecayMode::Slow},
      {"mixed-decay", 0x01U, 0x00U, DecayMode::Mixed},
      {"fast-decay", 0x01U, 0x00U, DecayMode::Fast},
  };

  runSweepSteps("heater", kSteps, sizeof(kSteps) / sizeof(kSteps[0]));
}

void runDecaySweep() {
  const DRV8704CurrentPresetConfig presetConfig = motorDriver().currentModePresetConfig();
  static const DecayMode kDecayModes[] = {
      DecayMode::Slow,
      DecayMode::AutoMixed,
      DecayMode::Mixed,
      DecayMode::Fast,
  };
  SweepStep steps[sizeof(kDecayModes) / sizeof(kDecayModes[0])];

  for (size_t i = 0; i < (sizeof(kDecayModes) / sizeof(kDecayModes[0])); ++i) {
    steps[i].label = decayModeName(kDecayModes[i]);
    steps[i].offTime = effectiveOffTime(presetConfig);
    steps[i].blankTime = effectiveBlankTime(presetConfig);
    steps[i].decayMode = kDecayModes[i];
  }

  runSweepSteps("decay", steps, sizeof(steps) / sizeof(steps[0]));
}

void runBlankSweep() {
  const DRV8704CurrentPresetConfig presetConfig = motorDriver().currentModePresetConfig();
  static const uint8_t kBlankValues[] = {0x00U, 0x08U, 0x10U, 0x20U, 0x40U, 0x80U};
  static const char* kLabels[] = {"0x00", "0x08", "0x10", "0x20", "0x40", "0x80"};
  SweepStep steps[sizeof(kBlankValues) / sizeof(kBlankValues[0])];

  for (size_t i = 0; i < (sizeof(kBlankValues) / sizeof(kBlankValues[0])); ++i) {
    steps[i].label = kLabels[i];
    steps[i].offTime = effectiveOffTime(presetConfig);
    steps[i].blankTime = kBlankValues[i];
    steps[i].decayMode = effectiveDecayMode(presetConfig);
  }

  runSweepSteps("blank", steps, sizeof(steps) / sizeof(steps[0]));
}

void runOffSweep() {
  const DRV8704CurrentPresetConfig presetConfig = motorDriver().currentModePresetConfig();
  static const uint8_t kOffValues[] = {0x01U, 0x04U, 0x08U, 0x10U, 0x20U, 0x40U};
  static const char* kLabels[] = {"0x01", "0x04", "0x08", "0x10", "0x20", "0x40"};
  SweepStep steps[sizeof(kOffValues) / sizeof(kOffValues[0])];

  for (size_t i = 0; i < (sizeof(kOffValues) / sizeof(kOffValues[0])); ++i) {
    steps[i].label = kLabels[i];
    steps[i].offTime = kOffValues[i];
    steps[i].blankTime = effectiveBlankTime(presetConfig);
    steps[i].decayMode = effectiveDecayMode(presetConfig);
  }

  runSweepSteps("off-time", steps, sizeof(steps) / sizeof(steps[0]));
}

void printStatus() {
  DRV8704& driver = motorDriver();
  const DRV8704BridgeState bridgeState = driver.bridgeState(kBridge);
  const DRV8704Status status = driver.readStatus();
  const DRV8704CurrentPresetConfig presetConfig = driver.currentModePresetConfig();

  Serial.println();
  Serial.print("Stored current (A): ");
  Serial.println(requestedCurrentAmps, 4);
  Serial.print("Stored PWM current limit (A): ");
  Serial.println(requestedCurrentLimitAmps, 4);
  Serial.print("Configured shunt (ohms): ");
  Serial.println(configuredShuntOhms, 6);
  Serial.print("Current preset: ");
  Serial.println(presetName(currentPreset));
  Serial.print("Stored duty (%): ");
  Serial.println(requestedDutyPercent, 2);
  Serial.print("Stored PWM frequency (Hz): ");
  Serial.println(requestedPwmFrequencyHz);
  Serial.print("Stored resolution bits: ");
  Serial.println(requestedResolutionBits);
  Serial.print("Drive mode: ");
  Serial.println(driveModeName(driveMode));
  Serial.print("Stored direction: ");
  Serial.println(directionName(requestedDirection));
  Serial.print("Drive enabled: ");
  Serial.println(driveEnabled ? "yes" : "no");
  Serial.print("PWM mode enabled: ");
  Serial.println(driver.isPwmModeEnabled() ? "yes" : "no");
  Serial.print("Bridge runtime state: ");
  Serial.println(runtimeStateName(bridgeState.runtimeState));
  Serial.print("Fault pin active: ");
  Serial.println(driver.isFaultPinActive() ? "yes" : "no");
  Serial.print("Fault present: ");
  Serial.println(driver.hasFault() ? "yes" : "no");
  Serial.print("STATUS raw: 0x");
  Serial.println(status.raw, HEX);
  Serial.print("STATUS flags: OT=");
  Serial.print(status.overTemperature ? "1" : "0");
  Serial.print(" AOCP=");
  Serial.print(status.overCurrentA ? "1" : "0");
  Serial.print(" BOCP=");
  Serial.print(status.overCurrentB ? "1" : "0");
  Serial.print(" APDF=");
  Serial.print(status.predriverFaultA ? "1" : "0");
  Serial.print(" BPDF=");
  Serial.print(status.predriverFaultB ? "1" : "0");
  Serial.print(" UVLO=");
  Serial.println(status.undervoltage ? "1" : "0");
  Serial.print("Preset registers: TOFF=0x");
  Serial.print(effectiveOffTime(presetConfig), HEX);
  Serial.print(" TBLANK=0x");
  Serial.print(effectiveBlankTime(presetConfig), HEX);
  Serial.print(" DECAY=");
  Serial.print(decayModeName(effectiveDecayMode(presetConfig)));
  Serial.print(" TDECAY=0x");
  Serial.println(presetConfig.decayTime, HEX);
  Serial.print("Timing overrides active: TOFF=");
  Serial.print(offTimeOverrideEnabled ? "yes" : "no");
  Serial.print(" TBLANK=");
  Serial.print(blankTimeOverrideEnabled ? "yes" : "no");
  Serial.print(" DECAY=");
  Serial.println(decayModeOverrideEnabled ? "yes" : "no");

  if (driver.isCurrentLimitEnabled()) {
    printCurrentLimit(Serial, driver.currentLimitResult());
  } else {
    Serial.println("Current limit disabled");
  }

  if (driver.isPwmModeEnabled()) {
    printPwmCapability(Serial, driver.pwmCapability());
  } else {
    Serial.println("PWM capability unavailable");
  }

  Serial.println();
}

bool applyDriveSettings() {
  DRV8704& driver = motorDriver();

  if (!driver.setDirection(kBridge, requestedDirection)) {
    Serial.println("Failed to apply direction");
    return false;
  }

  if (driveMode == DriveMode::FullBridge) {
    if (!driver.disableCurrentLimit()) {
      Serial.println("Failed to disable current limit");
      return false;
    }

    driver.setBridgeMode(
        kBridge,
        requestedDirection == Direction::Forward ? BridgeMode::Forward : BridgeMode::Reverse);
    driveEnabled = true;
    Serial.println("Current limit disabled for full-bridge drive");
    return true;
  }

  if (!driver.setCurrentModePreset(currentPreset)) {
    Serial.println("Failed to apply current preset");
    return false;
  }

  if (driveMode == DriveMode::CurrentLimited) {
    if (!driver.setCurrent(kBridge, requestedCurrentAmps)) {
      Serial.println("Failed to apply current request");
      return false;
    }

    if (!applyTimingOverrides()) {
      return false;
    }

    driveEnabled = true;
    printCurrentLimit(Serial, driver.currentLimitResult());
    if (requestedCurrentAmps < driver.currentLimitResult().ampsPerTorqueCount) {
      Serial.println("Warning: requested current is below one DAC step for the configured shunt");
    }
    return true;
  }

  if (!driver.setCurrentLimit(requestedCurrentLimitAmps)) {
    Serial.println("Failed to apply current limit");
    return false;
  }

  if (!driver.isPwmModeEnabled() && !beginOrRebeginPwmMode()) {
    return false;
  }

  if (!driver.setSpeed(kBridge, requestedDutyPercent)) {
    Serial.println("Failed to apply PWM duty");
    return false;
  }

  driveEnabled = requestedDutyPercent > 0.0f;
  printCurrentLimit(Serial, driver.currentLimitResult());
  printPwmCapability(Serial, driver.pwmCapability());
  return true;
}

void handleModeCommand(const char* token) {
  if (token == nullptr) {
    Serial.println("Usage: mode <current|pwm|full>");
    return;
  }

  if (strcmp(token, "current") == 0) {
    driveMode = DriveMode::CurrentLimited;
  } else if (strcmp(token, "pwm") == 0) {
    driveMode = DriveMode::PwmLimited;
  } else if (strcmp(token, "full") == 0 || strcmp(token, "fullbridge") == 0) {
    driveMode = DriveMode::FullBridge;
  } else {
    Serial.println("Usage: mode <current|pwm|full>");
    return;
  }

  Serial.print("Drive mode set to ");
  Serial.println(driveModeName(driveMode));

  if (driveEnabled) {
    applyDriveSettings();
  }
}

void handlePresetCommand(const char* token) {
  CurrentModePreset preset;
  if (!parsePresetToken(token, preset)) {
    Serial.println("Usage: preset <heater|tec|ms|mm|ml>");
    return;
  }

  if (!motorDriver().setCurrentModePreset(preset)) {
    Serial.println("Failed to apply preset");
    return;
  }

  clearTimingOverrides();
  currentPreset = preset;
  Serial.print("Current preset set to ");
  Serial.println(presetName(currentPreset));
  Serial.println("Timing overrides cleared");

  if (driveEnabled && driveMode != DriveMode::FullBridge) {
    applyDriveSettings();
  }
}

void handleShuntCommand(const char* token) {
  float ohms = 0.0f;
  if (!parsePositiveFloat(token, ohms)) {
    Serial.println("Usage: shunt <ohms>, for example: shunt 0.005");
    return;
  }

  if (!motorDriver().setShuntResistance(ohms)) {
    Serial.println("Failed to apply shunt resistance");
    return;
  }

  configuredShuntOhms = ohms;
  Serial.print("Configured shunt updated to ");
  Serial.print(configuredShuntOhms, 6);
  Serial.println(" ohm");

  if (driveEnabled && driveMode != DriveMode::FullBridge) {
    applyDriveSettings();
  }
}

void handleDirectionCommand(const char* token) {
  Direction direction;
  if (!parseDirectionToken(token, direction)) {
    Serial.println("Usage: direction <f|r>");
    return;
  }

  requestedDirection = direction;
  Serial.print("Stored direction updated to ");
  Serial.println(directionName(requestedDirection));

  if (driveEnabled) {
    applyDriveSettings();
  } else if (!motorDriver().setDirection(kBridge, requestedDirection)) {
    Serial.println("Failed to store direction in driver");
  }
}

void handleCurrentCommand(const char* token) {
  float amps = 0.0f;
  if (!parsePositiveFloat(token, amps)) {
    Serial.println("Usage: current <amps>, for example: current 0.25");
    return;
  }

  requestedCurrentAmps = amps;
  Serial.print("Stored current updated to ");
  Serial.print(requestedCurrentAmps, 4);
  Serial.println(" A");

  if (driveEnabled && driveMode == DriveMode::CurrentLimited) {
    applyDriveSettings();
  }
}

void handleLimitCommand(const char* token) {
  float amps = 0.0f;
  if (!parsePositiveFloat(token, amps)) {
    Serial.println("Usage: limit <amps>, for example: limit 4.0");
    return;
  }

  requestedCurrentLimitAmps = amps;
  Serial.print("Stored current limit updated to ");
  Serial.print(requestedCurrentLimitAmps, 4);
  Serial.println(" A");

  if (driveEnabled && driveMode == DriveMode::PwmLimited) {
    applyDriveSettings();
  }
}

void handleDutyCommand(const char* token) {
  float duty = 0.0f;
  if (!parseDutyPercent(token, duty)) {
    Serial.println("Usage: duty <0..100>");
    return;
  }

  requestedDutyPercent = duty;
  Serial.print("Stored duty updated to ");
  Serial.print(requestedDutyPercent, 2);
  Serial.println(" %");

  if (driveEnabled && driveMode == DriveMode::PwmLimited) {
    applyDriveSettings();
  }
}

void handleFrequencyCommand(const char* token) {
  uint32_t frequencyHz = 0UL;
  if (!parsePositiveUint32(token, frequencyHz)) {
    Serial.println("Usage: freq <hz>");
    return;
  }

  requestedPwmFrequencyHz = frequencyHz;
  Serial.print("Stored PWM frequency updated to ");
  Serial.print(requestedPwmFrequencyHz);
  Serial.println(" Hz");

  DRV8704& driver = motorDriver();
  if (driver.isPwmModeEnabled()) {
    if (!driver.setPwmFrequency(requestedPwmFrequencyHz)) {
      Serial.println("Failed to set PWM frequency");
      return;
    }
    printPwmCapability(Serial, driver.pwmCapability());
  }

  if (driveEnabled && driveMode == DriveMode::PwmLimited) {
    applyDriveSettings();
  }
}

void handleResolutionCommand(const char* token) {
  uint8_t bits = 0U;
  if (!parseResolutionBits(token, bits)) {
    Serial.println("Usage: res <bits>");
    return;
  }

  requestedResolutionBits = bits;
  Serial.print("Stored PWM resolution updated to ");
  Serial.print(requestedResolutionBits);
  Serial.println(" bits");

  if (!beginOrRebeginPwmMode()) {
    return;
  }

  printPwmCapability(Serial, motorDriver().pwmCapability());
  if (driveEnabled && driveMode == DriveMode::PwmLimited) {
    applyDriveSettings();
  }
}

void handleOffCommand(const char* token) {
  uint8_t value = 0U;
  if (!parseByteValue(token, value)) {
    Serial.println("Usage: off <0..255>");
    return;
  }

  if (!motorDriver().setOffTime(value)) {
    Serial.println("Failed to set off-time register");
    return;
  }

  offTimeOverrideEnabled = true;
  offTimeOverride = value;
  Serial.print("TOFF set to 0x");
  Serial.println(value, HEX);
}

void handleBlankCommand(const char* token) {
  uint8_t value = 0U;
  if (!parseByteValue(token, value)) {
    Serial.println("Usage: blank <0..255>");
    return;
  }

  if (!motorDriver().setBlankTime(value)) {
    Serial.println("Failed to set blanking register");
    return;
  }

  blankTimeOverrideEnabled = true;
  blankTimeOverride = value;
  Serial.print("TBLANK set to 0x");
  Serial.println(value, HEX);
}

void handleDecayCommand(const char* token) {
  DecayMode mode;
  if (!parseDecayModeToken(token, mode)) {
    Serial.println("Usage: decay <slow|fast|mixed|auto>");
    return;
  }

  if (!motorDriver().setDecayMode(mode)) {
    Serial.println("Failed to set decay mode");
    return;
  }

  decayModeOverrideEnabled = true;
  decayModeOverride = mode;
  Serial.print("Decay mode set to ");
  Serial.println(decayModeName(mode));
}

void handleDriveCommand(char* ampsToken, char* directionToken) {
  float amps = requestedCurrentAmps;
  Direction direction = requestedDirection;

  if (ampsToken != nullptr) {
    if (!parsePositiveFloat(ampsToken, amps)) {
      if (directionToken == nullptr && parseDirectionToken(ampsToken, direction)) {
        ampsToken = nullptr;
      } else {
        Serial.println("Usage: drive [amps] [f|r]");
        return;
      }
    }
  }

  if (directionToken != nullptr && !parseDirectionToken(directionToken, direction)) {
    Serial.println("Usage: drive [amps] [f|r]");
    return;
  }

  requestedCurrentAmps = amps;
  requestedDirection = direction;
  driveMode = DriveMode::CurrentLimited;

  if (applyDriveSettings()) {
    Serial.print("Driving ");
    Serial.print(requestedCurrentAmps, 4);
    Serial.print(" A requested, ");
    Serial.print(motorDriver().currentLimitResult().appliedCurrentLimitAmpsA, 4);
    Serial.print(" A effective, ");
    Serial.println(directionName(requestedDirection));
  }
}

void handlePwmCommand(char* dutyToken, char* directionToken) {
  float duty = requestedDutyPercent;
  Direction direction = requestedDirection;

  if (dutyToken != nullptr) {
    if (!parseDutyPercent(dutyToken, duty)) {
      if (directionToken == nullptr && parseDirectionToken(dutyToken, direction)) {
        dutyToken = nullptr;
      } else {
        Serial.println("Usage: pwm [duty] [f|r]");
        return;
      }
    }
  }

  if (directionToken != nullptr && !parseDirectionToken(directionToken, direction)) {
    Serial.println("Usage: pwm [duty] [f|r]");
    return;
  }

  requestedDutyPercent = duty;
  requestedDirection = direction;
  driveMode = DriveMode::PwmLimited;

  if (applyDriveSettings()) {
    Serial.print("PWM drive ");
    Serial.print(requestedDutyPercent, 2);
    Serial.print(" % ");
    Serial.println(directionName(requestedDirection));
  }
}

void handleFullCommand(char* directionToken) {
  Direction direction = requestedDirection;
  if (directionToken != nullptr && !parseDirectionToken(directionToken, direction)) {
    Serial.println("Usage: full [f|r]");
    return;
  }

  requestedDirection = direction;
  driveMode = DriveMode::FullBridge;

  if (applyDriveSettings()) {
    Serial.print("Driving full-bridge ");
    Serial.println(directionName(requestedDirection));
  }
}

void handleCommand(char* line) {
  for (char* p = line; *p != '\0'; ++p) {
    *p = static_cast<char>(tolower(static_cast<unsigned char>(*p)));
  }

  char* command = strtok(line, " \t");
  if (command == nullptr) {
    return;
  }

  if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
    printHelp();
    return;
  }

  if (strcmp(command, "status") == 0) {
    printStatus();
    return;
  }

  if (strcmp(command, "mode") == 0) {
    handleModeCommand(strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "preset") == 0) {
    handlePresetCommand(strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "sweep") == 0) {
    char* sweepTarget = strtok(nullptr, " \t");
    if (sweepTarget != nullptr && strcmp(sweepTarget, "heater") == 0) {
      runHeaterSweep();
    } else if (sweepTarget != nullptr && strcmp(sweepTarget, "decay") == 0) {
      runDecaySweep();
    } else if (sweepTarget != nullptr && strcmp(sweepTarget, "blank") == 0) {
      runBlankSweep();
    } else if (sweepTarget != nullptr && strcmp(sweepTarget, "off") == 0) {
      runOffSweep();
    } else {
      Serial.println("Usage: sweep <heater|decay|blank|off>");
    }
    return;
  }

  if (strcmp(command, "off") == 0) {
    handleOffCommand(strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "blank") == 0) {
    handleBlankCommand(strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "decay") == 0) {
    handleDecayCommand(strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "current") == 0) {
    handleCurrentCommand(strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "limit") == 0) {
    handleLimitCommand(strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "duty") == 0) {
    handleDutyCommand(strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "shunt") == 0) {
    handleShuntCommand(strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "direction") == 0 || strcmp(command, "dir") == 0) {
    handleDirectionCommand(strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "freq") == 0) {
    handleFrequencyCommand(strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "res") == 0) {
    handleResolutionCommand(strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "drive") == 0 || strcmp(command, "on") == 0) {
    handleDriveCommand(strtok(nullptr, " \t"), strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "pwm") == 0) {
    handlePwmCommand(strtok(nullptr, " \t"), strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "full") == 0) {
    handleFullCommand(strtok(nullptr, " \t"));
    return;
  }

  if (strcmp(command, "coast") == 0 || strcmp(command, "stop") == 0) {
    driveEnabled = false;
    motorDriver().coast(kBridge);
    Serial.println("Bridge A set to coast");
    return;
  }

  if (strcmp(command, "brake") == 0) {
    driveEnabled = false;
    motorDriver().brake(kBridge);
    Serial.println("Bridge A brake applied");
    return;
  }

  if (strcmp(command, "clearfaults") == 0) {
    if (motorDriver().clearFaults()) {
      Serial.println("Faults cleared");
    } else {
      Serial.println("Failed to clear faults");
    }
    return;
  }

  Serial.println("Unknown command. Type 'help' for a command list.");
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      commandBuffer[commandLength] = '\0';
      handleCommand(commandBuffer);
      commandLength = 0;
      continue;
    }

    if (commandLength + 1U < kCommandBufferSize) {
      commandBuffer[commandLength++] = incoming;
    } else {
      commandLength = 0;
      Serial.println("Command too long");
    }
  }
}

}  // namespace

void setup() {

  // disable other SPIs and peripherals
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);

  Serial.begin(115200);
  delay(1000);
  currentLogLevel = LOG_LEVEL_INFO;

  DRV8704& driver = motorDriver();
  if (!driver.begin()) {
    Serial.println("DRV8704 begin failed");
    return;
  }

  if (!driver.setShuntResistance(configuredShuntOhms)) {
    Serial.println("Invalid shunt resistance");
    return;
  }

  if (!driver.setCurrentModePreset(currentPreset)) {
    Serial.println("Failed to apply default current preset");
    return;
  }

  if (!driver.setCurrentLimit(requestedCurrentLimitAmps)) {
    Serial.println("Failed to apply default current limit");
    return;
  }

  if (!beginOrRebeginPwmMode()) {
    return;
  }

  if (!driver.setDirection(kBridge, requestedDirection)) {
    Serial.println("Failed to apply initial direction");
    return;
  }

  driver.coast(kBridge);
  driverReady = true;

  Serial.println();
  Serial.println("DRV8704 engineering test ready");
  Serial.println("Bridge A starts in coast for safety");
  printHelp();
  printStatus();
}

void loop() {
  if (!driverReady) {
    return;
  }

  readSerialCommands();
}
