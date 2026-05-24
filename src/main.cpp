#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

constexpr uint8_t TICK_SOLENOID_PIN = 5;
constexpr uint8_t TOCK_SOLENOID_PIN = 6;
constexpr uint8_t HALL_LEFT_PIN = 2;   // Reserved for a future pendulum sensor.
constexpr uint8_t HALL_RIGHT_PIN = 3;  // Reserved for a future pendulum sensor.
constexpr uint8_t STATUS_LED_PIN = 13;

constexpr bool AUTO_START = false;

constexpr uint16_t DEFAULT_TICK_PULSE_MS = 20;
constexpr uint16_t DEFAULT_TOCK_PULSE_MS = 20;
constexpr uint32_t DEFAULT_INTERVAL_MS = 1000;

constexpr uint16_t HARD_MAX_PULSE_MS = 80;
constexpr uint16_t DEFAULT_MAX_PULSE_MS = HARD_MAX_PULSE_MS;
constexpr uint32_t MIN_INTERVAL_MS = 200;

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr size_t COMMAND_BUFFER_SIZE = 80;

class SolenoidPulse {
public:
  SolenoidPulse(uint8_t pin, uint16_t pulseMs)
      : pin_(pin), pulseMs_(pulseMs), active_(false), startedAtMs_(0) {}

  void begin() {
    pinMode(pin_, OUTPUT);
    forceOff();
  }

  bool startPulse(uint32_t nowMs) {
    if (pulseMs_ == 0 || active_) {
      return false;
    }

    active_ = true;
    startedAtMs_ = nowMs;
    digitalWrite(pin_, HIGH);
    return true;
  }

  void update(uint32_t nowMs) {
    if (active_ && elapsed(nowMs, startedAtMs_) >= pulseMs_) {
      forceOff();
    }
  }

  void forceOff() {
    digitalWrite(pin_, LOW);
    active_ = false;
  }

  bool isActive() const {
    return active_;
  }

  uint16_t pulseMs() const {
    return pulseMs_;
  }

  void setPulseMs(uint16_t pulseMs) {
    pulseMs_ = pulseMs;
  }

private:
  static uint32_t elapsed(uint32_t nowMs, uint32_t sinceMs) {
    return nowMs - sinceMs;
  }

  uint8_t pin_;
  uint16_t pulseMs_;
  bool active_;
  uint32_t startedAtMs_;
};

class ClockSoundController {
public:
  ClockSoundController()
      : tick_(TICK_SOLENOID_PIN, DEFAULT_TICK_PULSE_MS),
        tock_(TOCK_SOLENOID_PIN, DEFAULT_TOCK_PULSE_MS),
        autoMode_(AUTO_START), nextIsTick_(true), lastTriggerMs_(0),
        intervalMs_(DEFAULT_INTERVAL_MS), maxPulseMs_(DEFAULT_MAX_PULSE_MS) {}

  void begin() {
    // Put all driven outputs into a known LOW state as early as possible.
    tick_.begin();
    tock_.begin();
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    pinMode(HALL_LEFT_PIN, INPUT_PULLUP);
    pinMode(HALL_RIGHT_PIN, INPUT_PULLUP);

    if (autoMode_) {
      lastTriggerMs_ = millis() - intervalMs_;
    }
  }

  void update(uint32_t nowMs) {
    tick_.update(nowMs);
    tock_.update(nowMs);
    digitalWrite(STATUS_LED_PIN, anyActive() ? HIGH : LOW);

    if (!autoMode_) {
      return;
    }

    if (elapsed(nowMs, lastTriggerMs_) >= intervalMs_) {
      bool triggered = nextIsTick_ ? triggerTick(nowMs, false) : triggerTock(nowMs, false);
      if (triggered) {
        nextIsTick_ = !nextIsTick_;
        lastTriggerMs_ = nowMs;
      }
    }
  }

  bool triggerTick(uint32_t nowMs, bool manual) {
    return triggerOne(tick_, "tick", nowMs, manual);
  }

  bool triggerTock(uint32_t nowMs, bool manual) {
    return triggerOne(tock_, "tock", nowMs, manual);
  }

  void run(uint32_t nowMs) {
    autoMode_ = true;
    nextIsTick_ = true;
    lastTriggerMs_ = nowMs - intervalMs_;
    Serial.println(F("auto mode: run"));
  }

  void stop() {
    autoMode_ = false;
    allOff();
    Serial.println(F("auto mode: stop, outputs LOW"));
  }

  void allOff() {
    tick_.forceOff();
    tock_.forceOff();
    digitalWrite(STATUS_LED_PIN, LOW);
  }

  void holdTickGate(bool enabled) {
    autoMode_ = false;
    tick_.forceOff();
    tock_.forceOff();
    digitalWrite(TICK_SOLENOID_PIN, enabled ? HIGH : LOW);
    digitalWrite(STATUS_LED_PIN, enabled ? HIGH : LOW);
    Serial.println(enabled ? F("diagnostic: D5 HIGH") : F("diagnostic: D5 LOW"));
  }

  bool setTickPulseMs(uint32_t value) {
    tick_.setPulseMs(clampPulse(value));
    return value <= maxPulseMs_;
  }

  bool setTockPulseMs(uint32_t value) {
    tock_.setPulseMs(clampPulse(value));
    return value <= maxPulseMs_;
  }

  bool setMaxPulseMs(uint32_t value) {
    bool clamped = false;
    if (value > HARD_MAX_PULSE_MS) {
      value = HARD_MAX_PULSE_MS;
      clamped = true;
    }

    maxPulseMs_ = static_cast<uint16_t>(value);
    tick_.setPulseMs(clampPulse(tick_.pulseMs()));
    tock_.setPulseMs(clampPulse(tock_.pulseMs()));
    return !clamped;
  }

  bool setIntervalMs(uint32_t value) {
    bool clamped = false;
    if (value < MIN_INTERVAL_MS) {
      value = MIN_INTERVAL_MS;
      clamped = true;
    }

    intervalMs_ = value;
    return !clamped;
  }

  void printStatus() const {
    Serial.println(F("status:"));
    Serial.print(F("  auto_mode: "));
    Serial.println(autoMode_ ? F("run") : F("stop"));
    Serial.print(F("  tick_ms: "));
    Serial.println(tick_.pulseMs());
    Serial.print(F("  tock_ms: "));
    Serial.println(tock_.pulseMs());
    Serial.print(F("  interval_ms: "));
    Serial.println(intervalMs_);
    Serial.print(F("  max_pulse_ms: "));
    Serial.println(maxPulseMs_);
    Serial.print(F("  hard_max_pulse_ms: "));
    Serial.println(HARD_MAX_PULSE_MS);
    Serial.print(F("  min_interval_ms: "));
    Serial.println(MIN_INTERVAL_MS);
    Serial.print(F("  outputs_active: "));
    Serial.println(anyActive() ? F("yes") : F("no"));
  }

private:
  static uint32_t elapsed(uint32_t nowMs, uint32_t sinceMs) {
    return nowMs - sinceMs;
  }

  bool anyActive() const {
    return tick_.isActive() || tock_.isActive();
  }

  bool triggerOne(SolenoidPulse &solenoid, const char *name, uint32_t nowMs, bool manual) {
    if (anyActive()) {
      if (manual) {
        Serial.println(F("busy: another solenoid is active"));
      }
      return false;
    }

    bool ok = solenoid.startPulse(nowMs);
    if (!ok) {
      if (manual) {
        Serial.print(name);
        Serial.println(F(": skipped (pulse_ms is 0 or already active)"));
      }
      return false;
    }

    if (manual) {
      Serial.print(name);
      Serial.println(F(": pulse"));
    }
    return true;
  }

  uint16_t clampPulse(uint32_t value) const {
    if (value > maxPulseMs_) {
      return maxPulseMs_;
    }
    return static_cast<uint16_t>(value);
  }

  SolenoidPulse tick_;
  SolenoidPulse tock_;
  bool autoMode_;
  bool nextIsTick_;
  uint32_t lastTriggerMs_;
  uint32_t intervalMs_;
  uint16_t maxPulseMs_;
};

ClockSoundController controller;

char commandBuffer[COMMAND_BUFFER_SIZE];
size_t commandLength = 0;

void printHelp() {
  Serial.println(F("commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  run"));
  Serial.println(F("  stop"));
  Serial.println(F("  status"));
  Serial.println(F("  tick"));
  Serial.println(F("  tock"));
  Serial.println(F("  gate on"));
  Serial.println(F("  gate off"));
  Serial.println(F("  set tick_ms 20"));
  Serial.println(F("  set tock_ms 20"));
  Serial.println(F("  set interval_ms 1000"));
  Serial.println(F("  set max_pulse_ms 50"));
  Serial.println(F("notes: pulse_ms 0 disables that sound; recommended pulse range is 3-30ms."));
}

char *nextToken(char **cursor) {
  char *token = strtok_r(*cursor, " \t", cursor);
  return token;
}

void toLowerAscii(char *text) {
  while (*text) {
    *text = static_cast<char>(tolower(*text));
    ++text;
  }
}

bool parseUnsigned(const char *text, uint32_t &value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char *end = nullptr;
  unsigned long parsed = strtoul(text, &end, 10);
  if (*end != '\0') {
    return false;
  }

  value = static_cast<uint32_t>(parsed);
  return true;
}

void handleSetCommand(char *cursor) {
  char *key = nextToken(&cursor);
  char *valueText = nextToken(&cursor);

  if (key == nullptr || valueText == nullptr) {
    Serial.println(F("error: usage is set <tick_ms|tock_ms|interval_ms|max_pulse_ms> <value>"));
    return;
  }

  toLowerAscii(key);

  uint32_t value = 0;
  if (!parseUnsigned(valueText, value)) {
    Serial.println(F("error: value must be a non-negative integer"));
    return;
  }

  bool unclamped = true;
  if (strcmp(key, "tick_ms") == 0) {
    unclamped = controller.setTickPulseMs(value);
  } else if (strcmp(key, "tock_ms") == 0) {
    unclamped = controller.setTockPulseMs(value);
  } else if (strcmp(key, "interval_ms") == 0) {
    unclamped = controller.setIntervalMs(value);
  } else if (strcmp(key, "max_pulse_ms") == 0) {
    unclamped = controller.setMaxPulseMs(value);
  } else {
    Serial.println(F("error: unknown setting"));
    return;
  }

  Serial.print(F("ok: "));
  Serial.print(key);
  Serial.println(unclamped ? F(" updated") : F(" updated (clamped to safe limit)"));
}

void handleGateCommand(char *cursor) {
  char *state = nextToken(&cursor);
  if (state == nullptr) {
    Serial.println(F("error: usage is gate <on|off>"));
    return;
  }

  toLowerAscii(state);

  if (strcmp(state, "on") == 0) {
    controller.holdTickGate(true);
  } else if (strcmp(state, "off") == 0) {
    controller.holdTickGate(false);
  } else {
    Serial.println(F("error: usage is gate <on|off>"));
  }
}

void handleCommand(char *line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }

  if (*line == '\0') {
    return;
  }

  char *cursor = line;
  char *command = nextToken(&cursor);
  if (command == nullptr) {
    return;
  }

  toLowerAscii(command);

  if (strcmp(command, "help") == 0) {
    printHelp();
  } else if (strcmp(command, "run") == 0) {
    controller.run(millis());
  } else if (strcmp(command, "stop") == 0) {
    controller.stop();
  } else if (strcmp(command, "status") == 0) {
    controller.printStatus();
  } else if (strcmp(command, "tick") == 0) {
    controller.triggerTick(millis(), true);
  } else if (strcmp(command, "tock") == 0) {
    controller.triggerTock(millis(), true);
  } else if (strcmp(command, "gate") == 0) {
    handleGateCommand(cursor);
  } else if (strcmp(command, "set") == 0) {
    handleSetCommand(cursor);
  } else {
    Serial.println(F("error: unknown command, type help"));
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      commandBuffer[commandLength] = '\0';
      handleCommand(commandBuffer);
      commandLength = 0;
      continue;
    }

    if (commandLength < COMMAND_BUFFER_SIZE - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;
      controller.allOff();
      Serial.println(F("error: command too long, outputs LOW"));
    }
  }
}

void setup() {
  controller.begin();

  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 1500) {
    // Nano Every native USB may need a short wait; classic boards continue after timeout.
  }

  Serial.println();
  Serial.println(F("Antique clock tick/tock solenoid controller"));
  printHelp();
  controller.printStatus();
}

void loop() {
  uint32_t nowMs = millis();
  controller.update(nowMs);
  readSerialCommands();
}
