/*
  ============================================================
  Pendulum Controller v1.0
  ============================================================

  Hardware:
    - Arduino Mega
    - 2 x TB6612FNG dual H-bridge boards
    - 3 x electromagnet coils
    - 2 momentary buttons per coil: NORTH and SOUTH
    - 1 momentary ALL OFF button

  Arduino pin map:
    Coil 1: PWM D2,  IN1 D3,  IN2 D4
    Coil 2: PWM D6,  IN1 D7,  IN2 D8
    Coil 3: PWM D10, IN1 D11, IN2 D12

    Coil 1 buttons: NORTH D22, SOUTH D23
    Coil 2 buttons: NORTH D24, SOUTH D25
    Coil 3 buttons: NORTH D26, SOUTH D27

    ALL OFF: D28
    Shared TB6612 STBY: D52

  Playing behaviour:
    - Coil responds as soon as a direction button is pressed.
    - Short press: momentary "ping"; coil turns off on release.
    - Long press (>= LATCH_TIME_MS): polarity latches on after release.
    - Short press of the currently latched direction: unlatches immediately.
      Keeping that same press held does not latch again; release first, then
      press-and-hold again if you want to relatch.
    - Pressing the opposite direction while latched reverses safely:
        short press -> opposite-direction ping, then off on release
        long press  -> opposite direction remains latched on release
    - ALL OFF clears every output and latch immediately.
    - Direct polarity reversals include non-blocking dead-time.
    - Simultaneous NORTH + SOUTH on one coil forces that coil off until both
      buttons are released.

  Notes:
    - Buttons connect between their Arduino pin and GND.
    - INPUT_PULLUP is used; no external button resistors are required.
    - TB6612 VCC connects to Arduino 5 V.
    - TB6612 VM connects to the 12 V coil supply.
    - Arduino GND, TB6612 GND and 12 V supply negative must be common.
    - External bridge rectifiers are not used with the TB6612 outputs.
*/

#include <Arduino.h>

// ============================================================
// User-adjustable settings
// ============================================================

constexpr uint8_t NUM_COILS = 3;

constexpr unsigned long LATCH_TIME_MS = 400;
constexpr unsigned long DEBOUNCE_MS   = 3;
constexpr unsigned long DEAD_TIME_MS  = 2;

constexpr uint8_t FULL_POWER = 255;

// Set to true while testing with Serial Monitor at 115200 baud.
constexpr bool DEBUG_SERIAL = true;

// ============================================================
// Pin assignments
// ============================================================

constexpr uint8_t STBY_PIN    = 52;
constexpr uint8_t ALL_OFF_PIN = 28;

constexpr uint8_t PWM_PINS[NUM_COILS]   = {2, 6, 10};
constexpr uint8_t IN1_PINS[NUM_COILS]   = {3, 7, 11};
constexpr uint8_t IN2_PINS[NUM_COILS]   = {4, 8, 12};

constexpr uint8_t NORTH_BUTTON_PINS[NUM_COILS] = {22, 24, 26};
constexpr uint8_t SOUTH_BUTTON_PINS[NUM_COILS] = {23, 25, 27};

// ============================================================
// Basic types
// ============================================================

enum class Direction : uint8_t {
  Off,
  North,
  South
};

enum class DriverPhase : uint8_t {
  Ready,
  DeadTime
};

// ============================================================
// Debounced active-low momentary button
// ============================================================

struct Button {
  uint8_t pin = 0;

  bool rawPressed = false;
  bool stablePressed = false;
  bool previousStablePressed = false;

  unsigned long rawChangedAt = 0;

  bool justPressed = false;
  bool justReleased = false;

  void begin(uint8_t assignedPin) {
    pin = assignedPin;
    pinMode(pin, INPUT_PULLUP);

    const bool initialPressed = (digitalRead(pin) == LOW);
    rawPressed = initialPressed;
    stablePressed = initialPressed;
    previousStablePressed = initialPressed;
    rawChangedAt = millis();

    justPressed = false;
    justReleased = false;
  }

  void update(unsigned long now) {
    justPressed = false;
    justReleased = false;

    const bool newRawPressed = (digitalRead(pin) == LOW);

    if (newRawPressed != rawPressed) {
      rawPressed = newRawPressed;
      rawChangedAt = now;
    }

    if ((now - rawChangedAt) >= DEBOUNCE_MS &&
        stablePressed != rawPressed) {
      previousStablePressed = stablePressed;
      stablePressed = rawPressed;

      justPressed = (!previousStablePressed && stablePressed);
      justReleased = (previousStablePressed && !stablePressed);
    }
  }

  bool isPressed() const {
    return stablePressed;
  }
};

// ============================================================
// Coil channel state
// ============================================================

struct CoilChannel {
  uint8_t index = 0;

  uint8_t pwmPin = 0;
  uint8_t in1Pin = 0;
  uint8_t in2Pin = 0;

  Button northButton;
  Button southButton;

  // What the bridge is physically outputting now.
  Direction outputDirection = Direction::Off;

  // Direction requested after safe dead-time.
  Direction pendingDirection = Direction::Off;

  DriverPhase driverPhase = DriverPhase::Ready;
  unsigned long deadTimeStartedAt = 0;

  // Persistent musical state.
  Direction latchedDirection = Direction::Off;

  // Current press gesture that may become a ping or latch.
  bool gestureActive = false;
  Direction gestureDirection = Direction::Off;
  unsigned long gestureStartedAt = 0;
  bool gesturePassedLatchTime = false;

  // Same-direction press used only to cancel an existing latch.
  // It must be released before it can be used to latch again.
  bool suppressNorthUntilRelease = false;
  bool suppressSouthUntilRelease = false;

  // Safety state if both direction buttons are held simultaneously.
  bool buttonConflict = false;
};

CoilChannel coils[NUM_COILS];
Button allOffButton;

// ============================================================
// Debug helpers
// ============================================================

const __FlashStringHelper* directionName(Direction direction) {
  switch (direction) {
    case Direction::North: return F("NORTH");
    case Direction::South: return F("SOUTH");
    default:               return F("OFF");
  }
}

void debugEvent(uint8_t coilIndex,
                const __FlashStringHelper* event,
                Direction direction = Direction::Off) {
  if (!DEBUG_SERIAL) {
    return;
  }

  Serial.print(F("Coil "));
  Serial.print(coilIndex + 1);
  Serial.print(F(": "));
  Serial.print(event);

  if (direction != Direction::Off) {
    Serial.print(F(" "));
    Serial.print(directionName(direction));
  }

  Serial.println();
}

// ============================================================
// Low-level TB6612 output functions
// ============================================================

void writeBridgeOff(CoilChannel& coil) {
  analogWrite(coil.pwmPin, 0);
  digitalWrite(coil.in1Pin, LOW);
  digitalWrite(coil.in2Pin, LOW);
  coil.outputDirection = Direction::Off;
}

void writeBridgeDirection(CoilChannel& coil, Direction direction) {
  if (direction == Direction::North) {
    digitalWrite(coil.in1Pin, HIGH);
    digitalWrite(coil.in2Pin, LOW);
    analogWrite(coil.pwmPin, FULL_POWER);
    coil.outputDirection = Direction::North;
  } else if (direction == Direction::South) {
    digitalWrite(coil.in1Pin, LOW);
    digitalWrite(coil.in2Pin, HIGH);
    analogWrite(coil.pwmPin, FULL_POWER);
    coil.outputDirection = Direction::South;
  } else {
    writeBridgeOff(coil);
  }
}

// Request a direction without ever switching directly from one polarity
// to the other. Reversals first go OFF, then wait DEAD_TIME_MS.
void requestOutput(CoilChannel& coil,
                   Direction requested,
                   unsigned long now) {
  if (requested == Direction::Off) {
    coil.pendingDirection = Direction::Off;
    coil.driverPhase = DriverPhase::Ready;
    writeBridgeOff(coil);
    return;
  }

  if (coil.driverPhase == DriverPhase::DeadTime) {
    // The bridge is already off. Update the intended destination while
    // preserving the dead-time that has already elapsed.
    coil.pendingDirection = requested;
    return;
  }

  if (coil.outputDirection == requested) {
    return;
  }

  if (coil.outputDirection == Direction::Off) {
    // Starting from OFF does not require reversal dead-time.
    coil.pendingDirection = Direction::Off;
    writeBridgeDirection(coil, requested);
    return;
  }

  // A true polarity reversal: switch off now, then wait.
  writeBridgeOff(coil);
  coil.pendingDirection = requested;
  coil.deadTimeStartedAt = now;
  coil.driverPhase = DriverPhase::DeadTime;
}

void updateDriver(CoilChannel& coil, unsigned long now) {
  if (coil.driverPhase != DriverPhase::DeadTime) {
    return;
  }

  if ((now - coil.deadTimeStartedAt) < DEAD_TIME_MS) {
    return;
  }

  const Direction destination = coil.pendingDirection;

  coil.pendingDirection = Direction::Off;
  coil.driverPhase = DriverPhase::Ready;

  if (destination != Direction::Off) {
    writeBridgeDirection(coil, destination);
  }
}

// ============================================================
// Musical state helpers
// ============================================================

void clearGesture(CoilChannel& coil) {
  coil.gestureActive = false;
  coil.gestureDirection = Direction::Off;
  coil.gestureStartedAt = 0;
  coil.gesturePassedLatchTime = false;
}

void forceCoilOff(CoilChannel& coil) {
  coil.latchedDirection = Direction::Off;
  clearGesture(coil);

  coil.pendingDirection = Direction::Off;
  coil.driverPhase = DriverPhase::Ready;

  coil.suppressNorthUntilRelease = false;
  coil.suppressSouthUntilRelease = false;

  writeBridgeOff(coil);
}

void forceAllOff() {
  for (uint8_t i = 0; i < NUM_COILS; ++i) {
    forceCoilOff(coils[i]);
  }

  if (DEBUG_SERIAL) {
    Serial.println(F("ALL OFF"));
  }
}

void beginGesture(CoilChannel& coil,
                  Direction direction,
                  unsigned long now) {
  coil.latchedDirection = Direction::Off;

  coil.gestureActive = true;
  coil.gestureDirection = direction;
  coil.gestureStartedAt = now;
  coil.gesturePassedLatchTime = false;

  requestOutput(coil, direction, now);
  debugEvent(coil.index, F("PRESS"), direction);
}

void cancelExistingLatchWithSameButton(CoilChannel& coil,
                                       Direction direction,
                                       unsigned long now) {
  (void)now;

  coil.latchedDirection = Direction::Off;
  clearGesture(coil);
  requestOutput(coil, Direction::Off, now);

  if (direction == Direction::North) {
    coil.suppressNorthUntilRelease = true;
  } else {
    coil.suppressSouthUntilRelease = true;
  }

  debugEvent(coil.index, F("UNLATCH"), direction);
}

void handleDirectionPress(CoilChannel& coil,
                          Direction direction,
                          unsigned long now) {
  const bool cancellingSameLatch =
      (coil.latchedDirection == direction);

  if (cancellingSameLatch) {
    cancelExistingLatchWithSameButton(coil, direction, now);
    return;
  }

  // Pressing the opposite direction, whether from OFF or from a latch,
  // starts a fresh gesture immediately.
  beginGesture(coil, direction, now);
}

void handleDirectionRelease(CoilChannel& coil,
                            Direction direction,
                            unsigned long now) {
  bool& suppressed =
      (direction == Direction::North)
          ? coil.suppressNorthUntilRelease
          : coil.suppressSouthUntilRelease;

  if (suppressed) {
    suppressed = false;
    return;
  }

  if (!coil.gestureActive || coil.gestureDirection != direction) {
    return;
  }

  const unsigned long heldFor = now - coil.gestureStartedAt;
  const bool shouldLatch =
      coil.gesturePassedLatchTime || heldFor >= LATCH_TIME_MS;

  if (shouldLatch) {
    coil.latchedDirection = direction;
    clearGesture(coil);
    requestOutput(coil, direction, now);
    debugEvent(coil.index, F("LATCH"), direction);
  } else {
    coil.latchedDirection = Direction::Off;
    clearGesture(coil);
    requestOutput(coil, Direction::Off, now);
    debugEvent(coil.index, F("PING END"), direction);
  }
}

void updateGestureLatchQualification(CoilChannel& coil,
                                     unsigned long now) {
  if (!coil.gestureActive || coil.gesturePassedLatchTime) {
    return;
  }

  const bool matchingButtonStillHeld =
      (coil.gestureDirection == Direction::North)
          ? coil.northButton.isPressed()
          : coil.southButton.isPressed();

  if (!matchingButtonStillHeld) {
    return;
  }

  if ((now - coil.gestureStartedAt) >= LATCH_TIME_MS) {
    coil.gesturePassedLatchTime = true;
    debugEvent(coil.index, F("LATCH ARMED"), coil.gestureDirection);
  }
}

// ============================================================
// Per-coil input and state update
// ============================================================

void updateCoil(CoilChannel& coil, unsigned long now) {
  coil.northButton.update(now);
  coil.southButton.update(now);

  const bool bothPressed =
      coil.northButton.isPressed() &&
      coil.southButton.isPressed();

  if (bothPressed) {
    if (!coil.buttonConflict) {
      coil.buttonConflict = true;
      forceCoilOff(coil);
      debugEvent(coil.index, F("BUTTON CONFLICT - OFF"));
    }

    updateDriver(coil, now);
    return;
  }

  if (coil.buttonConflict) {
    // Stay locked out until both direction buttons are released.
    if (!coil.northButton.isPressed() &&
        !coil.southButton.isPressed()) {
      coil.buttonConflict = false;
      debugEvent(coil.index, F("BUTTON CONFLICT CLEARED"));
    }

    updateDriver(coil, now);
    return;
  }

  if (coil.northButton.justPressed) {
    handleDirectionPress(coil, Direction::North, now);
  }

  if (coil.southButton.justPressed) {
    handleDirectionPress(coil, Direction::South, now);
  }

  if (coil.northButton.justReleased) {
    handleDirectionRelease(coil, Direction::North, now);
  }

  if (coil.southButton.justReleased) {
    handleDirectionRelease(coil, Direction::South, now);
  }

  updateGestureLatchQualification(coil, now);
  updateDriver(coil, now);
}

// ============================================================
// Setup
// ============================================================

void setup() {
  // Keep both TB6612 boards disabled while pins are configured.
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);

  if (DEBUG_SERIAL) {
    Serial.begin(115200);
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("Pendulum Controller v1.0"));
    Serial.println(F("Three-coil performance firmware"));
    Serial.println(F("========================================"));
  }

  allOffButton.begin(ALL_OFF_PIN);

  for (uint8_t i = 0; i < NUM_COILS; ++i) {
    CoilChannel& coil = coils[i];

    coil.index = i;
    coil.pwmPin = PWM_PINS[i];
    coil.in1Pin = IN1_PINS[i];
    coil.in2Pin = IN2_PINS[i];

    pinMode(coil.pwmPin, OUTPUT);
    pinMode(coil.in1Pin, OUTPUT);
    pinMode(coil.in2Pin, OUTPUT);

    coil.northButton.begin(NORTH_BUTTON_PINS[i]);
    coil.southButton.begin(SOUTH_BUTTON_PINS[i]);

    forceCoilOff(coil);
  }

  // Outputs are now known-safe; enable the driver boards.
  digitalWrite(STBY_PIN, HIGH);

  if (DEBUG_SERIAL) {
    Serial.println(F("Ready - all coils OFF"));
  }
}

// ============================================================
// Main loop
// ============================================================

void loop() {
  const unsigned long now = millis();

  allOffButton.update(now);

  // Holding ALL OFF prevents any coil input from re-energising outputs.
  if (allOffButton.isPressed()) {
    if (allOffButton.justPressed) {
      forceAllOff();
    }

    // Continue updating direction buttons so their debounce state remains
    // current, but keep every output firmly off.
    for (uint8_t i = 0; i < NUM_COILS; ++i) {
      coils[i].northButton.update(now);
      coils[i].southButton.update(now);
      writeBridgeOff(coils[i]);
    }

    return;
  }

  for (uint8_t i = 0; i < NUM_COILS; ++i) {
    updateCoil(coils[i], now);
  }
}
