// =============================================================================
// RowSense Rover ESP32 firmware
// AUTO  : USB-serial cmd_vel from the Pi (autodemopi5.py)
// MANUAL: BLE Nordic UART from the phone app (RoverBleLink)
//
// The Pi owns mode. It sends "AUTO" or "MANUAL" on USB serial. The ESP gates
// motor writes accordingly:
//   AUTO    -> only Pi CMD_VEL drives the servos. BLE writes are echoed but
//              not applied (returns "[BLE] IGNORED (rover in AUTO)").
//   MANUAL  -> only BLE F/B/L/R/S/C drives the servos. CMD_VEL is still
//              echoed back to the Pi for debugging but not applied.
// BLE advertising is always running. The phone can see and connect to the
// rover in either mode; commands are simply gated.
//
// Both subsystems keep their own auto-stop watchdog at 300 ms.
// =============================================================================

#include <Arduino.h>
#include <ESP32Servo.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ----------------------------
// Pins
// ----------------------------
static const int ESC_PIN   = 4;
static const int STEER_PIN = 5;

Servo esc;
Servo steer;

// ----------------------------
// Pulse tuning (microseconds)
// AUTO uses a tighter band tuned for cmd_vel; MANUAL uses the wider
// band the old BLE sketch used. Keeping them separate so neither feel
// changes from what was working before.
// ----------------------------
static const int ESC_NEUTRAL_US      = 1500;

// AUTO mode (cmd_vel)
static const int ESC_AUTO_FWD_MAX_US = 1800;
static const int ESC_AUTO_REV_MAX_US = 1200;
static const int STEER_AUTO_MAX_US   = 2000;
static const int STEER_AUTO_MIN_US   = 1000;
static const int STEER_TRIM_US       = 0;

// MANUAL/BLE mode (F/B/L/R commands)
static const int ESC_BLE_FWD_MIN_US     = 1500;
static const int ESC_BLE_FWD_MAX_US     = 2000;
static const int ESC_BLE_REV_MIN_US     = 1500;
static const int ESC_BLE_REV_MAX_US     = 1000;
static const int STEER_CENTER_US        = 1500;
static const int STEER_BLE_LEFT_MIN_US  = 1500;
static const int STEER_BLE_LEFT_MAX_US  = 2000;
static const int STEER_BLE_RIGHT_MIN_US = 1500;
static const int STEER_BLE_RIGHT_MAX_US = 1000;

// ----------------------------
// Safety
// ----------------------------
static const uint32_t AUTO_FAILSAFE_MS        = 300;
static const uint32_t MANUAL_DRIVE_TIMEOUT_MS = 300;
static const float    THROTTLE_DEADBAND       = 0.03f;

// AUTO model
static const float MAX_LIN_X = 1.0f;
static const float MAX_ANG_Z = 1.0f;

// ----------------------------
// Serial
// ----------------------------
static const uint32_t BAUD = 115200;

// ----------------------------
// BLE NUS
// ----------------------------
#define SERVICE_UUID_UART        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone -> ESP
#define CHARACTERISTIC_UUID_TX   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP -> phone
#define BLE_NAME                 "RowSense_Rover_S3"

BLEServer*         pServer           = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
volatile bool      deviceConnected   = false;

// ----------------------------
// Internal state
// ----------------------------
static bool auto_mode = false;   // boot in MANUAL, safer (Pi will flip to AUTO when ready)

// AUTO state
static uint32_t last_cmd_ms  = 0;
static uint32_t last_stat_ms = 0;
static float    last_lin_x   = 0.0f;
static float    last_ang_z   = 0.0f;
static int      last_esc_us   = ESC_NEUTRAL_US;
static int      last_steer_us = STEER_CENTER_US;

// MANUAL state
static bool     manual_driving           = false;
static uint32_t manual_last_drive_cmd_ms = 0;

// ----------------------------
// Helpers
// ----------------------------
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float applyDeadband(float x, float db) {
  if (fabsf(x) < db) return 0.0f;
  return x;
}

static int constrainI(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Write both servos and remember last commanded pulse widths.
static void writeServos(int esc_us, int steer_us) {
  last_esc_us   = esc_us;
  last_steer_us = steer_us;
  esc.writeMicroseconds(esc_us);
  steer.writeMicroseconds(steer_us);
}

static void applySafeOutputs() {
  writeServos(ESC_NEUTRAL_US, STEER_CENTER_US);
}

// Send a status string to USB serial AND BLE notify (when connected).
// The phone parses these via RoverBleLink.statusFeed.
static void announce(const char* msg) {
  Serial.println(msg);
  if (deviceConnected && pTxCharacteristic) {
    pTxCharacteristic->setValue((uint8_t*)msg, strlen(msg));
    pTxCharacteristic->notify();
  }
}

static void announceMode() {
  announce(auto_mode ? "[ROVER] MODE=AUTO" : "[ROVER] MODE=MANUAL");
}

// =============================================================================
// AUTO MODE: cmd_vel -> servo us
// =============================================================================

static int autoEscUsFromThrottle(float t) {
  t = clampf(t, -1.0f, 1.0f);
  if (t >= 0.0f) {
    float span = float(ESC_AUTO_FWD_MAX_US - ESC_NEUTRAL_US);
    return int(float(ESC_NEUTRAL_US) + (t * span) + 0.5f);
  } else {
    float span = float(ESC_NEUTRAL_US - ESC_AUTO_REV_MAX_US);
    return int(float(ESC_NEUTRAL_US) + (t * span) - 0.5f);
  }
}

static int autoSteerUsFromNorm(float s) {
  s = clampf(s, -1.0f, 1.0f);
  int us;
  if (s >= 0.0f) {
    float span = float(STEER_AUTO_MAX_US - STEER_CENTER_US);
    us = int(float(STEER_CENTER_US) + (s * span) + 0.5f);
  } else {
    float span = float(STEER_CENTER_US - STEER_AUTO_MIN_US);
    us = int(float(STEER_CENTER_US) + (s * span) - 0.5f);
  }
  us += STEER_TRIM_US;
  if (us < STEER_AUTO_MIN_US) us = STEER_AUTO_MIN_US;
  if (us > STEER_AUTO_MAX_US) us = STEER_AUTO_MAX_US;
  return us;
}

static void autoApply(float lin_x, float ang_z) {
  float t = lin_x / MAX_LIN_X;
  t = clampf(t, -1.0f, 1.0f);
  t = applyDeadband(t, THROTTLE_DEADBAND);

  float s = ang_z / MAX_ANG_Z;
  s = clampf(s, -1.0f, 1.0f);

  writeServos(autoEscUsFromThrottle(t), autoSteerUsFromNorm(s));
}

static void sendEchoCmdVel(float lin_x, float ang_z) {
  Serial.print("ECHO CMD_VEL ");
  Serial.print(lin_x, 4);
  Serial.print(" ");
  Serial.print(ang_z, 4);
  Serial.print(" ");
  Serial.print(last_esc_us);
  Serial.print(" ");
  Serial.println(last_steer_us);
}

static void sendStat() {
  Serial.print("STAT ");
  Serial.print(millis());
  Serial.print(" MODE=");
  Serial.print(auto_mode ? "AUTO" : "MANUAL");
  Serial.print(" LIN=");
  Serial.print(last_lin_x, 4);
  Serial.print(" ANG=");
  Serial.print(last_ang_z, 4);
  Serial.print(" ESC_US=");
  Serial.print(last_esc_us);
  Serial.print(" STEER_US=");
  Serial.println(last_steer_us);
}

// =============================================================================
// MANUAL MODE: BLE F/B/L/R/S/C
// =============================================================================

static void manualDriveStop() {
  // Throttle to neutral, keep current steering angle.
  writeServos(ESC_NEUTRAL_US, last_steer_us);
  manual_driving = false;
  announce("[ROVER] STOP");
}

static void manualDriveForward(int speedPct) {
  speedPct = constrainI(speedPct, 0, 100);
  int us = map(speedPct, 0, 100, ESC_BLE_FWD_MIN_US, ESC_BLE_FWD_MAX_US);
  writeServos(us, last_steer_us);
  manual_driving = true;
  manual_last_drive_cmd_ms = millis();
  char buf[40];
  snprintf(buf, sizeof(buf), "[ROVER] FORWARD %d%%", speedPct);
  announce(buf);
}

static void manualDriveBackward(int speedPct) {
  speedPct = constrainI(speedPct, 0, 100);
  int us = map(speedPct, 0, 100, ESC_BLE_REV_MIN_US, ESC_BLE_REV_MAX_US);
  writeServos(us, last_steer_us);
  manual_driving = true;
  manual_last_drive_cmd_ms = millis();
  char buf[40];
  snprintf(buf, sizeof(buf), "[ROVER] BACKWARD %d%%", speedPct);
  announce(buf);
}

static void manualSteerCenter() {
  writeServos(last_esc_us, STEER_CENTER_US);
  announce("[STEER] CENTER");
}

static void manualSteerLeft(int anglePct) {
  anglePct = constrainI(anglePct, 0, 100);
  int us = map(anglePct, 0, 100, STEER_BLE_LEFT_MIN_US, STEER_BLE_LEFT_MAX_US);
  writeServos(last_esc_us, us);
  char buf[40];
  snprintf(buf, sizeof(buf), "[STEER] LEFT %d%%", anglePct);
  announce(buf);
}

static void manualSteerRight(int anglePct) {
  anglePct = constrainI(anglePct, 0, 100);
  int us = map(anglePct, 0, 100, STEER_BLE_RIGHT_MIN_US, STEER_BLE_RIGHT_MAX_US);
  writeServos(last_esc_us, us);
  char buf[40];
  snprintf(buf, sizeof(buf), "[STEER] RIGHT %d%%", anglePct);
  announce(buf);
}

// One single-char-prefixed command (F50, B75, L30, R80, S, C).
static void manualHandleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  char c = cmd[0];
  int value = 50;
  if (cmd.length() > 1) {
    value = cmd.substring(1).toInt();
    value = constrainI(value, 0, 100);
  }

  switch (c) {
    case 'F': manualDriveForward(value);  break;
    case 'B': manualDriveBackward(value); break;
    case 'S': manualDriveStop();          break;
    case 'L': manualSteerLeft(value);     break;
    case 'R': manualSteerRight(value);    break;
    case 'C': manualSteerCenter();        break;
    default: {
      char buf[40];
      snprintf(buf, sizeof(buf), "Unknown cmd: %c", c);
      announce(buf);
      break;
    }
  }
}

// Map a BLE write payload (Bluefruit Control Pad legacy or new app) to handler.
static void manualHandlePacket(String rxValue) {
  rxValue.trim();

  Serial.print("[BLE] Packet: '");
  Serial.print(rxValue);
  Serial.println("'");

  if (rxValue.startsWith("!B")) {
    // Legacy Bluefruit Control Pad mappings.
    char mapped = 0;
    if      (rxValue == "!B507") mapped = 'F';   // UP
    else if (rxValue == "!B606") mapped = 'B';   // DOWN
    else if (rxValue == "!B804") mapped = 'R';   // RIGHT
    else if (rxValue == "!B714") mapped = 'L';   // LEFT
    else if (rxValue == "!B10;") mapped = 'S';   // Button 1
    else if (rxValue == "!B20:") mapped = 'C';   // Button 2

    if (mapped != 0) {
      manualHandleCommand(String(mapped));
    } else {
      Serial.println("[BLE] Unmapped Control Pad packet");
    }
    return;
  }

  // New app commands.
  manualHandleCommand(rxValue);
}

// =============================================================================
// BLE callbacks
// =============================================================================

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("[BLE] Device connected");
    // Tell the phone what mode we're in right now so the badge in
    // BleControlScreen is correct from the first frame.
    announceMode();
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("[BLE] Device disconnected");
    pServer->getAdvertising()->start();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String rxValue = pCharacteristic->getValue();
    if (rxValue.length() == 0) return;

    Serial.print("[BLE] RAW RX bytes: ");
    for (size_t i = 0; i < rxValue.length(); i++) {
      Serial.print((int)rxValue[i]);
      Serial.print(' ');
    }
    Serial.println();

    if (!auto_mode) {
      manualHandlePacket(rxValue);
    } else {
      // Pi owns the rover in AUTO. Drop the command, tell the phone why.
      announce("[BLE] IGNORED (rover in AUTO)");
    }
  }
};

// =============================================================================
// Mode transitions
// =============================================================================

static void enterAutoMode() {
  auto_mode = true;
  manual_driving = false;
  applySafeOutputs();
  last_cmd_ms = millis();
  last_lin_x  = 0.0f;
  last_ang_z  = 0.0f;
  Serial.println("ACK AUTO");
  announceMode();
}

static void enterManualMode() {
  auto_mode = false;
  last_lin_x = 0.0f;
  last_ang_z = 0.0f;
  manual_driving = false;
  applySafeOutputs();
  Serial.println("ACK MANUAL");
  announceMode();
}

// =============================================================================
// Pi serial line handler (USB)
// =============================================================================

static void handleSerialLine(const char* line) {
  if (line[0] == '\0') return;

  if (strcmp(line, "AUTO") == 0)   { enterAutoMode();   return; }
  if (strcmp(line, "MANUAL") == 0) { enterManualMode(); return; }

  if (strncmp(line, "CMD_VEL ", 8) == 0) {
    float lin_x = 0.0f, ang_z = 0.0f;
    int matched = sscanf(line + 8, "%f %f", &lin_x, &ang_z);
    if (matched == 2) {
      last_lin_x  = lin_x;
      last_ang_z  = ang_z;
      last_cmd_ms = millis();
      // Apply to servos only in AUTO. Always echo, so the Pi can debug.
      if (auto_mode) {
        autoApply(lin_x, ang_z);
      }
      sendEchoCmdVel(lin_x, ang_z);
    } else {
      Serial.print("ERR BAD_CMD_VEL_FORMAT ");
      Serial.println(line);
    }
    return;
  }

  if (strncmp(line, "PING ", 5) == 0) {
    int seq = 0;
    int matched = sscanf(line + 5, "%d", &seq);
    if (matched == 1) {
      Serial.print("PONG ");
      Serial.print(seq);
      Serial.print(" ");
      Serial.println(millis());
    } else {
      Serial.print("ERR BAD_PING_FORMAT ");
      Serial.println(line);
    }
    return;
  }

  Serial.print("ERR UNKNOWN ");
  Serial.println(line);
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
  Serial.begin(BAUD);
  delay(200);
  Serial.println();
  Serial.println("=== RowSense Rover ESP32: AUTO (USB serial) + MANUAL (BLE) ===");

  esc.setPeriodHertz(50);
  steer.setPeriodHertz(50);
  esc.attach(ESC_PIN,   1000, 2000);
  steer.attach(STEER_PIN, 1000, 2000);

  // Long ESC arming sequence carried over from the old BLE sketch.
  // Some ESCs need a power-up wait + a neutral hold before they arm.
  Serial.println("Waiting 3s for ESC power...");
  delay(3000);
  Serial.println("Arming ESC with NEUTRAL (1500us) for 3s...");
  applySafeOutputs();
  delay(3000);

  // BLE bring-up. Always running, regardless of mode.
  BLEDevice::init(BLE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID_UART);

  pTxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxCharacteristic->setCallbacks(new RxCallbacks());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID_UART);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.print("[BLE] Advertising as '");
  Serial.print(BLE_NAME);
  Serial.println("'.");

  last_cmd_ms  = millis();
  last_stat_ms = millis();

  // Boot in MANUAL: rover stays at neutral until the phone presses something
  // or the Pi explicitly sends AUTO. Avoids "Pi sends nothing yet" -> drift.
  auto_mode = false;
  Serial.println("READY 1 USB_SERIAL+BLE");
  Serial.println("HINT: Pi sends AUTO/MANUAL/CMD_VEL/PING; phone sends F/B/L/R/S/C over BLE.");
}

// =============================================================================
// Main loop
// =============================================================================

void loop() {
  // Non-blocking line reader for the Pi serial link.
  static char buf[128];
  static size_t idx = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[idx] = '\0';
      handleSerialLine(buf);
      idx = 0;
    } else {
      if (idx < sizeof(buf) - 1) {
        buf[idx++] = c;
      } else {
        idx = 0;  // overflow reset
      }
    }
  }

  uint32_t now = millis();

  if (auto_mode) {
    // AUTO failsafe: if Pi stops streaming CMD_VEL, fall back to neutral.
    if ((now - last_cmd_ms) > AUTO_FAILSAFE_MS) {
      if (last_esc_us != ESC_NEUTRAL_US || last_steer_us != STEER_CENTER_US) {
        applySafeOutputs();
      }
    }
  } else {
    // MANUAL auto-stop: if no F/B for the timeout window, cut throttle but
    // keep the current steering angle (matches the old BLE sketch behavior).
    if (manual_driving && (now - manual_last_drive_cmd_ms) > MANUAL_DRIVE_TIMEOUT_MS) {
      Serial.println("[ROVER] Auto-stop due to timeout");
      manualDriveStop();
    }
  }

  // 5 Hz STAT to the Pi.
  if ((now - last_stat_ms) >= 200) {
    last_stat_ms = now;
    sendStat();
  }
}
