#include <VescUart.h>
#include <TFT_eSPI.h>
#include <SimpleKalmanFilter.h>
#include "Free_Fonts.h"
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <cmath>
#include <climits>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Ошибка: В Arduino IDE выбрать сборку 'ESP32', далее Tools -> Partition Scheme -> Minimal SPIFFS (1.9MB APP with OTA/128 KB SPIFFS)."
#endif

namespace pins {
  constexpr int TOUCH_MOSI =  32;
  constexpr int TOUCH_MISO =  39;
  constexpr int TOUCH_CLK  =  25;
  constexpr int TOUCH_CS   =  33;
  constexpr int VESC_RX    =  22;
  constexpr int VESC_TX    =  27;
}

namespace timing {
  constexpr unsigned long VESC_TIMEOUT         = 3000;
  constexpr unsigned long VESC_LOOP_DELAY      =   40;
  constexpr unsigned long SECONDARY_BACKOFF    =  500;
  constexpr unsigned long SECONDARY_CACHE_TTL  = 1500;
  constexpr unsigned long TOUCH_DEBOUNCE       =  250;
  constexpr unsigned long TOUCH_HOLD_REPEAT    =  150;
  constexpr unsigned long STATS_REFRESH        =  500;
  constexpr unsigned long DIAG_REFRESH         =  600;
  constexpr unsigned long RECORDS_SAVE_COOLDOWN = 10000;
  constexpr unsigned long SETTINGS_SAVE_DELAY  = 3000;
  constexpr unsigned long NO_DATA_LOG_PERIOD   = 2000;
  constexpr unsigned long OTA_CONNECT_TIMEOUT  = 15000;
}

// =============================================================================
//  Peripherals / globals
// =============================================================================
TFT_eSPI          tft;
VescUart          vesc;
Preferences       prefs;
HardwareSerial    VESCSerial(1);
SPIClass          touchSPI(VSPI);
XPT2046_Touchscreen ts(pins::TOUCH_CS);

struct CockpitTile;
struct RecItem;

// =============================================================================
//  Drivetrain / battery configuration
// =============================================================================
constexpr int   MOTOR_POLES_DEFAULT = 30;
constexpr int   WHEEL_DIA_MM_DEFAULT = 280;
constexpr int   DRIVE_RATIO_X100_DEFAULT = 100;

constexpr int   MOTOR_POLES_MIN      = 2;
constexpr int   MOTOR_POLES_MAX      = 60;
constexpr int   WHEEL_DIA_MM_MIN     = 80;
constexpr int   WHEEL_DIA_MM_MAX     = 800;
constexpr int   DRIVE_RATIO_X100_MIN = 10;
constexpr int   DRIVE_RATIO_X100_MAX = 1000;

volatile int motorPoles      = MOTOR_POLES_DEFAULT;
volatile int wheelDiaMm      = WHEEL_DIA_MM_DEFAULT;
volatile int driveRatioX100  = DRIVE_RATIO_X100_DEFAULT;

static int normalizeMotorPoles(int v) {
  if (v < MOTOR_POLES_MIN) return MOTOR_POLES_DEFAULT;
  if (v > MOTOR_POLES_MAX) return MOTOR_POLES_DEFAULT;
  if (v & 1) ++v;
  if (v > MOTOR_POLES_MAX) v = MOTOR_POLES_MAX;
  return v;
}
static int normalizeWheelDiaMm(int v) {
  if (v < WHEEL_DIA_MM_MIN) return WHEEL_DIA_MM_DEFAULT;
  if (v > WHEEL_DIA_MM_MAX) return WHEEL_DIA_MM_DEFAULT;
  return v;
}
static int normalizeDriveRatioX100(int v) {
  if (v < DRIVE_RATIO_X100_MIN) return DRIVE_RATIO_X100_DEFAULT;
  if (v > DRIVE_RATIO_X100_MAX) return DRIVE_RATIO_X100_DEFAULT;
  return v;
}

constexpr int BATTERY_CELLS_MIN     = 10;
constexpr int BATTERY_CELLS_MAX     = 24;
constexpr int BATTERY_CELLS_DEFAULT = 10;
volatile int batteryCells = BATTERY_CELLS_DEFAULT;

// =============================================================================
//  Voltage calibration
// =============================================================================
constexpr int VOLTAGE_CAL_X1000_DEFAULT = 1000;
constexpr int VOLTAGE_CAL_X1000_MIN     = 800;
constexpr int VOLTAGE_CAL_X1000_MAX     = 1200;
volatile int  voltageCalX1000 = VOLTAGE_CAL_X1000_DEFAULT;

static int normalizeVoltageCalX1000(int v) {
  if (v < VOLTAGE_CAL_X1000_MIN) return VOLTAGE_CAL_X1000_DEFAULT;
  if (v > VOLTAGE_CAL_X1000_MAX) return VOLTAGE_CAL_X1000_DEFAULT;
  return v;
}
static inline float voltageCalMultiplier() {
  return normalizeVoltageCalX1000(voltageCalX1000) / 1000.0f;
}

// =============================================================================
//  Runtime state
// =============================================================================
int32_t       start_tachometer = -1;
float         trip_km          = 0.0f;
float         tripMaxSpeedKmh  = 0.0f;
float         tripMaxPowerW    = 0.0f;
float         tripMaxCurrentA  = 0.0f;
bool          vesc_online      = false;
unsigned long lastVescUpdate   = 0;

enum ScreenId : int {
  SCREEN_HOME          = 0,
  SCREEN_STATS         = 1,
  SCREEN_SETTINGS      = 2,
  SCREEN_LOCK_CONFIRM  = 3,
  SCREEN_NUMPAD        = 4,
  SCREEN_OTA           = 5,
  SCREEN_RECORDS       = 6,
  SCREEN_SETTINGS_2    = 7,
  SCREEN_DIAG          = 8,
  SCREEN_CAN_ID        = 9,
  SCREEN_CELLS         = 10,
  SCREEN_WIFI_CFG      = 11,
  SCREEN_WIFI_KEYBOARD = 12,
  SCREEN_BT_BRIDGE     = 13,
  SCREEN_RECORDS_CONFIRM = 14,
  SCREEN_MOTOR_CFG     = 15,
  SCREEN_SETTINGS_3    = 16,
  SCREEN_DRAGY         = 17,
  SCREEN_WEB           = 18,
  SCREEN_AUTOLOCK      = 19,
  SCREEN_SPEEDLIMIT    = 20,
  SCREEN_MUSIC         = 21,
};

enum NumpadMode : int {
  NUMPAD_NEW_PIN        = 0,
  NUMPAD_ENTER_PIN      = 1,
  NUMPAD_UNLOCK         = 2,
  NUMPAD_CONFIRM_RESET  = 3,
};

int  currentScreen      = SCREEN_HOME;
int  lastScreen         = -1;
bool screenNeedsRedraw  = true;
bool animationDone      = false;
bool hasRunStartupAnim  = false;
bool otaActive          = false;

enum OtaConnectState : int {
  OTA_STATE_IDLE       = 0,
  OTA_STATE_CONNECTING = 1,
  OTA_STATE_READY      = 2,
  OTA_STATE_FAILED     = 3,
};
int           otaConnectState   = OTA_STATE_IDLE;
unsigned long otaConnectStarted = 0;
bool firstRunAfterAnim  = false;

uint32_t      globalSeconds   = 0;
unsigned long precisionTimer  = 0;
unsigned long lastTouchTime   = 0;
unsigned long lastStatsUpdate = 0;
unsigned long lastHoldTime    = 0;

String savedPin      = "";
String inputPin      = "";
bool   isLocked      = false;
bool   passAtStartup = true;
bool   restoreLockedAfterBoot = false;
int    numpadMode    = NUMPAD_NEW_PIN;

unsigned long unlockLockoutUntil = 0;

volatile int canId_2nd = 1;

enum MotorCfgField : int {
  MCF_POLES = 0,
  MCF_WHEEL = 1,
  MCF_RATIO = 2,
};
int motorCfgField         = MCF_POLES;
int motorCfgPolesDraft    = MOTOR_POLES_DEFAULT;
int motorCfgWheelDraft    = WHEEL_DIA_MM_DEFAULT;
int motorCfgRatioX100Draft = DRIVE_RATIO_X100_DEFAULT;

volatile bool vescTaskPauseRequested = false;
volatile bool vescTaskPaused          = false;
volatile bool vescThrottleLockDesired = false;

bool          autoLockEnabled    = false;
int           autoLockTimeoutSec = 120;
volatile bool autoThrottleLock   = false;
volatile bool autoLockKick       = false;
static const int AUTOLOCK_PRESETS[] = { 15, 30, 60, 120, 300, 600, 900 };
static const int AUTOLOCK_PRESET_COUNT =
    (int)(sizeof(AUTOLOCK_PRESETS) / sizeof(AUTOLOCK_PRESETS[0]));

volatile bool speedLimitEnabled = false;
int           speedLimitKmh     = 25;
static const int SPEEDLIMIT_MIN_KMH = 5;
static const int SPEEDLIMIT_MAX_KMH = 999;
static const int SPEEDLIMIT_STEP_KMH = 1;

static const char* const kBtDeviceName = "VESC-Tool";

static const char* const kNusServiceUuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char* const kNusRxCharUuid  = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
static const char* const kNusTxCharUuid  = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

BLEServer*         bleServer         = nullptr;
BLECharacteristic* bleTxChar         = nullptr;
BLECharacteristic* bleRxChar         = nullptr;
bool               btBridgeActive    = false;
volatile int       melodyRequest     = -1;
int                musicPage         = 0;
TaskHandle_t       btBridgeTaskHandle = nullptr;
volatile bool      btBridgeStop      = false;
volatile bool      btClientConnected = false;
volatile uint16_t  bleMtuPayload     = 20;

StreamBufferHandle_t bleRxStream = nullptr;
static const size_t  BLE_RX_STREAM_SIZE = 2048;

static const char* const kLockBleDeviceName = "VESC-Lock";
static const char* const kLockServiceUuid   = "a1b2c3d4-0001-4a5b-8c6d-1234567890ab";
static const char* const kLockCmdCharUuid   = "a1b2c3d4-0002-4a5b-8c6d-1234567890ab";
static const char* const kLockStatusCharUuid= "a1b2c3d4-0003-4a5b-8c6d-1234567890ab";
static const uint8_t     LOCK_STATUS_UNLOCKED = 0;
static const uint8_t     LOCK_STATUS_LOCKED   = 1;
static const uint8_t     LOCK_STATUS_BAD_PIN  = 2;
static const uint8_t     LOCK_STATUS_NO_PIN   = 3;

BLEServer*         lockBleServer     = nullptr;
BLECharacteristic* lockStatusChar     = nullptr;
bool               lockBleActive     = false;
volatile bool      lockBleConnected  = false;
volatile bool      lockBleCmdPending = false;
volatile uint8_t   lockBleCmdData[8] = {0};
volatile uint8_t   lockBleCmdLen     = 0;

static const char*   kWebApSsid = "VESC-Dash";
static const char*   kWebApPass = "vesc12345";
WebServer            webServer(80);
DNSServer            webDns;
static const byte    WEB_DNS_PORT = 53;
static bool          webRoutesRegistered = false;
bool                 webDashActive = false;
IPAddress            webApIp;

String userWifiSsid = "";
String userWifiPass = "";

enum KeyboardTarget : int {
  KB_TARGET_SSID = 0,
  KB_TARGET_PASS = 1,
};
int     kbTarget   = KB_TARGET_SSID;
String  kbBuffer   = "";
bool    kbShift    = false;
bool    kbSymbols  = false;
static const size_t WIFI_SSID_MAX  = 32;
static const size_t WIFI_PASS_MAX  = 63;
static bool        wifiCredentialsReady();
static void        loadWifiCredentialsFromPrefs();
static void        saveWifiCredentialsToPrefs();
static void        drawWifiCfgScreen();
static void        drawKeyboardScreen();
static void        releaseGaugeCenterSprite();
static void        startBtBridge();
static void        stopBtBridge();
static void        drawBtBridgeScreen();
static void        startWebDash();
static void        stopWebDash();
static void        drawWebDashScreen();
static void        handleWebServer();
static void        updateWebDashStatus();
static void        pollOtaConnect();
static void        handleOtaFailedCleanup();
static void        resetRecordsAll();
static void        drawRecordsConfirmScreen();
static void        drawMotorCfgScreen();
static void        drawMotorCfgValues();
static void        drawAutoLockScreen();
static void        drawAutoLockValues();
static void        drawAutoLockToggleBtn();
static void        drawAutoLockStatusLive(bool force);
static int         normalizeAutoLockTimeoutSec(int value);
static void        drawSpeedLimitScreen();
static void        drawSpeedLimitValues();
static void        drawSpeedLimitToggleBtn();
static void        drawSpeedLimitStatusLive(bool force);
static void        speedLimitStepKmh(int dir);
static void        drawBluetoothGlyph(int cx, int cy, int size, uint16_t fg, uint16_t bg);
static void        updateOtaMarqueeBar();
static void        refreshPopupConfirmUpdate();
static void        releaseGaugeBatterySprite();
static void        updateGaugeBatteryAnimation();
static void        renderGaugeBatteryImmediate();
static void        releaseGaugePowerSprite();
static void        updateGaugePowerAnimation();
static void        renderGaugePowerImmediate();
static void        startLockBleService();
static void        stopLockBleService();
static void        notifyLockBleStatus(uint8_t status);

enum CockpitIcon : uint8_t {
  CPI_NONE = 0,
  CPI_LOCK,
  CPI_KEY,
  CPI_PIN,
  CPI_TROPHY,
  CPI_DOWNLOAD,
  CPI_CAN,
  CPI_BATTERY,
  CPI_WIFI,
  CPI_THEME,
  CPI_BT,
  CPI_DIAG,
  CPI_WARN,
  CPI_CHECK,
  CPI_CROSS,
  CPI_SHIELD,
  CPI_CHIP,
  CPI_THERMO,
  CPI_BOLT,
  CPI_GLOBE,
};

struct CockpitTile {
  const char* label;
  const char* valueChip;
  uint16_t    accent;
  uint8_t     icon;
  bool        active;
};

// =============================================================================
//  Forward declarations
// =============================================================================
void        stopOTA();
static void applyThemePalette();
static int  normalizeCanIdSetting(int value);
static int  normalizeBatteryCells(int value);
static int  normalizeThemeId(int value);
static bool isValidPinText(const String& pin);
static void formatTempText(float t, char* out, size_t outSize);
static void formatFaultText(int code, bool online, bool enabled,
                            char* out, size_t outSize);

// =============================================================================
//  Telemetry helpers
// =============================================================================
static bool tempLooksLikeSensorError(float t) {
  return isfinite(t) && t <= -19.5f;
}

static bool tempLooksValid(float t) {
  return isfinite(t) && t > -40.0f && t < 180.0f && !tempLooksLikeSensorError(t);
}

static const char* faultCodeToText(int code) {
  switch (code) {
    case 0:  return "NONE";
    case 1:  return "OV_V";
    case 2:  return "UV_V";
    case 3:  return "DRV";
    case 4:  return "OCP";
    case 5:  return "OT_FET";
    case 6:  return "OT_MOT";
    case 7:  return "G_OV";
    case 8:  return "G_UV";
    case 9:  return "MCU_UV";
    case 10: return "BOOT_WD";
    case 11: return "ENC_SPI";
    case 12: return "ENC_SINL";
    case 13: return "ENC_SINH";
    case 14: return "FLASH";
    case 15: return "CS1";
    case 16: return "CS2";
    case 17: return "CS3";
    case 18: return "CUR_IMB";
    case 19: return "BRK";
    case 20: return "R_LOT";
    case 21: return "R_DOS";
    case 22: return "R_LOS";
    case 23: return "CFG_APP";
    case 24: return "CFG_MC";
    case 25: return "NO_MAG";
    case 26: return "MAG_STR";
    case 27: return "PH_FILT";
    case 28: return "ENC_FLT";
    case 29: return "LV_OUT";
    case 30: return "SLIP";
    case 31: return "OVSPD";
    case 32: return "UNSPD";
    case 33: return "ABS_OVSPD";
    default: return "FAULT";
  }
}

static void formatTempText(float t, char* out, size_t outSize) {
  if (!out || outSize == 0) return;
  if (!isfinite(t)) {
    snprintf(out, outSize, "N/A");
    return;
  }
  if (tempLooksLikeSensorError(t)) {
    snprintf(out, outSize, "SENS?");
    return;
  }
  if (t < -40.0f || t > 180.0f) {
    snprintf(out, outSize, "N/A");
    return;
  }
  snprintf(out, outSize, "%.1fC", t);
}

static void formatFaultText(int code, bool online, bool enabled, char* out, size_t outSize) {
  if (!out || outSize == 0) return;
  if (!enabled) {
    snprintf(out, outSize, "OFF");
    return;
  }
  if (!online) {
    snprintf(out, outSize, "NO RESP");
    return;
  }
  if (code < 0) {
    snprintf(out, outSize, "N/A");
    return;
  }
  snprintf(out, outSize, "%s", faultCodeToText(code));
}

static int normalizeCanIdSetting(int value) {
  if (value <= 0) return -1;
  if (value > 127) return 127;
  return value;
}

static float erpmToKmh(float erpm) {
  const int   poles    = normalizeMotorPoles(motorPoles);
  const int   dMm      = normalizeWheelDiaMm(wheelDiaMm);
  const int   ratioX100 = normalizeDriveRatioX100(driveRatioX100);
  const float polePairs = poles / 2.0f;
  const float ratio     = ratioX100 / 100.0f;
  if (polePairs <= 0.0f || ratio <= 0.0f) return 0.0f;
  const float wheelCircM = 3.1415926535f * (dMm / 1000.0f);
  const float wheelRpm   = erpm / polePairs / ratio;
  return (wheelRpm * wheelCircM * 60.0f) / 1000.0f;
}

static float kmhToErpm(float kmh) {
  const int   poles    = normalizeMotorPoles(motorPoles);
  const int   dMm      = normalizeWheelDiaMm(wheelDiaMm);
  const int   ratioX100 = normalizeDriveRatioX100(driveRatioX100);
  const float polePairs = poles / 2.0f;
  const float ratio     = ratioX100 / 100.0f;
  const float wheelCircM = 3.1415926535f * (dMm / 1000.0f);
  if (wheelCircM <= 0.0f) return 0.0f;
  const float wheelRpm = (kmh * 1000.0f) / 60.0f / wheelCircM;
  return wheelRpm * polePairs * ratio;
}

static int normalizeSpeedLimitKmh(int value) {
  if (value < SPEEDLIMIT_MIN_KMH) return SPEEDLIMIT_MIN_KMH;
  if (value > SPEEDLIMIT_MAX_KMH) return SPEEDLIMIT_MAX_KMH;
  return value;
}

static float tachToKm(int32_t tachCounts) {
  const int   poles    = normalizeMotorPoles(motorPoles);
  const int   dMm      = normalizeWheelDiaMm(wheelDiaMm);
  const int   ratioX100 = normalizeDriveRatioX100(driveRatioX100);
  const float tachPerRev = 3.0f * (float)poles;
  const float ratio      = ratioX100 / 100.0f;
  if (tachPerRev <= 0.0f || ratio <= 0.0f) return 0.0f;
  const float wheelCircM = 3.1415926535f * (dMm / 1000.0f);
  const float motorRev   = (float)tachCounts / tachPerRev;
  const float wheelRev   = motorRev / ratio;
  return (wheelRev * wheelCircM) / 1000.0f;
}

static float clampFiniteFloat(float value, float minValue, float maxValue, float fallback = 0.0f) {
  if (!isfinite(value)) return fallback;
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

static void formatDecimal(float v, int decimals, char* out, size_t sz) {
  if (!out || sz == 0) return;
  float threshold = 0.5f;
  for (int i = 0; i < decimals; ++i) threshold *= 0.1f;
  if (!isfinite(v) || fabsf(v) < threshold) v = 0.0f;
  snprintf(out, sz, "%.*f", decimals, v);
}

static bool isValidPinText(const String& pin) {
  if (pin.length() != 4) return false;
  for (size_t i = 0; i < pin.length(); ++i) {
    if (pin[i] < '0' || pin[i] > '9') return false;
  }
  return true;
}

float disp_voltage = 0;
float disp_currentA = 0;

float disp_powerW = 0;
float disp_speedKmh = 0;
float start_ah = -1.0;
float start_wh = -1.0;

float disp_trip_km = 0;
int   disp_batPercent = 0;
int   disp_tempMosfet = 0;

int   disp_tempMotor = 0;
float disp_ampHours = 0;
float disp_wattHours = 0;

float ui_voltage = 0.0f;
float ui_currentA = 0.0f;
float ui_powerW = 0.0f;
float ui_speedKmh = 0.0f;
float ui_trip_km = 0.0f;
float ui_tempMosfet = 0.0f;
float ui_tempMotor = 0.0f;
float ui_batPercent = 0.0f;

bool uiSmoothingInitialized = false;

static float safeCombinedTemp(float primary, bool primaryOk,
                              float secondary, bool secondaryUsable) {
  const bool pOk = primaryOk      && tempLooksValid(primary);
  const bool sOk = secondaryUsable && tempLooksValid(secondary);
  if (pOk && sOk) return fmaxf(primary, secondary);
  if (pOk)        return primary;
  if (sOk)        return secondary;
  return 0.0f;
}

static float smoothTowards(float current, float target, float factor, float snapThreshold = -1.0f) {
  if (!isfinite(target)) return current;
  if (!isfinite(current)) return target;
  if (snapThreshold >= 0.0f && fabsf(target - current) > snapThreshold) return target;
  return current + (target - current) * factor;
}

static void syncDisplayToRaw() {
  ui_voltage = disp_voltage;
  ui_currentA = disp_currentA;
  ui_powerW = disp_powerW;
  ui_speedKmh = disp_speedKmh;
  ui_trip_km = disp_trip_km;
  ui_tempMosfet = (float)disp_tempMosfet;
  ui_tempMotor = (float)disp_tempMotor;
  ui_batPercent = (float)disp_batPercent;
  uiSmoothingInitialized = true;
}

static void updateDisplaySmoothing() {
  if (!uiSmoothingInitialized) {
    syncDisplayToRaw();
    return;
  }

  ui_speedKmh = smoothTowards(ui_speedKmh, disp_speedKmh, 0.20f, 6.0f);
  ui_currentA = smoothTowards(ui_currentA, disp_currentA, 0.16f, 12.0f);
  ui_powerW = smoothTowards(ui_powerW, disp_powerW, 0.16f, 120.0f);
  ui_trip_km = smoothTowards(ui_trip_km, disp_trip_km, 0.08f, 1.0f);
  ui_tempMosfet = smoothTowards(ui_tempMosfet, (float)disp_tempMosfet, 0.20f, 8.0f);
  ui_tempMotor = smoothTowards(ui_tempMotor, (float)disp_tempMotor, 0.20f, 8.0f);
  ui_voltage = disp_voltage;
  ui_batPercent = (float)disp_batPercent;
}

float allTimeSpeed    = 0;
float allTimePower    = 0;
float allTimeCurrent  = 0;
float allTimeFetTemp  = 0;
float allTimeMotTemp  = 0;

float         allTimeOdoKm = 0;
float         odoSavedKm   = 0;
unsigned long lastOdoSave  = 0;

int disp_primaryFault = -1;
int disp_secondaryFault = -1;
float disp_primaryTempMotor = NAN;
float disp_primaryTempMosfet = NAN;
float disp_secondaryTempMotor = NAN;
float disp_secondaryTempMosfet = NAN;
bool disp_secondaryOnline = false;
int disp_secondaryCanId = -1;

static bool diagStaticDrawn = false;
static unsigned long lastDiagRefresh = 0;
static char diagCachePrimaryMotor[16] = "";
static char diagCachePrimaryFet[16] = "";
static char diagCachePrimaryFault[18] = "";
static char diagCacheSecondaryMotor[16] = "";
static char diagCacheSecondaryFet[16] = "";
static char diagCacheSecondaryFault[18] = "";
static char diagCacheEsc2Header[24] = "";

unsigned long lastRecordSave = 0;
bool pendingRecordSave = false;

struct __attribute__((packed)) SettingsData {
  uint32_t magic;
  bool passStart;
  int32_t canId;
  char pin[5];
  int32_t batteryCells;
  int32_t themeId;
  int32_t motorPoles;
  int32_t wheelDiaMm;
  int32_t driveRatioX100;
  int32_t voltageCalX1000;
  bool    autoLockEnabled;
  int32_t autoLockTimeoutSec;
  bool    speedLimitEnabled;
  int32_t speedLimitKmh;
  bool    displayLocked;
};

static const uint32_t SETTINGS_MAGIC = 0x56455343;

enum ThemeId : int {
  THEME_DARK  = 0,
  THEME_LIGHT = 1,
};
static const int THEME_COUNT = 2;
int currentTheme = THEME_DARK;

SettingsData settingsData{};
bool settingsDirty = false;
unsigned long settingsDirtyAt = 0;

SemaphoreHandle_t telemetryMutex = nullptr;
TaskHandle_t vescTaskHandle = nullptr;
bool prefsReady = false;

static bool wifiCredentialsReady() {
  return userWifiSsid.length() > 0;
}

static void loadWifiCredentialsFromPrefs() {
  if (!prefsReady) return;
  String s = prefs.getString("wifiSsid", "");
  String p = prefs.getString("wifiPass", "");
  if (s.length() > WIFI_SSID_MAX) s.remove(WIFI_SSID_MAX);
  if (p.length() > WIFI_PASS_MAX) p.remove(WIFI_PASS_MAX);
  userWifiSsid = s;
  userWifiPass = p;
}

static void saveWifiCredentialsToPrefs() {
  if (!prefsReady) return;
  String cur_s = prefs.getString("wifiSsid", "");
  String cur_p = prefs.getString("wifiPass", "");
  if (cur_s != userWifiSsid) prefs.putString("wifiSsid", userWifiSsid);
  if (cur_p != userWifiPass) prefs.putString("wifiPass", userWifiPass);
}

struct SharedTelemetry {
  bool vesc_online;
  unsigned long lastVescUpdate;

  float disp_voltage;

  float disp_currentA;
  float disp_powerW;
  float disp_speedKmh;
  float start_ah;
  float start_wh;
  float disp_trip_km;
  int   disp_batPercent;

  int   disp_tempMosfet;
  int   disp_tempMotor;
  
  float disp_ampHours;
  float disp_wattHours;

  float trip_km;
  float tripMaxSpeedKmh;
  float tripMaxPowerW;
  float tripMaxCurrentA;

  float allTimeSpeed;
  float allTimePower;
  float allTimeCurrent;
  float allTimeFetTemp;
  float allTimeMotTemp;
  float allTimeOdoKm;

  int primaryFault;
  int secondaryFault;
  float primaryTempMotor;
  float primaryTempMosfet;
  float secondaryTempMotor;
  float secondaryTempMosfet;
  bool secondaryOnline;
  int secondaryCanId;

  unsigned long lastRecordSave;
  bool pendingRecordSave;
  int32_t start_tachometer;
};

SharedTelemetry telemetry{};

uint16_t BG_COLOR    = TFT_BLACK;
uint16_t LABEL_COLOR = 0x7BEF;
uint16_t VALUE_COLOR = TFT_WHITE;

namespace cockpit {
  uint16_t NEON_CYAN    = 0x07FF;
  uint16_t NEON_LIME    = 0x07E4;
  uint16_t NEON_AMBER   = 0xFD20;
  uint16_t NEON_RED     = 0xF9E0;
  uint16_t NEON_MAGENTA = 0xF81F;
  uint16_t NEON_BLUE    = 0x3D3F;

  uint16_t GLASS_FILL   = 0x1082;
  uint16_t GLASS_HI     = 0x3186;
  uint16_t GLASS_EDGE   = 0x2145;
  uint16_t TEXT_MAIN    = 0xFFFF;
  uint16_t TEXT_MUTED   = 0x9CD3;
  uint16_t CHIP_FILL    = 0x18E3;
}

SimpleKalmanFilter powerFilter(2, 2, 0.01);

const int BAT_X = 285;
const int BAT_Y = 50;
const int BAT_W = 28;

const int BAT_H = 170;
const int BAT_SEGMENTS = 10;

const int BAT_INFO_X = 250;
const int BAT_INFO_Y = 0;
const int BAT_INFO_W = 70;
const int BAT_INFO_H = 45;

bool inRect(int tx, int ty, int rx, int ry, int rw, int rh) {
  return (tx >= rx && tx <= rx + rw && ty >= ry && ty <= ry + rh);
}

void drawSettingsIcon(int x, int y, uint16_t color) {
  const int r = 13;
  tft.drawSmoothArc(x, y, r, r - 1, 0, 360, color, BG_COLOR);

  for (int i = 0; i < 6; i++) {
    const float a  = i * 60.0f * 0.01745329f;
    const float ca = cosf(a);
    const float sa = sinf(a);
    const int x1 = x + (int)lroundf(ca * 4.5f);
    const int y1 = y + (int)lroundf(sa * 4.5f);
    const int x2 = x + (int)lroundf(ca * 8.5f);
    const int y2 = y + (int)lroundf(sa * 8.5f);
    tft.drawWideLine(x1, y1, x2, y2, 3.0f, color, BG_COLOR);
  }

  tft.fillSmoothCircle(x, y, 4, color, BG_COLOR);
  tft.fillCircle(x, y, 2, BG_COLOR);
}

void drawHomeIcon(int x, int y) {
  uint16_t color = LABEL_COLOR;
  int r = 13;

  tft.drawSmoothArc(x, y, r, r - 1, 0, 360, color, BG_COLOR);

  int offsetX = -1;

  tft.fillRect(x - 4 + offsetX, y, 10, 2, color); 

  tft.drawLine(x - 4 + offsetX, y, x + offsetX, y - 4, color);

  tft.drawLine(x - 4 + offsetX, y + 1, x + offsetX, y - 3, color);

  tft.drawLine(x - 4 + offsetX, y, x + offsetX, y + 4, color);

  tft.drawLine(x - 4 + offsetX, y + 1, x + offsetX, y + 5, color);

}

static bool numpadInputAreaInvalidated = true;

void drawNumpadInputArea() {
  const uint16_t COCKPIT_GLASS_FILL = cockpit::GLASS_FILL;
  const uint16_t COCKPIT_NEON_CYAN  = cockpit::NEON_CYAN;
  const uint16_t COCKPIT_TEXT_MUTED = cockpit::TEXT_MUTED;

  const int x = 110, y = 46, w = 100, h = 22;
  const int maxDigits = 4;
  const int dotGap = 16;
  const int totalW = maxDigits * dotGap - dotGap;
  const int startX = x + w / 2 - totalW / 2;
  const int cy = y + h / 2;
  const int n = inputPin.length();

  static int lastCount = 0;

  if (numpadInputAreaInvalidated) {
    numpadInputAreaInvalidated = false;
    tft.fillRect(x + 2, y + 2, w - 4, h - 4, COCKPIT_GLASS_FILL);
    for (int i = 0; i < maxDigits; ++i) {
      const int dx = startX + i * dotGap;
      if (i < n) tft.fillSmoothCircle(dx, cy, 4, COCKPIT_NEON_CYAN, COCKPIT_GLASS_FILL);
      else       tft.drawCircle(dx, cy, 4, COCKPIT_TEXT_MUTED);
    }
    lastCount = n;
    return;
  }

  if (n == lastCount) return;

  if (n > lastCount) {
    for (int i = lastCount; i < n && i < maxDigits; ++i) {
      const int dx = startX + i * dotGap;
      tft.fillRect(dx - 5, cy - 5, 11, 11, COCKPIT_GLASS_FILL);
      tft.fillSmoothCircle(dx, cy, 4, COCKPIT_NEON_CYAN, COCKPIT_GLASS_FILL);
    }
  } else {
    for (int i = n; i < lastCount && i < maxDigits; ++i) {
      const int dx = startX + i * dotGap;
      tft.fillRect(dx - 5, cy - 5, 11, 11, COCKPIT_GLASS_FILL);
      tft.drawCircle(dx, cy, 4, COCKPIT_TEXT_MUTED);
    }
  }
  lastCount = n;
}

// =============================================================================
//  Cockpit settings UI
// =============================================================================

namespace cockpit {
  constexpr int TILE_W  = 190;
  constexpr int TILE_H  = 26;
  constexpr int SIDE_W  = 50;
  constexpr int GAP_Y   = 6;
  constexpr int GAP_X   = 10;
  constexpr int RADIUS  = 6;
}

static void drawCockpitIcon(int cx, int cy, uint8_t icon, uint16_t color, int holeColor = -1) {
  using namespace cockpit;
  const uint16_t bg = BG_COLOR;
  const uint16_t hole = (holeColor < 0) ? bg : (uint16_t)holeColor;
  switch (icon) {
    case CPI_LOCK: {
      tft.drawSmoothArc(cx, cy - 2, 5, 3, 90, 270, color, bg);
      tft.fillRoundRect(cx - 5, cy - 2, 11, 9, 2, color);
      tft.fillCircle(cx, cy + 1, 1, bg);
      tft.fillRect(cx, cy + 2, 1, 3, bg);
    } break;
    case CPI_KEY: {
      tft.drawSmoothArc(cx - 3, cy, 4, 2, 0, 360, color, bg);
      tft.fillRect(cx + 1, cy - 1, 6, 2, color);
      tft.fillRect(cx + 5, cy - 3, 2, 2, color);
    } break;
    case CPI_PIN: {
      tft.fillRoundRect(cx - 6, cy - 5, 12, 10, 1, color);
      tft.drawRoundRect(cx - 6, cy - 5, 12, 10, 1, color);
      tft.fillRect(cx - 4, cy - 3, 2, 2, hole);
      tft.fillRect(cx - 1, cy - 3, 2, 2, hole);
      tft.fillRect(cx + 2, cy - 3, 2, 2, hole);
      tft.fillRect(cx - 4, cy,     2, 2, hole);
      tft.fillRect(cx - 1, cy,     2, 2, hole);
      tft.fillRect(cx + 2, cy,     2, 2, hole);
    } break;
    case CPI_TROPHY: {
      tft.fillRect(cx - 4, cy - 5, 8, 6, color);
      tft.drawLine(cx - 6, cy - 4, cx - 4, cy - 2, color);
      tft.drawLine(cx + 6, cy - 4, cx + 4, cy - 2, color);
      tft.fillRect(cx - 2, cy + 1, 4, 3, color);
      tft.fillRect(cx - 5, cy + 4, 10, 2, color);
    } break;
    case CPI_DOWNLOAD: {
      tft.fillRect(cx - 1, cy - 6, 3, 7, color);
      tft.fillTriangle(cx - 5, cy - 1, cx + 5, cy - 1, cx, cy + 5, color);
      tft.fillRect(cx - 6, cy + 5, 13, 2, color);
    } break;
    case CPI_CAN: {
      tft.drawSmoothArc(cx, cy, 7, 6, 0, 360, color, bg);
      tft.fillCircle(cx, cy, 2, color);
      tft.drawLine(cx, cy, cx + 5, cy - 4, color);
      tft.drawLine(cx, cy, cx - 5, cy - 4, color);
      tft.drawLine(cx, cy, cx,     cy + 5, color);
    } break;
    case CPI_BATTERY: {
      tft.drawRoundRect(cx - 7, cy - 4, 12, 9, 1, color);
      tft.fillRect(cx + 5, cy - 2, 2, 5, color);
      tft.fillRect(cx - 5, cy - 2, 3, 5, color);
      tft.fillRect(cx - 1, cy - 2, 3, 5, color);
    } break;
    case CPI_WIFI: {
      tft.drawSmoothArc(cx, cy + 4, 8, 7, 225, 315, color, bg);
      tft.drawSmoothArc(cx, cy + 4, 5, 4, 225, 315, color, bg);
      tft.fillCircle(cx, cy + 3, 1, color);
    } break;
    case CPI_THEME: {
      tft.drawSmoothArc(cx, cy, 7, 6, 0, 360, color, bg);
      tft.fillCircle(cx - 2, cy - 2, 2, color);
      tft.fillCircle(cx + 3, cy - 1, 2, color);
      tft.fillCircle(cx + 1, cy + 3, 2, color);
      tft.fillCircle(cx - 3, cy + 2, 2, color);
    } break;
    case CPI_BT: {
      drawBluetoothGlyph(cx, cy, 14, color, bg);
    } break;
    case CPI_DIAG: {
      tft.drawLine(cx - 7, cy + 3, cx - 4, cy + 3, color);
      tft.drawLine(cx - 4, cy + 3, cx - 2, cy - 3, color);
      tft.drawLine(cx - 2, cy - 3, cx,     cy + 5, color);
      tft.drawLine(cx,     cy + 5, cx + 2, cy - 1, color);
      tft.drawLine(cx + 2, cy - 1, cx + 4, cy + 3, color);
      tft.drawLine(cx + 4, cy + 3, cx + 7, cy + 3, color);
    } break;
    case CPI_WARN: {
      tft.fillTriangle(cx, cy - 7, cx - 7, cy + 5, cx + 7, cy + 5, color);
      tft.fillRect(cx - 1, cy - 3, 2, 5, hole);
      tft.fillRect(cx - 1, cy + 3, 2, 2, hole);
    } break;
    case CPI_CHECK: {
      tft.drawLine(cx - 5, cy + 1, cx - 2, cy + 4, color);
      tft.drawLine(cx - 5, cy,     cx - 2, cy + 3, color);
      tft.drawLine(cx - 2, cy + 4, cx + 5, cy - 3, color);
      tft.drawLine(cx - 2, cy + 3, cx + 5, cy - 4, color);
    } break;
    case CPI_CROSS: {
      tft.drawLine(cx - 5, cy - 5, cx + 5, cy + 5, color);
      tft.drawLine(cx - 4, cy - 5, cx + 6, cy + 5, color);
      tft.drawLine(cx + 5, cy - 5, cx - 5, cy + 5, color);
      tft.drawLine(cx + 6, cy - 5, cx - 4, cy + 5, color);
    } break;
    case CPI_SHIELD: {
      tft.drawLine(cx - 6, cy - 5, cx, cy - 7, color);
      tft.drawLine(cx, cy - 7, cx + 6, cy - 5, color);
      tft.drawLine(cx - 6, cy - 5, cx - 6, cy + 1, color);
      tft.drawLine(cx + 6, cy - 5, cx + 6, cy + 1, color);
      tft.drawLine(cx - 6, cy + 1, cx, cy + 6, color);
      tft.drawLine(cx + 6, cy + 1, cx, cy + 6, color);
    } break;
    case CPI_CHIP: {
      tft.drawRoundRect(cx - 6, cy - 6, 12, 12, 2, color);
      tft.fillRect(cx - 3, cy - 3, 6, 6, color);
      for (int i = 0; i < 3; i++) {
        tft.fillRect(cx - 5 + i * 4, cy - 8, 2, 2, color);
        tft.fillRect(cx - 5 + i * 4, cy + 6, 2, 2, color);
        tft.fillRect(cx - 8, cy - 5 + i * 4, 2, 2, color);
        tft.fillRect(cx + 6, cy - 5 + i * 4, 2, 2, color);
      }
    } break;
    case CPI_THERMO: {
      tft.drawLine(cx - 2, cy - 6, cx + 2, cy - 6, color);
      tft.drawLine(cx - 2, cy - 6, cx - 2, cy + 3, color);
      tft.drawLine(cx + 2, cy - 6, cx + 2, cy + 3, color);
      tft.fillCircle(cx, cy + 4, 3, color);
      tft.fillRect(cx - 1, cy - 3, 2, 6, color);
    } break;
    case CPI_BOLT: {
      tft.fillTriangle(cx + 1, cy - 7, cx - 4, cy + 1, cx + 1, cy + 1, color);
      tft.fillTriangle(cx - 1, cy - 1, cx + 4, cy + 7, cx - 1, cy + 7, color);
    } break;
    case CPI_GLOBE: {
      tft.drawCircle(cx, cy, 7, color);
      tft.drawFastHLine(cx - 7, cy, 15, color);
      tft.drawFastVLine(cx, cy - 7, 15, color);
      tft.drawEllipse(cx, cy, 3, 7, color);
    } break;
    default: break;
  }
}

static int drawValueChip(int rightX, int cy, const char* text, uint16_t accent) {
  using namespace cockpit;
  tft.setFreeFont(GLCD);
  const int textW = tft.textWidth(text);
  const int chipW = textW + 12;
  const int chipH = 16;
  const int x = rightX - chipW;
  const int y = cy - chipH / 2;
  tft.fillRoundRect(x, y, chipW, chipH, 3, CHIP_FILL);
  tft.drawRoundRect(x, y, chipW, chipH, 3, accent);
  tft.setTextColor(accent, CHIP_FILL);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(text, x + chipW / 2, y + chipH / 2 + 1);
  return chipW;
}

static void drawCockpitTile(int x, int y, const CockpitTile& t) {
  using namespace cockpit;
  const int w = TILE_W;
  const int h = TILE_H;

  tft.fillRoundRect(x, y, w, h, RADIUS, GLASS_FILL);
  tft.drawRoundRect(x, y, w, h, RADIUS, GLASS_EDGE);
  tft.drawFastHLine(x + 3, y + 1, w - 6, GLASS_HI);

  tft.fillRoundRect(x + 2, y + 3, 3, h - 6, 1, t.accent);

  const int iconCx = x + 18;
  const int iconCy = y + h / 2;
  if (t.icon != CPI_NONE) {
    drawCockpitIcon(iconCx, iconCy, t.icon, t.accent);
  }

  int rightEdge = x + w - 8;
  if (t.active) {
    tft.fillSmoothCircle(rightEdge - 4, iconCy, 4, t.accent, GLASS_FILL);
    tft.drawCircle(rightEdge - 4, iconCy, 6, t.accent);
    rightEdge -= 14;
  }
  if (t.valueChip && t.valueChip[0]) {
    const int chipW = drawValueChip(rightEdge, iconCy, t.valueChip, t.accent);
    rightEdge -= chipW + 6;
  }

  tft.setFreeFont(FF1);
  tft.setTextColor(TEXT_MAIN, GLASS_FILL);
  tft.setTextDatum(ML_DATUM);
  const int labelX    = x + 32;
  const int labelMaxW = rightEdge - labelX - 4;
  if (labelMaxW < 10) return;

  if (tft.textWidth(t.label) <= labelMaxW) {
    tft.drawString(t.label, labelX, iconCy + 1);
  } else {
    char buf[40];
    snprintf(buf, sizeof(buf), "%s", t.label);
    const int ellW = tft.textWidth("...");
    while (buf[0] && tft.textWidth(buf) + ellW > labelMaxW) {
      buf[strlen(buf) - 1] = '\0';
    }
    char outBuf[44];
    snprintf(outBuf, sizeof(outBuf), "%s...", buf);
    tft.drawString(outBuf, labelX, iconCy + 1);
  }
}

static void drawCockpitChevron(int x, int y, int dir) {
  using namespace cockpit;
  const int w = SIDE_W;
  const int h = TILE_H;
  tft.fillRoundRect(x, y, w, h, RADIUS, GLASS_FILL);
  tft.drawRoundRect(x, y, w, h, RADIUS, GLASS_EDGE);
  tft.drawFastHLine(x + 3, y + 1, w - 6, GLASS_HI);

  const int cx = x + w / 2;
  const int cy = y + h / 2;
  if (dir < 0) {
    tft.drawLine(cx + 2, cy - 6, cx - 4, cy,     NEON_CYAN);
    tft.drawLine(cx + 2, cy + 6, cx - 4, cy,     NEON_CYAN);
    tft.drawLine(cx + 3, cy - 6, cx - 3, cy,     NEON_CYAN);
    tft.drawLine(cx + 3, cy + 6, cx - 3, cy,     NEON_CYAN);
    tft.drawLine(cx + 7, cy - 6, cx + 1, cy,     NEON_CYAN);
    tft.drawLine(cx + 7, cy + 6, cx + 1, cy,     NEON_CYAN);
  } else {
    tft.drawLine(cx - 2, cy - 6, cx + 4, cy,     NEON_CYAN);
    tft.drawLine(cx - 2, cy + 6, cx + 4, cy,     NEON_CYAN);
    tft.drawLine(cx - 3, cy - 6, cx + 3, cy,     NEON_CYAN);
    tft.drawLine(cx - 3, cy + 6, cx + 3, cy,     NEON_CYAN);
    tft.drawLine(cx - 7, cy - 6, cx - 1, cy,     NEON_CYAN);
    tft.drawLine(cx - 7, cy + 6, cx - 1, cy,     NEON_CYAN);
  }
}

static void drawCockpitHeader(const char* section) {
  using namespace cockpit;
  tft.fillRect(10, 10, 3, 14, NEON_CYAN);
  tft.setFreeFont(GLCD);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(NEON_CYAN, BG_COLOR);
  tft.drawString("COCKPIT", 18, 10);
  tft.setTextColor(TEXT_MUTED, BG_COLOR);
  tft.drawString("/", 18 + tft.textWidth("COCKPIT") + 3, 10);
  tft.setTextColor(TEXT_MAIN, BG_COLOR);
  tft.drawString(section, 18 + tft.textWidth("COCKPIT") + 10, 10);
  tft.drawFastHLine(10, 26, 280, GLASS_EDGE);
  tft.drawFastHLine(10, 27, 60,  NEON_CYAN);
}

static void drawCockpitPager(int pageIdx, int pageTotal) {
  using namespace cockpit;
  const int segW = 22, segH = 3, gap = 6;
  const int totalW = pageTotal * segW + (pageTotal - 1) * gap;
  int x = (320 - totalW) / 2;
  const int y = 232;
  for (int i = 0; i < pageTotal; ++i) {
    const bool active = (i == pageIdx);
    tft.fillRoundRect(x, y, segW, segH, 1, active ? NEON_CYAN : GLASS_EDGE);
    x += segW + gap;
  }
}

// -----------------------------------------------------------------------------
// Shared cockpit OS toolkit
// -----------------------------------------------------------------------------

static void drawCockpitPanel(int x, int y, int w, int h, uint16_t accent, const char* title = nullptr) {
  using namespace cockpit;
  const uint16_t border = accent ? accent : GLASS_EDGE;
  tft.fillRoundRect(x, y, w, h, 10, BG_COLOR);
  tft.drawRoundRect(x, y, w, h, 10, border);
  tft.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 9, GLASS_EDGE);
  tft.drawFastHLine(x + 6, y + 2, w - 12, GLASS_HI);
  if (title) {
    tft.fillRect(x + 8, y + 8, 3, 10, accent ? accent : NEON_CYAN);
    tft.setFreeFont(GLCD);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(accent ? accent : NEON_CYAN, BG_COLOR);
    tft.drawString(title, x + 16, y + 8);
  }
}

static void drawCockpitActionBtn(int x, int y, int w, int h, const char* label,
                                 uint8_t kind, uint8_t icon = CPI_NONE) {
  using namespace cockpit;
  uint16_t accent;
  switch (kind) {
    case 1:  accent = NEON_LIME;  break;
    case 2:  accent = NEON_RED;   break;
    case 3:  accent = TEXT_MUTED; break;
    default: accent = NEON_CYAN;  break;
  }
  tft.fillRoundRect(x, y, w, h, 8, GLASS_FILL);
  tft.drawRoundRect(x, y, w, h, 8, accent);
  tft.drawFastHLine(x + 4, y + 1, w - 8, GLASS_HI);
  tft.fillRoundRect(x + 2, y + 3, 3, h - 6, 1, accent);

  tft.setFreeFont(FF1);
  const int textW    = (label && label[0]) ? tft.textWidth(label) : 0;
  const int padding  = 10;
  const int maxInner = w - padding;

  int iconW = (icon != CPI_NONE) ? 14 : 0;
  int gap   = (iconW > 0 && textW > 0) ? 6 : 0;
  if (iconW + gap + textW > maxInner) {
    iconW = 0;
    gap   = 0;
  }

  char   truncated[36];
  const char* drawLabel = label;
  int    effTextW       = textW;
  if (effTextW > maxInner && textW > 0) {
    snprintf(truncated, sizeof(truncated), "%s", label);
    const int ellW = tft.textWidth("..");
    while (truncated[0] && tft.textWidth(truncated) + ellW > maxInner) {
      truncated[strlen(truncated) - 1] = '\0';
    }
    if (truncated[0]) {
      size_t len = strlen(truncated);
      if (len + 2 < sizeof(truncated)) {
        truncated[len]     = '.';
        truncated[len + 1] = '.';
        truncated[len + 2] = '\0';
      }
    }
    drawLabel = truncated;
    effTextW  = tft.textWidth(drawLabel);
  }

  const int totalW = iconW + gap + effTextW;
  const int startX = x + (w - totalW) / 2;
  const int cy     = y + h / 2;

  if (iconW > 0) {
    drawCockpitIcon(startX + iconW / 2, cy, icon, accent);
  }
  if (effTextW > 0) {
    tft.setTextColor(TEXT_MAIN, GLASS_FILL);
    tft.setTextDatum(ML_DATUM);
    tft.drawString(drawLabel, startX + iconW + gap, cy + 1);
  }
}

static void drawCockpitStepperBtn(int x, int y, int w, int h, bool minus, uint16_t accent) {
  using namespace cockpit;
  tft.fillRoundRect(x, y, w, h, 8, GLASS_FILL);
  tft.drawRoundRect(x, y, w, h, 8, accent);
  tft.drawFastHLine(x + 4, y + 1, w - 8, GLASS_HI);
  const int cx = x + w / 2;
  const int cy = y + h / 2;
  tft.drawRoundRect(x + 3, y + 3, w - 6, h - 6, 6, GLASS_EDGE);
  tft.fillRoundRect(cx - 9, cy - 2, 18, 4, 1, accent);
  if (!minus) tft.fillRoundRect(cx - 2, cy - 9, 4, 18, 1, accent);
}

static void drawCockpitStatusDot(int x, int y, uint16_t color, const char* label) {
  using namespace cockpit;
  tft.fillSmoothCircle(x, y, 4, color, BG_COLOR);
  tft.drawCircle(x, y, 6, color);
  tft.setFreeFont(GLCD);
  tft.setTextColor(color, BG_COLOR);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(label, x + 12, y);
}

static int drawCockpitBigChip(int cx, int cy, const char* text, uint16_t accent) {
  using namespace cockpit;
  tft.setFreeFont(FF2);
  const int textW = tft.textWidth(text);
  const int w = textW + 20;
  const int h = 30;
  const int x = cx - w / 2;
  const int y = cy - h / 2;
  tft.fillRoundRect(x, y, w, h, 6, GLASS_FILL);
  tft.drawRoundRect(x, y, w, h, 6, accent);
  tft.drawFastHLine(x + 4, y + 1, w - 8, GLASS_HI);
  tft.setTextColor(accent, GLASS_FILL);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(text, cx, cy + 2);
  return w;
}

// =============================================================================
//  Cockpit Telemetry Hub
// =============================================================================

namespace stats_layout {
  constexpr int MARGIN  = 6;

  constexpr int SBAT_X = MARGIN, SBAT_Y = 34, SBAT_W = 156, SBAT_H = 76;
  constexpr int PWR_X  = SBAT_X + SBAT_W + MARGIN;
  constexpr int PWR_Y  = SBAT_Y;
  constexpr int PWR_W  = 320 - PWR_X - MARGIN;
  constexpr int PWR_H  = SBAT_H;

  constexpr int MID_Y  = SBAT_Y + SBAT_H + MARGIN;
  constexpr int MID_H  = 50;
  constexpr int MID_W  = 100;
  constexpr int MID1_X = MARGIN;
  constexpr int MID2_X = MID1_X + MID_W + MARGIN;
  constexpr int MID3_X = MID2_X + MID_W + MARGIN;
  constexpr int MID3_W = 320 - MID3_X - MARGIN;

  constexpr int RIDE_Y = MID_Y + MID_H + MARGIN;
  constexpr int RIDE_H = 38;
  constexpr int TRIP_X = MARGIN;
  constexpr int TRIP_W = (320 - 3 * MARGIN) / 2;
  constexpr int VMAX_X = TRIP_X + TRIP_W + MARGIN;
  constexpr int VMAX_W = TRIP_W;

  constexpr int BAR_Y  = 220, BAR_H  = 18;
}

struct StatsCache {
  int      batPct     = -1;
  int      voltMv     = -1;
  int      ahMilli    = -1;
  int      powerW10   = INT_MIN;
  int      currA10    = INT_MIN;
  int      effWhKm10  = INT_MIN;
  int      motorC     = INT_MIN;
  int      fetC       = INT_MIN;
  uint32_t uptime     = UINT32_MAX;
  int      tripM      = -1;
  int      tripVmax10 = -1;
  uint8_t  statusBits = 0xFF;
};
static StatsCache statsCache;

static void statsCacheReset() {
  statsCache = StatsCache{};
}

static void drawStatsCard(int x, int y, int w, int h, uint16_t accent) {
  using namespace cockpit;
  tft.fillRoundRect(x, y, w, h, 6, GLASS_FILL);
  tft.drawRoundRect(x, y, w, h, 6, GLASS_EDGE);
  tft.drawFastHLine(x + 8, y + 3, w - 16, accent);
}

static void drawStatsCardLabel(int x, int y, const char* label, uint16_t accent) {
  using namespace cockpit;
  tft.setFreeFont(GLCD);
  tft.setTextColor(accent, GLASS_FILL);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(label, x, y);
}

static void drawChargeBar(int x, int y, int w, int h, int pct) {
  using namespace cockpit;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  tft.drawRoundRect(x, y, w, h, 2, GLASS_EDGE);
  tft.fillRect(x + 1, y + 1, w - 2, h - 2, GLASS_FILL);
  const uint16_t color = (pct < 25) ? NEON_RED
                       : (pct < 65) ? NEON_AMBER
                                    : NEON_LIME;
  const int fillW = ((w - 2) * pct) / 100;
  if (fillW > 0) tft.fillRect(x + 1, y + 1, fillW, h - 2, color);
}

static TFT_eSprite statsCellBigSpr(&tft);
static TFT_eSprite statsCellSmallSpr(&tft);
static TFT_eSprite statsCellMidSpr(&tft);
static bool        statsCellBigInit   = false;
static bool        statsCellSmallInit = false;
static bool        statsCellMidInit   = false;
static const int   STATS_BIG_W    = 88;
static const int   STATS_BIG_H    = 26;
static const int   STATS_SMALL_W  = 130;
static const int   STATS_SMALL_H  = 14;
static const int   STATS_MID_W    = 88;
static const int   STATS_MID_H    = 22;

static bool ensureStatsBigSprite() {
  if (statsCellBigInit) return true;
  statsCellBigSpr.setColorDepth(16);
  if (statsCellBigSpr.createSprite(STATS_BIG_W, STATS_BIG_H) == nullptr) {
    Serial.println("stats big sprite: createSprite failed");
    return false;
  }
  statsCellBigInit = true;
  return true;
}
static bool ensureStatsSmallSprite() {
  if (statsCellSmallInit) return true;
  statsCellSmallSpr.setColorDepth(16);
  if (statsCellSmallSpr.createSprite(STATS_SMALL_W, STATS_SMALL_H) == nullptr) {
    Serial.println("stats small sprite: createSprite failed");
    return false;
  }
  statsCellSmallInit = true;
  return true;
}
static bool ensureStatsMidSprite() {
  if (statsCellMidInit) return true;
  statsCellMidSpr.setColorDepth(16);
  if (statsCellMidSpr.createSprite(STATS_MID_W, STATS_MID_H) == nullptr) {
    Serial.println("stats mid sprite: createSprite failed");
    return false;
  }
  statsCellMidInit = true;
  return true;
}

static void releaseStatsSprites() {
  if (statsCellBigInit)   { statsCellBigSpr.deleteSprite();   statsCellBigInit   = false; }
  if (statsCellSmallInit) { statsCellSmallSpr.deleteSprite(); statsCellSmallInit = false; }
  if (statsCellMidInit)   { statsCellMidSpr.deleteSprite();   statsCellMidInit   = false; }
}

static void releasePopupChipSprite();

static void drawStatsBigValue(int dstX, int dstY, int cellW,
                              const char* big, const char* unit,
                              uint16_t bigColor, uint16_t unitColor,
                              uint16_t bgColor) {
  if (cellW > STATS_BIG_W) cellW = STATS_BIG_W;
  if (!ensureStatsBigSprite()) {
    tft.fillRect(dstX, dstY, cellW, STATS_BIG_H, bgColor);
    tft.setFreeFont(FF2);
    tft.setTextColor(bigColor, bgColor);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(big, dstX + 2, dstY + 2);
    const int bigW = tft.textWidth(big);
    if (unit && unit[0]) {
      tft.setFreeFont(GLCD);
      tft.setTextColor(unitColor, bgColor);
      tft.drawString(unit, dstX + 2 + bigW + 3, dstY + 14);
    }
    return;
  }
  statsCellBigSpr.fillSprite(bgColor);
  statsCellBigSpr.setFreeFont(FF2);
  statsCellBigSpr.setTextColor(bigColor, bgColor);
  statsCellBigSpr.setTextDatum(TL_DATUM);
  statsCellBigSpr.drawString(big, 2, 2);
  const int bigW = statsCellBigSpr.textWidth(big);
  if (unit && unit[0]) {
    statsCellBigSpr.setFreeFont(GLCD);
    statsCellBigSpr.setTextColor(unitColor, bgColor);
    statsCellBigSpr.drawString(unit, 2 + bigW + 3, 14);
  }
  statsCellBigSpr.pushSprite(dstX, dstY);
}

static void drawStatsMidValue(int dstX, int dstY, int cellW,
                              const char* big, const char* unit,
                              uint16_t bigColor, uint16_t unitColor,
                              uint16_t bgColor) {
  if (cellW > STATS_MID_W) cellW = STATS_MID_W;
  if (!ensureStatsMidSprite()) {
    tft.fillRect(dstX, dstY, cellW, STATS_MID_H, bgColor);
    tft.setFreeFont(FF1);
    tft.setTextColor(bigColor, bgColor);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(big, dstX + 2, dstY + 2);
    const int bigW = tft.textWidth(big);
    if (unit && unit[0]) {
      tft.setFreeFont(GLCD);
      tft.setTextColor(unitColor, bgColor);
      tft.drawString(unit, dstX + 2 + bigW + 4, dstY + 8);
    }
    return;
  }
  statsCellMidSpr.fillSprite(bgColor);
  statsCellMidSpr.setFreeFont(FF1);
  statsCellMidSpr.setTextColor(bigColor, bgColor);
  statsCellMidSpr.setTextDatum(TL_DATUM);
  statsCellMidSpr.drawString(big, 2, 2);
  const int bigW = statsCellMidSpr.textWidth(big);
  if (unit && unit[0]) {
    statsCellMidSpr.setFreeFont(GLCD);
    statsCellMidSpr.setTextColor(unitColor, bgColor);
    statsCellMidSpr.drawString(unit, 2 + bigW + 4, 8);
  }
  statsCellMidSpr.pushSprite(dstX, dstY);
}

static void drawStatsCentered(int dstX, int dstY, int cellW,
                              const char* text,
                              uint16_t fg, uint16_t bg) {
  if (cellW > STATS_MID_W) cellW = STATS_MID_W;
  if (!ensureStatsMidSprite()) {
    tft.fillRect(dstX, dstY, cellW, STATS_MID_H, bg);
    tft.setFreeFont(FF1);
    tft.setTextColor(fg, bg);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(text, dstX + cellW / 2, dstY + STATS_MID_H / 2);
    return;
  }
  statsCellMidSpr.fillSprite(bg);
  statsCellMidSpr.setFreeFont(FF1);
  statsCellMidSpr.setTextColor(fg, bg);
  statsCellMidSpr.setTextDatum(MC_DATUM);
  statsCellMidSpr.drawString(text, cellW / 2, STATS_MID_H / 2);
  statsCellMidSpr.pushSprite(dstX, dstY);
}

static void drawStatsGlcdLine(int dstX, int dstY, int cellW,
                              const char* text, uint16_t fg, uint16_t bg) {
  if (cellW > STATS_SMALL_W) cellW = STATS_SMALL_W;
  if (!ensureStatsSmallSprite()) {
    tft.fillRect(dstX, dstY, cellW, STATS_SMALL_H, bg);
    tft.setFreeFont(GLCD);
    tft.setTextColor(fg, bg);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(text, dstX + 2, dstY + 1);
    return;
  }
  statsCellSmallSpr.fillSprite(bg);
  statsCellSmallSpr.setFreeFont(GLCD);
  statsCellSmallSpr.setTextColor(fg, bg);
  statsCellSmallSpr.setTextDatum(TL_DATUM);
  statsCellSmallSpr.drawString(text, 2, 1);
  statsCellSmallSpr.pushSprite(dstX, dstY);
}

void drawStatsStatic() {
  using namespace cockpit;
  using namespace stats_layout;

  statsCacheReset();

  drawCockpitHeader("TELEMETRY");
  drawSettingsIcon(300, 20, NEON_CYAN);

  drawStatsCard(SBAT_X, SBAT_Y, SBAT_W, SBAT_H, NEON_LIME);
  drawStatsCardLabel(SBAT_X + 8, SBAT_Y + 8, "BATTERY", NEON_LIME);

  drawStatsCard(PWR_X, PWR_Y, PWR_W, PWR_H, NEON_CYAN);
  drawStatsCardLabel(PWR_X + 8, PWR_Y + 8, "POWER MAX", NEON_CYAN);

  drawStatsCard(MID1_X, MID_Y, MID_W, MID_H, NEON_AMBER);
  drawStatsCardLabel(MID1_X + 8, MID_Y + 6, "MOTOR", NEON_AMBER);

  drawStatsCard(MID2_X, MID_Y, MID_W, MID_H, NEON_AMBER);
  drawStatsCardLabel(MID2_X + 8, MID_Y + 6, "MOSFET", NEON_AMBER);

  drawStatsCard(MID3_X, MID_Y, MID3_W, MID_H, NEON_MAGENTA);
  drawStatsCardLabel(MID3_X + 8, MID_Y + 6, "UPTIME", NEON_MAGENTA);

  drawStatsCard(TRIP_X, RIDE_Y, TRIP_W, RIDE_H, NEON_CYAN);
  drawStatsCardLabel(TRIP_X + 8, RIDE_Y + 6, "TRIP", NEON_CYAN);

  drawStatsCard(VMAX_X, RIDE_Y, VMAX_W, RIDE_H, NEON_MAGENTA);
  drawStatsCardLabel(VMAX_X + 8, RIDE_Y + 6, "SPEED MAX", NEON_MAGENTA);
}

void updateStatsValues() {
  using namespace cockpit;
  using namespace stats_layout;

  if (millis() - lastStatsUpdate < timing::STATS_REFRESH) return;
  lastStatsUpdate = millis();

  char buf[32];

  {
    const int pct = disp_batPercent;
    if (pct != statsCache.batPct) {
      statsCache.batPct = pct;
      snprintf(buf, sizeof(buf), "%d", pct);
      drawStatsBigValue(SBAT_X + 8, SBAT_Y + 18, SBAT_W - 16,
                        buf, "%", TEXT_MAIN, TEXT_MUTED, GLASS_FILL);
      drawChargeBar(SBAT_X + 10, SBAT_Y + 46, SBAT_W - 20, 6, pct);
    }
    const int voltMv = (int)(disp_voltage * 100.0f + 0.5f);
    const int ahMa   = (int)(disp_ampHours * 1000.0f + 0.5f);
    if (voltMv != statsCache.voltMv || ahMa != statsCache.ahMilli) {
      statsCache.voltMv  = voltMv;
      statsCache.ahMilli = ahMa;
      char line[32];
      snprintf(line, sizeof(line), "%.1fV   %.1fAh", disp_voltage, disp_ampHours);
      drawStatsGlcdLine(SBAT_X + 8, SBAT_Y + 58, SBAT_W - 16,
                        line, NEON_LIME, GLASS_FILL);
    }
  }

  {
    const int peakW = (int)lroundf(tripMaxPowerW);
    if (peakW != statsCache.powerW10) {
      statsCache.powerW10  = peakW;
      const uint16_t col   = (peakW > 50) ? NEON_CYAN : TEXT_MUTED;
      char b[12];
      snprintf(b, sizeof(b), "%d", peakW);
      drawStatsBigValue(PWR_X + 8, PWR_Y + 18, PWR_W - 16,
                        b, "W", col, TEXT_MUTED, GLASS_FILL);
    }
    const int a10 = (int)(tripMaxCurrentA * 10.0f);
    const float eff = (trip_km > 0.05f) ? (disp_wattHours / trip_km) : 0.0f;
    const int e10 = (int)(eff * 10.0f);
    if (a10 != statsCache.currA10 || e10 != statsCache.effWhKm10) {
      statsCache.currA10   = a10;
      statsCache.effWhKm10 = e10;
      char line[32];
      char aBuf[10];
      char effBuf[10];
      formatDecimal(tripMaxCurrentA, 1, aBuf, sizeof(aBuf));
      formatDecimal(eff, 0, effBuf, sizeof(effBuf));
      snprintf(line, sizeof(line), "%sA   %sWh/km", aBuf, effBuf);
      drawStatsGlcdLine(PWR_X + 8, PWR_Y + 58, PWR_W - 16,
                        line, NEON_CYAN, GLASS_FILL);
    }
  }

  {
    const int mC = (int)(disp_tempMotor + 0.5f);
    if (mC != statsCache.motorC) {
      statsCache.motorC = mC;
      snprintf(buf, sizeof(buf), "%d", mC);
      drawStatsBigValue(MID1_X + 8, MID_Y + 18, MID_W - 8,
                        buf, "C", TEXT_MAIN, NEON_AMBER, GLASS_FILL);
    }
  }

  {
    const int fC = (int)(disp_tempMosfet + 0.5f);
    if (fC != statsCache.fetC) {
      statsCache.fetC = fC;
      snprintf(buf, sizeof(buf), "%d", fC);
      drawStatsBigValue(MID2_X + 8, MID_Y + 18, MID_W - 8,
                        buf, "C", TEXT_MAIN, NEON_AMBER, GLASS_FILL);
    }
  }

  {
    if (globalSeconds != statsCache.uptime) {
      statsCache.uptime = globalSeconds;
      int h = globalSeconds / 3600;
      if (h > 99) h = 99;
      const int m = (globalSeconds % 3600) / 60;
      const int s = globalSeconds % 60;
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d", h, m, s);
      drawStatsCentered(MID3_X + 4, MID_Y + MID_H / 2 - STATS_MID_H / 2 + 2,
                        MID3_W - 8, buf, TEXT_MAIN, GLASS_FILL);
    }
  }

  {
    const int tripM = (int)(trip_km * 1000.0f);
    if (tripM != statsCache.tripM) {
      statsCache.tripM = tripM;
      char t[16];
      if (trip_km < 1.0f) snprintf(t, sizeof(t), "%.0f", trip_km * 1000.0f);
      else                snprintf(t, sizeof(t), "%.2f", trip_km);
      drawStatsMidValue(TRIP_X + 8, RIDE_Y + 14, TRIP_W - 8,
                        t, trip_km < 1.0f ? "m" : "km",
                        TEXT_MAIN, TEXT_MUTED, GLASS_FILL);
    }
  }

  {
    const int v10 = (int)(tripMaxSpeedKmh * 10.0f);
    if (v10 != statsCache.tripVmax10) {
      statsCache.tripVmax10 = v10;
      char vbuf[16];
      snprintf(vbuf, sizeof(vbuf), "%.1f", tripMaxSpeedKmh);
      drawStatsMidValue(VMAX_X + 8, RIDE_Y + 14, VMAX_W - 8,
                        vbuf, "km/h", TEXT_MAIN, TEXT_MUTED, GLASS_FILL);
    }
  }

  {
    const uint8_t bits =
        (vesc_online            ? 0x01 : 0) |
        (disp_secondaryOnline   ? 0x02 : 0) |
        (btBridgeActive         ? 0x04 : 0) |
        (disp_currentA < -0.5f  ? 0x08 : 0);
    if (bits != statsCache.statusBits) {
      statsCache.statusBits = bits;
      tft.fillRect(0, BAR_Y, 320, BAR_H, BG_COLOR);
      tft.drawFastHLine(MARGIN, BAR_Y, 320 - 2 * MARGIN, GLASS_EDGE);

      struct Tok { const char* label; uint16_t col; };
      const Tok toks[4] = {
        { "VESC",  (bits & 0x01) ? NEON_LIME  : TEXT_MUTED },
        { (disp_secondaryCanId >= 1) ? "ESC2" : "--",
                   (bits & 0x02) ? NEON_CYAN  : TEXT_MUTED },
        { "BT",    (bits & 0x04) ? NEON_BLUE  : TEXT_MUTED },
        { "REGEN", (bits & 0x08) ? cockpit::NEON_BLUE : TEXT_MUTED },
      };

      tft.setFreeFont(GLCD);
      tft.setTextDatum(ML_DATUM);

      const int dotW  = tft.textWidth(" . ");
      int totalW = 0;
      for (int i = 0; i < 4; ++i) {
        totalW += tft.textWidth(toks[i].label);
        if (i < 3) totalW += dotW;
      }
      int x = (320 - totalW) / 2;
      const int y = BAR_Y + BAR_H / 2 + 1;

      for (int i = 0; i < 4; ++i) {
        tft.setTextColor(toks[i].col, BG_COLOR);
        tft.drawString(toks[i].label, x, y);
        x += tft.textWidth(toks[i].label);
        if (i < 3) {
          tft.setTextColor(TEXT_MUTED, BG_COLOR);
          tft.drawString(" . ", x, y);
          x += dotW;
        }
      }
    }
  }
}

void drawSettingsStatic() {
  using namespace cockpit;
  drawCockpitHeader("SYSTEM");
  drawHomeIcon(300, 20);

  const int btnW     = TILE_W;
  const int btnH     = TILE_H;
  const int sX       = (320 - btnW) / 2;
  const int gY       = GAP_Y;
  const int sideBtnW = SIDE_W;
  const int gapX     = GAP_X;

  const bool havePin = (savedPin != "");
  const int  contentRows = havePin ? 6 : 5;
  const int  totalH = contentRows * btnH + (contentRows - 1) * gY;
  const int  topPad = 30;
  const int  bottomPad = 18;
  const int  avail = 240 - topPad - bottomPad;
  int        sY    = topPad + (avail - totalH) / 2;
  if (sY < topPad) sY = topPad;

  if (havePin) {
    CockpitTile lockTile { "LOCK", nullptr, NEON_RED, CPI_LOCK, false };
    drawCockpitTile(sX, sY, lockTile);
    sY += btnH + gY;
  }

  CockpitTile btTile {
    "VESC BT",
    btBridgeActive ? "LIVE" : nullptr,
    NEON_BLUE,
    CPI_BT,
    btBridgeActive
  };
  drawCockpitTile(sX, sY, btTile);
  sY += btnH + gY;

  CockpitTile diagTile { "VESC DIAG", nullptr, NEON_AMBER, CPI_DIAG, false };
  drawCockpitTile(sX, sY, diagTile);
  sY += btnH + gY;

  char motorChip[12];
  snprintf(motorChip, sizeof(motorChip), "%dmm",
           normalizeWheelDiaMm(wheelDiaMm));
  CockpitTile motorTile { "MOTOR", motorChip, NEON_AMBER, CPI_CHIP, false };
  drawCockpitTile(sX, sY, motorTile);
  sY += btnH + gY;

  CockpitTile recordsTile { "RECORDS", nullptr, NEON_CYAN, CPI_TROPHY, false };
  drawCockpitTile(sX, sY, recordsTile);
  sY += btnH + gY;

  drawCockpitChevron(sX - sideBtnW - gapX, sY, -1);
  CockpitTile updateTile { "OTA UPDATE", nullptr, NEON_LIME, CPI_DOWNLOAD, false };
  drawCockpitTile(sX, sY, updateTile);
  drawCockpitChevron(sX + btnW + gapX, sY, +1);

  drawCockpitPager(0, 3);
}

void drawSettingsStatic2() {
  using namespace cockpit;
  drawCockpitHeader("HARDWARE");
  drawHomeIcon(300, 20);

  const int btnW     = TILE_W;
  const int btnH     = TILE_H;
  const int sX       = (320 - btnW) / 2;
  const int sideBtnW = SIDE_W;
  const int gapX     = GAP_X;

  const bool havePin = (savedPin != "");
  const int  contentRows = havePin ? 6 : 5;
  const int  topPad = 30;
  const int  bottomPad = 18;
  const int  avail = 240 - topPad - bottomPad;
  int gY = GAP_Y;
  while (contentRows * btnH + (contentRows - 1) * gY > avail && gY > 0) --gY;
  const int totalH = contentRows * btnH + (contentRows - 1) * gY;
  int sY = topPad + (avail - totalH) / 2;
  if (sY < topPad) sY = topPad;

  const bool wifiReady = wifiCredentialsReady();
  CockpitTile wifiTile {
    "WI-FI",
    wifiReady ? "READY" : "SET",
    wifiReady ? NEON_LIME : NEON_AMBER,
    CPI_WIFI,
    wifiReady
  };
  drawCockpitTile(sX, sY, wifiTile);
  sY += btnH + gY;

  char cellsChip[8];
  snprintf(cellsChip, sizeof(cellsChip), "%dS", normalizeBatteryCells(batteryCells));
  CockpitTile cellsTile { "BATTERY", cellsChip, NEON_LIME, CPI_BATTERY, false };
  drawCockpitTile(sX, sY, cellsTile);
  sY += btnH + gY;

  CockpitTile webTile { "WEB DASH", webDashActive ? "ON" : "OFF",
                        webDashActive ? NEON_LIME : NEON_CYAN, CPI_GLOBE, webDashActive };
  drawCockpitTile(sX, sY, webTile);
  sY += btnH + gY;

  if (havePin) {
    CockpitTile startupTile {
      "STARTUP PIN",
      passAtStartup ? "ON" : "OFF",
      passAtStartup ? NEON_LIME : TEXT_MUTED,
      CPI_KEY,
      passAtStartup
    };
    drawCockpitTile(sX, sY, startupTile);
    sY += btnH + gY;
  }

  CockpitTile passTile {
    havePin ? "DELETE PIN" : "SET PIN",
    nullptr,
    havePin ? NEON_AMBER : NEON_CYAN,
    CPI_PIN,
    false
  };
  drawCockpitTile(sX, sY, passTile);
  sY += btnH + gY;

  drawCockpitChevron(sX - sideBtnW - gapX, sY, -1);
  const int tId = normalizeThemeId(currentTheme);
  CockpitTile themeTile {
    "THEME",
    tId == THEME_LIGHT ? "LIGHT" : "DARK",
    NEON_MAGENTA,
    CPI_THEME,
    false
  };
  drawCockpitTile(sX, sY, themeTile);
  drawCockpitChevron(sX + btnW + gapX, sY, +1);

  drawCockpitPager(1, 3);
}

// ============================================================
//  DRAGY
// ============================================================
enum DragyState : uint8_t { DRAGY_IDLE, DRAGY_ARMED, DRAGY_RUNNING, DRAGY_DONE };

static const float    DRAGY_ARM_SPEED    = 0.8f;
static const float    DRAGY_LAUNCH_SPEED = 3.0f;
static const float    DRAGY_ABORT_SPEED  = 0.8f;
static const uint32_t DRAGY_ARM_HOLD_MS  = 300;
static const int      DRAGY_TARGET_COUNT = 4;
static const int      dragyTargets[DRAGY_TARGET_COUNT] = { 30, 60, 90, 100 };

DragyState dragyState     = DRAGY_IDLE;
uint32_t   dragyRunStart  = 0;
uint32_t   dragyBelowArm  = 0;
uint32_t   dragyLastStop  = 0;
uint32_t   dragySplit[DRAGY_TARGET_COUNT] = {0};
uint32_t   dragyBest[DRAGY_TARGET_COUNT]  = {0};
float      dragyPeakSpeed = 0.0f;
float      dragyPrevSpeed = 0.0f;
uint32_t   dragyPrevMs    = 0;
bool       dragyBestLoaded = false;

DragyState dragyDrawnState   = (DragyState)255;
uint32_t   dragyLastDrawMs   = 0;
int        dragyDrawnElapsed = -1;
uint32_t   dragyDrawnSplit[DRAGY_TARGET_COUNT] = { 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };

void loadDragyBest() {
  if (dragyBestLoaded) return;
  dragyBestLoaded = true;
  if (!prefsReady) return;
  size_t n = prefs.getBytes("dragyBest", dragyBest, sizeof(dragyBest));
  if (n != sizeof(dragyBest)) {
    for (int i = 0; i < DRAGY_TARGET_COUNT; ++i) dragyBest[i] = 0;
  }
}

void saveDragyBest() {
  if (!prefsReady) return;
  prefs.putBytes("dragyBest", dragyBest, sizeof(dragyBest));
}

void dragyReset() {
  dragyState     = DRAGY_IDLE;
  dragyRunStart  = 0;
  dragyBelowArm  = 0;
  dragyLastStop  = 0;
  dragyPeakSpeed = 0.0f;
  for (int i = 0; i < DRAGY_TARGET_COUNT; ++i) dragySplit[i] = 0;
  dragyPrevSpeed = disp_speedKmh;
  dragyPrevMs    = millis();
}

void updateDragy() {
  const uint32_t now = millis();
  const float    spd = disp_speedKmh;

  switch (dragyState) {
    case DRAGY_IDLE:
      if (spd < DRAGY_ARM_SPEED) {
        if (dragyBelowArm == 0) dragyBelowArm = now;
        if (now - dragyBelowArm >= DRAGY_ARM_HOLD_MS) {
          dragyState     = DRAGY_ARMED;
          dragyLastStop  = now;
          dragyPeakSpeed = 0.0f;
          for (int i = 0; i < DRAGY_TARGET_COUNT; ++i) dragySplit[i] = 0;
        }
      } else {
        dragyBelowArm = 0;
      }
      break;

    case DRAGY_ARMED:
      if (spd < DRAGY_ARM_SPEED) {
        dragyLastStop = now;
      } else if (spd >= DRAGY_LAUNCH_SPEED) {
        dragyRunStart  = dragyLastStop;
        dragyPeakSpeed = spd;
        dragyState     = DRAGY_RUNNING;
      }
      break;

    case DRAGY_RUNNING: {
      if (spd > dragyPeakSpeed) dragyPeakSpeed = spd;
      for (int i = 0; i < DRAGY_TARGET_COUNT; ++i) {
        if (dragySplit[i] == 0 && spd >= dragyTargets[i] && dragyPrevSpeed < dragyTargets[i]) {
          uint32_t crossMs = now;
          const float dv = spd - dragyPrevSpeed;
          if (dv > 0.001f) {
            const float frac = ((float)dragyTargets[i] - dragyPrevSpeed) / dv;
            const float dt   = (float)(now - dragyPrevMs);
            crossMs = dragyPrevMs + (uint32_t)(frac * dt);
          }
          const uint32_t t = (crossMs > dragyRunStart) ? (crossMs - dragyRunStart) : 0;
          dragySplit[i] = t;
          if (dragyBest[i] == 0 || t < dragyBest[i]) {
            dragyBest[i] = t;
            saveDragyBest();
          }
        }
      }
      if (dragySplit[DRAGY_TARGET_COUNT - 1] != 0) {
        dragyState = DRAGY_DONE;
      }
      else if (spd < DRAGY_ABORT_SPEED) {
        dragyState    = DRAGY_IDLE;
        dragyBelowArm = now;
      }
      break;
    }

    case DRAGY_DONE:
      if (spd < DRAGY_ARM_SPEED) {
        if (dragyBelowArm == 0) dragyBelowArm = now;
        if (now - dragyBelowArm >= DRAGY_ARM_HOLD_MS) {
          dragyState     = DRAGY_ARMED;
          dragyLastStop  = now;
          dragyPeakSpeed = 0.0f;
          for (int i = 0; i < DRAGY_TARGET_COUNT; ++i) dragySplit[i] = 0;
        }
      } else {
        dragyBelowArm = 0;
      }
      break;
  }

  dragyPrevSpeed = spd;
  dragyPrevMs    = now;
}

static void dragyFmt(uint32_t ms, char* out, size_t n) {
  if (ms == 0) { snprintf(out, n, "--.--"); return; }
  snprintf(out, n, "%lu.%02lu", (unsigned long)(ms / 1000),
           (unsigned long)((ms % 1000) / 10));
}

void renderDragyDynamic(bool force) {
  using namespace cockpit;
  const uint32_t now = millis();
  if (!force && now - dragyLastDrawMs < 100) return;
  dragyLastDrawMs = now;

  uint32_t elapsed;
  if (dragyState == DRAGY_RUNNING)   elapsed = now - dragyRunStart;
  else if (dragyState == DRAGY_DONE) elapsed = dragySplit[DRAGY_TARGET_COUNT - 1];
  else                               elapsed = 0;

  const int tenths = (int)(elapsed / 10);
  if (force || tenths != dragyDrawnElapsed || dragyState != dragyDrawnState) {
    dragyDrawnElapsed = tenths;
    dragyDrawnState   = dragyState;
    char buf[12];
    snprintf(buf, sizeof(buf), "%lu.%02lu",
             (unsigned long)(elapsed / 1000),
             (unsigned long)((elapsed % 1000) / 10));
    const uint16_t tcol = (dragyState == DRAGY_DONE) ? NEON_LIME : NEON_AMBER;
    tft.fillRect(16, 37, 288, 46, BG_COLOR);
    tft.setTextFont(7);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(tcol, BG_COLOR);
    tft.drawString(buf, 160, 60);
  }

  const int rowYc0   = 124;
  const int rowPitch = 20;
  for (int i = 0; i < DRAGY_TARGET_COUNT; ++i) {
    if (!force && dragySplit[i] == dragyDrawnSplit[i]) continue;
    dragyDrawnSplit[i] = dragySplit[i];
    const int yc = rowYc0 + i * rowPitch;
    tft.fillRect(118, yc - 10, 188, 18, BG_COLOR);

    char cur[12];
    dragyFmt(dragySplit[i], cur, sizeof(cur));
    tft.setFreeFont(FF1);
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(dragySplit[i] ? NEON_LIME : TEXT_MUTED, BG_COLOR);
    tft.drawString(cur, 274, yc);
  }
}

void drawDragyStatic() {
  using namespace cockpit;
  loadDragyBest();
  drawCockpitHeader("DRAGY");
  drawHomeIcon(300, 20);

  drawCockpitPanel(10, 30, 300, 60, NEON_AMBER);
  drawCockpitPanel(10, 94, 300, 104, NEON_CYAN, "ACCELERATION (s)");

  tft.setFreeFont(FF1);
  tft.setTextColor(TEXT_MAIN, BG_COLOR);
  tft.setTextDatum(ML_DATUM);
  for (int i = 0; i < DRAGY_TARGET_COUNT; ++i) {
    char lab[10];
    snprintf(lab, sizeof(lab), "0-%d", dragyTargets[i]);
    tft.drawString(lab, 46, 124 + i * 20);
  }

  drawCockpitActionBtn(110, 206, 100, 28, "RESET", 2, CPI_WARN);

  dragyDrawnState   = (DragyState)255;
  dragyDrawnElapsed = -1;
  for (int i = 0; i < DRAGY_TARGET_COUNT; ++i) dragyDrawnSplit[i] = 0xFFFFFFFF;
  renderDragyDynamic(true);
}

void drawSettingsStatic3() {
  using namespace cockpit;
  drawCockpitHeader("TUNING");
  drawHomeIcon(300, 20);

  const int btnW     = TILE_W;
  const int btnH     = TILE_H;
  const int sX       = (320 - btnW) / 2;
  const int sideBtnW = SIDE_W;
  const int gapX     = GAP_X;

  const int topPad    = 30;
  const int bottomPad = 18;
  const int avail     = 240 - topPad - bottomPad;
  const int gY        = 10;
  const int totalH    = btnH * 5 + gY * 4;
  int       sY        = topPad + (avail - totalH) / 2;
  if (sY < topPad) sY = topPad;

  const int dragyY = sY;
  const int autoY  = sY + (btnH + gY) * 1;
  const int limitY = sY + (btnH + gY) * 2;
  const int musicY = sY + (btnH + gY) * 3;
  const int navY   = sY + (btnH + gY) * 4;

  CockpitTile dragyTile { "DRAGY", "0-100", NEON_AMBER, CPI_BOLT, false };
  drawCockpitTile(sX, dragyY, dragyTile);

  CockpitTile autoTile { "AUTO LOCK",
                         autoLockEnabled ? "ON" : "OFF",
                         autoLockEnabled ? NEON_LIME : TEXT_MUTED,
                         CPI_LOCK, autoLockEnabled };
  drawCockpitTile(sX, autoY, autoTile);

  char spdChip[8];
  snprintf(spdChip, sizeof(spdChip), "%d", normalizeSpeedLimitKmh(speedLimitKmh));
  CockpitTile limitTile { "SPEED LIMIT",
                          speedLimitEnabled ? spdChip : "OFF",
                          speedLimitEnabled ? NEON_AMBER : TEXT_MUTED,
                          CPI_BOLT, speedLimitEnabled };
  drawCockpitTile(sX, limitY, limitTile);

  CockpitTile musicTile { "MUSIC", "PLAY", NEON_MAGENTA, CPI_BOLT, false };
  drawCockpitTile(sX, musicY, musicTile);

  char canIdChip[8];
  const int canIdNorm = normalizeCanIdSetting(canId_2nd);
  if (canIdNorm <= 0) snprintf(canIdChip, sizeof(canIdChip), "OFF");
  else                snprintf(canIdChip, sizeof(canIdChip), "%d", canIdNorm);

  drawCockpitChevron(sX - sideBtnW - gapX, navY, -1);
  CockpitTile canTile { "CAN ID", canIdChip, NEON_CYAN, CPI_CAN, canIdNorm > 0 };
  drawCockpitTile(sX, navY, canTile);
  drawCockpitChevron(sX + btnW + gapX, navY, +1);

  drawCockpitPager(2, 3);
}

static TFT_eSprite popupChipSpr(&tft);
static bool        popupChipInit = false;
static const int   POPUP_CHIP_W  = 80;
static const int   POPUP_CHIP_H  = 36;

static bool ensurePopupChipSprite() {
  if (popupChipInit) return true;
  popupChipSpr.setColorDepth(16);
  if (popupChipSpr.createSprite(POPUP_CHIP_W, POPUP_CHIP_H) == nullptr) {
    return false;
  }
  popupChipInit = true;
  return true;
}
static void releasePopupChipSprite() {
  if (!popupChipInit) return;
  popupChipSpr.deleteSprite();
  popupChipInit = false;
}

static void drawPopupBigChipSprite(int cx, int cy, const char* text, uint16_t accent) {
  using namespace cockpit;
  if (!ensurePopupChipSprite()) {
    tft.fillRect(cx - POPUP_CHIP_W / 2, cy - POPUP_CHIP_H / 2,
                 POPUP_CHIP_W, POPUP_CHIP_H, BG_COLOR);
    drawCockpitBigChip(cx, cy, text, accent);
    return;
  }
  popupChipSpr.fillSprite(BG_COLOR);

  popupChipSpr.setFreeFont(FF2);
  const int textW = popupChipSpr.textWidth(text);
  const int w = textW + 20;
  const int h = 30;
  const int x = (POPUP_CHIP_W - w) / 2;
  const int y = (POPUP_CHIP_H - h) / 2;
  popupChipSpr.fillRoundRect(x, y, w, h, 6, GLASS_FILL);
  popupChipSpr.drawRoundRect(x, y, w, h, 6, accent);
  popupChipSpr.drawFastHLine(x + 4, y + 1, w - 8, GLASS_HI);
  popupChipSpr.setTextColor(accent, GLASS_FILL);
  popupChipSpr.setTextDatum(MC_DATUM);
  popupChipSpr.drawString(text, POPUP_CHIP_W / 2, POPUP_CHIP_H / 2 + 2);

  popupChipSpr.pushSprite(cx - POPUP_CHIP_W / 2, cy - POPUP_CHIP_H / 2);
}

void drawCanIdValue() {
  using namespace cockpit;
  const int canIdNorm = normalizeCanIdSetting(canId_2nd);
  char buf[8];
  if (canIdNorm < 1) snprintf(buf, sizeof(buf), "OFF");
  else               snprintf(buf, sizeof(buf), "%d", canIdNorm);
  const uint16_t accent = (canIdNorm < 1) ? TEXT_MUTED : NEON_CYAN;
  drawPopupBigChipSprite(160, 110, buf, accent);
}

void drawPopupCanId() {
  using namespace cockpit;
  drawCockpitHeader("CAN DEVICES");

  drawCockpitPanel(14, 34, 292, 158, NEON_CYAN, "2ND ESC / CAN BUS");

  tft.setFreeFont(GLCD);
  tft.setTextColor(TEXT_MUTED, BG_COLOR);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("OFF / 1 - 127", 160, 62);

  drawCockpitStepperBtn(70,  90, 50, 40, true,  NEON_CYAN);
  drawCockpitStepperBtn(200, 90, 50, 40, false, NEON_CYAN);

  drawCanIdValue();

  drawCockpitActionBtn(110, 145, 100, 35, "CANCEL", 3, CPI_CROSS);
}

namespace cells_layout {
  constexpr int STEP_W = 46, STEP_H = 36;
  constexpr int MINUS_X = 60, PLUS_X = 214;
  constexpr int CELLS_STEP_Y = 64;
  constexpr int VCAL_STEP_Y  = 128;
  constexpr int CHIP_OFFS_Y  = STEP_H / 2;
  constexpr int LIVE_X = 40, LIVE_Y = 176, LIVE_W = 240;
  constexpr int CANCEL_X = 110, CANCEL_Y = 204, CANCEL_W = 100, CANCEL_H = 28;
}

void drawCellsValue() {
  using namespace cockpit;
  using namespace cells_layout;
  char buf[8];
  snprintf(buf, sizeof(buf), "%dS", normalizeBatteryCells(batteryCells));
  drawPopupBigChipSprite(160, CELLS_STEP_Y + CHIP_OFFS_Y, buf, NEON_LIME);
}

void drawVoltCalValue() {
  using namespace cockpit;
  using namespace cells_layout;
  const int v = normalizeVoltageCalX1000(voltageCalX1000);
  char buf[12];
  snprintf(buf, sizeof(buf), "%d.%03d", v / 1000, v % 1000);
  drawPopupBigChipSprite(160, VCAL_STEP_Y + CHIP_OFFS_Y, buf, NEON_CYAN);
}

void drawPopupCells() {
  using namespace cockpit;
  using namespace cells_layout;
  drawCockpitHeader("BATTERY");

  drawCockpitPanel(14, 30, 292, 170, NEON_LIME, "PACK / CALIBRATION");

  char header[32];
  snprintf(header, sizeof(header), "CELLS  %d - %d",
           BATTERY_CELLS_MIN, BATTERY_CELLS_MAX);
  tft.setFreeFont(GLCD);
  tft.setTextColor(TEXT_MUTED, BG_COLOR);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(header, 160, 52);

  drawCockpitStepperBtn(MINUS_X, CELLS_STEP_Y, STEP_W, STEP_H, true,  NEON_LIME);
  drawCockpitStepperBtn(PLUS_X,  CELLS_STEP_Y, STEP_W, STEP_H, false, NEON_LIME);
  drawCellsValue();

  tft.setFreeFont(GLCD);
  tft.setTextColor(TEXT_MUTED, BG_COLOR);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("VOLTAGE CALIBRATION", 160, 114);

  drawCockpitStepperBtn(MINUS_X, VCAL_STEP_Y, STEP_W, STEP_H, true,  NEON_CYAN);
  drawCockpitStepperBtn(PLUS_X,  VCAL_STEP_Y, STEP_W, STEP_H, false, NEON_CYAN);
  drawVoltCalValue();

  char lv[24];
  if (vesc_online) snprintf(lv, sizeof(lv), "VESC: %.1f V", disp_voltage);
  else             snprintf(lv, sizeof(lv), "VESC: -- V");
  tft.setFreeFont(GLCD);
  tft.setTextColor(NEON_LIME, BG_COLOR);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(lv, 160, LIVE_Y);

  drawCockpitActionBtn(CANCEL_X, CANCEL_Y, CANCEL_W, CANCEL_H, "CANCEL", 3, CPI_CROSS);
}

// =============================================================================
//  Auto-lock config screen
// =============================================================================
namespace autolock_layout {
  constexpr int CARD_X = 14,  CARD_Y = 30, CARD_W = 292, CARD_H = 170;
  constexpr int TOGGLE_X = 28, TOGGLE_Y = 56, TOGGLE_W = 264, TOGGLE_H = 34;
  constexpr int STEP_MINUS_X = 40, STEP_PLUS_X = 234, STEP_Y = 118, STEP_W = 46, STEP_H = 40;
  constexpr int CHIP_CX = 160, CHIP_CY = 138;
  constexpr int STATUS_Y = 180;
  constexpr int BACK_X = 110, BACK_Y = 206, BACK_W = 100, BACK_H = 28;
}

static void autoLockFormatTimeout(char* out, size_t sz) {
  const int s = normalizeAutoLockTimeoutSec(autoLockTimeoutSec);
  if (s < 60) snprintf(out, sz, "%d s", s);
  else        snprintf(out, sz, "%d min", s / 60);
}

static void autoLockStepTimeout(int dir) {
  int idx = 0, bestDiff = 2147483647;
  const int cur = normalizeAutoLockTimeoutSec(autoLockTimeoutSec);
  for (int k = 0; k < AUTOLOCK_PRESET_COUNT; ++k) {
    int d = cur - AUTOLOCK_PRESETS[k]; if (d < 0) d = -d;
    if (d < bestDiff) { bestDiff = d; idx = k; }
  }
  idx += dir;
  if (idx < 0) idx = 0;
  if (idx >= AUTOLOCK_PRESET_COUNT) idx = AUTOLOCK_PRESET_COUNT - 1;
  autoLockTimeoutSec = AUTOLOCK_PRESETS[idx];
}

static void drawAutoLockValues() {
  using namespace cockpit;
  using namespace autolock_layout;
  char buf[16];
  autoLockFormatTimeout(buf, sizeof(buf));
  drawPopupBigChipSprite(CHIP_CX, CHIP_CY, buf, NEON_CYAN);
}

static void drawAutoLockToggleBtn() {
  using namespace autolock_layout;
  drawCockpitActionBtn(TOGGLE_X, TOGGLE_Y, TOGGLE_W, TOGGLE_H,
                       autoLockEnabled ? "AUTO-LOCK: ON" : "AUTO-LOCK: OFF",
                       autoLockEnabled ? 1 : 3, CPI_LOCK);
}

static char autoLockStatusShown[28] = "";
static void drawAutoLockStatusLive(bool force) {
  using namespace cockpit;
  using namespace autolock_layout;
  const char* txt; uint16_t col;
  if (!autoLockEnabled)       { txt = "STATUS: DISABLED";       col = TEXT_MUTED; }
  else if (isLocked)          { txt = "STATUS: DISPLAY LOCKED"; col = NEON_RED;   }
  else if (autoThrottleLock)  { txt = "STATUS: THROTTLE OFF";   col = NEON_AMBER; }
  else                        { txt = "STATUS: ARMED";          col = NEON_LIME;  }
  if (!force && strcmp(txt, autoLockStatusShown) == 0) return;
  snprintf(autoLockStatusShown, sizeof(autoLockStatusShown), "%s", txt);
  tft.fillRect(CARD_X + 6, STATUS_Y - 8, CARD_W - 12, 16, BG_COLOR);
  tft.setFreeFont(GLCD);
  tft.setTextColor(col, BG_COLOR);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(txt, 160, STATUS_Y);
}

static void drawAutoLockScreen() {
  using namespace cockpit;
  using namespace autolock_layout;
  drawCockpitHeader("AUTO LOCK");
  drawHomeIcon(300, 20);

  drawCockpitPanel(CARD_X, CARD_Y, CARD_W, CARD_H, NEON_LIME, "THROTTLE AUTO-LOCK");

  drawAutoLockToggleBtn();

  tft.setFreeFont(GLCD);
  tft.setTextColor(TEXT_MUTED, BG_COLOR);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("DISABLE THROTTLE AFTER", 160, 104);

  drawCockpitStepperBtn(STEP_MINUS_X, STEP_Y, STEP_W, STEP_H, true,  NEON_CYAN);
  drawCockpitStepperBtn(STEP_PLUS_X,  STEP_Y, STEP_W, STEP_H, false, NEON_CYAN);
  drawAutoLockValues();

  drawAutoLockStatusLive(true);

  drawCockpitActionBtn(BACK_X, BACK_Y, BACK_W, BACK_H, "BACK", 0, CPI_NONE);
}

static void speedLimitStepKmh(int dir) {
  const int cur = normalizeSpeedLimitKmh(speedLimitKmh);
  speedLimitKmh = normalizeSpeedLimitKmh(cur + dir * SPEEDLIMIT_STEP_KMH);
}

static void drawSpeedLimitValues() {
  using namespace cockpit;
  using namespace autolock_layout;
  char buf[16];
  snprintf(buf, sizeof(buf), "%d", normalizeSpeedLimitKmh(speedLimitKmh));
  drawPopupBigChipSprite(CHIP_CX, CHIP_CY, buf, NEON_AMBER);
}

static void drawSpeedLimitToggleBtn() {
  using namespace autolock_layout;
  drawCockpitActionBtn(TOGGLE_X, TOGGLE_Y, TOGGLE_W, TOGGLE_H,
                       speedLimitEnabled ? "LIMITER: ON" : "LIMITER: OFF",
                       speedLimitEnabled ? 1 : 3, CPI_BOLT);
}

static char speedLimitStatusShown[28] = "";
static void drawSpeedLimitStatusLive(bool force) {
  using namespace cockpit;
  using namespace autolock_layout;
  const char* txt; uint16_t col;
  if (!speedLimitEnabled) { txt = "STATUS: DISABLED"; col = TEXT_MUTED; }
  else                    { txt = "STATUS: ACTIVE";   col = NEON_LIME;  }
  if (!force && strcmp(txt, speedLimitStatusShown) == 0) return;
  snprintf(speedLimitStatusShown, sizeof(speedLimitStatusShown), "%s", txt);
  tft.fillRect(CARD_X + 6, STATUS_Y - 8, CARD_W - 12, 16, BG_COLOR);
  tft.setFreeFont(GLCD);
  tft.setTextColor(col, BG_COLOR);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(txt, 160, STATUS_Y);
}

static void drawSpeedLimitScreen() {
  using namespace cockpit;
  using namespace autolock_layout;
  drawCockpitHeader("SPEED LIMIT");
  drawHomeIcon(300, 20);

  drawCockpitPanel(CARD_X, CARD_Y, CARD_W, CARD_H, NEON_AMBER, "MAX SPEED LIMIT");

  drawSpeedLimitToggleBtn();

  tft.setFreeFont(GLCD);
  tft.setTextColor(TEXT_MUTED, BG_COLOR);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("LIMIT TOP SPEED TO (KM/H)", 160, 104);

  drawCockpitStepperBtn(STEP_MINUS_X, STEP_Y, STEP_W, STEP_H, true,  NEON_CYAN);
  drawCockpitStepperBtn(STEP_PLUS_X,  STEP_Y, STEP_W, STEP_H, false, NEON_CYAN);
  drawSpeedLimitValues();

  drawSpeedLimitStatusLive(true);

  drawCockpitActionBtn(BACK_X, BACK_Y, BACK_W, BACK_H, "BACK", 0, CPI_NONE);
}

// =============================================================================
//  Motor / wheel config screen
// -----------------------------------------------------------------------------
namespace motor_cfg_layout {
  constexpr int CARD_X = 14,  CARD_Y = 34;
  constexpr int CARD_W = 292, CARD_H = 100;
  constexpr int ROW_H = 24;
  constexpr int STEP_MINUS_X = 24,  STEP_PLUS_X = 246;
  constexpr int STEP_Y       = 144, STEP_W = 50, STEP_H = 40;
  constexpr int FIELD_BTN_X  = 82,  FIELD_BTN_Y = 144, FIELD_BTN_W = 156, FIELD_BTN_H = 40;
  constexpr int SAVE_X = 24,  SAVE_Y = 194, SAVE_W = 130, SAVE_H = 34;
  constexpr int CANCEL_X = 166, CANCEL_Y = 194, CANCEL_W = 130, CANCEL_H = 34;
}

static int* motorCfgDraftPtr(int field) {
  switch (field) {
    case MCF_POLES: return &motorCfgPolesDraft;
    case MCF_WHEEL: return &motorCfgWheelDraft;
    case MCF_RATIO: return &motorCfgRatioX100Draft;
  }
  return &motorCfgPolesDraft;
}

static void motorCfgDraftClamp(int field) {
  int* p = motorCfgDraftPtr(field);
  switch (field) {
    case MCF_POLES:
      if (*p < MOTOR_POLES_MIN) *p = MOTOR_POLES_MIN;
      if (*p > MOTOR_POLES_MAX) *p = MOTOR_POLES_MAX;
      if (*p & 1) ++(*p);
      if (*p > MOTOR_POLES_MAX) *p = MOTOR_POLES_MAX;
      break;
    case MCF_WHEEL:
      if (*p < WHEEL_DIA_MM_MIN) *p = WHEEL_DIA_MM_MIN;
      if (*p > WHEEL_DIA_MM_MAX) *p = WHEEL_DIA_MM_MAX;
      break;
    case MCF_RATIO:
      if (*p < DRIVE_RATIO_X100_MIN) *p = DRIVE_RATIO_X100_MIN;
      if (*p > DRIVE_RATIO_X100_MAX) *p = DRIVE_RATIO_X100_MAX;
      break;
  }
}

static void motorCfgStep(int field, int dir) {
  int* p = motorCfgDraftPtr(field);
  switch (field) {
    case MCF_POLES:  *p += dir * 2; break;
    case MCF_WHEEL:  *p += dir * 5; break;
    case MCF_RATIO:  *p += dir;     break;
  }
  motorCfgDraftClamp(field);
}

static void motorCfgFormatValue(int field, char* out, size_t sz) {
  if (!out || sz == 0) return;
  int v = *motorCfgDraftPtr(field);
  switch (field) {
    case MCF_POLES: snprintf(out, sz, "%d", v);                     break;
    case MCF_WHEEL: snprintf(out, sz, "%d mm", v);                  break;
    case MCF_RATIO: snprintf(out, sz, "%d.%02d", v / 100, v % 100); break;
    default:        snprintf(out, sz, "?");                         break;
  }
}

static const char* motorCfgLabel(int field) {
  switch (field) {
    case MCF_POLES: return "MOTOR POLES";
    case MCF_WHEEL: return "WHEEL DIAMETER";
    case MCF_RATIO: return "DRIVE RATIO";
    default:        return "?";
  }
}

static void drawMotorCfgValues() {
  using namespace cockpit;
  using namespace motor_cfg_layout;
  const int innerX = CARD_X + 8;
  const int innerY = CARD_Y + 26;
  const int innerW = CARD_W - 16;
  const int innerH = 3 * ROW_H + 2;
  tft.fillRect(innerX, innerY, innerW, innerH, BG_COLOR);

  for (int f = 0; f < 3; ++f) {
    const int rowY = innerY + f * ROW_H;
    const bool selected = (f == motorCfgField);
    const uint16_t accent = selected ? NEON_CYAN : TEXT_MUTED;

    tft.fillRoundRect(innerX, rowY + 3, 4, ROW_H - 6, 1, accent);

    tft.setFreeFont(GLCD);
    tft.setTextColor(accent, BG_COLOR);
    tft.setTextDatum(ML_DATUM);
    tft.drawString(motorCfgLabel(f), innerX + 14, rowY + ROW_H / 2);

    char buf[24];
    motorCfgFormatValue(f, buf, sizeof(buf));
    tft.setFreeFont(FF1);
    tft.setTextColor(selected ? TEXT_MAIN : TEXT_MUTED, BG_COLOR);
    tft.setTextDatum(MR_DATUM);
    tft.drawString(buf, innerX + innerW - 8, rowY + ROW_H / 2 + 1);

    if (f < 2) {
      tft.drawFastHLine(innerX + 4, rowY + ROW_H - 1, innerW - 8, GLASS_EDGE);
    }
  }
}

static void drawMotorCfgFieldBtn() {
  using namespace motor_cfg_layout;
  char fieldLabel[28];
  snprintf(fieldLabel, sizeof(fieldLabel), "FIELD: %s", motorCfgLabel(motorCfgField));
  drawCockpitActionBtn(FIELD_BTN_X, FIELD_BTN_Y, FIELD_BTN_W, FIELD_BTN_H,
                       fieldLabel, 0, CPI_CHIP);
}

static void drawMotorCfgScreen() {
  using namespace cockpit;
  using namespace motor_cfg_layout;

  drawCockpitHeader("DRIVETRAIN");
  drawHomeIcon(300, 20);

  drawCockpitPanel(CARD_X, CARD_Y, CARD_W, CARD_H, NEON_CYAN, "MOTOR / WHEEL SETUP");

  drawMotorCfgValues();

  drawCockpitStepperBtn(STEP_MINUS_X, STEP_Y, STEP_W, STEP_H, true,  NEON_CYAN);
  drawCockpitStepperBtn(STEP_PLUS_X,  STEP_Y, STEP_W, STEP_H, false, NEON_CYAN);

  drawMotorCfgFieldBtn();

  drawCockpitActionBtn(SAVE_X,   SAVE_Y,   SAVE_W,   SAVE_H,   "SAVE",   1, CPI_CHECK);
  drawCockpitActionBtn(CANCEL_X, CANCEL_Y, CANCEL_W, CANCEL_H, "CANCEL", 3, CPI_CROSS);
}

static void motorCfgLoadDrafts() {
  motorCfgPolesDraft    = normalizeMotorPoles(motorPoles);
  motorCfgWheelDraft    = normalizeWheelDiaMm(wheelDiaMm);
  motorCfgRatioX100Draft = normalizeDriveRatioX100(driveRatioX100);
  motorCfgField          = MCF_POLES;
}

static void motorCfgCommit() {
  motorPoles      = normalizeMotorPoles(motorCfgPolesDraft);
  wheelDiaMm      = normalizeWheelDiaMm(motorCfgWheelDraft);
  driveRatioX100  = normalizeDriveRatioX100(motorCfgRatioX100Draft);
  requestSettingsSave();
  if (telemetryMutex && xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
    telemetry.start_tachometer = -1;
    telemetry.trip_km          = 0.0f;
    telemetry.disp_trip_km     = 0.0f;
    telemetry.tripMaxSpeedKmh  = 0.0f;
    telemetry.tripMaxPowerW    = 0.0f;
    telemetry.tripMaxCurrentA  = 0.0f;
    xSemaphoreGive(telemetryMutex);
  }
}

// =============================================================================
//  WiFi credentials screens
// =============================================================================

static void wifiFitString(char* out, size_t outSize, const String& src, size_t maxChars) {
  if (outSize == 0) return;
  if (src.length() <= maxChars) {
    snprintf(out, outSize, "%s", src.c_str());
  } else {
    size_t cut = (maxChars >= 3) ? (maxChars - 3) : 1;
    if (cut > src.length()) cut = src.length();
    String sub = src.substring(0, cut);
    snprintf(out, outSize, "%s...", sub.c_str());
  }
}

namespace wifi_cfg_layout {
  constexpr int BTN_W    = 220;
  constexpr int BTN_H    = 36;
  constexpr int BTN_X    = (320 - BTN_W) / 2;
  constexpr int SSID_Y   = 60;
  constexpr int PASS_Y   = SSID_Y + BTN_H + 10;
  constexpr int CLEAR_Y  = PASS_Y + BTN_H + 16;
  constexpr int CLEAR_W  = 120;
  constexpr int CLEAR_X  = (320 - CLEAR_W) / 2;
  constexpr int CLEAR_H  = 32;
}

static void drawWifiCfgScreen() {
  using namespace cockpit;
  using namespace wifi_cfg_layout;

  drawCockpitHeader("WI-FI");
  drawHomeIcon(300, 20);

  const bool hasSsid = userWifiSsid.length() > 0;
  const bool hasPass = userWifiPass.length() > 0;
  const bool ready   = hasSsid && hasPass;

  const uint16_t statusColor = ready    ? NEON_LIME
                             : hasSsid  ? NEON_AMBER
                                        : TEXT_MUTED;
  const char* statusText = ready    ? "NETWORK READY"
                         : hasSsid  ? "PASSWORD MISSING"
                                    : "NETWORK NOT CONFIGURED";
  drawCockpitStatusDot(32, 44, statusColor, statusText);

  tft.fillRoundRect(BTN_X, SSID_Y, BTN_W, BTN_H, 8, GLASS_FILL);
  tft.drawRoundRect(BTN_X, SSID_Y, BTN_W, BTN_H, 8, hasSsid ? NEON_CYAN : GLASS_EDGE);
  tft.drawFastHLine(BTN_X + 4, SSID_Y + 1, BTN_W - 8, GLASS_HI);
  tft.fillRoundRect(BTN_X + 2, SSID_Y + 3, 3, BTN_H - 6, 1,
                    hasSsid ? NEON_CYAN : TEXT_MUTED);
  drawCockpitIcon(BTN_X + 18, SSID_Y + BTN_H / 2, CPI_WIFI,
                  hasSsid ? NEON_CYAN : TEXT_MUTED);
  tft.setFreeFont(GLCD);
  tft.setTextColor(TEXT_MUTED, GLASS_FILL);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("SSID", BTN_X + 32, SSID_Y + 6);
  tft.setFreeFont(FF1);
  tft.setTextColor(hasSsid ? TEXT_MAIN : TEXT_MUTED, GLASS_FILL);
  tft.setTextDatum(BL_DATUM);
  char buf[40];
  if (hasSsid) wifiFitString(buf, sizeof(buf), userWifiSsid, 16);
  else         snprintf(buf, sizeof(buf), "tap to enter");
  tft.drawString(buf, BTN_X + 32, SSID_Y + BTN_H - 6);

  tft.fillRoundRect(BTN_X, PASS_Y, BTN_W, BTN_H, 8, GLASS_FILL);
  tft.drawRoundRect(BTN_X, PASS_Y, BTN_W, BTN_H, 8, hasPass ? NEON_CYAN : GLASS_EDGE);
  tft.drawFastHLine(BTN_X + 4, PASS_Y + 1, BTN_W - 8, GLASS_HI);
  tft.fillRoundRect(BTN_X + 2, PASS_Y + 3, 3, BTN_H - 6, 1,
                    hasPass ? NEON_CYAN : TEXT_MUTED);
  drawCockpitIcon(BTN_X + 18, PASS_Y + BTN_H / 2, CPI_LOCK,
                  hasPass ? NEON_CYAN : TEXT_MUTED);
  tft.setFreeFont(GLCD);
  tft.setTextColor(TEXT_MUTED, GLASS_FILL);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("PASSWORD", BTN_X + 32, PASS_Y + 6);
  tft.setFreeFont(FF1);
  tft.setTextColor(hasPass ? TEXT_MAIN : TEXT_MUTED, GLASS_FILL);
  tft.setTextDatum(BL_DATUM);
  if (hasPass) {
    size_t n = userWifiPass.length();
    if (n > 16) n = 16;
    char stars[20];
    for (size_t i = 0; i < n; ++i) stars[i] = '*';
    stars[n] = '\0';
    snprintf(buf, sizeof(buf), "%s", stars);
  } else {
    snprintf(buf, sizeof(buf), "tap to enter");
  }
  tft.drawString(buf, BTN_X + 32, PASS_Y + BTN_H - 6);

  drawCockpitActionBtn(CLEAR_X, CLEAR_Y, CLEAR_W, CLEAR_H, "CLEAR", 2, CPI_CROSS);

  tft.setFreeFont(GLCD);
  tft.setTextColor(TEXT_MUTED, BG_COLOR);
  tft.setTextDatum(BC_DATUM);
  tft.drawString("OTA update uses this network", 160, 232);
}

// =============================================================================
//  On-screen QWERTY keyboard
// =============================================================================
namespace kb_layout {
  constexpr int ROWS     = 4;
  constexpr int COLS     = 10;
  constexpr int KEY_W    = 30;
  constexpr int KEY_H    = 28;
  constexpr int GAP_X    = 2;
  constexpr int GAP_Y    = 3;
  constexpr int AREA_W   = COLS * KEY_W + (COLS - 1) * GAP_X;
  constexpr int ORIGIN_X = (320 - AREA_W) / 2;
  constexpr int ORIGIN_Y = 56;
  constexpr int INPUT_X  = 8;
  constexpr int INPUT_Y  = 26;
  constexpr int INPUT_W  = 304;
  constexpr int INPUT_H  = 24;
  constexpr int ACT_Y = ORIGIN_Y + ROWS * KEY_H + (ROWS - 1) * GAP_Y + 6;
  constexpr int ACT_H = 28;
}

static const char* const kKbRowLower[kb_layout::ROWS] = {
  "1234567890",
  "qwertyuiop",
  "asdfghjkl-",
  "zxcvbnm_.!",
};
static const char* const kKbRowUpper[kb_layout::ROWS] = {
  "1234567890",
  "QWERTYUIOP",
  "ASDFGHJKL-",
  "ZXCVBNM_.!",
};
static const char* const kKbRowSymbols[kb_layout::ROWS] = {
  "1234567890",
  "!@#$%^&*()",
  "+=/\\:;\"\'<>",
  "?,.~`|{}[]",
};

static const char* const* activeKbLayout() {
  if (kbSymbols) return kKbRowSymbols;
  return kbShift ? kKbRowUpper : kKbRowLower;
}

static char kbKeyAt(int row, int col) {
  const char* const* L = activeKbLayout();
  if (row < 0 || row >= kb_layout::ROWS) return 0;
  const char* s = L[row];
  if (col < 0 || col >= (int)strlen(s)) return 0;
  return s[col];
}

static size_t kbMaxLen() {
  return (kbTarget == KB_TARGET_SSID) ? WIFI_SSID_MAX : WIFI_PASS_MAX;
}

static void drawKeyboardInputField() {
  using namespace kb_layout;
  using namespace cockpit;

  tft.fillRoundRect(INPUT_X, INPUT_Y, INPUT_W, INPUT_H, 5, GLASS_FILL);
  tft.drawRoundRect(INPUT_X, INPUT_Y, INPUT_W, INPUT_H, 5, NEON_CYAN);
  tft.drawFastHLine(INPUT_X + 4, INPUT_Y + 1, INPUT_W - 8, GLASS_HI);

  tft.setFreeFont(FF1);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(NEON_CYAN, GLASS_FILL);

  char buf[48];
  if (kbTarget == KB_TARGET_PASS) {
    size_t n = kbBuffer.length();
    size_t show = n > 28 ? 28 : n;
    for (size_t i = 0; i < show; ++i) buf[i] = '*';
    buf[show] = '\0';
  } else {
    wifiFitString(buf, sizeof(buf), kbBuffer, 28);
  }
  tft.drawString(buf, INPUT_X + 10, INPUT_Y + INPUT_H / 2 + 1);

  const int cx = INPUT_X + 10 + tft.textWidth(buf) + 2;
  if (cx < INPUT_X + INPUT_W - 6) {
    tft.fillRect(cx, INPUT_Y + 4, 2, INPUT_H - 8, NEON_CYAN);
  }
}

static void drawKeyboardScreen() {
  using namespace kb_layout;

  drawCockpitHeader(kbTarget == KB_TARGET_SSID ? "WI-FI / SSID" : "WI-FI / PASSWORD");

  drawKeyboardInputField();

  tft.setFreeFont(FF1);
  tft.setTextDatum(MC_DATUM);
  for (int r = 0; r < ROWS; ++r) {
    for (int c = 0; c < COLS; ++c) {
      char ch = kbKeyAt(r, c);
      if (ch == 0) continue;
      int kx = ORIGIN_X + c * (KEY_W + GAP_X);
      int ky = ORIGIN_Y + r * (KEY_H + GAP_Y);
      tft.fillRoundRect(kx, ky, KEY_W, KEY_H, 4, cockpit::GLASS_FILL);
      tft.drawRoundRect(kx, ky, KEY_W, KEY_H, 4, cockpit::GLASS_EDGE);
      tft.drawFastHLine(kx + 2, ky + 1, KEY_W - 4, cockpit::GLASS_HI);
      char tmp[2] = { ch, 0 };
      tft.setTextColor(cockpit::TEXT_MAIN, cockpit::GLASS_FILL);
      tft.drawString(tmp, kx + KEY_W / 2, ky + KEY_H / 2 + 1);
    }
  }

  const int y = ACT_Y;
  const int h = ACT_H;

  {
    const uint16_t accent = kbShift ? cockpit::NEON_LIME : cockpit::TEXT_MUTED;
    tft.fillRoundRect(6, y, 46, h, 5, cockpit::GLASS_FILL);
    tft.drawRoundRect(6, y, 46, h, 5, accent);
    tft.drawFastHLine(10, y + 1, 38, cockpit::GLASS_HI);
    tft.setTextColor(accent, cockpit::GLASS_FILL);
    tft.drawString("SHIFT", 6 + 23, y + h / 2 + 1);
    if (kbShift) tft.fillSmoothCircle(10, y + 5, 2, cockpit::NEON_LIME, cockpit::GLASS_FILL);
  }

  {
    const uint16_t accent = kbSymbols ? cockpit::NEON_LIME : cockpit::TEXT_MUTED;
    tft.fillRoundRect(56, y, 40, h, 5, cockpit::GLASS_FILL);
    tft.drawRoundRect(56, y, 40, h, 5, accent);
    tft.drawFastHLine(60, y + 1, 32, cockpit::GLASS_HI);
    tft.setTextColor(accent, cockpit::GLASS_FILL);
    tft.drawString("?!", 56 + 20, y + h / 2 + 1);
    if (kbSymbols) tft.fillSmoothCircle(60, y + 5, 2, cockpit::NEON_LIME, cockpit::GLASS_FILL);
  }

  tft.fillRoundRect(100, y, 80, h, 5, cockpit::GLASS_FILL);
  tft.drawRoundRect(100, y, 80, h, 5, cockpit::NEON_CYAN);
  tft.drawFastHLine(104, y + 1, 72, cockpit::GLASS_HI);
  tft.fillRect(112, y + h - 7, 56, 2, cockpit::NEON_CYAN);

  tft.fillRoundRect(184, y, 40, h, 5, cockpit::GLASS_FILL);
  tft.drawRoundRect(184, y, 40, h, 5, cockpit::TEXT_MUTED);
  tft.drawFastHLine(188, y + 1, 32, cockpit::GLASS_HI);
  tft.setTextColor(cockpit::TEXT_MAIN, cockpit::GLASS_FILL);
  tft.drawString("<", 184 + 20, y + h / 2 + 1);

  tft.fillRoundRect(228, y, 40, h, 5, cockpit::GLASS_FILL);
  tft.drawRoundRect(228, y, 40, h, 5, cockpit::NEON_LIME);
  tft.drawFastHLine(232, y + 1, 32, cockpit::GLASS_HI);
  drawCockpitIcon(228 + 20, y + h / 2, CPI_CHECK, cockpit::NEON_LIME);

  tft.fillRoundRect(272, y, 42, h, 5, cockpit::GLASS_FILL);
  tft.drawRoundRect(272, y, 42, h, 5, cockpit::NEON_RED);
  tft.drawFastHLine(276, y + 1, 34, cockpit::GLASS_HI);
  drawCockpitIcon(272 + 21, y + h / 2, CPI_CROSS, cockpit::NEON_RED);
}

// =============================================================================
//  BT-bridge screen
// =============================================================================
namespace bt_layout {
  constexpr int EXIT_X = 60;
  constexpr int EXIT_Y = 195;
  constexpr int EXIT_W = 200;
  constexpr int EXIT_H = 34;

  constexpr int STATUS_Y = 162;
}

namespace web_layout {
  constexpr int BTN_Y  = 195;
  constexpr int BTN_H  = 34;
  constexpr int BACK_X = 14;
  constexpr int BACK_W = 292;
}

static void drawBluetoothGlyph(int cx, int cy, int size, uint16_t fg, uint16_t bg) {
  const float H  = size * 0.5f;
  const float W  = size * 0.30f;
  const float hh = H * 0.5f;

  const float Tx  = cx,     Ty  = cy - H;
  const float Bx  = cx,     By  = cy + H;
  const float RUx = cx + W, RUy = cy - hh;
  const float RLx = cx + W, RLy = cy + hh;
  const float LUx = cx - W, LUy = cy - hh;
  const float LLx = cx - W, LLy = cy + hh;

  float t = size / 8.0f;
  if (t < 2.0f) t = 2.0f;

  tft.drawWideLine(Tx, Ty, Bx, By, t, fg, bg);
  tft.drawWideLine(Tx, Ty, RUx, RUy, t, fg, bg);
  tft.drawWideLine(Bx, By, RLx, RLy, t, fg, bg);
  tft.drawWideLine(RUx, RUy, LLx, LLy, t, fg, bg);
  tft.drawWideLine(RLx, RLy, LUx, LUy, t, fg, bg);
}

static void drawBtBridgeScreen() {
  using namespace cockpit;
  using namespace bt_layout;

  drawCockpitHeader("BLUETOOTH");

  const bool connected = btClientConnected;
  const uint16_t accent = connected ? NEON_LIME : NEON_BLUE;

  drawCockpitPanel(14, 36, 292, 150, accent, "VESC TOOL BRIDGE");

  const int bcx = 160;
  const int bcy = 92;
  const int rIn  = 22;
  const int rMid = 26;
  const int rOut = 32;

  tft.fillCircle(bcx, bcy, rIn, GLASS_FILL);
  tft.drawCircle(bcx, bcy, rMid, accent);
  if (connected) {
    tft.drawCircle(bcx, bcy, rOut,     accent);
    tft.drawCircle(bcx, bcy, rOut - 1, accent);
  } else {
    tft.drawSmoothArc(bcx, bcy, rOut, rOut - 1, 330, 30,  accent, BG_COLOR);
    tft.drawSmoothArc(bcx, bcy, rOut, rOut - 1, 90,  150, accent, BG_COLOR);
    tft.drawSmoothArc(bcx, bcy, rOut, rOut - 1, 210, 270, accent, BG_COLOR);
  }

  drawBluetoothGlyph(bcx, bcy, 30, accent, GLASS_FILL);

  tft.setFreeFont(FF1);
  tft.setTextColor(TEXT_MAIN, BG_COLOR);
  tft.setTextDatum(TC_DATUM);
  tft.drawString(kBtDeviceName, 160, 138);

  tft.setFreeFont(GLCD);
  tft.setTextColor(TEXT_MUTED, BG_COLOR);
  tft.drawString("Tap Scan BLE in VESC Tool", 160, 160);

  const int statusY = 178;
  const char* statusLabel = connected ? "CONNECTED" : "WAITING FOR CLIENT";
  tft.setFreeFont(GLCD);
  const int textW = tft.textWidth(statusLabel);
  const int totalW = 6 + 4 + textW;
  const int dotX = (320 - totalW) / 2 + 3;
  drawCockpitStatusDot(dotX, statusY, accent, statusLabel);

  drawCockpitActionBtn(EXIT_X, EXIT_Y, EXIT_W, EXIT_H, "EXIT BRIDGE", 2, CPI_CROSS);
}

// ----------------------------------------------------------------------------
// OTA marquee progress bar
// -----------------------------------------------------------------------------
namespace ota_marquee {
  constexpr int BAR_X   = 44;
  constexpr int BAR_Y   = 108;
  constexpr int BAR_W   = 232;
  constexpr int BAR_H   = 4;
  constexpr int SEG_W   = 48;
  constexpr int STEP_PX = 3;
  constexpr unsigned long FRAME_MS = 32;
  constexpr int CYCLE_W = BAR_W + SEG_W;
}

static int           otaMarqueeOffset    = 0;
static unsigned long otaMarqueeLastFrame = 0;
static bool          otaMarqueeActive    = false;

void resetOtaMarqueeBar() {
  otaMarqueeOffset    = 0;
  otaMarqueeLastFrame = 0;
  otaMarqueeActive    = true;
}

static void updateOtaMarqueeBar() {
  using namespace cockpit;
  using namespace ota_marquee;

  if (currentScreen != SCREEN_OTA) {
    otaMarqueeActive = false;
    return;
  }
  if (!(otaActive || otaConnectState == OTA_STATE_CONNECTING)) {
    otaMarqueeActive = false;
    return;
  }
  if (!otaMarqueeActive) return;

  const unsigned long now = millis();
  if (now - otaMarqueeLastFrame < FRAME_MS) return;
  otaMarqueeLastFrame = now;

  const uint16_t accent =
    otaActive                                ? NEON_LIME  :
    otaConnectState == OTA_STATE_CONNECTING  ? NEON_AMBER :
                                                NEON_CYAN;

  const int innerX = BAR_X + 1;
  const int innerY = BAR_Y + 1;
  const int innerW = BAR_W - 2;
  const int innerH = BAR_H - 2;

  const int prev = otaMarqueeOffset;
  int next = prev + STEP_PX;
  if (next >= CYCLE_W) next -= CYCLE_W;
  otaMarqueeOffset = next;

  auto drawSeg = [&](int offset, uint16_t col) {
    int head = offset;
    int tail = offset - SEG_W;
    if (tail < 0) tail = 0;
    if (head > innerW) head = innerW;
    if (head <= tail) return;
    tft.fillRect(innerX + tail, innerY, head - tail, innerH, col);
  };

  drawSeg(prev, BG_COLOR);
  drawSeg(next, accent);
}

void drawPopupConfirmUpdate() {
  using namespace cockpit;
  drawCockpitHeader("OTA UPDATE");

  const uint16_t accent =
    otaConnectState == OTA_STATE_FAILED       ? NEON_RED  :
    otaActive                                  ? NEON_LIME :
    otaConnectState == OTA_STATE_CONNECTING   ? NEON_AMBER :
                                                  NEON_CYAN;

  drawCockpitPanel(20, 36, 280, 152, accent, "FIRMWARE / OVER-THE-AIR");

  const int iconX = 44, iconY = 74;
  if (otaActive) {
    drawCockpitIcon(iconX, iconY, CPI_DOWNLOAD, NEON_LIME);
  } else if (otaConnectState == OTA_STATE_CONNECTING) {
    drawCockpitIcon(iconX, iconY, CPI_WIFI, NEON_AMBER);
  } else if (otaConnectState == OTA_STATE_FAILED) {
    drawCockpitIcon(iconX, iconY, CPI_WARN, NEON_RED);
  } else {
    drawCockpitIcon(iconX, iconY, CPI_SHIELD, NEON_CYAN);
  }

  tft.setFreeFont(FF1);
  tft.setTextColor(TEXT_MAIN, BG_COLOR);
  tft.setTextDatum(ML_DATUM);
  const char* stateTitle =
    otaActive                                ? "WAITING FOR UPDATE"     :
    otaConnectState == OTA_STATE_CONNECTING  ? "CONNECTING TO NETWORK"  :
    otaConnectState == OTA_STATE_FAILED      ? "CONNECTION FAILED"      :
                                                "START OTA UPDATE?";
  tft.drawString(stateTitle, 62, iconY - 2);

  tft.setFreeFont(GLCD);
  tft.setTextDatum(ML_DATUM);
  if (otaActive) {
    tft.setTextColor(NEON_LIME, BG_COLOR);
    tft.drawString(WiFi.localIP().toString().c_str(), 62, iconY + 14);
  } else if (otaConnectState == OTA_STATE_CONNECTING) {
    tft.setTextColor(NEON_AMBER, BG_COLOR);
    tft.drawString(userWifiSsid.c_str(), 62, iconY + 14);
  } else if (otaConnectState == OTA_STATE_FAILED) {
    tft.setTextColor(NEON_RED, BG_COLOR);
    tft.drawString("check SSID / password", 62, iconY + 14);
  } else {
    tft.setTextColor(TEXT_MUTED, BG_COLOR);
    tft.drawString(userWifiSsid.length() > 0 ? userWifiSsid.c_str()
                                             : "no SSID set",
                   62, iconY + 14);
  }

  if (otaActive || otaConnectState == OTA_STATE_CONNECTING) {
    const int barX = 44, barY = 108, barW = 232, barH = 4;
    tft.drawRoundRect(barX, barY, barW, barH, 2, GLASS_EDGE);
    resetOtaMarqueeBar();
  }

  if (otaActive) {
    drawCockpitActionBtn(106, 135, 108, 35, "STOP", 2, CPI_CROSS);
  } else if (otaConnectState == OTA_STATE_CONNECTING ||
             otaConnectState == OTA_STATE_FAILED) {
    drawCockpitActionBtn(106, 135, 108, 35, "CANCEL", 3, CPI_CROSS);
  } else {
    drawCockpitActionBtn(46,  135, 108, 35, "GO",     1, CPI_CHECK);
    drawCockpitActionBtn(166, 135, 108, 35, "CANCEL", 3, CPI_CROSS);
  }
}

static void refreshPopupConfirmUpdate() {
  using namespace cockpit;
  if (currentScreen != SCREEN_OTA) return;

  const uint16_t accent =
    otaConnectState == OTA_STATE_FAILED       ? NEON_RED  :
    otaActive                                  ? NEON_LIME :
    otaConnectState == OTA_STATE_CONNECTING   ? NEON_AMBER :
                                                  NEON_CYAN;

  const int px = 20, py = 36, pw = 280, ph = 152;
  tft.drawRoundRect(px,     py,     pw,     ph,     10, accent);
  tft.drawRoundRect(px + 1, py + 1, pw - 2, ph - 2, 9,  GLASS_EDGE);
  tft.drawFastHLine(px + 6, py + 2, pw - 12, GLASS_HI);
  tft.fillRect(px + 8, py + 8, 3, 10, accent);
  tft.setFreeFont(GLCD);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(accent, BG_COLOR);
  tft.fillRect(px + 16, py + 8, pw - 24, 10, BG_COLOR);
  tft.drawString("FIRMWARE / OVER-THE-AIR", px + 16, py + 8);

  const int iconX = 44, iconY = 74;
  const int blockX = px + 4;
  const int blockY = iconY - 14;
  const int blockW = pw - 8;
  const int blockH = 40;
  tft.fillRect(blockX, blockY, blockW, blockH, BG_COLOR);

  if (otaActive) {
    drawCockpitIcon(iconX, iconY, CPI_DOWNLOAD, NEON_LIME);
  } else if (otaConnectState == OTA_STATE_CONNECTING) {
    drawCockpitIcon(iconX, iconY, CPI_WIFI, NEON_AMBER);
  } else if (otaConnectState == OTA_STATE_FAILED) {
    drawCockpitIcon(iconX, iconY, CPI_WARN, NEON_RED);
  } else {
    drawCockpitIcon(iconX, iconY, CPI_SHIELD, NEON_CYAN);
  }

  tft.setFreeFont(FF1);
  tft.setTextColor(TEXT_MAIN, BG_COLOR);
  tft.setTextDatum(ML_DATUM);
  const char* stateTitle =
    otaActive                                ? "WAITING FOR UPDATE"     :
    otaConnectState == OTA_STATE_CONNECTING  ? "CONNECTING TO NETWORK"  :
    otaConnectState == OTA_STATE_FAILED      ? "CONNECTION FAILED"      :
                                                "START OTA UPDATE?";
  tft.drawString(stateTitle, 62, iconY - 2);

  tft.setFreeFont(GLCD);
  tft.setTextDatum(ML_DATUM);
  if (otaActive) {
    tft.setTextColor(NEON_LIME, BG_COLOR);
    tft.drawString(WiFi.localIP().toString().c_str(), 62, iconY + 14);
  } else if (otaConnectState == OTA_STATE_CONNECTING) {
    tft.setTextColor(NEON_AMBER, BG_COLOR);
    tft.drawString(userWifiSsid.c_str(), 62, iconY + 14);
  } else if (otaConnectState == OTA_STATE_FAILED) {
    tft.setTextColor(NEON_RED, BG_COLOR);
    tft.drawString("check SSID / password", 62, iconY + 14);
  } else {
    tft.setTextColor(TEXT_MUTED, BG_COLOR);
    tft.drawString(userWifiSsid.length() > 0 ? userWifiSsid.c_str()
                                             : "no SSID set",
                   62, iconY + 14);
  }

  if (otaActive || otaConnectState == OTA_STATE_CONNECTING) {
    const int barX = 44, barY = 108, barW = 232, barH = 4;
    tft.fillRect(barX + 1, barY + 1, barW - 2, barH - 2, BG_COLOR);
    tft.drawRoundRect(barX, barY, barW, barH, 2, GLASS_EDGE);
    resetOtaMarqueeBar();
  }

  const int btnAreaY = 135;
  const int btnAreaH = 35;
  tft.fillRect(px + 4, btnAreaY - 2, pw - 8, btnAreaH + 4, BG_COLOR);
  if (otaActive) {
    drawCockpitActionBtn(106, btnAreaY, 108, btnAreaH, "STOP", 2, CPI_CROSS);
  } else if (otaConnectState == OTA_STATE_CONNECTING ||
             otaConnectState == OTA_STATE_FAILED) {
    drawCockpitActionBtn(106, btnAreaY, 108, btnAreaH, "CANCEL", 3, CPI_CROSS);
  } else {
    drawCockpitActionBtn(46,  btnAreaY, 108, btnAreaH, "GO",     1, CPI_CHECK);
    drawCockpitActionBtn(166, btnAreaY, 108, btnAreaH, "CANCEL", 3, CPI_CROSS);
  }
}

void drawPopupDiagStatic() {
  using namespace cockpit;
  drawCockpitHeader("DIAGNOSTICS");

  drawCockpitPanel(14, 30, 292, 178, NEON_AMBER, "VESC / THERMAL & FAULT");

  tft.setFreeFont(GLCD);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(NEON_CYAN, BG_COLOR);
  tft.drawString("ESC 1", 140, 62);
  tft.setTextColor(NEON_MAGENTA, BG_COLOR);
  tft.drawString("ESC 2", 232, 62);

  drawCockpitIcon(32, 96,  CPI_THERMO, NEON_AMBER);
  drawCockpitIcon(32, 124, CPI_CHIP,   NEON_AMBER);
  drawCockpitIcon(32, 152, CPI_WARN,   NEON_AMBER);

  tft.setFreeFont(GLCD);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TEXT_MUTED, BG_COLOR);
  tft.drawString("MOTOR",  46, 96);
  tft.drawString("MOSFET", 46, 124);
  tft.drawString("FAULT",  46, 152);

  tft.drawFastHLine(24, 108, 268, GLASS_EDGE);
  tft.drawFastHLine(24, 136, 268, GLASS_EDGE);

  drawCockpitActionBtn(110, 175, 100, 26, "CLOSE", 3, CPI_CROSS);

  diagStaticDrawn = true;
}

static TFT_eSprite diagCellSpr(&tft);
static bool        diagCellInit = false;
static const int   DIAG_CELL_W  = 80;
static const int   DIAG_CELL_H  = 20;

static bool ensureDiagCellSprite() {
  if (diagCellInit) return true;
  diagCellSpr.setColorDepth(16);
  if (diagCellSpr.createSprite(DIAG_CELL_W, DIAG_CELL_H) == nullptr) {
    return false;
  }
  diagCellInit = true;
  return true;
}

static void drawDiagCell(int cx, int y, int w, const char* text) {
  (void)w;
  if (ensureDiagCellSprite()) {
    diagCellSpr.fillSprite(BG_COLOR);
    diagCellSpr.setFreeFont(FF1);
    diagCellSpr.setTextColor(cockpit::TEXT_MAIN, BG_COLOR);
    diagCellSpr.setTextDatum(MC_DATUM);
    diagCellSpr.drawString(text, DIAG_CELL_W / 2, DIAG_CELL_H / 2);
    diagCellSpr.pushSprite(cx - DIAG_CELL_W / 2, y - DIAG_CELL_H / 2);
    return;
  }
  tft.fillRect(cx - DIAG_CELL_W / 2, y - DIAG_CELL_H / 2, DIAG_CELL_W, DIAG_CELL_H, BG_COLOR);
  tft.setFreeFont(FF1);
  tft.setTextColor(cockpit::TEXT_MAIN, BG_COLOR);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(text, cx, y);
}

void updatePopupDiagValues(bool force = false) {
  char esc2Header[24];
  if (disp_secondaryCanId >= 1) {
    snprintf(esc2Header, sizeof(esc2Header), "ID %d", disp_secondaryCanId);
  } else {
    snprintf(esc2Header, sizeof(esc2Header), "OFF");
  }

  const int colX1 = 140;
  const int colX2 = 232;
  const int cellW = 80;

  if (force || strcmp(diagCacheEsc2Header, esc2Header) != 0) {
    strncpy(diagCacheEsc2Header, esc2Header, sizeof(diagCacheEsc2Header) - 1);
    diagCacheEsc2Header[sizeof(diagCacheEsc2Header) - 1] = '\0';
    tft.fillRect(196, 70, 72, 14, BG_COLOR);
    tft.setTextColor(cockpit::NEON_MAGENTA, BG_COLOR);
    tft.setTextDatum(MC_DATUM);
    tft.setFreeFont(GLCD);
    tft.drawString(diagCacheEsc2Header, 232, 76);
  }

  char buf[24];

  formatTempText(disp_primaryTempMotor, buf, sizeof(buf));
  if (force || strcmp(diagCachePrimaryMotor, buf) != 0) {
    strncpy(diagCachePrimaryMotor, buf, sizeof(diagCachePrimaryMotor) - 1);
    diagCachePrimaryMotor[sizeof(diagCachePrimaryMotor) - 1] = '\0';
    drawDiagCell(colX1, 96, cellW, diagCachePrimaryMotor);
  }

  formatTempText(disp_secondaryTempMotor, buf, sizeof(buf));
  if (force || strcmp(diagCacheSecondaryMotor, buf) != 0) {
    strncpy(diagCacheSecondaryMotor, buf, sizeof(diagCacheSecondaryMotor) - 1);
    diagCacheSecondaryMotor[sizeof(diagCacheSecondaryMotor) - 1] = '\0';
    drawDiagCell(colX2, 96, cellW, diagCacheSecondaryMotor);
  }

  formatTempText(disp_primaryTempMosfet, buf, sizeof(buf));
  if (force || strcmp(diagCachePrimaryFet, buf) != 0) {
    strncpy(diagCachePrimaryFet, buf, sizeof(diagCachePrimaryFet) - 1);
    diagCachePrimaryFet[sizeof(diagCachePrimaryFet) - 1] = '\0';
    drawDiagCell(colX1, 124, cellW, diagCachePrimaryFet);
  }

  formatTempText(disp_secondaryTempMosfet, buf, sizeof(buf));
  if (force || strcmp(diagCacheSecondaryFet, buf) != 0) {
    strncpy(diagCacheSecondaryFet, buf, sizeof(diagCacheSecondaryFet) - 1);
    diagCacheSecondaryFet[sizeof(diagCacheSecondaryFet) - 1] = '\0';
    drawDiagCell(colX2, 124, cellW, diagCacheSecondaryFet);
  }

  formatFaultText(disp_primaryFault, vesc_online, true, buf, sizeof(buf));
  if (force || strcmp(diagCachePrimaryFault, buf) != 0) {
    strncpy(diagCachePrimaryFault, buf, sizeof(diagCachePrimaryFault) - 1);
    diagCachePrimaryFault[sizeof(diagCachePrimaryFault) - 1] = '\0';
    drawDiagCell(colX1, 152, cellW, diagCachePrimaryFault);
  }

  formatFaultText(disp_secondaryFault, disp_secondaryOnline, disp_secondaryCanId >= 1,
                  buf, sizeof(buf));
  if (force || strcmp(diagCacheSecondaryFault, buf) != 0) {
    strncpy(diagCacheSecondaryFault, buf, sizeof(diagCacheSecondaryFault) - 1);
    diagCacheSecondaryFault[sizeof(diagCacheSecondaryFault) - 1] = '\0';
    drawDiagCell(colX2, 152, cellW, diagCacheSecondaryFault);
  }
}

void drawPopupConfirmLock() {
  using namespace cockpit;
  drawCockpitHeader("SECURITY");

  drawCockpitPanel(20, 44, 280, 140, NEON_RED, "LOCK SYSTEM?");

  drawCockpitIcon(50, 98, CPI_LOCK, NEON_RED);
  tft.setFreeFont(FF1);
  tft.setTextColor(TEXT_MAIN, BG_COLOR);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("Lock this dashboard", 72, 94);
  tft.setFreeFont(GLCD);
  tft.setTextColor(TEXT_MUTED, BG_COLOR);
  tft.drawString("PIN will be required to unlock", 72, 108);

  drawCockpitActionBtn(46,  128, 108, 44, "YES",    2, CPI_CHECK);
  drawCockpitActionBtn(166, 128, 108, 44, "CANCEL", 3, CPI_CROSS);
}

void drawNumpad() {
  using namespace cockpit;
  const char* headText;
  const char* section;
  switch (numpadMode) {
    case NUMPAD_NEW_PIN:       headText = "SET NEW PIN";        section = "SECURITY / NEW PIN"; break;
    case NUMPAD_CONFIRM_RESET: headText = "PIN TO RESET DATA";  section = "SECURITY / CONFIRM"; break;
    case NUMPAD_UNLOCK:        headText = "UNLOCK DASHBOARD";   section = "SECURITY / UNLOCK";  break;
    default:                   headText = "ENTER PIN";           section = "SECURITY / VERIFY";  break;
  }
  drawCockpitHeader(section);

  const bool lockoutActive = (numpadMode == NUMPAD_UNLOCK) && (unlockLockoutUntil > millis());
  const unsigned long remMs = lockoutActive ? (unlockLockoutUntil - millis()) : 0;

  tft.fillRect(20, 30, 280, 16, BG_COLOR);
  tft.setFreeFont(GLCD);
  tft.setTextDatum(MC_DATUM);
  if (lockoutActive) {
    char buf[40];
    const unsigned long remSec = (remMs + 999) / 1000;
    snprintf(buf, sizeof(buf), "LOCKED - TRY AGAIN IN %lus", remSec);
    tft.setTextColor(NEON_RED, BG_COLOR);
    tft.drawString(buf, 160, 38);
  } else {
    tft.setTextColor(isLocked ? NEON_RED : TEXT_MUTED, BG_COLOR);
    tft.drawString(headText, 160, 38);
  }

  const int fieldX = 110, fieldY = 46, fieldW = 100, fieldH = 22;
  const uint16_t fieldBorder = lockoutActive ? NEON_RED : NEON_CYAN;
  tft.fillRoundRect(fieldX, fieldY, fieldW, fieldH, 4, GLASS_FILL);
  tft.drawRoundRect(fieldX, fieldY, fieldW, fieldH, 4, fieldBorder);
  tft.drawFastHLine(fieldX + 4, fieldY + 1, fieldW - 8, GLASS_HI);
  numpadInputAreaInvalidated = true;
  drawNumpadInputArea();

  const int kw = 46, kh = 32, gX = 14, gY = 6;
  const int sX = 160 - (3 * kw + 2 * gX) / 2;
  const int sY = 78;

  const char* keys[12] = {"1","2","3","4","5","6","7","8","9","C","0","X"};
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 3; c++) {
      int idx = r * 3 + c;
      if (numpadMode == NUMPAD_UNLOCK && idx == 11) continue;

      int kx = sX + c * (kw + gX);
      int ky = sY + r * (kh + gY);
      uint16_t accent = NEON_CYAN;
      if (idx == 9)       { accent = NEON_AMBER; }
      else if (idx == 11) { accent = NEON_RED;   }
      if (lockoutActive) accent = TEXT_MUTED;

      tft.fillRoundRect(kx, ky, kw, kh, 6, GLASS_FILL);
      tft.drawRoundRect(kx, ky, kw, kh, 6, accent);
      tft.drawFastHLine(kx + 3, ky + 1, kw - 6, GLASS_HI);
      tft.fillRoundRect(kx + 2, ky + 3, 3, kh - 6, 1, accent);

      tft.setFreeFont(FF1);
      tft.setTextDatum(MC_DATUM);
      if (idx == 11) {
        drawCockpitIcon(kx + kw / 2, ky + kh / 2, CPI_CROSS, accent);
      } else {
        uint16_t txtColor;
        if (lockoutActive)      txtColor = TEXT_MUTED;
        else if (idx == 9)      txtColor = NEON_AMBER;
        else                    txtColor = TEXT_MAIN;
        tft.setTextColor(txtColor, GLASS_FILL);
        tft.drawString(keys[idx], kx + kw / 2, ky + kh / 2 + 2);
      }
    }
  }
}

static const uint32_t SETTINGS_VERSION = 8;
static const char* SETTINGS_VERSION_KEY = "cfgVer";
static const char* SETTINGS_CRC_KEY = "cfgCrc";

static uint32_t fnv1a32(const uint8_t* data, size_t len) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < len; ++i) {
    hash ^= data[i];
    hash *= 16777619u;
  }
  return hash;
}

static uint32_t calcSettingsBlobCrc() {
  return fnv1a32(reinterpret_cast<const uint8_t*>(&settingsData), sizeof(settingsData));
}

static int normalizeBatteryCells(int value) {
  if (value < BATTERY_CELLS_MIN) return BATTERY_CELLS_DEFAULT;
  if (value > BATTERY_CELLS_MAX) return BATTERY_CELLS_DEFAULT;
  return value;
}

static int normalizeThemeId(int value) {
  if (value < 0 || value >= THEME_COUNT) return THEME_DARK;
  return value;
}

static int normalizeAutoLockTimeoutSec(int value) {
  int best = AUTOLOCK_PRESETS[0];
  int bestDiff = 2147483647;
  for (int k = 0; k < AUTOLOCK_PRESET_COUNT; ++k) {
    int d = value - AUTOLOCK_PRESETS[k];
    if (d < 0) d = -d;
    if (d < bestDiff) { bestDiff = d; best = AUTOLOCK_PRESETS[k]; }
  }
  return best;
}

static void loadLegacyDefaults() {
  settingsData.magic = SETTINGS_MAGIC;
  settingsData.passStart = true;
  settingsData.canId = -1;
  memset(settingsData.pin, 0, sizeof(settingsData.pin));
  settingsData.batteryCells = BATTERY_CELLS_DEFAULT;
  settingsData.themeId = THEME_DARK;
  settingsData.motorPoles     = MOTOR_POLES_DEFAULT;
  settingsData.wheelDiaMm     = WHEEL_DIA_MM_DEFAULT;
  settingsData.driveRatioX100 = DRIVE_RATIO_X100_DEFAULT;
  settingsData.voltageCalX1000 = VOLTAGE_CAL_X1000_DEFAULT;
  settingsData.autoLockEnabled = false;
  settingsData.autoLockTimeoutSec = 120;
  settingsData.speedLimitEnabled = false;
  settingsData.speedLimitKmh = 25;
  settingsData.displayLocked = false;
}

void requestSettingsSave() {
  settingsDirty = true;
  settingsDirtyAt = millis();
}

void requestSettingsSaveNow() {
  settingsDirty = true;
  settingsDirtyAt = millis() - timing::SETTINGS_SAVE_DELAY;
}

void syncRuntimeToSettingsBlob() {
  settingsData.magic = SETTINGS_MAGIC;
  settingsData.passStart = passAtStartup;
  settingsData.canId = normalizeCanIdSetting(canId_2nd);
  snprintf(settingsData.pin, sizeof(settingsData.pin), "%s", savedPin.c_str());
  settingsData.pin[sizeof(settingsData.pin) - 1] = '\0';
  settingsData.batteryCells = normalizeBatteryCells(batteryCells);
  settingsData.themeId = normalizeThemeId(currentTheme);
  settingsData.motorPoles     = normalizeMotorPoles(motorPoles);
  settingsData.wheelDiaMm     = normalizeWheelDiaMm(wheelDiaMm);
  settingsData.driveRatioX100 = normalizeDriveRatioX100(driveRatioX100);
  settingsData.voltageCalX1000 = normalizeVoltageCalX1000(voltageCalX1000);
  settingsData.autoLockEnabled = autoLockEnabled;
  settingsData.autoLockTimeoutSec = normalizeAutoLockTimeoutSec(autoLockTimeoutSec);
  settingsData.speedLimitEnabled = speedLimitEnabled;
  settingsData.speedLimitKmh = normalizeSpeedLimitKmh(speedLimitKmh);
  settingsData.displayLocked = (isLocked && savedPin.length() == 4);
}

void applySettingsBlobToRuntime() {
  passAtStartup = settingsData.passStart;
  canId_2nd = normalizeCanIdSetting(settingsData.canId);
  settingsData.pin[sizeof(settingsData.pin) - 1] = '\0';

  savedPin = String(settingsData.pin);
  if (!isValidPinText(savedPin)) savedPin = "";

  batteryCells = normalizeBatteryCells(settingsData.batteryCells);
  currentTheme = normalizeThemeId(settingsData.themeId);
  motorPoles     = normalizeMotorPoles(settingsData.motorPoles);
  wheelDiaMm     = normalizeWheelDiaMm(settingsData.wheelDiaMm);
  driveRatioX100 = normalizeDriveRatioX100(settingsData.driveRatioX100);
  voltageCalX1000 = normalizeVoltageCalX1000(settingsData.voltageCalX1000);
  autoLockEnabled = settingsData.autoLockEnabled;
  autoLockTimeoutSec = normalizeAutoLockTimeoutSec(settingsData.autoLockTimeoutSec);
  speedLimitEnabled = settingsData.speedLimitEnabled;
  speedLimitKmh = normalizeSpeedLimitKmh(settingsData.speedLimitKmh);
  restoreLockedAfterBoot = settingsData.displayLocked && (savedPin.length() == 4);
}

void loadSettingsFromPrefs() {
  if (!prefsReady) {
    loadLegacyDefaults();
    applySettingsBlobToRuntime();
    settingsDirty = false;
    settingsDirtyAt = 0;
    return;
  }

  SettingsData defaults{};
  loadLegacyDefaults();
  defaults = settingsData;

  SettingsData tmp = defaults;
  const size_t bytes = prefs.getBytes("cfg", &tmp, sizeof(tmp));

  const bool hasMeta = prefs.isKey(SETTINGS_VERSION_KEY) && prefs.isKey(SETTINGS_CRC_KEY);
  const uint32_t storedVersion = hasMeta ? prefs.getUInt(SETTINGS_VERSION_KEY, 0) : 0;
  const uint32_t storedCrc     = hasMeta ? prefs.getUInt(SETTINGS_CRC_KEY, 0) : 0;

  const bool blobOk      = (bytes > 0 && tmp.magic == SETTINGS_MAGIC);
  const bool sizeMatch   = (bytes == sizeof(SettingsData));
  const bool versionMatch = hasMeta && (storedVersion == SETTINGS_VERSION);

  if (!blobOk) {
    settingsData = defaults;
  } else if (sizeMatch && versionMatch) {
    settingsData = tmp;
  } else {
    settingsData = defaults;
    if (bytes > 0) {
      const size_t copyBytes = (bytes > sizeof(SettingsData)) ? sizeof(SettingsData) : bytes;
      memcpy(&settingsData, &tmp, copyBytes);
      settingsData.magic = SETTINGS_MAGIC;
    }
  }

  const bool crcMatch = (sizeMatch && versionMatch && hasMeta &&
                         storedCrc == calcSettingsBlobCrc());
  if (sizeMatch && versionMatch && !crcMatch) {
    settingsData = defaults;
  }

  settingsData.pin[sizeof(settingsData.pin) - 1] = '\0';
  applySettingsBlobToRuntime();

  const bool needsRewrite = !sizeMatch || !hasMeta || !versionMatch || !crcMatch;
  settingsDirty   = needsRewrite;
  settingsDirtyAt = millis();
}

void saveSettingsIfNeeded() {
  if (!prefsReady) return;
  if (!settingsDirty) return;
  if (millis() - settingsDirtyAt < timing::SETTINGS_SAVE_DELAY) return;

  syncRuntimeToSettingsBlob();
  const uint32_t newCrc = calcSettingsBlobCrc();

  const uint32_t storedCrc = prefs.getUInt(SETTINGS_CRC_KEY, 0);
  if (storedCrc != newCrc) {
    prefs.putBytes("cfg", &settingsData, sizeof(settingsData));
    prefs.putUInt(SETTINGS_CRC_KEY, newCrc);
  }
  if (prefs.getUInt(SETTINGS_VERSION_KEY, 0) != SETTINGS_VERSION) {
    prefs.putUInt(SETTINGS_VERSION_KEY, SETTINGS_VERSION);
  }

  settingsDirty = false;
}

void syncRecordSaveStateToTelemetry(unsigned long savedAt, bool pending) {
  if (!telemetryMutex) return;

  if (xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
    telemetry.lastRecordSave = savedAt;
    telemetry.pendingRecordSave = pending;
    xSemaphoreGive(telemetryMutex);
  }
}

void saveRecordStatsIfNeeded() {
  if (!prefsReady) return;
  if (!pendingRecordSave) return;
  if (millis() - lastRecordSave < timing::RECORDS_SAVE_COOLDOWN) return;

  if (prefs.getFloat("recSpeed",   0)   != allTimeSpeed)    prefs.putFloat("recSpeed",   allTimeSpeed);
  if (prefs.getFloat("recPower",   0)   != allTimePower)    prefs.putFloat("recPower",   allTimePower);
  if (prefs.getFloat("recCurrent", 0)   != allTimeCurrent)  prefs.putFloat("recCurrent", allTimeCurrent);
  if (prefs.getFloat("recFetT",    0)   != allTimeFetTemp)  prefs.putFloat("recFetT",    allTimeFetTemp);
  if (prefs.getFloat("recMotT",    0)   != allTimeMotTemp)  prefs.putFloat("recMotT",    allTimeMotTemp);

  unsigned long now = millis();
  lastRecordSave = now;
  pendingRecordSave = false;
  syncRecordSaveStateToTelemetry(now, false);
}

namespace odo {
  constexpr float         SAVE_STEP_KM  = 0.1f;
  constexpr unsigned long SAVE_COOLDOWN = 30000;
}

void saveOdometerIfNeeded() {
  if (!prefsReady) return;
  if (allTimeOdoKm - odoSavedKm < odo::SAVE_STEP_KM) return;
  if (millis() - lastOdoSave < odo::SAVE_COOLDOWN)   return;
  prefs.putFloat("odoKm", allTimeOdoKm);
  odoSavedKm  = allTimeOdoKm;
  lastOdoSave = millis();
}

void flushOdometer() {
  if (!prefsReady) return;
  if (allTimeOdoKm <= odoSavedKm) return;
  prefs.putFloat("odoKm", allTimeOdoKm);
  odoSavedKm  = allTimeOdoKm;
  lastOdoSave = millis();
}

static void resetRecordsAll() {
  allTimeSpeed    = 0.0f;
  allTimePower    = 0.0f;
  allTimeCurrent  = 0.0f;
  allTimeFetTemp  = 0.0f;
  allTimeMotTemp  = 0.0f;

  if (telemetryMutex && xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
    telemetry.allTimeSpeed    = 0.0f;
    telemetry.allTimePower    = 0.0f;
    telemetry.allTimeCurrent  = 0.0f;
    telemetry.allTimeFetTemp  = 0.0f;
    telemetry.allTimeMotTemp  = 0.0f;
    telemetry.pendingRecordSave = false;
    telemetry.lastRecordSave    = millis();
    xSemaphoreGive(telemetryMutex);
  }

  if (prefsReady) {
    prefs.putFloat("recSpeed",   0.0f);
    prefs.putFloat("recPower",   0.0f);
    prefs.putFloat("recCurrent", 0.0f);
    prefs.putFloat("recFetT",    0.0f);
    prefs.putFloat("recMotT",    0.0f);
  }

  pendingRecordSave = false;
  lastRecordSave    = millis();
  Serial.println("Records: reset");
}

void syncTelemetryToGlobals() {
  if (!telemetryMutex) return;

  if (xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
    vesc_online = telemetry.vesc_online;
    lastVescUpdate = telemetry.lastVescUpdate;

    disp_voltage = telemetry.disp_voltage;
    disp_currentA = telemetry.disp_currentA;
    disp_powerW = telemetry.disp_powerW;
    disp_speedKmh = telemetry.disp_speedKmh;
    start_ah = telemetry.start_ah;
    start_wh = telemetry.start_wh;
    disp_trip_km = telemetry.disp_trip_km;
    disp_batPercent = telemetry.disp_batPercent;
    disp_tempMosfet = telemetry.disp_tempMosfet;
    disp_tempMotor = telemetry.disp_tempMotor;
    disp_ampHours = telemetry.disp_ampHours;
    disp_wattHours = telemetry.disp_wattHours;

    trip_km = telemetry.trip_km;
    tripMaxSpeedKmh = telemetry.tripMaxSpeedKmh;
    tripMaxPowerW   = telemetry.tripMaxPowerW;
    tripMaxCurrentA = telemetry.tripMaxCurrentA;
    disp_primaryFault = telemetry.primaryFault;
    disp_secondaryFault = telemetry.secondaryFault;
    disp_primaryTempMotor = telemetry.primaryTempMotor;
    disp_primaryTempMosfet = telemetry.primaryTempMosfet;
    disp_secondaryTempMotor = telemetry.secondaryTempMotor;
    disp_secondaryTempMosfet = telemetry.secondaryTempMosfet;
    disp_secondaryOnline = telemetry.secondaryOnline;
    disp_secondaryCanId = telemetry.secondaryCanId;

    allTimeSpeed    = telemetry.allTimeSpeed;
    allTimePower    = telemetry.allTimePower;
    allTimeCurrent  = telemetry.allTimeCurrent;
    allTimeFetTemp  = telemetry.allTimeFetTemp;
    allTimeMotTemp  = telemetry.allTimeMotTemp;
    allTimeOdoKm    = telemetry.allTimeOdoKm;

    lastRecordSave = telemetry.lastRecordSave;
    pendingRecordSave = telemetry.pendingRecordSave;
    start_tachometer = telemetry.start_tachometer;

    xSemaphoreGive(telemetryMutex);
  }
}

// =============================================================================
//  Блокировка курка газа
// =============================================================================
namespace vesclock {
  constexpr uint8_t COMM_GET_MCCONF_TEMP = 91;
  constexpr uint8_t COMM_SET_MCCONF_TEMP = 48;
  constexpr uint8_t COMM_FORWARD_CAN     = 34;

  struct TempCfg {
    float cur_min_scale = 1.0f, cur_max_scale = 1.0f;
    float min_erpm = 0, max_erpm = 0;
    float min_duty = 0, max_duty = 0;
    float watt_min = 0, watt_max = 0;
    float in_curr_min = 0, in_curr_max = 0;
    bool  valid = false;
  };

  static const uint16_t crc16_tab[256] = {0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0};
  static uint16_t crc16(const uint8_t* buf, uint16_t len) {
    uint16_t c = 0;
    for (uint16_t i = 0; i < len; i++) c = crc16_tab[((c >> 8) ^ buf[i]) & 0xFF] ^ (c << 8);
    return c;
  }
  static void appU32(uint8_t* b, uint32_t n, int* i) {
    b[(*i)++] = (uint8_t)(n >> 24); b[(*i)++] = (uint8_t)(n >> 16);
    b[(*i)++] = (uint8_t)(n >> 8);  b[(*i)++] = (uint8_t)(n);
  }
  static uint32_t getU32(const uint8_t* b, int* i) {
    uint32_t r = ((uint32_t)b[*i] << 24) | ((uint32_t)b[*i + 1] << 16) |
                 ((uint32_t)b[*i + 2] << 8) | (uint32_t)b[*i + 3];
    *i += 4; return r;
  }
  static void appF(uint8_t* b, float number, int* i) {
    if (fabsf(number) < 1.5e-38f) number = 0.0f;
    int e = 0; float sig = frexpf(number, &e); float sa = fabsf(sig); uint32_t si = 0;
    if (sa >= 0.5f) { si = (uint32_t)((sa - 0.5f) * 2.0f * 8388608.0f); e += 126; }
    uint32_t res = ((uint32_t)(e & 0xFF) << 23) | (si & 0x7FFFFF);
    if (sig < 0) res |= 1U << 31;
    appU32(b, res, i);
  }
  static float getF(const uint8_t* b, int* i) {
    uint32_t res = getU32(b, i);
    int e = (res >> 23) & 0xFF; uint32_t si = res & 0x7FFFFF; bool neg = res & (1U << 31);
    float sig = 0.0f;
    if (e != 0 || si != 0) { sig = (float)si / (8388608.0f * 2.0f) + 0.5f; e -= 126; }
    if (neg) sig = -sig;
    return ldexpf(sig, e);
  }

  static void sendFrame(const uint8_t* payload, uint8_t len) {
    uint8_t f[261]; int n = 0;
    f[n++] = 2; f[n++] = len;
    for (uint8_t k = 0; k < len; k++) f[n++] = payload[k];
    uint16_t c = crc16(payload, len);
    f[n++] = (uint8_t)(c >> 8); f[n++] = (uint8_t)(c & 0xFF); f[n++] = 3;
    VESCSerial.write(f, n);
    VESCSerial.flush();
  }

  static int readFrame(uint8_t* out, int outMax, uint32_t timeoutMs) {
    uint32_t start = millis();
    int st = 0, len = 0, idx = 0; uint16_t rxcrc = 0;
    while (millis() - start < timeoutMs) {
      if (!VESCSerial.available()) { vTaskDelay(1); continue; }
      uint8_t b = (uint8_t)VESCSerial.read();
      switch (st) {
        case 0: if (b == 2) st = 1; break;
        case 1: len = b; if (len < 1 || len > outMax) { st = 0; } else { idx = 0; st = 2; } break;
        case 2: out[idx++] = b; if (idx >= len) st = 3; break;
        case 3: rxcrc = (uint16_t)b << 8; st = 4; break;
        case 4: rxcrc |= b; st = 5; break;
        case 5: if (b == 3 && crc16(out, (uint16_t)len) == rxcrc) return len; st = 0; break;
      }
    }
    return -1;
  }

  static bool readTempCfg(int canId, TempCfg& out) {
    while (VESCSerial.available()) VESCSerial.read();
    uint8_t pl[4]; int n = 0;
    if (canId >= 1) { pl[n++] = COMM_FORWARD_CAN; pl[n++] = (uint8_t)canId; pl[n++] = COMM_GET_MCCONF_TEMP; }
    else            { pl[n++] = COMM_GET_MCCONF_TEMP; }
    sendFrame(pl, (uint8_t)n);
    uint8_t rx[128];
    int rl = readFrame(rx, sizeof(rx), 200);
    if (rl < 1 + 10 * 4) return false;
    int i = 0;
    if (rx[i] != COMM_GET_MCCONF_TEMP) return false;
    i++;
    out.cur_min_scale = getF(rx, &i); out.cur_max_scale = getF(rx, &i);
    out.min_erpm      = getF(rx, &i); out.max_erpm      = getF(rx, &i);
    out.min_duty      = getF(rx, &i); out.max_duty      = getF(rx, &i);
    out.watt_min      = getF(rx, &i); out.watt_max      = getF(rx, &i);
    out.in_curr_min   = getF(rx, &i); out.in_curr_max   = getF(rx, &i);
    out.valid = true;
    return true;
  }

  static void sendSetTemp(int canId, const TempCfg& c, float minScale, float maxScale) {
    uint8_t inner[80]; int n = 0;
    inner[n++] = COMM_SET_MCCONF_TEMP;
    inner[n++] = 0;
    inner[n++] = 0;
    inner[n++] = 0;
    inner[n++] = 0;
    appF(inner, minScale,      &n);
    appF(inner, maxScale,      &n);
    appF(inner, c.min_erpm,    &n);
    appF(inner, c.max_erpm,    &n);
    appF(inner, c.min_duty,    &n);
    appF(inner, c.max_duty,    &n);
    appF(inner, c.watt_min,    &n);
    appF(inner, c.watt_max,    &n);
    appF(inner, c.in_curr_min, &n);
    appF(inner, c.in_curr_max, &n);
    if (canId >= 1) {
      uint8_t fwd[96]; int m = 0;
      fwd[m++] = COMM_FORWARD_CAN; fwd[m++] = (uint8_t)canId;
      for (int k = 0; k < n; k++) fwd[m++] = inner[k];
      sendFrame(fwd, (uint8_t)m);
    } else {
      sendFrame(inner, (uint8_t)n);
    }
  }

  constexpr uint8_t COMM_LISP_REPL_CMD = 138;
  static void sendLispRepl(const char* code) {
    uint8_t pl[160];
    int n = 0;
    pl[n++] = COMM_LISP_REPL_CMD;
    for (const char* p = code; *p && n < (int)sizeof(pl); ++p) {
      pl[n++] = (uint8_t)*p;
    }
    sendFrame(pl, (uint8_t)n);
  }
}

struct MusicNote { uint16_t freq; float dur; uint16_t gapMs; };

static const MusicNote MELODY_0[] = {
  {1318, 0.15f, 180}, {1174, 0.15f, 180}, {739, 0.30f, 330}, {830, 0.30f, 330},
  {1109, 0.15f, 180}, {987, 0.15f, 180}, {587, 0.30f, 330}, {659, 0.30f, 330},
  {987, 0.15f, 180}, {880, 0.15f, 180}, {554, 0.30f, 330}, {659, 0.30f, 330},
  {880, 0.50f, 600},
};
static const MusicNote MELODY_1[] = {
  {440, 0.50f, 550}, {440, 0.50f, 550}, {440, 0.50f, 550}, {349, 0.35f, 400},
  {523, 0.15f, 180}, {440, 0.50f, 550}, {349, 0.35f, 400}, {523, 0.15f, 180},
  {440, 0.70f, 800},
};
static const MusicNote MELODY_2[] = {
  {660, 0.10f, 150}, {660, 0.10f, 300}, {660, 0.10f, 300}, {510, 0.10f, 100},
  {660, 0.10f, 300}, {770, 0.10f, 550}, {380, 0.10f, 550}, {510, 0.10f, 400},
  {380, 0.10f, 300}, {320, 0.10f, 300}, {440, 0.10f, 300}, {480, 0.08f, 250},
  {450, 0.10f, 300}, {430, 0.10f, 300},
};

static const MusicNote MELODY_3[] = {
  {659,0.25f,280},{494,0.13f,150},{523,0.13f,150},{587,0.25f,280},{523,0.13f,150},
  {494,0.13f,150},{440,0.25f,280},{440,0.13f,150},{523,0.13f,150},{659,0.25f,280},
  {587,0.13f,150},{523,0.13f,150},{494,0.35f,400},{587,0.25f,280},{659,0.30f,330},
  {523,0.30f,330},{440,0.30f,330},{440,0.40f,450},
};
static const MusicNote MELODY_4[] = {
  {523,0.20f,180},{523,0.13f,150},{587,0.30f,330},{523,0.30f,330},{698,0.30f,330},
  {659,0.50f,550},{523,0.20f,180},{523,0.13f,150},{587,0.30f,330},{523,0.30f,330},
  {784,0.30f,330},{698,0.50f,550},
};
static const MusicNote MELODY_5[] = {
  {523,0.30f,330},{523,0.30f,330},{784,0.30f,330},{784,0.30f,330},{880,0.30f,330},
  {880,0.30f,330},{784,0.50f,550},{698,0.30f,330},{698,0.30f,330},{659,0.30f,330},
  {659,0.30f,330},{587,0.30f,330},{587,0.30f,330},{523,0.50f,550},
};
static const MusicNote MELODY_6[] = {
  {659,0.30f,330},{659,0.30f,330},{698,0.30f,330},{784,0.30f,330},{784,0.30f,330},
  {698,0.30f,330},{659,0.30f,330},{587,0.30f,330},{523,0.30f,330},{523,0.30f,330},
  {587,0.30f,330},{659,0.30f,330},{659,0.45f,400},{587,0.15f,180},{587,0.50f,550},
};
static const MusicNote MELODY_7[] = {
  {659,0.25f,280},{659,0.25f,280},{659,0.45f,500},{659,0.25f,280},{659,0.25f,280},
  {659,0.45f,500},{659,0.25f,280},{784,0.25f,280},{523,0.25f,280},{587,0.25f,280},
  {659,0.60f,650},
};
static const MusicNote MELODY_8[] = {
  {392,0.18f,200},{392,0.18f,200},{392,0.18f,200},{523,0.55f,580},{784,0.55f,580},
  {698,0.18f,200},{659,0.18f,200},{587,0.18f,200},{1047,0.55f,580},{784,0.40f,430},
  {698,0.18f,200},{659,0.18f,200},{587,0.18f,200},{1047,0.55f,580},{784,0.50f,560},
};
static const MusicNote MELODY_9[] = {
  {659,0.18f,200},{622,0.18f,200},{659,0.18f,200},{622,0.18f,200},{659,0.18f,200},
  {494,0.18f,200},{587,0.18f,200},{523,0.18f,200},{440,0.40f,450},{262,0.18f,200},
  {330,0.18f,200},{440,0.18f,200},{494,0.40f,450},
};

static const MusicNote MELODY_10[] = {
  {494,0.30f,330},{659,0.45f,500},{784,0.15f,170},{740,0.30f,330},{659,0.50f,560},
  {988,0.50f,560},{880,0.80f,900},{740,0.50f,560},{659,0.45f,500},{784,0.15f,170},
  {740,0.30f,330},{622,0.50f,560},{698,0.30f,330},{494,0.80f,900},{494,0.30f,330},
  {659,0.45f,500},{784,0.15f,170},{740,0.30f,330},{659,0.50f,560},{988,0.50f,560},
  {1175,0.50f,560},{1109,0.50f,560},{1047,0.50f,560},{831,0.50f,560},{1047,0.45f,500},
  {988,0.15f,170},{932,0.30f,330},{466,0.50f,560},{784,0.30f,330},{659,0.80f,900},
};
static const MusicNote MELODY_11[] = {
  {440,0.20f,150},{523,0.20f,150},{587,0.40f,300},{587,0.20f,150},{587,0.20f,150},
  {659,0.20f,150},{698,0.40f,300},{698,0.20f,150},{698,0.20f,150},{784,0.20f,150},
  {659,0.40f,300},{659,0.20f,150},{587,0.20f,150},{523,0.20f,150},{523,0.20f,150},
  {587,0.40f,450},{440,0.20f,150},{523,0.20f,150},{587,0.40f,300},{587,0.20f,150},
  {587,0.20f,150},{659,0.20f,150},{698,0.40f,300},{698,0.20f,150},{698,0.20f,150},
  {784,0.20f,150},{880,0.40f,300},{880,0.20f,150},{932,0.20f,150},{880,0.20f,150},
  {784,0.20f,150},{659,0.50f,560},
};
static const MusicNote MELODY_12[] = {
  {392,0.40f,300},{523,0.40f,300},{622,0.20f,150},{698,0.20f,150},{392,0.40f,300},
  {523,0.40f,300},{622,0.20f,150},{698,0.20f,150},{392,0.40f,300},{523,0.40f,300},
  {622,0.20f,150},{698,0.20f,150},{392,0.40f,300},{523,0.40f,300},{622,0.20f,150},
  {698,0.20f,150},{587,0.40f,300},{392,0.40f,300},{466,0.20f,150},{523,0.20f,150},
  {587,0.40f,300},{392,0.40f,300},{466,0.20f,150},{523,0.20f,150},{587,0.40f,300},
  {392,0.40f,300},{466,0.20f,150},{523,0.20f,150},{587,0.70f,750},
};
static const MusicNote MELODY_13[] = {
  {294,0.15f,160},{294,0.15f,160},{587,0.30f,330},{440,0.30f,360},{415,0.30f,360},
  {392,0.30f,360},{349,0.15f,160},{294,0.15f,160},{349,0.15f,160},{392,0.15f,170},
  {262,0.15f,160},{262,0.15f,160},{587,0.30f,330},{440,0.30f,360},{415,0.30f,360},
  {392,0.30f,360},{349,0.15f,160},{294,0.15f,160},{349,0.15f,160},{392,0.15f,170},
  {247,0.15f,160},{247,0.15f,160},{587,0.30f,330},{440,0.30f,360},{415,0.30f,360},
  {392,0.30f,360},{349,0.15f,160},{294,0.15f,160},{349,0.15f,160},{392,0.15f,170},
  {233,0.15f,160},{233,0.15f,160},{587,0.30f,330},{440,0.30f,360},{415,0.30f,360},
  {392,0.30f,360},{349,0.15f,160},{294,0.15f,160},{349,0.15f,160},{392,0.30f,360},
};
static const MusicNote MELODY_14[] = {
  {494,0.45f,500},{587,0.20f,220},{440,0.80f,880},{392,0.20f,220},{440,0.20f,220},
  {494,0.45f,500},{587,0.20f,220},{440,0.80f,880},{494,0.45f,500},{587,0.20f,220},
  {784,0.45f,500},{740,0.20f,220},{659,0.20f,220},{587,0.45f,500},{494,0.20f,220},
  {440,0.45f,500},{392,0.80f,880},{494,0.45f,500},{587,0.20f,220},{440,0.80f,880},
  {392,0.20f,220},{440,0.20f,220},{494,0.45f,500},{523,0.20f,220},{494,0.45f,500},
  {440,0.90f,950},
};
static const MusicNote MELODY_15[] = {
  {370,0.18f,400},{440,0.18f,200},{554,0.18f,400},{370,0.18f,400},{370,0.18f,200},
  {440,0.18f,200},{622,0.18f,400},{587,0.18f,400},{587,0.18f,200},{554,0.18f,200},
  {440,0.18f,400},{554,0.18f,200},{440,0.30f,500},{370,0.18f,400},{440,0.18f,200},
  {554,0.18f,400},{370,0.18f,400},{370,0.18f,200},{440,0.18f,200},{622,0.18f,400},
  {831,0.18f,400},{740,0.18f,400},{554,0.30f,500},{440,0.40f,600},
};
struct Melody { const MusicNote* notes; uint16_t count; const char* name; };
static const Melody MELODIES[] = {
  { MELODY_1,  (uint16_t)(sizeof(MELODY_1)  / sizeof(MELODY_1[0])),  "IMPERIAL MARCH" },
  { MELODY_2,  (uint16_t)(sizeof(MELODY_2)  / sizeof(MELODY_2[0])),  "SUPER MARIO" },
  { MELODY_3,  (uint16_t)(sizeof(MELODY_3)  / sizeof(MELODY_3[0])),  "TETRIS" },
  { MELODY_8,  (uint16_t)(sizeof(MELODY_8)  / sizeof(MELODY_8[0])),  "STAR WARS" },
  { MELODY_10, (uint16_t)(sizeof(MELODY_10) / sizeof(MELODY_10[0])), "HARRY POTTER" },
  { MELODY_13, (uint16_t)(sizeof(MELODY_13) / sizeof(MELODY_13[0])), "MEGALOVANIA" },
};
static const int MUSIC_COUNT = (int)(sizeof(MELODIES) / sizeof(MELODIES[0]));
static const int MUSIC_PER_PAGE = 6;

static void playMelody(int idx) {
  if (idx < 0 || idx >= MUSIC_COUNT) return;
  if (btBridgeActive) return;
  if (fabsf(ui_speedKmh) > 2.0f) return;
  const Melody& m = MELODIES[idx];
  for (uint16_t k = 0; k < m.count; ++k) {
    char buf[40];
    snprintf(buf, sizeof(buf), "(foc-beep %u %.2f 12)",
             (unsigned)m.notes[k].freq, m.notes[k].dur);
    vesclock::sendLispRepl(buf);
    vTaskDelay(pdMS_TO_TICKS(m.notes[k].gapMs));
  }
}

void vescTask(void *pvParameters) {
  (void)pvParameters;

  float lastSecCurrent = 0.0f;
  float lastSecAh = 0.0f;
  float lastSecWh = 0.0f;
  float lastSecTempMotor = NAN;
  float lastSecTempMosfet = NAN;
  bool  haveSecCache = false;
  unsigned long lastSecGoodMs = 0;
  unsigned long lastSecAttemptMs = 0;

  float startSecAh = -1.0f;
  float startSecWh = -1.0f;

  for (;;) {
    if (vescTaskPauseRequested) {
      vescTaskPaused = true;
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    vescTaskPaused = false;

    const bool primaryOk = vesc.getVescValues();
    if (primaryOk) {
      const float primaryCurrentRaw = vesc.data.avgInputCurrent;
      const float primaryVoltageRaw = vesc.data.inpVoltage * voltageCalMultiplier();
      const float primaryRpmRaw = vesc.data.rpm;
      const float primaryTempMotRaw = vesc.data.tempMotor;
      const float primaryTempFetRaw = vesc.data.tempMosfet;
      const float primaryAhRaw = vesc.data.ampHours;
      const float primaryWhRaw = vesc.data.wattHours;
      const int32_t primaryTachometer = vesc.data.tachometerAbs;
      const int primaryFault = (int)vesc.data.error;

      if (!isfinite(primaryVoltageRaw) || primaryVoltageRaw < 5.0f) {
        vTaskDelay(pdMS_TO_TICKS(timing::VESC_LOOP_DELAY));
        continue;
      }

      const float primaryVoltage = clampFiniteFloat(primaryVoltageRaw, 0.0f, 200.0f, 0.0f);
      const float primaryCurrent = clampFiniteFloat(primaryCurrentRaw, -500.0f, 500.0f, 0.0f);
      const float primaryRpm = clampFiniteFloat(primaryRpmRaw, -100000.0f, 100000.0f, 0.0f);
      const float primaryTempMotor = clampFiniteFloat(primaryTempMotRaw, -40.0f, 200.0f, 0.0f);
      const float primaryTempMosfet = clampFiniteFloat(primaryTempFetRaw, -40.0f, 200.0f, 0.0f);
      const float primaryAh = clampFiniteFloat(primaryAhRaw, 0.0f, 100000.0f, 0.0f);
      const float primaryWh = clampFiniteFloat(primaryWhRaw, 0.0f, 1000000.0f, 0.0f);

      const float wheelSpeedKmh = erpmToKmh(primaryRpm);
      const float speed = (wheelSpeedKmh <= 0.2f) ? 0.0f : wheelSpeedKmh;

      int32_t localStartTach = primaryTachometer;
      bool    sessionReset   = false;
      if (telemetryMutex && xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
        if (telemetry.start_tachometer == -1 ||
            primaryTachometer < telemetry.start_tachometer) {
          telemetry.start_tachometer = primaryTachometer;
          sessionReset = true;
        }
        localStartTach = telemetry.start_tachometer;

        if (telemetry.start_ah < 0.0f || primaryAh + 0.01f < telemetry.start_ah) {
          telemetry.start_ah = primaryAh;
        }
        if (telemetry.start_wh < 0.0f || primaryWh + 0.1f < telemetry.start_wh) {
          telemetry.start_wh = primaryWh;
        }
        if (sessionReset) {
          telemetry.tripMaxSpeedKmh = 0.0f;
          telemetry.tripMaxPowerW   = 0.0f;
          telemetry.tripMaxCurrentA = 0.0f;
          startSecAh = -1.0f;
          startSecWh = -1.0f;
        }
        xSemaphoreGive(telemetryMutex);
      }

      float trip = tachToKm(primaryTachometer - localStartTach);
      if (!isfinite(trip) || trip < 0.0f) trip = 0.0f;

      const int cellsForSoc = (batteryCells >= BATTERY_CELLS_MIN && batteryCells <= BATTERY_CELLS_MAX)
                                ? batteryCells
                                : BATTERY_CELLS_DEFAULT;
      const float cell_v = primaryVoltage / (float)cellsForSoc;
      float batPercentF = (cell_v - 3.30f) / (4.15f - 3.30f) * 100.0f;
      if (!isfinite(batPercentF)) batPercentF = 0.0f;
      if (batPercentF < 0.0f)   batPercentF = 0.0f;
      if (batPercentF > 100.0f) batPercentF = 100.0f;
      static float batPercentFilt = -1.0f;
      static int   lastCellsUsed  = -1;
      if (lastCellsUsed != cellsForSoc) {
        batPercentFilt = -1.0f;
        lastCellsUsed  = cellsForSoc;
      }
      if (batPercentFilt < 0.0f || fabsf(batPercentF - batPercentFilt) > 15.0f) {
        batPercentFilt = batPercentF;
      } else {
        batPercentFilt = batPercentFilt + (batPercentF - batPercentFilt) * 0.05f;
      }
      const int batPercent = (int)lroundf(batPercentFilt);

      const int secondaryCanId = normalizeCanIdSetting(canId_2nd);
      const bool secondaryEnabled = (secondaryCanId >= 1);

      float secondaryCurrent = 0.0f;
      float secondaryAh = 0.0f;
      float secondaryWh = 0.0f;
      float secondaryTempMotor = NAN;
      float secondaryTempMosfet = NAN;
      int secondaryFault = -1;
      bool secondaryOk = false;

      if (secondaryEnabled) {
        const unsigned long now = millis();
        const bool recentlyGood = haveSecCache && (now - lastSecGoodMs < timing::SECONDARY_CACHE_TTL);
        const unsigned long minInterval = recentlyGood ? 0 : timing::SECONDARY_BACKOFF;

        if (now - lastSecAttemptMs >= minInterval) {
          lastSecAttemptMs = now;
          secondaryOk = vesc.getVescValues((uint8_t)secondaryCanId);
          if (secondaryOk) {
            secondaryCurrent = clampFiniteFloat(vesc.data.avgInputCurrent, -500.0f, 500.0f, 0.0f);
            secondaryAh = clampFiniteFloat(vesc.data.ampHours, 0.0f, 100000.0f, 0.0f);
            secondaryWh = clampFiniteFloat(vesc.data.wattHours, 0.0f, 1000000.0f, 0.0f);
            secondaryTempMotor = clampFiniteFloat(vesc.data.tempMotor, -40.0f, 200.0f, 0.0f);
            secondaryTempMosfet = clampFiniteFloat(vesc.data.tempMosfet, -40.0f, 200.0f, 0.0f);
            secondaryFault = (int)vesc.data.error;

            if (startSecAh < 0.0f || secondaryAh + 0.01f < startSecAh) {
              startSecAh = secondaryAh;
            }
            if (startSecWh < 0.0f || secondaryWh + 0.1f  < startSecWh) {
              startSecWh = secondaryWh;
            }

            lastSecCurrent = secondaryCurrent;
            lastSecAh = secondaryAh;
            lastSecWh = secondaryWh;
            lastSecTempMotor = secondaryTempMotor;
            lastSecTempMosfet = secondaryTempMosfet;
            haveSecCache = true;
            lastSecGoodMs = now;
          }
        }

        if (!secondaryOk && haveSecCache && (now - lastSecGoodMs < timing::SECONDARY_CACHE_TTL)) {
          secondaryCurrent = lastSecCurrent;
          secondaryAh = lastSecAh;
          secondaryWh = lastSecWh;
          secondaryTempMotor = lastSecTempMotor;
          secondaryTempMosfet = lastSecTempMosfet;
        } else if (!secondaryOk) {
          haveSecCache = false;
          secondaryCurrent = 0.0f;
          secondaryAh = 0.0f;
          secondaryWh = 0.0f;
          secondaryTempMotor = NAN;
          secondaryTempMosfet = NAN;
        }
      } else {
        haveSecCache = false;
      }

      const bool secondaryUsable = secondaryEnabled && (secondaryOk || haveSecCache);

      const float motorTempForDisplay = safeCombinedTemp(
          primaryTempMotor, true, secondaryTempMotor, secondaryUsable);
      const float fetTempForDisplay = safeCombinedTemp(
          primaryTempMosfet, true, secondaryTempMosfet, secondaryUsable);
      float current = clampFiniteFloat(primaryCurrent + secondaryCurrent, -500.0f, 500.0f, 0.0f);

      if (speed < 1.0f && fabsf(current) < 0.8f) {
        current = 0.0f;
        powerFilter.updateEstimate(0.0f);
      }

      const float measuredPower = primaryVoltage * current;
      float power = isfinite(measuredPower) ? powerFilter.updateEstimate(measuredPower) : 0.0f;
      if (fabsf(power) < 1.0f) power = 0.0f;

      if (telemetryMutex && xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
        telemetry.vesc_online = true;
        telemetry.lastVescUpdate = millis();

        telemetry.disp_voltage = primaryVoltage;
        telemetry.disp_currentA = current;
        telemetry.disp_powerW = isfinite(power) ? power : 0.0f;
        telemetry.disp_speedKmh = speed;
        telemetry.disp_trip_km = trip;

        telemetry.trip_km = trip;
        if (isfinite(speed) && speed > telemetry.tripMaxSpeedKmh) {
          telemetry.tripMaxSpeedKmh = speed;
        }
        if (isfinite(power) && power > telemetry.tripMaxPowerW) {
          telemetry.tripMaxPowerW = power;
        }
        if (isfinite(current) && current > telemetry.tripMaxCurrentA) {
          telemetry.tripMaxCurrentA = current;
        }
        telemetry.disp_batPercent = batPercent;
        telemetry.disp_tempMosfet = (int)lroundf(fetTempForDisplay);
        telemetry.disp_tempMotor = (int)lroundf(motorTempForDisplay);

        const float primaryAhDelta = primaryAh - telemetry.start_ah;
        const float primaryWhDelta = primaryWh - telemetry.start_wh;
        float secondaryAhDelta = 0.0f;
        float secondaryWhDelta = 0.0f;
        if (startSecAh >= 0.0f) secondaryAhDelta = secondaryAh - startSecAh;
        if (startSecWh >= 0.0f) secondaryWhDelta = secondaryWh - startSecWh;

        telemetry.disp_ampHours  = primaryAhDelta + secondaryAhDelta;
        telemetry.disp_wattHours = primaryWhDelta + secondaryWhDelta;
        if (telemetry.disp_ampHours  < 0.0f) telemetry.disp_ampHours  = 0.0f;
        if (telemetry.disp_wattHours < 0.0f) telemetry.disp_wattHours = 0.0f;

        telemetry.start_tachometer = localStartTach;

        telemetry.primaryFault = primaryFault;
        telemetry.secondaryFault = secondaryFault;
        telemetry.primaryTempMotor = primaryTempMotor;
        telemetry.primaryTempMosfet = primaryTempMosfet;
        telemetry.secondaryTempMotor = secondaryUsable ? secondaryTempMotor : NAN;
        telemetry.secondaryTempMosfet = secondaryUsable ? secondaryTempMosfet : NAN;
        telemetry.secondaryOnline = secondaryOk || (secondaryEnabled && haveSecCache);
        telemetry.secondaryCanId = secondaryCanId;

        if (speed > telemetry.allTimeSpeed) {
          telemetry.allTimeSpeed = speed;
          telemetry.pendingRecordSave = true;
        }
        if (power > telemetry.allTimePower) {
          telemetry.allTimePower = power;
          telemetry.pendingRecordSave = true;
        }
        if (current > telemetry.allTimeCurrent) {
          telemetry.allTimeCurrent = current;
          telemetry.pendingRecordSave = true;
        }
        if (tempLooksValid(fetTempForDisplay) &&
            fetTempForDisplay > telemetry.allTimeFetTemp) {
          telemetry.allTimeFetTemp = fetTempForDisplay;
          telemetry.pendingRecordSave = true;
        }
        if (tempLooksValid(motorTempForDisplay) &&
            motorTempForDisplay > telemetry.allTimeMotTemp) {
          telemetry.allTimeMotTemp = motorTempForDisplay;
          telemetry.pendingRecordSave = true;
        }

        static bool    odoTachInit = false;
        static int32_t odoLastTach = 0;
        if (!odoTachInit) {
          odoTachInit = true;
          odoLastTach = primaryTachometer;
        } else if (primaryTachometer > odoLastTach) {
          const float dKm = tachToKm(primaryTachometer - odoLastTach);
          if (isfinite(dKm) && dKm > 0.0f && dKm < 10.0f) {
            telemetry.allTimeOdoKm += dKm;
          }
          odoLastTach = primaryTachometer;
        } else if (primaryTachometer < odoLastTach) {
          odoLastTach = primaryTachometer;
        }

        xSemaphoreGive(telemetryMutex);
      }

      {
        static unsigned long lastActiveMs = 0;
        const unsigned long nowAL = millis();
        if (lastActiveMs == 0) lastActiveMs = nowAL;

        if (autoLockKick) {
          autoLockKick = false;
          lastActiveMs = nowAL;
          autoThrottleLock = false;
        }

        const bool moving = fabsf(wheelSpeedKmh) > 0.8f;

        if (isLocked || !autoLockEnabled) {
          autoThrottleLock = false;
          lastActiveMs = nowAL;
        } else if (moving) {
          lastActiveMs = nowAL;
          autoThrottleLock = false;
        } else if (!autoThrottleLock) {
          if ((nowAL - lastActiveMs) >= (unsigned long)autoLockTimeoutSec * 1000UL) {
            autoThrottleLock = true;
          }
        } else {
        }
      }

      {
        static vesclock::TempCfg masterCfg;
        static vesclock::TempCfg slaveCfg;
        static bool          lockApplied = false;
        static unsigned long lastReapply = 0;
        static int           restoreTries = 0;
        const unsigned long  nowLock = millis();

        const bool lockWanted  = vescThrottleLockDesired || autoThrottleLock;
        const bool limitWanted = speedLimitEnabled;
        const bool overrideWanted = lockWanted || limitWanted;

        static bool prevLockWanted  = false;
        static bool prevLimitWanted = false;
        if (lockWanted != prevLockWanted || limitWanted != prevLimitWanted) {
          lastReapply = 0;
        }
        prevLockWanted  = lockWanted;
        prevLimitWanted = limitWanted;

        if (overrideWanted) {
          if (!masterCfg.valid) vesclock::readTempCfg(-1, masterCfg);
          if (secondaryEnabled && secondaryOk && !slaveCfg.valid)
            vesclock::readTempCfg(secondaryCanId, slaveCfg);

          const float capErpm = limitWanted
              ? kmhToErpm((float)normalizeSpeedLimitKmh(speedLimitKmh)) : 0.0f;

          if (masterCfg.valid && (!lockApplied || (nowLock - lastReapply) >= 1500)) {
            const float mMin = lockWanted ? 0.0f
                : ((masterCfg.cur_min_scale <= 0.0f) ? 1.0f : masterCfg.cur_min_scale);
            const float mMax = lockWanted ? 0.0f
                : ((masterCfg.cur_max_scale <= 0.0f) ? 1.0f : masterCfg.cur_max_scale);
            vesclock::TempCfg mEff = masterCfg;
            if (capErpm > 0.0f) {
              if (mEff.max_erpm <= 0.0f || mEff.max_erpm > capErpm)  mEff.max_erpm = capErpm;
              if (mEff.min_erpm >= 0.0f || mEff.min_erpm < -capErpm) mEff.min_erpm = -capErpm;
            }
            vesclock::sendSetTemp(-1, mEff, mMin, mMax);
            if (secondaryEnabled && slaveCfg.valid) {
              const float sMin = lockWanted ? 0.0f
                  : ((slaveCfg.cur_min_scale <= 0.0f) ? 1.0f : slaveCfg.cur_min_scale);
              const float sMax = lockWanted ? 0.0f
                  : ((slaveCfg.cur_max_scale <= 0.0f) ? 1.0f : slaveCfg.cur_max_scale);
              vesclock::TempCfg sEff = slaveCfg;
              if (capErpm > 0.0f) {
                if (sEff.max_erpm <= 0.0f || sEff.max_erpm > capErpm)  sEff.max_erpm = capErpm;
                if (sEff.min_erpm >= 0.0f || sEff.min_erpm < -capErpm) sEff.min_erpm = -capErpm;
              }
              vesclock::sendSetTemp(secondaryCanId, sEff, sMin, sMax);
            }
            lockApplied = true;
            lastReapply = nowLock;
          }
        } else {
          if (lockApplied) {
            restoreTries = 6;
            lockApplied  = false;
            lastReapply  = 0;
          }
          if (restoreTries > 0 && (nowLock - lastReapply) >= 400) {
            if (masterCfg.valid) {
              const float mn = (masterCfg.cur_min_scale <= 0.0f) ? 1.0f : masterCfg.cur_min_scale;
              const float mx = (masterCfg.cur_max_scale <= 0.0f) ? 1.0f : masterCfg.cur_max_scale;
              vesclock::sendSetTemp(-1, masterCfg, mn, mx);
            }
            if (secondaryEnabled && slaveCfg.valid) {
              const float mn = (slaveCfg.cur_min_scale <= 0.0f) ? 1.0f : slaveCfg.cur_min_scale;
              const float mx = (slaveCfg.cur_max_scale <= 0.0f) ? 1.0f : slaveCfg.cur_max_scale;
              vesclock::sendSetTemp(secondaryCanId, slaveCfg, mn, mx);
            }
            restoreTries--;
            lastReapply = nowLock;
          }
        }
      }

    }

    {
      int mreq = melodyRequest;
      if (mreq >= 0) {
        melodyRequest = -1;
        playMelody(mreq);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(timing::VESC_LOOP_DELAY));
  }
}

static bool otaCallbacksRegistered = false;

static void setupOtaCallbacks() {
  if (otaCallbacksRegistered) return;
  otaCallbacksRegistered = true;

  ArduinoOTA.setHostname("Vesc-OTA");
#ifdef VESC_OTA_PASSWORD
  if (sizeof(VESC_OTA_PASSWORD) > 1) ArduinoOTA.setPassword(VESC_OTA_PASSWORD);
#endif

  ArduinoOTA.onStart([]() {
    using namespace cockpit;
    tft.fillScreen(BG_COLOR);
    drawCockpitHeader("OTA / FLASHING");
    drawCockpitPanel(20, 40, 280, 150, NEON_LIME, "FIRMWARE UPDATE");
    drawCockpitIcon(60, 96, CPI_DOWNLOAD, NEON_LIME);
    tft.setFreeFont(FF1);
    tft.setTextColor(TEXT_MAIN, BG_COLOR);
    tft.setTextDatum(ML_DATUM);
    tft.drawString("Updating firmware...", 80, 92);
    tft.setFreeFont(GLCD);
    tft.setTextColor(TEXT_MUTED, BG_COLOR);
    tft.drawString("do not power off the device", 80, 108);
    tft.drawRoundRect(40, 140, 180, 20, 4, NEON_LIME);
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    using namespace cockpit;
    int p = total ? (int)((progress * 100UL) / total) : 0;
    if (p > 100) p = 100;
    static int lastShown = -1;
    if (p == lastShown) return;
    lastShown = p;

    const int innerW = 176;
    const int fillW = (p * innerW) / 100;
    if (fillW > 0) {
      tft.fillRoundRect(42, 142, fillW, 16, 1, NEON_LIME);
    }

    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", p);
    tft.setFreeFont(FF1);
    tft.setTextColor(NEON_LIME, BG_COLOR);
    tft.setTextDatum(MC_DATUM);
    tft.setTextPadding(50);
    tft.drawString(buf, 245, 151);
    tft.setTextPadding(0);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    using namespace cockpit;
    tft.setFreeFont(FF1);
    tft.setTextColor(NEON_RED, BG_COLOR);
    tft.setTextDatum(MC_DATUM);
    tft.fillRect(40, 170, 240, 20, BG_COLOR);
    switch (error) {
      case OTA_AUTH_ERROR:    tft.drawString("OTA AUTH ERROR",    160, 180); break;
      case OTA_BEGIN_ERROR:   tft.drawString("OTA BEGIN ERROR",   160, 180); break;
      case OTA_CONNECT_ERROR: tft.drawString("OTA CONNECT ERROR", 160, 180); break;
      case OTA_RECEIVE_ERROR: tft.drawString("OTA RECEIVE ERROR", 160, 180); break;
      case OTA_END_ERROR:     tft.drawString("OTA END ERROR",     160, 180); break;
      default:                tft.drawString("OTA ERROR",         160, 180); break;
    }
    Serial.printf("OTA error: %d\n", (int)error);
  });
}

void startOTA() {
  if (otaConnectState == OTA_STATE_CONNECTING) return;

  releaseStatsSprites();
  releaseGaugeCenterSprite();
  releaseGaugeBatterySprite();
  releaseGaugePowerSprite();
  releasePopupChipSprite();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(userWifiSsid.c_str(), userWifiPass.c_str());

  otaConnectState   = OTA_STATE_CONNECTING;
  otaConnectStarted = millis();
  screenNeedsRedraw = true;
}

static void pollOtaConnect() {
  if (otaConnectState != OTA_STATE_CONNECTING) return;

  if (WiFi.status() == WL_CONNECTED) {
    setupOtaCallbacks();
    if (MDNS.begin("vesc-ota")) {
      MDNS.addService("arduino", "tcp", 3232);
    }
    ArduinoOTA.begin();
    otaActive = true;
    otaConnectState = OTA_STATE_READY;
    if (currentScreen == SCREEN_OTA) refreshPopupConfirmUpdate();

    Serial.println("");
    Serial.println("WiFi Connected!");
    Serial.println(WiFi.localIP());
    return;
  }

  if (millis() - otaConnectStarted > timing::OTA_CONNECT_TIMEOUT) {
    otaConnectState = OTA_STATE_FAILED;
    if (currentScreen == SCREEN_OTA) refreshPopupConfirmUpdate();

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    otaActive = false;
    otaConnectStarted = millis();
  }
}

static void handleOtaFailedCleanup() {
  if (otaConnectState != OTA_STATE_FAILED) return;
  if (millis() - otaConnectStarted < 3000) return;
  otaConnectState = OTA_STATE_IDLE;
  if (currentScreen == SCREEN_OTA) {
    currentScreen = SCREEN_SETTINGS;
    screenNeedsRedraw = true;
  }
}

void stopOTA() {
  if (otaActive) {
    ArduinoOTA.end();
    MDNS.end();
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  otaActive = false;
  otaConnectState = OTA_STATE_IDLE;

  Serial.println("OTA & WiFi Disabled");
}

// =============================================================================
//  Android lock remote BLE service
// =============================================================================

static uint8_t currentLockBleStatus() {
  return isLocked ? LOCK_STATUS_LOCKED : LOCK_STATUS_UNLOCKED;
}

static bool lockBlePinMatches(const uint8_t* data, size_t len) {
  if (savedPin.length() != 4 || !isValidPinText(savedPin)) return false;
  if (data == nullptr || len < 5) return false;
  for (int i = 0; i < 4; ++i) {
    char c = (char)data[1 + i];
    if (c < '0' || c > '9' || c != savedPin.charAt(i)) return false;
  }
  return true;
}

static void notifyLockBleStatus(uint8_t status) {
  if (lockStatusChar == nullptr) return;
  lockStatusChar->setValue(&status, 1);
  if (lockBleConnected) lockStatusChar->notify();
}

class LockBleServerCallbacks : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer* server) override {
    (void)server;
    lockBleConnected = true;
    notifyLockBleStatus(currentLockBleStatus());
  }

  void onDisconnect(BLEServer* server) override {
    (void)server;
    lockBleConnected = false;
    if (lockBleActive) BLEDevice::startAdvertising();
  }
};

class LockBleCmdCallbacks : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic* ch) override {
    if (ch == nullptr) return;
    const uint8_t* data = ch->getData();
    const size_t len = ch->getLength();
    if (data == nullptr || len == 0) return;

    uint8_t n = (len > sizeof(lockBleCmdData)) ? (uint8_t)sizeof(lockBleCmdData)
                                               : (uint8_t)len;
    for (uint8_t i = 0; i < n; ++i) lockBleCmdData[i] = data[i];
    lockBleCmdLen = n;
    lockBleCmdPending = true;
  }
};

static LockBleServerCallbacks lockBleServerCallbacks;
static LockBleCmdCallbacks    lockBleCmdCallbacks;

static void processLockBleCommand() {
  if (!lockBleCmdPending) return;

  uint8_t data[8];
  uint8_t len = lockBleCmdLen;
  if (len > sizeof(data)) len = sizeof(data);
  for (uint8_t i = 0; i < len; ++i) data[i] = lockBleCmdData[i];
  lockBleCmdPending = false;

  if (len == 0) return;

  static uint8_t       bleFailedCount  = 0;
  static unsigned long bleLockoutUntil = 0;
  if (bleLockoutUntil != 0 && bleLockoutUntil > millis()) {
    notifyLockBleStatus(LOCK_STATUS_BAD_PIN);
    return;
  }

  if (savedPin.length() != 4 || !isValidPinText(savedPin)) {
    notifyLockBleStatus(LOCK_STATUS_NO_PIN);
    return;
  }
  if (!lockBlePinMatches(data, len)) {
    if (bleFailedCount < 255) ++bleFailedCount;
    if (bleFailedCount >= 3) {
      unsigned long d = (unsigned long)(bleFailedCount - 2) * 3000UL;
      if (d > 30000UL) d = 30000UL;
      bleLockoutUntil = millis() + d;
    }
    notifyLockBleStatus(LOCK_STATUS_BAD_PIN);
    return;
  }
  bleFailedCount  = 0;
  bleLockoutUntil = 0;

  const char cmd = (char)data[0];
  if (cmd == 'L') {
    isLocked = true;
    vescThrottleLockDesired = true;
    requestSettingsSaveNow();
    flushOdometer();
    inputPin = "";
    currentScreen = SCREEN_NUMPAD;
    numpadMode = NUMPAD_UNLOCK;
    screenNeedsRedraw = true;
    notifyLockBleStatus(LOCK_STATUS_LOCKED);
  } else if (cmd == 'U') {
    isLocked = false;
    vescThrottleLockDesired = false;
    autoLockKick = true;
    requestSettingsSaveNow();
    inputPin = "";
    if (currentScreen == SCREEN_NUMPAD) {
      currentScreen = SCREEN_HOME;
      screenNeedsRedraw = true;
    }
    if (!hasRunStartupAnim) {
      hasRunStartupAnim = true;
      animationDone = true;
      precisionTimer = millis();
    }
    notifyLockBleStatus(LOCK_STATUS_UNLOCKED);
  } else {
    notifyLockBleStatus(currentLockBleStatus());
  }
}

static void startLockBleService() {
  if (lockBleActive || btBridgeActive) return;

  BLEDevice::init(kLockBleDeviceName);
  lockBleServer = BLEDevice::createServer();
  if (lockBleServer == nullptr) {
    Serial.println("Lock BLE: createServer() failed");
    return;
  }
  lockBleServer->setCallbacks(&lockBleServerCallbacks);

  BLEService* svc = lockBleServer->createService(kLockServiceUuid);
  if (svc == nullptr) {
    Serial.println("Lock BLE: createService() failed");
    BLEDevice::deinit(false);
    lockBleServer = nullptr;
    return;
  }

  BLECharacteristic* cmdChar = svc->createCharacteristic(
      kLockCmdCharUuid,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  if (cmdChar == nullptr) {
    Serial.println("Lock BLE: create command characteristic failed");
    BLEDevice::deinit(false);
    lockBleServer = nullptr;
    return;
  }
  cmdChar->setCallbacks(&lockBleCmdCallbacks);

  lockStatusChar = svc->createCharacteristic(
      kLockStatusCharUuid,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  if (lockStatusChar == nullptr) {
    Serial.println("Lock BLE: create status characteristic failed");
    BLEDevice::deinit(false);
    lockBleServer = nullptr;
    return;
  }
  lockStatusChar->addDescriptor(new BLE2902());
  uint8_t st = currentLockBleStatus();
  lockStatusChar->setValue(&st, 1);

  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(kLockServiceUuid);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  lockBleActive = true;
  lockBleConnected = false;
  Serial.println("Lock BLE service started as 'VESC-Lock'");
}

static void stopLockBleService() {
  if (!lockBleActive && lockBleServer == nullptr) return;
  BLEDevice::stopAdvertising();
  if (lockBleServer != nullptr && lockBleConnected) lockBleServer->disconnect(0);
  BLEDevice::deinit(false);
  lockBleServer = nullptr;
  lockStatusChar = nullptr;
  lockBleActive = false;
  lockBleConnected = false;
  Serial.println("Lock BLE service stopped");
}

// =============================================================================
//  VESC Tool BT bridge
// =============================================================================

class BtServerCallbacks : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer* server) override {
    (void)server;
    btClientConnected = true;
  }

  void onDisconnect(BLEServer* server) override {
    (void)server;
    btClientConnected = false;
    BLEDevice::startAdvertising();
  }

  void onMtuChanged(BLEServer* server, esp_ble_gatts_cb_param_t* param) override {
    (void)server;
    if (param != nullptr) {
      uint16_t mtu = param->mtu.mtu;
      uint16_t payload = (mtu > 3) ? (mtu - 3) : 20;
      if (payload > 244) payload = 244;
      bleMtuPayload = payload;
    }
  }
};

class BtRxCallbacks : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic* ch) override {
    if (ch == nullptr || bleRxStream == nullptr) return;
    const uint8_t* data = ch->getData();
    size_t len = ch->getLength();
    if (data == nullptr || len == 0) return;
    xStreamBufferSend(bleRxStream, data, len, 0);
  }
};

static BtServerCallbacks bleServerCallbacks;
static BtRxCallbacks     bleRxCallbacks;

static void btBridgeTask(void* param) {
  (void)param;
  uint8_t rxBuf[256];
  uint8_t uartBuf[256];

  VESCSerial.setTimeout(5);

  while (!btBridgeStop) {
    if (bleRxStream != nullptr) {
      size_t got = xStreamBufferReceive(bleRxStream, rxBuf, sizeof(rxBuf),
                                        pdMS_TO_TICKS(2));
      if (got > 0) {
        VESCSerial.write(rxBuf, got);
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(2));
    }

    int availV = VESCSerial.available();
    if (availV > 0 && btClientConnected && bleTxChar != nullptr) {
      uint16_t chunk = bleMtuPayload;
      if (chunk < 20)  chunk = 20;
      if (chunk > (uint16_t)sizeof(uartBuf)) chunk = sizeof(uartBuf);

      int toRead = (availV > (int)chunk) ? (int)chunk : availV;
      int n = VESCSerial.readBytes(uartBuf, toRead);
      if (n > 0) {
        bleTxChar->setValue(uartBuf, n);
        bleTxChar->notify();
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    } else if (availV == 0 && bleRxStream == nullptr) {
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }

  btBridgeTaskHandle = nullptr;
  vTaskDelete(nullptr);
}

static void startBtBridge() {
  if (btBridgeActive) return;

  if (webDashActive) stopWebDash();

  if (lockBleActive) stopLockBleService();

  auto abortStartup = [&]() {
    if (bleRxStream != nullptr) {
      vStreamBufferDelete(bleRxStream);
      bleRxStream = nullptr;
    }
    BLEDevice::deinit(false);
    bleServer = nullptr;
    bleTxChar = nullptr;
    bleRxChar = nullptr;
    bleMtuPayload = 20;
    vescTaskPauseRequested = false;
  };

  vescTaskPauseRequested = true;
  for (int i = 0; i < 20 && !vescTaskPaused; ++i) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  while (VESCSerial.available()) VESCSerial.read();

  if (telemetryMutex && xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
    telemetry.vesc_online = false;
    xSemaphoreGive(telemetryMutex);
  }
  vesc_online = false;

  if (bleRxStream == nullptr) {
    bleRxStream = xStreamBufferCreate(BLE_RX_STREAM_SIZE, 1);
    if (bleRxStream == nullptr) {
      Serial.println("BLE: failed to create RX stream buffer");
      vescTaskPauseRequested = false;
      return;
    }
  } else {
    xStreamBufferReset(bleRxStream);
  }

  releaseStatsSprites();
  releaseGaugeCenterSprite();
  releaseGaugeBatterySprite();
  releaseGaugePowerSprite();
  releasePopupChipSprite();

  BLEDevice::init(kBtDeviceName);
  BLEDevice::setMTU(247);

  bleServer = BLEDevice::createServer();
  if (bleServer == nullptr) {
    Serial.println("BLE: createServer() failed");
    abortStartup();
    return;
  }
  bleServer->setCallbacks(&bleServerCallbacks);

  BLEService* svc = bleServer->createService(kNusServiceUuid);
  if (svc == nullptr) {
    Serial.println("BLE: createService() failed");
    abortStartup();
    return;
  }

  bleTxChar = svc->createCharacteristic(
      kNusTxCharUuid,
      BLECharacteristic::PROPERTY_NOTIFY);
  if (bleTxChar == nullptr) {
    Serial.println("BLE: create TX characteristic failed");
    abortStartup();
    return;
  }
  bleTxChar->addDescriptor(new BLE2902());

  bleRxChar = svc->createCharacteristic(
      kNusRxCharUuid,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  if (bleRxChar == nullptr) {
    Serial.println("BLE: create RX characteristic failed");
    abortStartup();
    return;
  }
  bleRxChar->setCallbacks(&bleRxCallbacks);

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(kNusServiceUuid);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  btBridgeStop = false;
  btClientConnected = false;
  btBridgeActive = true;

  if (btBridgeTaskHandle == nullptr) {
    xTaskCreatePinnedToCore(
        btBridgeTask, "btBridge", 4096, nullptr, 1, &btBridgeTaskHandle, 0);
  }
  Serial.println("BLE bridge started (NUS) as 'VESC-Tool'");
}

static void stopBtBridge() {
  if (!btBridgeActive) return;

  btBridgeStop = true;

  const int waitIter = 50;
  for (int i = 0; i < waitIter && btBridgeTaskHandle != nullptr; ++i) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (btBridgeTaskHandle != nullptr) {
    TaskHandle_t h = btBridgeTaskHandle;
    btBridgeTaskHandle = nullptr;
    vTaskDelete(h);
    Serial.println("BLE bridge task killed by timeout");
  }

  if (bleServer != nullptr) {
    BLEDevice::stopAdvertising();
    if (btClientConnected) {
      bleServer->disconnect(0);
    }
  }
  BLEDevice::deinit(false);
  bleServer = nullptr;
  bleTxChar = nullptr;
  bleRxChar = nullptr;
  bleMtuPayload = 20;

  if (bleRxStream != nullptr) {
    vStreamBufferDelete(bleRxStream);
    bleRxStream = nullptr;
  }

  btBridgeActive    = false;
  btClientConnected = false;

  while (VESCSerial.available()) VESCSerial.read();

  vescTaskPauseRequested = false;
  Serial.println("BLE bridge stopped");
  startLockBleService();
}

// =============================================================================
//  Веб-панель
// =============================================================================

static const char WEB_DASH_HTML[] PROGMEM = R"=====(<!DOCTYPE html>
<html lang="ru"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,viewport-fit=cover">
<title>VESC Telemetry</title>
<style>
:root{--bg:#05070d;--card:#0d121d;--edge:#1c2740;--cyan:#22e1ff;--lime:#39ff8b;--amber:#ffb53d;--red:#ff5470;--muted:#6c7a93;--txt:#eaf2ff}
*{box-sizing:border-box;margin:0;padding:0}
body{background:radial-gradient(900px 500px at 50% -10%,#0b1426 0%,var(--bg) 60%);color:var(--txt);font-family:"Segoe UI",Roboto,system-ui,sans-serif;min-height:100vh;-webkit-text-size-adjust:100%}
.wrap{max-width:640px;margin:0 auto;padding:10px 12px calc(20px + env(safe-area-inset-bottom))}
header{position:sticky;top:0;z-index:5;display:flex;align-items:center;gap:10px;padding:8px 2px;margin-bottom:10px;background:linear-gradient(180deg,var(--bg) 72%,rgba(5,7,13,0));backdrop-filter:blur(4px)}
.logo{width:30px;height:30px;border-radius:9px;background:linear-gradient(135deg,var(--cyan),#1166ff);box-shadow:0 0 16px rgba(34,225,255,.4);display:flex;align-items:center;justify-content:center;font-weight:800;color:#02101c;font-size:16px}
h1{font-size:14px;letter-spacing:.16em;font-weight:700}
.dot{margin-left:auto;display:flex;align-items:center;gap:7px;font-size:11px;letter-spacing:.1em;color:var(--muted)}
.led{width:9px;height:9px;border-radius:50%;background:var(--red);box-shadow:0 0 9px var(--red);transition:background .3s,box-shadow .3s}
.led.on{background:var(--lime);box-shadow:0 0 10px var(--lime)}
.card{background:linear-gradient(180deg,rgba(255,255,255,.03),rgba(255,255,255,0)),var(--card);border:1px solid var(--edge);border-radius:16px;padding:12px 14px}
.hero{display:flex;gap:10px;margin-bottom:10px}
.hero .sp{flex:1.2}.hero .pw{flex:1}
.k{font-size:10px;letter-spacing:.16em;color:var(--muted);text-transform:uppercase}
.big{font-size:54px;font-weight:800;line-height:.95;margin-top:2px;font-variant-numeric:tabular-nums}
.big small{font-size:15px;color:var(--muted);font-weight:600;margin-left:4px}
.pow{font-size:30px;font-weight:800;color:var(--amber);font-variant-numeric:tabular-nums;margin-top:2px}
.pack{font-size:16px;font-weight:700;margin-top:8px;font-variant-numeric:tabular-nums}
.pack .a{color:var(--cyan)}.pack u{color:var(--muted);font-size:12px;text-decoration:none}
.bat .lab{display:flex;justify-content:space-between;font-size:10px;letter-spacing:.14em;color:var(--muted);text-transform:uppercase;margin-bottom:5px}
.bat-wrap{height:12px;border-radius:7px;background:#0a0f18;border:1px solid var(--edge);overflow:hidden}
.bat-fill{height:100%;width:0;background:linear-gradient(90deg,var(--lime),var(--cyan))}
.ctrl{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:8px}
.tg{appearance:none;cursor:pointer;border:1px solid var(--edge);border-radius:13px;padding:11px 12px;font-size:14px;font-weight:700;letter-spacing:.03em;color:var(--txt);background:#10172a;display:flex;align-items:center;gap:10px;transition:transform .08s,filter .15s,border-color .2s}
.tg:active{transform:scale(.97)}.tg:hover{filter:brightness(1.15)}
.tg .ic{width:11px;height:11px;border-radius:50%;background:var(--muted);flex:none;transition:.2s}
.tg.on{border-color:rgba(57,255,139,.45)}.tg.on .ic{background:var(--lime);box-shadow:0 0 9px var(--lime)}
.tg.off{border-color:rgba(255,84,112,.45)}.tg.off .ic{background:var(--red);box-shadow:0 0 9px var(--red)}
.tg small{display:block;font-weight:500;font-size:11px;color:var(--muted);margin-top:2px;text-transform:uppercase;letter-spacing:.08em}
.al{display:flex;align-items:center;gap:9px;font-size:12px;color:var(--muted);background:#0a0f18;border:1px solid var(--edge);border-radius:11px;padding:9px 13px;margin-bottom:12px}
.al .d{width:9px;height:9px;border-radius:50%;background:var(--muted);flex:none}
.al .d.ok{background:var(--lime);box-shadow:0 0 8px var(--lime)}
.al .d.warn{background:var(--amber);box-shadow:0 0 8px var(--amber)}
.al b{color:var(--txt);letter-spacing:.04em}
.al .t{margin-left:auto;font-weight:700;letter-spacing:.05em;color:var(--txt)}
.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}
@media(max-width:560px){.grid{grid-template-columns:repeat(3,1fr)}}
.cell{background:var(--card);border:1px solid var(--edge);border-radius:12px;padding:9px 10px}
.cell .v{font-size:17px;font-weight:700;margin-top:3px;font-variant-numeric:tabular-nums}
.v.cyan{color:var(--cyan)}.v.lime{color:var(--lime)}.v.amber{color:var(--amber)}
.fault{color:var(--lime)}.fault.bad{color:var(--red)}
details{margin-top:10px;background:var(--card);border:1px solid var(--edge);border-radius:14px;overflow:hidden}
summary{cursor:pointer;list-style:none;padding:11px 14px;font-size:12px;letter-spacing:.14em;color:var(--muted);text-transform:uppercase;display:flex;align-items:center;gap:8px}
summary::-webkit-details-marker{display:none}
summary::after{content:"v";font-family:monospace;margin-left:auto;transition:transform .2s}
details[open] summary::after{transform:rotate(180deg)}
details .bd{padding:0 12px 12px}
.foot{text-align:center;color:var(--muted);font-size:10px;letter-spacing:.1em;margin-top:16px}
.toast{position:fixed;left:50%;bottom:calc(18px + env(safe-area-inset-bottom));transform:translateX(-50%) translateY(20px);background:#0d121d;border:1px solid var(--edge);border-radius:12px;padding:10px 18px;font-size:13px;opacity:0;pointer-events:none;transition:opacity .25s,transform .25s;z-index:9;max-width:90%;text-align:center}
.toast.show{opacity:1;transform:translateX(-50%) translateY(0)}
.toast.ok{border-color:rgba(57,255,139,.6)}.toast.err{border-color:rgba(255,84,112,.6)}
</style></head>
<body><div class="wrap">
<header>
  <div class="logo">V</div>
  <h1>VESC TELEMETRY</h1>
  <div class="dot"><span id="led" class="led"></span><span id="st">OFFLINE</span></div>
</header>
<div class="hero">
  <div class="card sp"><div class="k">Speed</div><div class="big"><span id="spd">0.0</span><small>km/h</small></div></div>
  <div class="card pw"><div class="k">Power</div><div class="pow"><span id="pw">0</span> W</div>
    <div class="pack"><span id="volt">0.0</span><u> V</u> &nbsp; <span class="a"><span id="cur">0.0</span></span><u> A</u></div></div>
</div>
<div class="card bat" style="margin-bottom:10px">
  <div class="lab"><span>Battery <b style="color:var(--txt)" id="batp">0</b>%</span><span><span id="trip2">0.00</span> km</span></div>
  <div class="bat-wrap"><div id="bat" class="bat-fill"></div></div>
</div>
<div class="ctrl">
  <button id="bThr" class="tg" onclick="tog('thr')"><span class="ic"></span><span>Газ<small id="bThrS">—</small></span></button>
  <button id="bDisp" class="tg" onclick="tog('disp')"><span class="ic"></span><span>Дисплей<small id="bDispS">—</small></span></button>
</div>
<div class="al"><span id="alD" class="d"></span><b>Авто-лок</b><span class="t" id="alT">—</span></div>
<div class="grid">
  <div class="cell"><div class="k">Trip</div><div class="v cyan"><span id="trip">0.00</span> km</div></div>
  <div class="cell"><div class="k">Energy</div><div class="v"><span id="wh">0</span> Wh</div></div>
  <div class="cell"><div class="k">Charge</div><div class="v"><span id="ah">0.0</span> Ah</div></div>
  <div class="cell"><div class="k">Wh/km</div><div class="v amber"><span id="eff">0</span></div></div>
  <div class="cell"><div class="k">FET</div><div class="v"><span id="tfet">0</span>&deg;</div></div>
  <div class="cell"><div class="k">Motor</div><div class="v"><span id="tmot">0</span>&deg;</div></div>
  <div class="cell"><div class="k">Fault</div><div class="v fault" id="flt">NONE</div></div>
  <div class="cell"><div class="k">2nd ESC</div><div class="v" id="esc2">--</div></div>
</div>
<details>
  <summary>Пики и рекорды</summary>
  <div class="bd">
  <div class="k" style="margin:2px 0 6px">Сессия</div>
  <div class="grid">
    <div class="cell"><div class="k">Max kmh</div><div class="v lime"><span id="mspd">0.0</span></div></div>
    <div class="cell"><div class="k">Max W</div><div class="v lime"><span id="mpw">0</span></div></div>
    <div class="cell"><div class="k">Max A</div><div class="v lime"><span id="mcur">0.0</span></div></div>
    <div class="cell"><div class="k">Uptime</div><div class="v"><span id="up">0:00</span></div></div>
  </div>
  <div class="k" style="margin:10px 0 6px">Рекорды</div>
  <div class="grid">
    <div class="cell"><div class="k">Top kmh</div><div class="v cyan"><span id="aspd">0.0</span></div></div>
    <div class="cell"><div class="k">Top W</div><div class="v cyan"><span id="apw">0</span></div></div>
    <div class="cell"><div class="k">Top A</div><div class="v cyan"><span id="acur">0.0</span></div></div>
    <div class="cell"><div class="k">RAM</div><div class="v"><span id="heap">0</span>k</div></div>
  </div>
  </div>
</details>
<div class="foot">VESC COCKPIT &middot; AP "VESC-Dash" &middot; 192.168.4.1</div>
</div>
<div id="toast" class="toast"></div>
<script>
function f1(x){return (Math.round(x*10)/10).toFixed(1)}
function ut(s){var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),x=s%60;return (h>0?h+":"+("0"+m).slice(-2):m)+":"+("0"+x).slice(-2)}
function $(i){return document.getElementById(i)}
var _tt;
function toast(m,ok){var t=$('toast');t.textContent=m;t.className='toast show '+(ok?'ok':'err');clearTimeout(_tt);_tt=setTimeout(function(){t.className='toast';},2200);}
var S={thrOff:false,dispLock:false};
async function cmd(a){
 try{const r=await fetch('/cmd?a='+a,{cache:'no-store'});const d=await r.json();toast(d.msg,d.ok);tick();}
 catch(e){toast('Нет связи с устройством',false);}
}
function tog(w){if(w=='thr')cmd(S.thrOff?'throttle_on':'throttle_off');else cmd(S.dispLock?'display_unlock':'display_lock');}
var A={spd:0,pw:0,volt:0,cur:0,bat:0},T={spd:0,pw:0,volt:0,cur:0,bat:0},raf=0;
function anim(){var done=true;for(var k in A){var d=T[k]-A[k];if(Math.abs(d)>0.05){A[k]+=d*0.18;done=false;}else A[k]=T[k];}
 $('spd').textContent=f1(A.spd);$('pw').textContent=Math.round(A.pw);$('volt').textContent=f1(A.volt);$('cur').textContent=f1(A.cur);
 $('bat').style.width=Math.max(0,Math.min(100,A.bat))+'%';raf=done?0:requestAnimationFrame(anim);}
function retarget(d){T.spd=d.speed;T.pw=d.power;T.volt=d.voltage;T.cur=d.current;T.bat=d.battery;if(!raf)raf=requestAnimationFrame(anim);}
async function tick(){
 try{
  const r=await fetch('/api',{cache:'no-store'});const d=await r.json();
  $('led').className='led'+(d.online?' on':'');$('st').textContent=d.online?'ONLINE':'NO VESC';
  retarget(d);
  $('batp').textContent=d.battery;$('trip').textContent=d.trip.toFixed(2);$('trip2').textContent=d.trip.toFixed(2);
  $('wh').textContent=Math.round(d.wh);$('ah').textContent=f1(d.ah);$('eff').textContent=Math.round(d.eff);
  $('tfet').textContent=d.tfet;$('tmot').textContent=d.tmot;
  var fe=$('flt');fe.textContent=d.fault;fe.className='v fault'+(d.fault!=='NONE'?' bad':'');
  $('esc2').textContent=d.esc2?('CAN '+d.esc2id):'--';
  $('mspd').textContent=f1(d.mspd);$('mpw').textContent=Math.round(d.mpw);$('mcur').textContent=f1(d.mcur);$('up').textContent=ut(d.up);
  $('aspd').textContent=f1(d.aspd);$('apw').textContent=Math.round(d.apw);$('acur').textContent=f1(d.acur);$('heap').textContent=Math.round(d.heap/1024);
  S.thrOff=d.thrOff;S.dispLock=d.dispLock;
  $('bThr').className='tg '+(d.thrOff?'off':'on');$('bThrS').textContent=d.thrOff?'заблокирован':'активен';
  $('bDisp').className='tg '+(d.dispLock?'off':'on');$('bDispS').textContent=d.dispLock?'закрыт':'открыт';
  var ad=$('alD');ad.className='d'+(d.alEn?(d.alEng?' warn':' ok'):'');
  $('alT').textContent=d.alEn?(d.alEng?'СРАБОТАЛ':('ВКЛ · '+(d.alSec>=60?(d.alSec/60+' мин'):(d.alSec+' с')))):'ВЫКЛ';
 }catch(e){$('st').textContent='...';}
}
setInterval(tick,1000);tick();
</script>
</body></html>)=====";

static void handleWebRoot() {
  webServer.send_P(200, "text/html", WEB_DASH_HTML);
}

static void handleWebApi() {
  const float eff = (trip_km > 0.05f) ? (disp_wattHours / trip_km) : 0.0f;
  const char* faultTxt = (disp_primaryFault > 0) ? faultCodeToText(disp_primaryFault) : "NONE";

  char buf[1400];
  snprintf(buf, sizeof(buf),
    "{\"online\":%s,\"speed\":%.1f,\"power\":%.0f,\"voltage\":%.1f,\"current\":%.1f,"
    "\"battery\":%d,\"trip\":%.2f,\"wh\":%.0f,\"ah\":%.2f,\"eff\":%.0f,"
    "\"tfet\":%d,\"tmot\":%d,\"fault\":\"%s\",\"esc2\":%s,\"esc2id\":%d,"
    "\"mspd\":%.1f,\"mpw\":%.0f,\"mcur\":%.1f,"
    "\"aspd\":%.1f,\"apw\":%.0f,\"acur\":%.1f,"
    "\"up\":%lu,\"heap\":%lu,"
    "\"thrOff\":%s,\"dispLock\":%s,\"alEn\":%s,\"alEng\":%s,\"alSec\":%d}",
    vesc_online ? "true" : "false",
    disp_speedKmh, disp_powerW, disp_voltage, disp_currentA,
    disp_batPercent, disp_trip_km, disp_wattHours, disp_ampHours, eff,
    disp_tempMosfet, disp_tempMotor, faultTxt,
    disp_secondaryOnline ? "true" : "false", disp_secondaryCanId,
    tripMaxSpeedKmh, tripMaxPowerW, tripMaxCurrentA,
    allTimeSpeed, allTimePower, allTimeCurrent,
    (unsigned long)globalSeconds, (unsigned long)ESP.getFreeHeap(),
    (vescThrottleLockDesired || autoThrottleLock) ? "true" : "false",
    isLocked ? "true" : "false",
    autoLockEnabled ? "true" : "false",
    autoThrottleLock ? "true" : "false",
    normalizeAutoLockTimeoutSec(autoLockTimeoutSec));

  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json", buf);
}

static void handleCaptiveRedirect() {
  webServer.sendHeader("Location", "http://192.168.4.1/", true);
  webServer.send(302, "text/plain", "");
}

static void handleWebCmd() {
  const String a = webServer.hasArg("a") ? webServer.arg("a") : String("");
  bool ok = true;
  const char* msg = "OK";

  if (a == "throttle_off") {
    vescThrottleLockDesired = true;
    notifyLockBleStatus(currentLockBleStatus());
  } else if (a == "throttle_on") {
    if (isLocked) { ok = false; msg = "Сначала разблокируйте дисплей"; }
    else {
      vescThrottleLockDesired = false;
      autoLockKick = true;
      notifyLockBleStatus(currentLockBleStatus());
    }
  } else if (a == "display_lock") {
    if (savedPin.length() != 4) { ok = false; msg = "Сначала задайте PIN на устройстве"; }
    else { isLocked = true; vescThrottleLockDesired = true; requestSettingsSaveNow(); flushOdometer(); notifyLockBleStatus(LOCK_STATUS_LOCKED); }
  } else if (a == "display_unlock") {
    isLocked = false;
    vescThrottleLockDesired = false;
    autoLockKick = true;
    requestSettingsSaveNow();
    notifyLockBleStatus(LOCK_STATUS_UNLOCKED);
    inputPin = "";
    if (currentScreen == SCREEN_NUMPAD) {
      currentScreen = SCREEN_HOME;
      screenNeedsRedraw = true;
    }
    if (!hasRunStartupAnim) {
      hasRunStartupAnim = true;
      animationDone = true;
      precisionTimer = millis();
    }
  } else {
    ok = false; msg = "Неизвестная команда";
  }

  char out[128];
  snprintf(out, sizeof(out), "{\"ok\":%s,\"msg\":\"%s\"}", ok ? "true" : "false", msg);
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json", out);
}

static void startWebDash() {
  if (webDashActive) return;

  if (btBridgeActive) stopBtBridge();
  if (otaActive || otaConnectState != OTA_STATE_IDLE) stopOTA();
  // Освобождаем RAM BLE-пульта блокировки на время работы Wi-Fi AP,
  // иначе BLE-стек + AP вместе могут не оставить памяти на softAP/спрайты.
  if (lockBleActive) stopLockBleService();

  releaseStatsSprites();
  releaseGaugeCenterSprite();
  releaseGaugeBatterySprite();
  releaseGaugePowerSprite();
  releasePopupChipSprite();

  WiFi.persistent(false);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  const IPAddress apIp(192, 168, 4, 1);
  const IPAddress apMask(255, 255, 255, 0);
  WiFi.softAPConfig(apIp, apIp, apMask);

  bool apOk = false;
  for (int attempt = 0; attempt < 3 && !apOk; ++attempt) {
    apOk = WiFi.softAP(kWebApSsid, kWebApPass);
    delay(150);
    webApIp = WiFi.softAPIP();
    if (apOk && webApIp != IPAddress(0, 0, 0, 0)) break;
    apOk = false;
    WiFi.softAPdisconnect(true);
    delay(120);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIp, apIp, apMask);
  }
  if (!apOk) {
    apOk = WiFi.softAP(kWebApSsid);
    delay(150);
    webApIp = WiFi.softAPIP();
  }

  if (!apOk || webApIp == IPAddress(0, 0, 0, 0)) {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    webDashActive = false;
    Serial.println("Web dashboard: softAP failed");
    return;
  }

  if (!webRoutesRegistered) {
    webServer.on("/", handleWebRoot);
    webServer.on("/api", handleWebApi);
    webServer.on("/cmd", handleWebCmd);
    webServer.on("/generate_204", handleCaptiveRedirect);
    webServer.on("/gen_204", handleCaptiveRedirect);
    webServer.on("/hotspot-detect.html", handleCaptiveRedirect);
    webServer.on("/library/test/success.html", handleCaptiveRedirect);
    webServer.on("/connecttest.txt", handleCaptiveRedirect);
    webServer.on("/ncsi.txt", handleCaptiveRedirect);
    webServer.on("/redirect", handleCaptiveRedirect);
    webServer.on("/canonical.html", handleCaptiveRedirect);
    webServer.on("/success.txt", handleCaptiveRedirect);
    webServer.onNotFound(handleCaptiveRedirect);
    webRoutesRegistered = true;
  }
  webServer.begin();

  webDns.setErrorReplyCode(DNSReplyCode::NoError);
  webDns.start(WEB_DNS_PORT, "*", apIp);

  webDashActive = true;
  Serial.print("Web dashboard up at http://");
  Serial.println(webApIp);
}

static void stopWebDash() {
  if (!webDashActive) return;
  webDns.stop();
  webServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  webDashActive = false;
  Serial.println("Web dashboard stopped");

  // Возвращаем BLE-пульт блокировки после выключения Wi-Fi.
  startLockBleService();
}

static void handleWebServer() {
  if (!webDashActive) return;
  webDns.processNextRequest();
  webServer.handleClient();
}

static void updateWebDashStatus() {
  if (currentScreen != SCREEN_WEB) return;

  static int lastState = -2;
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 600) return;
  lastCheck = millis();

  const int clients = webDashActive ? (int)WiFi.softAPgetStationNum() : 0;
  const int state   = webDashActive ? clients : -1;
  if (state == lastState) return;
  lastState = state;

  using namespace cockpit;
  const int statusY = 178;
  uint16_t accent;
  char label[28];
  if (!webDashActive) {
    accent = NEON_RED;
    snprintf(label, sizeof(label), "SERVER OFF");
  } else {
    accent = (clients > 0) ? NEON_LIME : NEON_CYAN;
    snprintf(label, sizeof(label), clients == 1 ? "%d CLIENT ONLINE" : "%d CLIENTS ONLINE", clients);
  }
  tft.fillRect(16, statusY - 9, 288, 20, BG_COLOR);
  tft.setFreeFont(GLCD);
  const int gw = tft.textWidth(label);
  const int totalW = 6 + 4 + gw;
  const int dotX = (320 - totalW) / 2 + 3;
  drawCockpitStatusDot(dotX, statusY, accent, label);
}

static void drawWebDashScreen() {
  using namespace cockpit;

  drawCockpitHeader("WEB DASH");

  const uint16_t accent = webDashActive ? NEON_LIME : NEON_CYAN;
  drawCockpitPanel(14, 36, 292, 150, accent, "TELEMETRY SERVER");

  const int bcx = 54, bcy = 92;
  tft.fillCircle(bcx, bcy, 22, GLASS_FILL);
  tft.drawCircle(bcx, bcy, 26, accent);
  drawCockpitIcon(bcx, bcy, CPI_GLOBE, accent);

  const int tx = 92;
  tft.setTextDatum(TL_DATUM);

  tft.setFreeFont(GLCD);
  tft.setTextColor(TEXT_MUTED, BG_COLOR);
  tft.drawString("WI-FI NETWORK", tx, 54);
  tft.setFreeFont(FF1);
  tft.setTextColor(TEXT_MAIN, BG_COLOR);
  tft.drawString(kWebApSsid, tx, 66);

  tft.setFreeFont(GLCD);
  tft.setTextColor(TEXT_MUTED, BG_COLOR);
  tft.drawString("PASSWORD", tx, 94);
  tft.setFreeFont(FF1);
  tft.setTextColor(TEXT_MAIN, BG_COLOR);
  tft.drawString(kWebApPass, tx, 106);

  tft.setFreeFont(GLCD);
  tft.setTextColor(TEXT_MUTED, BG_COLOR);
  tft.drawString("OPEN IN BROWSER", tx, 134);
  tft.setFreeFont(FF1);
  tft.setTextColor(NEON_CYAN, BG_COLOR);
  tft.drawString("http://192.168.4.1", tx, 146);

  const int statusY = 178;
  uint16_t sAccent;
  char label[28];
  if (!webDashActive) {
    sAccent = NEON_RED;
    snprintf(label, sizeof(label), "SERVER OFF");
  } else {
    const int clients = (int)WiFi.softAPgetStationNum();
    sAccent = (clients > 0) ? NEON_LIME : NEON_CYAN;
    snprintf(label, sizeof(label), clients == 1 ? "%d CLIENT ONLINE" : "%d CLIENTS ONLINE", clients);
  }
  tft.setFreeFont(GLCD);
  const int gw = tft.textWidth(label);
  const int totalW = 6 + 4 + gw;
  const int dotX = (320 - totalW) / 2 + 3;
  drawCockpitStatusDot(dotX, statusY, sAccent, label);

  drawCockpitActionBtn(web_layout::BACK_X, web_layout::BTN_Y, web_layout::BACK_W, web_layout::BTN_H,
                       "BACK", 0, CPI_NONE);
}

// =============================================================================
//  Тема 2
// =============================================================================
namespace gauge_batt {
  constexpr int X      = 6;
  constexpr int Y      = 22;
  constexpr int W      = 26;
  constexpr int H      = 172;
  constexpr int CAP_W  = 12;
  constexpr int CAP_H  = 4;
  constexpr int RADIUS = 6;
  constexpr int INSET  = 3;

  constexpr int SPR_W = W;
  constexpr int SPR_H = H + CAP_H + 1;

  constexpr float FILL_EASE        = 0.12f;
  constexpr unsigned long FRAME_MS = 33;
  constexpr float WAVE_SPEED       = 0.045f;
  constexpr float WAVE_AMP_IDLE    = 1.0f;
  constexpr float WAVE_AMP_CHARGE  = 2.6f;
  constexpr float WAVE_K           = 0.28f;
  constexpr float BOLT_PULSE_SPEED = 0.07f;
}

static TFT_eSprite gaugeBattSpr(&tft);
static bool        gaugeBattInit = false;

static bool ensureGaugeBatterySprite() {
  if (gaugeBattInit) return true;
  using namespace gauge_batt;
  gaugeBattSpr.setColorDepth(16);
  if (gaugeBattSpr.createSprite(SPR_W, SPR_H) == nullptr) {
    Serial.println("gauge battery sprite: createSprite failed (low RAM)");
    gaugeBattInit = false;
    return false;
  }
  gaugeBattInit = true;
  return true;
}

static void releaseGaugeBatterySprite() {
  if (!gaugeBattInit) return;
  gaugeBattSpr.deleteSprite();
  gaugeBattInit = false;
}

static float          battFillUi      = -1.0f;
static float          battWavePhase   = 0.0f;
static float          battWaveAmp     = gauge_batt::WAVE_AMP_IDLE;
static float          battBoltAlpha   = 0.0f;
static float          battBoltPhase   = 0.0f;
static unsigned long  battLastFrame   = 0;

static uint16_t lerpRgb565(uint16_t a, uint16_t b, float t) {
  if (t <= 0.0f) return a;
  if (t >= 1.0f) return b;
  const int ar = (a >> 11) & 0x1F, ag = (a >> 5) & 0x3F, ab = a & 0x1F;
  const int br = (b >> 11) & 0x1F, bg = (b >> 5) & 0x3F, bb = b & 0x1F;
  const int r = ar + (int)((br - ar) * t + 0.5f);
  const int g = ag + (int)((bg - ag) * t + 0.5f);
  const int bl = ab + (int)((bb - ab) * t + 0.5f);
  return (uint16_t)((r << 11) | (g << 5) | bl);
}

static uint16_t themeBoostColor(uint16_t c) {
  if (normalizeThemeId(currentTheme) != THEME_LIGHT) return c;
  return lerpRgb565(c, TFT_BLACK, 0.28f);
}

static uint16_t batteryAccentColor(float pct) {
  const uint16_t cBlue   = tft.color565(  40, 140, 255);
  const uint16_t cBlueLo = tft.color565(  60, 170, 255);
  const uint16_t cAmber  = tft.color565( 255, 170,  60);
  const uint16_t cRed    = tft.color565( 255,  70,  60);
  if (pct >= 60.0f) return cBlue;
  if (pct >= 30.0f) return lerpRgb565(cBlueLo, cBlue, (pct - 30.0f) / 30.0f);
  if (pct >= 12.0f) return lerpRgb565(cRed, cAmber, (pct - 12.0f) / 18.0f);
  return cRed;
}

static bool batteryIsCharging() {
  return ui_currentA < -1.5f;
}

static void drawGaugeBatterySprite(float pctNow, float boltPhase, float boltAlpha) {
  using namespace gauge_batt;
  if (!ensureGaugeBatterySprite()) return;

  const uint16_t bg          = BG_COLOR;
  const uint16_t accent      = themeBoostColor(batteryAccentColor(pctNow));
  const uint16_t shellEdge   = lerpRgb565(accent, bg, 0.55f);
  const uint16_t glow        = lerpRgb565(bg, accent, 0.18f);
  const uint16_t liqBottom   = lerpRgb565(bg, accent, 0.55f);
  const uint16_t liqMid      = accent;
  const uint16_t liqTop      = lerpRgb565(accent, 0xFFFF, 0.20f);
  const uint16_t surfaceHi   = lerpRgb565(accent, 0xFFFF, 0.55f);
  const uint16_t surfaceMid  = lerpRgb565(accent, 0xFFFF, 0.30f);
  const uint16_t wellFill    = lerpRgb565(bg, accent, 0.10f);

  gaugeBattSpr.fillSprite(bg);

  const int bodyY  = CAP_H + 1;
  const int innerX = INSET;
  const int innerY = bodyY + INSET;
  const int innerW = W - 2 * INSET;
  const int innerH = H - 2 * INSET;

  const float pctClamped0 = (pctNow < 0) ? 0 : (pctNow > 100 ? 100 : pctNow);
  const int   liquidPx0   = (int)lroundf((pctClamped0 / 100.0f) * innerH);
  const int   headroomPx  = innerH - liquidPx0;
  float headroomT = (float)headroomPx / 6.0f;
  if (headroomT < 0.0f) headroomT = 0.0f;
  if (headroomT > 1.0f) headroomT = 1.0f;
  const float headroomTarget = headroomT * headroomT * (3.0f - 2.0f * headroomT);
  static float headroomGainSmooth = 1.0f;
  headroomGainSmooth += (headroomTarget - headroomGainSmooth) * 0.12f;
  if (fabsf(headroomTarget - headroomGainSmooth) < 0.002f) {
    headroomGainSmooth = headroomTarget;
  }
  const float waveAmpEff   = battWaveAmp * headroomGainSmooth;

  const int capX = (W - CAP_W) / 2;
  gaugeBattSpr.fillRoundRect(capX - 1, 0, CAP_W + 2, CAP_H + 1, 2, glow);
  gaugeBattSpr.fillRoundRect(capX,     1, CAP_W,     CAP_H,     2, shellEdge);

  gaugeBattSpr.fillRoundRect(0, bodyY, W, H, RADIUS, glow);
  gaugeBattSpr.fillRoundRect(1, bodyY + 1, W - 2, H - 2, RADIUS - 1, bg);
  gaugeBattSpr.drawRoundRect(0, bodyY, W, H, RADIUS, shellEdge);
  gaugeBattSpr.drawRoundRect(1, bodyY + 1, W - 2, H - 2, RADIUS - 1,
                             lerpRgb565(shellEdge, bg, 0.4f));

  gaugeBattSpr.fillRoundRect(innerX, innerY, innerW, innerH, RADIUS - 2, wellFill);

  const float pctClamped = (pctNow < 0) ? 0 : (pctNow > 100 ? 100 : pctNow);
  const int   liquidH    = (int)lroundf((pctClamped / 100.0f) * innerH);

  if (liquidH > 0) {
    const int liquidTop = innerY + innerH - liquidH;

    const int CORNER_R = 3;
    const int cornerInset[CORNER_R] = { 3, 2, 1 };

    for (int row = 0; row < liquidH; ++row) {
      const float t = (float)row / (float)liquidH;
      uint16_t c;
      if (t < 0.30f) {
        c = lerpRgb565(liqTop, liqMid, t / 0.30f);
      } else {
        c = lerpRgb565(liqMid, liqBottom, (t - 0.30f) / 0.70f);
      }

      const int distFromBottom = liquidH - 1 - row;
      const int distFromWellTop = (innerH - liquidH) + row;
      int inset = 0;
      if (distFromBottom  < CORNER_R) inset = cornerInset[distFromBottom];
      if (distFromWellTop < CORNER_R) {
        const int topInset = cornerInset[distFromWellTop];
        if (topInset > inset) inset = topInset;
      }

      const int rowX = innerX + inset;
      const int rowW = innerW - 2 * inset;
      if (rowW > 0) {
        gaugeBattSpr.drawFastHLine(rowX, liquidTop + row, rowW, c);
      }
    }

    for (int col = 0; col < innerW; ++col) {
      const float x  = (float)col;
      const float w1 = sinf(WAVE_K * x + battWavePhase) * waveAmpEff;
      const float w2 = sinf(WAVE_K * 0.5f * x - battWavePhase * 0.55f + 1.1f)
                       * (waveAmpEff * 0.45f);
      const int   dy = (int)lroundf(w1 + w2);
      const int   surfaceY = liquidTop + dy;
      const int   colX     = innerX + col;

      if (surfaceY < innerY) continue;

      if (dy < 0) {
        for (int yy = surfaceY + 2; yy < liquidTop; ++yy) {
          gaugeBattSpr.drawPixel(colX, yy, liqTop);
        }
      } else if (dy > 0) {
        for (int yy = liquidTop; yy < surfaceY; ++yy) {
          gaugeBattSpr.drawPixel(colX, yy, wellFill);
        }
      }

      const int distFromTop = surfaceY - innerY;
      if (distFromTop < CORNER_R) {
        const int inset = cornerInset[distFromTop];
        if (col < inset || col >= innerW - inset) {
          continue;
        }
      }

      if (surfaceY - 1 >= innerY) {
        gaugeBattSpr.drawPixel(colX, surfaceY - 1, surfaceMid);
      }
      if (surfaceY < innerY + innerH) {
        gaugeBattSpr.drawPixel(colX, surfaceY, surfaceHi);
      }
      if (surfaceY + 1 < innerY + innerH) {
        gaugeBattSpr.drawPixel(colX, surfaceY + 1, surfaceMid);
      }
    }
  }

  if (boltAlpha > 0.02f) {
    const int bcx = W / 2;
    const int bcy = bodyY + H / 2;
    const int bs  = 11;
    const int p1x = bcx - 1, p1y = bcy - bs;
    const int p2x = bcx + 5, p2y = bcy - 2;
    const int p3x = bcx + 1, p3y = bcy - 1;
    const int p4x = bcx + 4, p4y = bcy + bs;
    const int p5x = bcx - 5, p5y = bcy + 2;
    const int p6x = bcx - 1, p6y = bcy + 1;

    const float pulse = (sinf(boltPhase) + 1.0f) * 0.5f;
    const float a     = boltAlpha * (0.6f + 0.4f * pulse);

    const uint16_t boltCore = tft.color565(255, 255, 255);
    const uint16_t boltHalo = tft.color565(170, 220, 255);
    const uint16_t blend1   = lerpRgb565(liqMid, boltCore, a);
    const uint16_t blend2   = lerpRgb565(liqMid, boltHalo, a * 0.7f);

    gaugeBattSpr.fillTriangle(p1x, p1y, p2x, p2y, p6x, p6y, blend2);
    gaugeBattSpr.fillTriangle(p3x, p3y, p4x, p4y, p5x, p5y, blend2);
    gaugeBattSpr.fillTriangle(p1x + 1, p1y + 1, p2x - 1, p2y, p6x + 1, p6y, blend1);
    gaugeBattSpr.fillTriangle(p3x, p3y, p4x - 1, p4y - 1, p5x + 1, p5y, blend1);
  }

  gaugeBattSpr.pushSprite(X, Y - CAP_H - 1);
}

static void updateGaugeBatteryAnimation() {
  using namespace gauge_batt;

  if (currentScreen != SCREEN_HOME)       return;

  const unsigned long now = millis();
  if (now - battLastFrame < FRAME_MS) return;
  battLastFrame = now;

  const float target = (ui_batPercent < 0) ? 0 : (ui_batPercent > 100 ? 100 : ui_batPercent);
  if (battFillUi < 0.0f) battFillUi = target;
  const float diff = target - battFillUi;
  battFillUi += diff * FILL_EASE;
  if (fabsf(diff) < 0.05f) battFillUi = target;

  const bool  charging = batteryIsCharging();
  const float ampTarget   = charging ? WAVE_AMP_CHARGE : WAVE_AMP_IDLE;
  const float boltTarget  = charging ? 1.0f            : 0.0f;
  battWaveAmp   += (ampTarget  - battWaveAmp)   * 0.10f;
  battBoltAlpha += (boltTarget - battBoltAlpha) * 0.10f;

  const float speed = charging ? (WAVE_SPEED * 1.6f) : WAVE_SPEED;
  battWavePhase += speed * 6.2831853f;
  if (battWavePhase > 1000.0f) battWavePhase -= 1000.0f;

  battBoltPhase += BOLT_PULSE_SPEED;
  if (battBoltPhase > 1000.0f) battBoltPhase -= 1000.0f;

  drawGaugeBatterySprite(battFillUi, battBoltPhase, battBoltAlpha);
}

static void renderGaugeBatteryImmediate() {
  if (currentScreen != SCREEN_HOME) return;
  if (battFillUi < 0.0f) {
    const float t = (ui_batPercent < 0) ? 0 : (ui_batPercent > 100 ? 100 : ui_batPercent);
    battFillUi = t;
  }
  drawGaugeBatterySprite(battFillUi, battBoltPhase, battBoltAlpha);
  battLastFrame = millis();
}

// =============================================================================
//  Тема 2
// =============================================================================
namespace gauge_pwr {
  constexpr int X      = 320 - 6 - 26;
  constexpr int Y      = 22 - 4 - 1;
  constexpr int W      = 26;
  constexpr int H      = 172 + 4 + 1;
  constexpr int RADIUS = 6;
  constexpr int INSET  = 3;
  constexpr int SPR_W  = W;
  constexpr int SPR_H  = H;

  constexpr float FILL_EASE        = 0.14f;
  constexpr unsigned long FRAME_MS = 33;
  constexpr float POWER_FULL_W     = 3500.0f;
}

static TFT_eSprite gaugePwrSpr(&tft);
static bool        gaugePwrInit = false;

static bool ensureGaugePowerSprite() {
  if (gaugePwrInit) return true;
  using namespace gauge_pwr;
  gaugePwrSpr.setColorDepth(16);
  if (gaugePwrSpr.createSprite(SPR_W, SPR_H) == nullptr) {
    Serial.println("gauge power sprite: createSprite failed (low RAM)");
    gaugePwrInit = false;
    return false;
  }
  gaugePwrInit = true;
  return true;
}

static void releaseGaugePowerSprite() {
  if (!gaugePwrInit) return;
  gaugePwrSpr.deleteSprite();
  gaugePwrInit = false;
}

static float          pwrFillUi      = -1.0f;
static unsigned long  pwrLastFrame   = 0;

static uint16_t powerZoneColor(float t) {
  const uint16_t cGreenLo = tft.color565(  10, 120,  35);
  const uint16_t cGreenHi = tft.color565(  60, 230,  90);
  const uint16_t cYellow  = tft.color565( 240, 220,  50);
  const uint16_t cOrange  = tft.color565( 255, 140,  30);
  const uint16_t cRed     = tft.color565( 255,  60,  40);
  if (t <= 0.0f)        return cGreenLo;
  if (t <  0.30f)       return lerpRgb565(cGreenLo, cGreenHi, t / 0.30f);
  if (t <  0.55f)       return cGreenHi;
  if (t <  0.70f)       return lerpRgb565(cGreenHi, cYellow,  (t - 0.55f) / 0.15f);
  if (t <  0.80f)       return cYellow;
  if (t <  0.92f)       return lerpRgb565(cYellow,  cOrange,  (t - 0.80f) / 0.12f);
  return                       lerpRgb565(cOrange,  cRed,     (t - 0.92f) / 0.08f);
}

static uint16_t regenPowerColor(float t) {
  const uint16_t cBlueLo = tft.color565( 10,  70, 170);
  const uint16_t cBlueHi = tft.color565( 80, 200, 255);
  const uint16_t cCyan   = tft.color565( 90, 240, 255);
  if (t <= 0.0f)        return cBlueLo;
  if (t <  0.35f)       return lerpRgb565(cBlueLo, cBlueHi, t / 0.35f);
  if (t <  0.75f)       return cBlueHi;
  if (t <  0.92f)       return lerpRgb565(cBlueHi, cCyan,   (t - 0.75f) / 0.17f);
  return                       cCyan;
}

static void drawGaugePowerSprite(float fill01, bool regen) {
  using namespace gauge_pwr;
  if (!ensureGaugePowerSprite()) return;

  const uint16_t bg        = BG_COLOR;
  const uint16_t accent    = themeBoostColor(regen ? regenPowerColor(fill01) : powerZoneColor(fill01));
  const uint16_t shellEdge  = lerpRgb565(accent, bg, 0.55f);
  const uint16_t glow      = lerpRgb565(bg, accent, 0.16f);
  const uint16_t wellFill  = lerpRgb565(bg, accent, 0.07f);

  gaugePwrSpr.fillSprite(bg);

  const int bodyY  = 0;
  const int bodyH  = H;
  const int innerX = INSET;
  const int innerY = bodyY + INSET;
  const int innerW = W - 2 * INSET;
  const int innerH = bodyH - 2 * INSET;

  gaugePwrSpr.fillRoundRect(0, bodyY, W, bodyH, RADIUS, glow);
  gaugePwrSpr.fillRoundRect(1, bodyY + 1, W - 2, bodyH - 2, RADIUS - 1, bg);
  gaugePwrSpr.drawRoundRect(0, bodyY, W, bodyH, RADIUS, shellEdge);
  gaugePwrSpr.drawRoundRect(1, bodyY + 1, W - 2, bodyH - 2, RADIUS - 1,
                            lerpRgb565(shellEdge, bg, 0.4f));

  gaugePwrSpr.fillRoundRect(innerX, innerY, innerW, innerH, RADIUS - 2, wellFill);

  const float clamped = (fill01 < 0.0f) ? 0.0f : (fill01 > 1.0f ? 1.0f : fill01);
  const int   fillH   = (int)lroundf(clamped * innerH);

  if (fillH > 0) {
    const int fillTop = innerY + innerH - fillH;

    const int CORNER_R = 3;
    const int cornerInset[CORNER_R] = { 3, 2, 1 };

    for (int row = 0; row < fillH; ++row) {
      const int   absRowFromBottom = fillH - 1 - row;
      const float t = (float)absRowFromBottom / (float)(innerH - 1);
      const uint16_t c = themeBoostColor(regen ? regenPowerColor(t) : powerZoneColor(t));

      int inset = 0;
      const int distFromBottom = fillH - 1 - row;
      const int distFromTop    = row;
      if (distFromBottom < CORNER_R) inset = cornerInset[distFromBottom];
      if (fillH >= innerH && distFromTop < CORNER_R) {
        const int topInset = cornerInset[distFromTop];
        if (topInset > inset) inset = topInset;
      }

      const int rowX = innerX + inset;
      const int rowW = innerW - 2 * inset;
      if (rowW > 0) {
        gaugePwrSpr.drawFastHLine(rowX, fillTop + row, rowW, c);
      }
    }

    for (int row = 1; row < fillH - 1; ++row) {
      const int   absRowFromBottom = fillH - 1 - row;
      const float t = (float)absRowFromBottom / (float)(innerH - 1);
      const uint16_t base = themeBoostColor(regen ? regenPowerColor(t) : powerZoneColor(t));
      const uint16_t hi   = lerpRgb565(base, 0xFFFF, 0.22f);
      const int distFromBottom = fillH - 1 - row;
      const int distFromTop    = row;
      int inset = 0;
      if (distFromBottom < CORNER_R) inset = cornerInset[distFromBottom];
      if (fillH >= innerH && distFromTop < CORNER_R) {
        const int topInset = cornerInset[distFromTop];
        if (topInset > inset) inset = topInset;
      }
      const int xPix = innerX + inset + 1;
      if (xPix < innerX + innerW - inset - 1) {
        gaugePwrSpr.drawPixel(xPix, fillTop + row, hi);
      }
    }

    if (fillH >= 2) {
      const float tTop = (float)(fillH - 1) / (float)(innerH - 1);
      const uint16_t cTop = themeBoostColor(regen ? regenPowerColor(tTop) : powerZoneColor(tTop));
      const uint16_t hi   = lerpRgb565(cTop, 0xFFFF, 0.50f);
      const uint16_t mid  = lerpRgb565(cTop, 0xFFFF, 0.22f);

      int topInset = 0;
      if (fillH >= innerH) topInset = cornerInset[0];

      const int yHi  = fillTop;
      const int yMid = fillTop - 1;
      if (yMid >= innerY && innerW - 2 * topInset > 0) {
        gaugePwrSpr.drawFastHLine(innerX + topInset, yMid, innerW - 2 * topInset, mid);
      }
      if (yHi >= innerY && innerW - 2 * topInset > 0) {
        gaugePwrSpr.drawFastHLine(innerX + topInset, yHi, innerW - 2 * topInset, hi);
      }
    }
  }

  gaugePwrSpr.pushSprite(X, Y);
}

static void updateGaugePowerAnimation() {
  using namespace gauge_pwr;

  if (currentScreen != SCREEN_HOME) return;

  const unsigned long now = millis();
  if (now - pwrLastFrame < FRAME_MS) return;
  pwrLastFrame = now;

  const bool targetRegen = (ui_powerW < -1.0f);
  float target = fabsf(ui_powerW) / POWER_FULL_W;
  if (target < 0.0f) target = 0.0f;
  if (target > 1.0f) target = 1.0f;

  if (pwrFillUi < 0.0f) pwrFillUi = target;
  const float diff = target - pwrFillUi;
  pwrFillUi += diff * FILL_EASE;
  if (fabsf(diff) < 0.002f) pwrFillUi = target;

  drawGaugePowerSprite(pwrFillUi, targetRegen);
}

static void renderGaugePowerImmediate() {
  using namespace gauge_pwr;
  if (currentScreen != SCREEN_HOME) return;

  const bool regen = (ui_powerW < -1.0f);
  if (pwrFillUi < 0.0f) {
    float t = fabsf(ui_powerW) / POWER_FULL_W;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    pwrFillUi = t;
  }
  drawGaugePowerSprite(pwrFillUi, regen);
  pwrLastFrame = millis();
}

// =============================================================================
//  Тема 2
// =============================================================================
namespace gauge {
  constexpr int   CENTER_X    = 160;
  constexpr int   CENTER_Y    = 108;
  constexpr int   RADIUS_OUT  = 102;
  constexpr int   ARC_WIDTH   = 14;

  constexpr int   ARC_START   = 30;
  constexpr int   ARC_END     = 330;
  constexpr int   ARC_SPAN    = ARC_END - ARC_START;

  constexpr float    SPEED_MAX    = 120.0f;
  uint16_t TRACK_COLOR  = 0x18C3;

  inline int speedToAngle(float speed) {
    float t = speed / SPEED_MAX;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return ARC_START + (int)lroundf(ARC_SPAN * t);
  }

  inline uint16_t speedColor(float t01) {
    if (t01 < 0) t01 = 0;
    if (t01 > 1) t01 = 1;
    struct ColorStop { float pos; uint8_t r, g, b; };
    static const ColorStop stops[] = {
      { 0.00f,   0, 229, 255 },
      { 0.45f,  57, 255,  90 },
      { 0.75f, 255, 170,   0 },
      { 1.00f, 255,  60,  30 },
    };
    const int N = (int)(sizeof(stops) / sizeof(stops[0]));
    int i = 0;
    while (i < N - 2 && t01 > stops[i + 1].pos) i++;
    const ColorStop& a = stops[i];
    const ColorStop& b = stops[i + 1];
    const float span = b.pos - a.pos;
    float f = (span > 0.0001f) ? (t01 - a.pos) / span : 0.0f;
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    const uint8_t r  = (uint8_t)lroundf(a.r + (b.r - a.r) * f);
    const uint8_t g  = (uint8_t)lroundf(a.g + (b.g - a.g) * f);
    const uint8_t bl = (uint8_t)lroundf(a.b + (b.b - a.b) * f);
    return tft.color565(r, g, bl);
  }
}

static void paintArcTrack() {
  using namespace gauge;
  tft.drawSmoothArc(CENTER_X, CENTER_Y,
                    RADIUS_OUT, RADIUS_OUT - ARC_WIDTH,
                    ARC_START, ARC_END, TRACK_COLOR, BG_COLOR);
}

static void paintArcActive(int toA, float t01) {
  using namespace gauge;
  if (toA > ARC_END)    toA = ARC_END;
  if (toA <= ARC_START) return;
  tft.drawSmoothArc(CENTER_X, CENTER_Y,
                    RADIUS_OUT, RADIUS_OUT - ARC_WIDTH,
                    ARC_START, toA, themeBoostColor(speedColor(t01)), TRACK_COLOR);
}

static void drawGaugeArc(float speedKmh, bool force) {
  using namespace gauge;

  static int lastAngle = ARC_START;

  float t01 = speedKmh / SPEED_MAX;
  if (t01 < 0.0f) t01 = 0.0f;
  if (t01 > 1.0f) t01 = 1.0f;

  const int targetAngle = speedToAngle(speedKmh);

  if (!force && targetAngle == lastAngle) return;

  if (targetAngle < lastAngle) {
    tft.drawSmoothArc(CENTER_X, CENTER_Y,
                      RADIUS_OUT, RADIUS_OUT - ARC_WIDTH,
                      targetAngle, lastAngle, TRACK_COLOR, BG_COLOR);
  }

  paintArcActive(targetAngle, t01);
  lastAngle = targetAngle;
}

static void drawGaugeStatic() {
  using namespace gauge;

  paintArcTrack();

    const int labelsY = 228;
  tft.setFreeFont(GLCD);
  tft.setTextColor(LABEL_COLOR, BG_COLOR);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("VOLT",    55,  labelsY);
  tft.drawString("WATT",    160, labelsY);
  tft.drawString("TRIP KM", 265, labelsY);
}

void drawUIStaticGauge() {
  drawGaugeStatic();
}

static TFT_eSprite gaugeCenterSpr(&tft);
static bool        gaugeCenterInit = false;
static bool        gaugeCenterDirty = true;
static const int   GAUGE_CENTER_W  = 144;
static const int   GAUGE_CENTER_H  = 92;

static void ensureGaugeCenterSprite() {
  if (gaugeCenterInit) return;
  gaugeCenterSpr.setColorDepth(16);
  void* buf = gaugeCenterSpr.createSprite(GAUGE_CENTER_W, GAUGE_CENTER_H);
  if (buf == nullptr) {
    Serial.println("gauge sprite: createSprite failed (low RAM)");
    gaugeCenterInit = false;
    return;
  }
  gaugeCenterInit = true;
  gaugeCenterDirty = true;
}

static void releaseGaugeCenterSprite() {
  if (!gaugeCenterInit) return;
  gaugeCenterSpr.deleteSprite();
  gaugeCenterInit = false;
  gaugeCenterDirty = true;
}

static void drawGaugeCenterText(int speed, bool regen, bool force) {
  using namespace gauge;
  ensureGaugeCenterSprite();
  if (!gaugeCenterInit) return;

  static int  lastSpeed  = -1;
  static bool lastRegen  = false;
  static bool lastOnline = false;

  const bool online = vesc_online;

  if (!force && !gaugeCenterDirty && speed == lastSpeed &&
      regen == lastRegen && online == lastOnline) return;

  lastSpeed = speed;
  lastRegen = regen; lastOnline = online;
  gaugeCenterDirty = false;

  gaugeCenterSpr.fillSprite(BG_COLOR);
  const int cx = GAUGE_CENTER_W / 2;

  uint16_t sCol = regen ? cockpit::NEON_BLUE : VALUE_COLOR;
  gaugeCenterSpr.setTextColor(sCol, BG_COLOR);
  gaugeCenterSpr.setTextFont(7);
  gaugeCenterSpr.setTextDatum(TL_DATUM);
  char spdBuf[6];
  snprintf(spdBuf, sizeof(spdBuf), "%d", speed);
  const int spdW = gaugeCenterSpr.textWidth(spdBuf);
  gaugeCenterSpr.drawString(spdBuf, cx - spdW / 2, 30);

  gaugeCenterSpr.setFreeFont(GLCD);
  gaugeCenterSpr.setTextColor(LABEL_COLOR, BG_COLOR);
  gaugeCenterSpr.setTextDatum(MC_DATUM);
  gaugeCenterSpr.drawString("km/h", cx, GAUGE_CENTER_H - 8);

  gaugeCenterSpr.pushSprite(CENTER_X - GAUGE_CENTER_W / 2,
                            CENTER_Y - GAUGE_CENTER_H / 2);
}

static const int MOTOR_TEMP_WARN_C = 85;
static const int FET_TEMP_WARN_C   = 80;

static void drawGaugeStatusBadges(bool force) {
  using namespace cockpit;

  struct Badge { uint8_t icon; uint16_t color; };
  Badge badges[4];
  int   n = 0;
  uint16_t mask = 0;

  if (autoThrottleLock) {
    badges[n++] = { (uint8_t)CPI_LOCK,   NEON_AMBER }; mask |= 1 << 0;
  }
  if (disp_tempMotor >= MOTOR_TEMP_WARN_C) {
    badges[n++] = { (uint8_t)CPI_THERMO, NEON_RED };   mask |= 1 << 1;
  }
  if (disp_tempMosfet >= FET_TEMP_WARN_C) {
    badges[n++] = { (uint8_t)CPI_CHIP,   NEON_RED };   mask |= 1 << 2;
  }
  if (disp_primaryFault > 0 || disp_secondaryFault > 0) {
    badges[n++] = { (uint8_t)CPI_WARN,   NEON_RED };   mask |= 1 << 3;
  }

  static uint16_t lastMask = 0xFFFF;
  if (!force && mask == lastMask) return;
  lastMask = mask;

  const int cy = 168, r = 10, pitch = 28;
  tft.fillRect(160 - 78, cy - 17, 156, 34, BG_COLOR);

  paintArcTrack();
  drawGaugeArc(ui_speedKmh, true);

  if (n == 0) return;

  const int startX = 160 - ((n - 1) * pitch) / 2;
  for (int i = 0; i < n; i++) {
    const int x = startX + i * pitch;
    tft.drawCircle(x, cy, r + 2, badges[i].color);
    tft.fillCircle(x, cy, r, badges[i].color);
    drawCockpitIcon(x, cy, badges[i].icon, BG_COLOR, badges[i].color);
  }
}

void drawMainValuesGauge(bool forceRedraw = false) {
  using namespace gauge;

  drawGaugeArc(ui_speedKmh, forceRedraw);

  const int  speedInt = (int)lroundf(ui_speedKmh);
  const bool regen    = (ui_currentA < -1.5f);

  drawGaugeCenterText(speedInt, regen, forceRedraw);

  const int valueY = 206;

  static char lastV[12]    = "";
  static char lastW[12]    = "";
  static char lastTrip[12] = "";

  char bufV[12];
  formatDecimal(ui_voltage, 0, bufV, sizeof(bufV));
  if (forceRedraw || strcmp(lastV, bufV) != 0) {
    tft.setFreeFont(FF1);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(VALUE_COLOR, BG_COLOR);
    tft.setTextPadding(70);
    tft.drawString(bufV, 55, valueY);
    tft.setTextPadding(0);
    snprintf(lastV, sizeof(lastV), "%s", bufV);
  }

  char bufW[12];
  formatDecimal(ui_powerW, 0, bufW, sizeof(bufW));
  if (forceRedraw || strcmp(lastW, bufW) != 0) {
    tft.setFreeFont(FF1);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(VALUE_COLOR, BG_COLOR);
    tft.setTextPadding(80);
    tft.drawString(bufW, 160, valueY);
    tft.setTextPadding(0);
    snprintf(lastW, sizeof(lastW), "%s", bufW);
  }

  char bufTrip[12];
  formatDecimal(ui_trip_km, 0, bufTrip, sizeof(bufTrip));
  if (forceRedraw || strcmp(lastTrip, bufTrip) != 0) {
    tft.setFreeFont(FF1);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(VALUE_COLOR, BG_COLOR);
    tft.setTextPadding(90);
    tft.drawString(bufTrip, 265, valueY);
    tft.setTextPadding(0);
    snprintf(lastTrip, sizeof(lastTrip), "%s", bufTrip);
  }

  drawGaugeStatusBadges(forceRedraw);
}

static void drawUIStaticByTheme() {
  drawUIStaticGauge();
}
static void drawMainValuesByTheme(bool force) {
  drawMainValuesGauge(force);
  if (force && currentScreen == SCREEN_HOME) {
    renderGaugeBatteryImmediate();
    renderGaugePowerImmediate();
  }
}

static const int RECORDS_VISIBLE  = 5;
static const int RECORDS_ROW_H    = 28;
static const int RECORDS_LIST_TOP = 42;
int recordsScroll = 0;

int           recSwipeStartY      = 0;
int           recSwipeStartScroll = 0;
bool          recSwipeActive      = false;
unsigned long recLastTouchMs      = 0;

struct RecItem {
  bool        header;
  const char* label;
  uint8_t     icon;
  uint16_t    accent;
  char        value[16];
  bool        valid;
};

static int buildRecordsItems(RecItem* it) {
  using namespace cockpit;
  loadDragyBest();
  int n = 0;

  it[n].header = true;  it[n].label = "LIFETIME"; it[n].icon = CPI_NONE;
  it[n].accent = NEON_MAGENTA; it[n].value[0] = 0; it[n].valid = true; n++;

  it[n].header = false; it[n].label = "ODOMETER"; it[n].icon = CPI_GLOBE;
  it[n].accent = NEON_CYAN; it[n].valid = true;
  if (allTimeOdoKm >= 1000.0f) snprintf(it[n].value, sizeof(it[n].value), "%.0f km", allTimeOdoKm);
  else                        snprintf(it[n].value, sizeof(it[n].value), "%.1f km", allTimeOdoKm);
  n++;

  it[n].header = false; it[n].label = "TOP SPEED"; it[n].icon = CPI_BOLT;
  it[n].accent = NEON_CYAN; it[n].valid = true;
  snprintf(it[n].value, sizeof(it[n].value), "%.1f km/h", allTimeSpeed); n++;

  it[n].header = false; it[n].label = "PEAK POWER"; it[n].icon = CPI_BOLT;
  it[n].accent = NEON_LIME; it[n].valid = true;
  snprintf(it[n].value, sizeof(it[n].value), "%.0f W", allTimePower); n++;

  it[n].header = false; it[n].label = "PEAK CURR"; it[n].icon = CPI_CHIP;
  it[n].accent = NEON_AMBER; it[n].valid = true;
  snprintf(it[n].value, sizeof(it[n].value), "%.1f A", allTimeCurrent); n++;

  it[n].header = false; it[n].label = "FET MAX"; it[n].icon = CPI_THERMO;
  it[n].accent = NEON_RED; it[n].valid = (allTimeFetTemp > 0.5f);
  if (it[n].valid) snprintf(it[n].value, sizeof(it[n].value), "%.0f C", allTimeFetTemp);
  else             snprintf(it[n].value, sizeof(it[n].value), "-- C");
  n++;

  it[n].header = false; it[n].label = "MOTOR MAX"; it[n].icon = CPI_THERMO;
  it[n].accent = NEON_MAGENTA; it[n].valid = (allTimeMotTemp > 0.5f);
  if (it[n].valid) snprintf(it[n].value, sizeof(it[n].value), "%.0f C", allTimeMotTemp);
  else             snprintf(it[n].value, sizeof(it[n].value), "-- C");
  n++;

  it[n].header = true;  it[n].label = "DRAGY  BEST"; it[n].icon = CPI_NONE;
  it[n].accent = NEON_AMBER; it[n].value[0] = 0; it[n].valid = true; n++;

  static char dragyLabels[DRAGY_TARGET_COUNT][8];
  for (int i = 0; i < DRAGY_TARGET_COUNT; ++i) {
    snprintf(dragyLabels[i], sizeof(dragyLabels[i]), "0-%d", dragyTargets[i]);
    it[n].header = false; it[n].label = dragyLabels[i]; it[n].icon = CPI_BOLT;
    it[n].accent = NEON_AMBER; it[n].valid = (dragyBest[i] != 0);
    char t[12];
    dragyFmt(dragyBest[i], t, sizeof(t));
    snprintf(it[n].value, sizeof(it[n].value), "%s s", t);
    n++;
  }

  return n;
}

static const int REC_LIST_X = 12;
static const int REC_LIST_W = 272;
static TFT_eSprite recordsRowSpr(&tft);
static bool recordsRowSprReady = false;
static bool recordsRowSprTried = false;

static void ensureRecordsRowSprite() {
  if (recordsRowSprTried) return;
  recordsRowSprTried = true;
  recordsRowSpr.setColorDepth(16);
  if (recordsRowSpr.createSprite(REC_LIST_W, RECORDS_ROW_H) != nullptr) {
    recordsRowSprReady = true;
  } else {
    Serial.println("records row sprite: createSprite failed (low RAM)");
  }
}

static void renderRecordRowToSprite(const RecItem& r, bool drawDivider) {
  using namespace cockpit;
  recordsRowSpr.fillSprite(BG_COLOR);
  const int yMid = RECORDS_ROW_H / 2;
  if (r.header) {
    recordsRowSpr.fillRect(18 - REC_LIST_X, yMid - 6, 3, 12, r.accent);
    recordsRowSpr.setFreeFont(GLCD);
    recordsRowSpr.setTextColor(r.accent, BG_COLOR);
    recordsRowSpr.setTextDatum(ML_DATUM);
    recordsRowSpr.drawString(r.label, 28 - REC_LIST_X, yMid);
  } else {
    recordsRowSpr.setFreeFont(GLCD);
    recordsRowSpr.setTextColor(TEXT_MUTED, BG_COLOR);
    recordsRowSpr.setTextDatum(ML_DATUM);
    recordsRowSpr.drawString(r.label, 44 - REC_LIST_X, yMid);

    recordsRowSpr.setFreeFont(FF1);
    recordsRowSpr.setTextColor(r.valid ? r.accent : TEXT_MUTED, BG_COLOR);
    recordsRowSpr.setTextDatum(MR_DATUM);
    recordsRowSpr.drawString(r.value, 276 - REC_LIST_X, yMid);
  }
  if (drawDivider) {
    recordsRowSpr.drawFastHLine(20 - REC_LIST_X, RECORDS_ROW_H - 1, 254, GLASS_EDGE);
  }
}

void drawRecordsList() {
  using namespace cockpit;
  RecItem items[14];
  const int total = buildRecordsItems(items);

  int maxScroll = total - RECORDS_VISIBLE;
  if (maxScroll < 0) maxScroll = 0;
  if (recordsScroll < 0) recordsScroll = 0;
  if (recordsScroll > maxScroll) recordsScroll = maxScroll;

  ensureRecordsRowSprite();

  for (int v = 0; v < RECORDS_VISIBLE; ++v) {
    const int  idx     = recordsScroll + v;
    const int  rowTop  = RECORDS_LIST_TOP + v * RECORDS_ROW_H;
    const int  yMidAbs = rowTop + RECORDS_ROW_H / 2;
    const bool has     = (idx < total);

    bool drawDivider = false;
    if (has) {
      const int idxNext = idx + 1;
      drawDivider = (v < RECORDS_VISIBLE - 1 && idxNext < total &&
                     !items[idx].header && !items[idxNext].header);
    }

    if (recordsRowSprReady) {
      if (has) renderRecordRowToSprite(items[idx], drawDivider);
      else     recordsRowSpr.fillSprite(BG_COLOR);
      recordsRowSpr.pushSprite(REC_LIST_X, rowTop);
      if (has && !items[idx].header)
        drawCockpitIcon(26, yMidAbs, items[idx].icon, items[idx].accent);
    } else {
      tft.fillRect(REC_LIST_X, rowTop, REC_LIST_W, RECORDS_ROW_H, BG_COLOR);
      if (has) {
        const RecItem& r = items[idx];
        if (r.header) {
          tft.fillRect(18, yMidAbs - 6, 3, 12, r.accent);
          tft.setFreeFont(GLCD);
          tft.setTextColor(r.accent, BG_COLOR);
          tft.setTextDatum(ML_DATUM);
          tft.drawString(r.label, 28, yMidAbs);
        } else {
          drawCockpitIcon(26, yMidAbs, r.icon, r.accent);
          tft.setFreeFont(GLCD);
          tft.setTextColor(TEXT_MUTED, BG_COLOR);
          tft.setTextDatum(ML_DATUM);
          tft.drawString(r.label, 44, yMidAbs);
          tft.setFreeFont(FF1);
          tft.setTextColor(r.valid ? r.accent : TEXT_MUTED, BG_COLOR);
          tft.setTextDatum(MR_DATUM);
          tft.drawString(r.value, 276, yMidAbs);
        }
        if (drawDivider)
          tft.drawFastHLine(20, rowTop + RECORDS_ROW_H - 1, 254, GLASS_EDGE);
      }
    }
  }

  const uint16_t upCol   = (recordsScroll > 0)         ? NEON_CYAN : GLASS_EDGE;
  const uint16_t downCol = (recordsScroll < maxScroll) ? NEON_CYAN : GLASS_EDGE;
  tft.fillTriangle(288, 56, 304, 56, 296, 44, upCol);
  tft.fillTriangle(288, 162, 304, 162, 296, 174, downCol);

  const int trackTop = 74, trackH = 84;
  tft.fillRect(294, trackTop, 7, trackH, BG_COLOR);
  tft.drawFastVLine(297, trackTop, trackH, GLASS_EDGE);
  int thumbH = (total > 0) ? (trackH * RECORDS_VISIBLE / total) : trackH;
  if (thumbH < 14)     thumbH = 14;
  if (thumbH > trackH) thumbH = trackH;
  int thumbY = trackTop;
  if (maxScroll > 0) thumbY = trackTop + (trackH - thumbH) * recordsScroll / maxScroll;
  tft.fillRoundRect(295, thumbY, 5, thumbH, 2, NEON_MAGENTA);
}

void drawRecordsScreen() {
  using namespace cockpit;
  drawCockpitHeader("RECORDS");
  drawHomeIcon(300, 20);
  drawCockpitPanel(8, 32, 304, 170, NEON_MAGENTA);
  drawCockpitActionBtn(90, 208, 140, 28, "RESET ALL", 2, CPI_WARN);
  drawRecordsList();
}

static void drawRecordsConfirmScreen() {
  using namespace cockpit;

  drawCockpitHeader("RECORDS / RESET");
  drawCockpitPanel(20, 44, 280, 140, NEON_RED, "DANGER ZONE");

  drawCockpitIcon(50, 98, CPI_WARN, NEON_RED);
  tft.setFreeFont(FF1);
  tft.setTextColor(TEXT_MAIN, BG_COLOR);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("Reset all records?", 72, 94);
  tft.setFreeFont(GLCD);
  tft.setTextColor(TEXT_MUTED, BG_COLOR);
  tft.drawString("this cannot be undone", 72, 108);

  drawCockpitActionBtn(46,  134, 108, 44, "YES", 2, CPI_CHECK);
  drawCockpitActionBtn(166, 134, 108, 44, "NO",  3, CPI_CROSS);
}

void updateAnimFrame(int val) {
  disp_speedKmh = val;
  disp_currentA = val;
  disp_powerW = (val / 11) * 111.0f;
  disp_tempMosfet = val;
  disp_tempMotor = val;

  disp_batPercent = val;
  disp_trip_km = val;
  disp_voltage = val / 10.0f;
  syncDisplayToRaw();

  tft.setTextDatum(TC_DATUM);
  drawMainValuesByTheme(false);
  {
    battFillUi = val;
    {
      float t = fabsf(ui_powerW) / gauge_pwr::POWER_FULL_W;
      if (t < 0.0f) t = 0.0f;
      if (t > 1.0f) t = 1.0f;
      pwrFillUi = t;
    }
    renderGaugeBatteryImmediate();
    renderGaugePowerImmediate();
  }
}

void runStartAnimation() {
  hasRunStartupAnim = true;

  tft.fillScreen(BG_COLOR);
  drawUIStaticByTheme();

  for (int v = 11; v <= 99; v += 11) { updateAnimFrame(v); delay(120); }
  for (int v = 99; v >= 0;  v -= 11) { updateAnimFrame(v); delay(120); }

  animationDone = true;
  precisionTimer = millis();
  firstRunAfterAnim = true;

  disp_speedKmh = 0;
  disp_currentA = 0;
  disp_powerW = 0;
  disp_trip_km = 0;
  disp_tempMosfet = 0;
  disp_tempMotor = 0;
  disp_batPercent = 0;
  disp_voltage = 0.0f;
  syncDisplayToRaw();

  drawMainValuesByTheme(true);

  screenNeedsRedraw = false;
}

// =============================================================================
//  Палитра темы
// =============================================================================
static void applyThemePalette() {
  using namespace cockpit;
  if (normalizeThemeId(currentTheme) == THEME_LIGHT) {
    BG_COLOR    = 0xFFFF;
    LABEL_COLOR = 0x528A;
    VALUE_COLOR = 0x0000;

    GLASS_FILL  = 0xEF5D;
    GLASS_HI    = 0xFFFF;
    GLASS_EDGE  = 0xAD55;
    TEXT_MAIN   = 0x0000;
    TEXT_MUTED  = 0x6B4D;
    CHIP_FILL   = 0xDEDB;

    NEON_CYAN    = 0x045B;
    NEON_LIME    = 0x04E4;
    NEON_AMBER   = 0xD3A0;
    NEON_RED     = 0xC0E2;
    NEON_MAGENTA = 0xA812;
    NEON_BLUE    = 0x12D9;

    gauge::TRACK_COLOR = 0xD69A;
  } else {
    BG_COLOR    = TFT_BLACK;
    LABEL_COLOR = 0x7BEF;
    VALUE_COLOR = TFT_WHITE;

    GLASS_FILL  = 0x1082;
    GLASS_HI    = 0x3186;
    GLASS_EDGE  = 0x2145;
    TEXT_MAIN   = 0xFFFF;
    TEXT_MUTED  = 0x9CD3;
    CHIP_FILL   = 0x18E3;

    NEON_CYAN    = 0x07FF;
    NEON_LIME    = 0x07E4;
    NEON_AMBER   = 0xFD20;
    NEON_RED     = 0xF9E0;
    NEON_MAGENTA = 0xF81F;
    NEON_BLUE    = 0x3D3F;

    gauge::TRACK_COLOR = 0x18C3;
  }
}

void setup() {
  Serial.begin(115200);
  VESCSerial.begin(115200, SERIAL_8N1, pins::VESC_RX, pins::VESC_TX);
  vesc.setSerialPort(&VESCSerial);

  prefsReady = prefs.begin("scooter", false);
  if (!prefsReady) {
    Serial.println("Preferences: failed to open NVS, using defaults");
  }
  loadSettingsFromPrefs();
  loadWifiCredentialsFromPrefs();
  applyThemePalette();

  telemetryMutex = xSemaphoreCreateMutex();
  if (telemetryMutex && xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
    telemetry.vesc_online         = false;
    telemetry.lastVescUpdate      = 0;
    telemetry.start_tachometer    = -1;
    telemetry.start_ah            = -1.0f;
    telemetry.start_wh            = -1.0f;
    telemetry.pendingRecordSave   = false;
    telemetry.lastRecordSave      = 0;
    telemetry.primaryFault        = -1;
    telemetry.secondaryFault      = -1;
    telemetry.primaryTempMotor    = NAN;
    telemetry.primaryTempMosfet   = NAN;
    telemetry.secondaryTempMotor  = NAN;
    telemetry.secondaryTempMosfet = NAN;
    telemetry.secondaryOnline     = false;
    telemetry.secondaryCanId      = normalizeCanIdSetting(canId_2nd);
    xSemaphoreGive(telemetryMutex);
  }

  if (prefsReady) {
    allTimeSpeed    = prefs.getFloat("recSpeed",   0);
    allTimePower    = prefs.getFloat("recPower",   0);
    allTimeCurrent  = prefs.getFloat("recCurrent", 0);
    allTimeFetTemp  = prefs.getFloat("recFetT",    0);
    allTimeMotTemp  = prefs.getFloat("recMotT",    0);
    allTimeOdoKm    = prefs.getFloat("odoKm",      0);
    odoSavedKm      = allTimeOdoKm;
  }

  if (telemetryMutex && xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
    telemetry.allTimeSpeed    = allTimeSpeed;
    telemetry.allTimePower    = allTimePower;
    telemetry.allTimeCurrent  = allTimeCurrent;
    telemetry.allTimeFetTemp  = allTimeFetTemp;
    telemetry.allTimeMotTemp  = allTimeMotTemp;
    telemetry.allTimeOdoKm    = allTimeOdoKm;
    xSemaphoreGive(telemetryMutex);
  }

  tft.init();
  tft.setRotation(1);
  tft.invertDisplay(false);
  tft.fillScreen(BG_COLOR);

  touchSPI.begin(pins::TOUCH_CLK, pins::TOUCH_MISO, pins::TOUCH_MOSI, pins::TOUCH_CS);
  ts.begin(touchSPI);
  ts.setRotation(1);

  startLockBleService();

  if (savedPin.length() == 4 && (passAtStartup || restoreLockedAfterBoot)) {
    isLocked          = true;
    vescThrottleLockDesired = true;
    currentScreen     = SCREEN_NUMPAD;
    numpadMode        = NUMPAD_UNLOCK;
    screenNeedsRedraw = true;
  } else {
    runStartAnimation();
    drawMainValuesByTheme(true);
    screenNeedsRedraw = true;
  }

  if (telemetryMutex && vescTaskHandle == nullptr) {
    xTaskCreatePinnedToCore(
        vescTask, "vescTask", 6144, nullptr, 1, &vescTaskHandle, 0);
  }
}

static void updateUptimeClock() {
  while (animationDone && (millis() - precisionTimer >= 1000)) {
    precisionTimer += 1000;
    globalSeconds++;
  }
}

static void enforceLockScreen() {
  if (isLocked && currentScreen != SCREEN_NUMPAD) {
    currentScreen = SCREEN_NUMPAD;
    numpadMode = NUMPAD_UNLOCK;
    screenNeedsRedraw = true;
  }
}

static void handleTouchEvent() {
  if (!ts.touched()) return;

  TS_Point p = ts.getPoint();
  if (p.z < 200) return;

  int touchX = constrain(map(p.x, 200, 3700, 0, 319), 0, 319);
  int touchY = constrain(map(p.y, 200, 3700, 0, 239), 0, 239);

  bool handledHold = false;

  if (currentScreen == SCREEN_CAN_ID) {
    if (inRect(touchX, touchY, 70, 90, 50, 40)) {
      handledHold = true;

      if (millis() - lastHoldTime > timing::TOUCH_HOLD_REPEAT) {
        lastHoldTime = millis();
        lastTouchTime = millis();

        if (normalizeCanIdSetting(canId_2nd) <= 1) canId_2nd = -1;
        else canId_2nd = normalizeCanIdSetting(canId_2nd) - 1;

        drawCanIdValue();
        requestSettingsSave();
      }
    }
    else if (inRect(touchX, touchY, 200, 90, 50, 40)) {
      handledHold = true;

      if (millis() - lastHoldTime > timing::TOUCH_HOLD_REPEAT) {
        lastHoldTime = millis();
        lastTouchTime = millis();

        if (normalizeCanIdSetting(canId_2nd) < 1) canId_2nd = 1;
        else if (normalizeCanIdSetting(canId_2nd) < 127) canId_2nd = normalizeCanIdSetting(canId_2nd) + 1;

        drawCanIdValue();
        requestSettingsSave();
      }
    }
  }
  else if (currentScreen == SCREEN_CELLS) {
    using namespace cells_layout;
    if (inRect(touchX, touchY, MINUS_X, CELLS_STEP_Y, STEP_W, STEP_H)) {
      handledHold = true;
      if (millis() - lastHoldTime > timing::TOUCH_HOLD_REPEAT) {
        lastHoldTime = millis();
        lastTouchTime = millis();
        int v = normalizeBatteryCells(batteryCells);
        if (v > BATTERY_CELLS_MIN) {
          batteryCells = v - 1;
          drawCellsValue();
          requestSettingsSave();
        }
      }
    }
    else if (inRect(touchX, touchY, PLUS_X, CELLS_STEP_Y, STEP_W, STEP_H)) {
      handledHold = true;
      if (millis() - lastHoldTime > timing::TOUCH_HOLD_REPEAT) {
        lastHoldTime = millis();
        lastTouchTime = millis();
        int v = normalizeBatteryCells(batteryCells);
        if (v < BATTERY_CELLS_MAX) {
          batteryCells = v + 1;
          drawCellsValue();
          requestSettingsSave();
        }
      }
    }
    else if (inRect(touchX, touchY, MINUS_X, VCAL_STEP_Y, STEP_W, STEP_H)) {
      handledHold = true;
      if (millis() - lastHoldTime > timing::TOUCH_HOLD_REPEAT) {
        lastHoldTime = millis();
        lastTouchTime = millis();
        int v = normalizeVoltageCalX1000(voltageCalX1000);
        if (v > VOLTAGE_CAL_X1000_MIN) {
          voltageCalX1000 = v - 1;
          drawVoltCalValue();
          requestSettingsSave();
        }
      }
    }
    else if (inRect(touchX, touchY, PLUS_X, VCAL_STEP_Y, STEP_W, STEP_H)) {
      handledHold = true;
      if (millis() - lastHoldTime > timing::TOUCH_HOLD_REPEAT) {
        lastHoldTime = millis();
        lastTouchTime = millis();
        int v = normalizeVoltageCalX1000(voltageCalX1000);
        if (v < VOLTAGE_CAL_X1000_MAX) {
          voltageCalX1000 = v + 1;
          drawVoltCalValue();
          requestSettingsSave();
        }
      }
    }
  }
  else if (currentScreen == SCREEN_RECORDS) {
    const unsigned long nowMs = millis();
    if (inRect(touchX, touchY, 280, 38, 32, 28)) {
      handledHold    = true;
      recSwipeActive = false;
      if (nowMs - lastHoldTime > timing::TOUCH_HOLD_REPEAT) {
        lastHoldTime  = nowMs;
        lastTouchTime = nowMs;
        if (recordsScroll > 0) { recordsScroll--; drawRecordsList(); }
      }
    }
    else if (inRect(touchX, touchY, 280, 160, 32, 28)) {
      handledHold    = true;
      recSwipeActive = false;
      if (nowMs - lastHoldTime > timing::TOUCH_HOLD_REPEAT) {
        lastHoldTime  = nowMs;
        lastTouchTime = nowMs;
        recordsScroll++;
        drawRecordsList();
      }
    }
    else if (touchX >= 12 && touchX < 280 && touchY >= 40 && touchY <= 192 &&
             !(touchX > 250 && touchY < 70)) {
      handledHold = true;
      if (!recSwipeActive || nowMs - recLastTouchMs > 140) {
        recSwipeActive      = true;
        recSwipeStartY      = touchY;
        recSwipeStartScroll = recordsScroll;
      }
      recLastTouchMs = nowMs;
      const int newScroll = recSwipeStartScroll -
                            ((touchY - recSwipeStartY) / RECORDS_ROW_H);
      if (newScroll != recordsScroll) {
        recordsScroll = newScroll;
        drawRecordsList();
      }
    }
  }

  if (!handledHold && millis() - lastTouchTime > timing::TOUCH_DEBOUNCE) {
    lastTouchTime = millis();

    if (currentScreen == SCREEN_HOME) {
      if (autoThrottleLock && inRect(touchX, touchY, 82, 134, 156, 68)) {
        autoLockKick = true;
      } else {
        currentScreen = SCREEN_STATS;
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_STATS) {
      if (touchX > 250 && touchY < 70) {
        currentScreen = SCREEN_SETTINGS;
      } else {
        currentScreen = SCREEN_HOME;
      }
      screenNeedsRedraw = true;
    }
    else if (currentScreen == SCREEN_SETTINGS) {
      const int btnW     = 190;
      const int btnH     = 26;
      const int sX       = (320 - btnW) / 2;
      const int gY       = 6;
      const int sideBtnW = 50;
      const int gapX     = 10;
      const int pitch    = btnH + gY;

      const bool havePin = (savedPin != "");
      const int  contentRows = havePin ? 6 : 5;
      const int  totalH = contentRows * btnH + (contentRows - 1) * gY;
      const int  topPad = 30;
      const int  bottomPad = 18;
      const int  avail = 240 - topPad - bottomPad;
      int        sY    = topPad + (avail - totalH) / 2;
      if (sY < topPad) sY = topPad;

      const int lockY     = havePin ? sY                 : -1;
      const int btRowY    = sY + pitch * (havePin ? 1 : 0);
      const int diagRowY  = sY + pitch * (havePin ? 2 : 1);
      const int motorY    = sY + pitch * (havePin ? 3 : 2);
      const int recordsY  = sY + pitch * (havePin ? 4 : 3);
      const int navY      = sY + pitch * (havePin ? 5 : 4);

      if (touchX > 250 && touchY < 70) {
        stopOTA();
        currentScreen = SCREEN_HOME;
        screenNeedsRedraw = true;
      }
      else if (havePin &&
               inRect(touchX, touchY, sX, lockY - gY / 2, btnW, pitch)) {
        currentScreen = SCREEN_LOCK_CONFIRM;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX, btRowY - gY / 2, btnW, pitch)) {
        if (otaActive) stopOTA();
        startBtBridge();
        if (btBridgeActive) {
          currentScreen = SCREEN_BT_BRIDGE;
          screenNeedsRedraw = true;
        }
      }
      else if (inRect(touchX, touchY, sX, diagRowY - gY / 2, btnW, pitch)) {
        currentScreen = SCREEN_DIAG;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX, motorY - gY / 2, btnW, pitch)) {
        motorCfgLoadDrafts();
        currentScreen = SCREEN_MOTOR_CFG;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX, recordsY - gY / 2, btnW, pitch)) {
        recordsScroll = 0;
        currentScreen = SCREEN_RECORDS;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX, navY, btnW, btnH)) {
        if (btBridgeActive) stopBtBridge();
        if (!wifiCredentialsReady()) {
          currentScreen = SCREEN_WIFI_CFG;
        } else {
          currentScreen = SCREEN_OTA;
        }
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX - sideBtnW - gapX, navY, sideBtnW, btnH)) {
        currentScreen = SCREEN_SETTINGS_3;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX + btnW + gapX, navY, sideBtnW, btnH)) {
        currentScreen = SCREEN_SETTINGS_2;
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_SETTINGS_2) {
      const int btnW     = 190;
      const int btnH     = 26;
      const int sX       = (320 - btnW) / 2;
      const int sideBtnW = 50;
      const int gapX     = 10;

      const bool havePin = (savedPin != "");
      const int contentRows = havePin ? 6 : 5;
      const int topPad = 30;
      const int bottomPad = 18;
      const int avail = 240 - topPad - bottomPad;
      int gY = 6;
      while (contentRows * btnH + (contentRows - 1) * gY > avail && gY > 0) --gY;
      const int pitch  = btnH + gY;
      const int totalH = contentRows * btnH + (contentRows - 1) * gY;
      int       sY     = topPad + (avail - totalH) / 2;
      if (sY < topPad) sY = topPad;

      const int wifiY     = sY;
      const int cellsY    = sY + pitch * 1;
      const int webY      = sY + pitch * 2;
      const int startupY  = havePin ? sY + pitch * 3 : -1;
      const int passY     = sY + pitch * (havePin ? 4 : 3);
      const int navY      = sY + pitch * (havePin ? 5 : 4);

      if (touchX > 250 && touchY < 70) {
        currentScreen = SCREEN_HOME;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX, wifiY - gY / 2, btnW, pitch)) {
        currentScreen = SCREEN_WIFI_CFG;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX, cellsY - gY / 2, btnW, pitch)) {
        currentScreen = SCREEN_CELLS;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX, webY - gY / 2, btnW, pitch)) {
        // Вход на экран веб-даша сразу поднимает сервер/Wi-Fi AP.
        currentScreen = SCREEN_WEB;
        if (!webDashActive) startWebDash();
        screenNeedsRedraw = true;
      }
      else if (havePin &&
               inRect(touchX, touchY, sX, startupY - gY / 2, btnW, pitch)) {
        passAtStartup = !passAtStartup;
        requestSettingsSave();
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX, passY - gY / 2, btnW, btnH + gY / 2)) {
        if (havePin) {
          currentScreen = SCREEN_NUMPAD;
          numpadMode = NUMPAD_ENTER_PIN;
        } else {
          currentScreen = SCREEN_NUMPAD;
          numpadMode = NUMPAD_NEW_PIN;
        }
        inputPin = "";
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX - sideBtnW - gapX, navY, sideBtnW, btnH)) {
        currentScreen = SCREEN_SETTINGS;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX + btnW + gapX, navY, sideBtnW, btnH)) {
        currentScreen = SCREEN_SETTINGS_3;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX, navY, btnW, btnH)) {
        currentTheme = normalizeThemeId((currentTheme + 1) % THEME_COUNT);
        applyThemePalette();
        releaseGaugeCenterSprite();
        releaseGaugeBatterySprite();
        releaseGaugePowerSprite();
        requestSettingsSave();
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_SETTINGS_3) {
      const int btnW     = 190;
      const int btnH     = 26;
      const int sX       = (320 - btnW) / 2;
      const int sideBtnW = 50;
      const int gapX     = 10;
      const int topPad = 30;
      const int bottomPad = 18;
      const int avail = 240 - topPad - bottomPad;
      const int gY     = 10;
      const int totalH = btnH * 5 + gY * 4;
      int       sY     = topPad + (avail - totalH) / 2;
      if (sY < topPad) sY = topPad;
      const int dragyY = sY;
      const int autoY  = sY + (btnH + gY) * 1;
      const int limitY = sY + (btnH + gY) * 2;
      const int musicY = sY + (btnH + gY) * 3;
      const int navY   = sY + (btnH + gY) * 4;

      if (touchX > 250 && touchY < 70) {
        currentScreen = SCREEN_HOME;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX, dragyY - gY / 2, btnW, btnH + gY / 2)) {
        dragyReset();
        currentScreen = SCREEN_DRAGY;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX, autoY - gY / 2, btnW, btnH + gY)) {
        currentScreen = SCREEN_AUTOLOCK;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX, limitY - gY / 2, btnW, btnH + gY)) {
        currentScreen = SCREEN_SPEEDLIMIT;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX, musicY - gY / 2, btnW, btnH + gY)) {
        musicPage = 0;
        currentScreen = SCREEN_MUSIC;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX, navY, btnW, btnH)) {
        currentScreen = SCREEN_CAN_ID;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX - sideBtnW - gapX, navY, sideBtnW, btnH)) {
        currentScreen = SCREEN_SETTINGS_2;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, sX + btnW + gapX, navY, sideBtnW, btnH)) {
        currentScreen = SCREEN_SETTINGS;
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_MUSIC) {
      const int n      = MUSIC_COUNT;
      const int btnW   = 248;
      const int btnX   = (320 - btnW) / 2;
      const int btnH   = 26;
      const int gap    = 6;
      const int top    = 42;
      const int bottom = 236;
      const int total  = n * btnH + (n - 1) * gap;
      int y0 = top + ((bottom - top) - total) / 2;
      if (y0 < top) y0 = top;
      if (touchX > 250 && touchY < 70) {
        currentScreen = SCREEN_SETTINGS_3;
        screenNeedsRedraw = true;
      }
      else {
        for (int i = 0; i < n; i++) {
          if (inRect(touchX, touchY, btnX, y0 + (btnH + gap) * i, btnW, btnH)) {
            melodyRequest = i;
            break;
          }
        }
      }
    }
    else if (currentScreen == SCREEN_DRAGY) {
      if (touchX > 250 && touchY < 70) {
        currentScreen = SCREEN_SETTINGS_3;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, 110, 206, 100, 28)) {
        dragyReset();
        renderDragyDynamic(true);
      }
    }
    else if (currentScreen == SCREEN_DIAG) {
      if (inRect(touchX, touchY, 110, 175, 100, 26)) {
        currentScreen = SCREEN_SETTINGS;
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_LOCK_CONFIRM) {
      if (inRect(touchX, touchY, 46, 128, 108, 44)) {
        isLocked = true;
        vescThrottleLockDesired = true;
        requestSettingsSaveNow();
        flushOdometer();
        notifyLockBleStatus(LOCK_STATUS_LOCKED);
        currentScreen = SCREEN_NUMPAD;
        numpadMode = NUMPAD_UNLOCK;
        inputPin = "";
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, 166, 128, 108, 44)) {
        currentScreen = SCREEN_SETTINGS;
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_NUMPAD) {
      if (numpadMode == NUMPAD_UNLOCK && unlockLockoutUntil > millis()) {
        return;
      }

      int kw = 46;
      int kh = 32;
      int gX = 14;
      int gY = 6;

      int stX = 160 - (3 * kw + 2 * gX) / 2;
      int stY = 78;

      const char* keys[12] = {"1","2","3","4","5","6","7","8","9","C","0","X"};
      for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 3; c++) {
          int idx = r * 3 + c;

          if (inRect(touchX, touchY, stX + c * (kw + gX), stY + r * (kh + gY), kw, kh)) {
            if (numpadMode == NUMPAD_UNLOCK && idx == 11) continue;

            if (idx == 9) {
              inputPin = "";
              drawNumpadInputArea();
            }
            else if (idx == 11) {
              switch (numpadMode) {
                case NUMPAD_CONFIRM_RESET: currentScreen = SCREEN_RECORDS;      break;
                case NUMPAD_NEW_PIN:
                case NUMPAD_ENTER_PIN:     currentScreen = SCREEN_SETTINGS_2;   break;
                default:                   currentScreen = SCREEN_SETTINGS;     break;
              }
              inputPin = "";
              screenNeedsRedraw = true;
            }
            else if (inputPin.length() < 4) {
              inputPin += keys[idx];
              drawNumpadInputArea();
            }

            if (inputPin.length() == 4) {
              if (numpadMode == NUMPAD_NEW_PIN) {
                savedPin = inputPin;
                requestSettingsSave();
                currentScreen = SCREEN_SETTINGS_2;
              }
              else if (numpadMode == NUMPAD_ENTER_PIN) {
                if (inputPin == savedPin) {
                  savedPin = "";
                  requestSettingsSave();
                  currentScreen = SCREEN_SETTINGS_2;
                } else {
                  inputPin = "";
                }
              }
              else if (numpadMode == NUMPAD_CONFIRM_RESET) {
                if (inputPin == savedPin) {
                  resetRecordsAll();
                  currentScreen = SCREEN_RECORDS;
                } else {
                  inputPin = "";
                }
              }
              else if (numpadMode == NUMPAD_UNLOCK) {
                static int  failedUnlockCount = 0;
                if (inputPin == savedPin) {
                  failedUnlockCount = 0;
                  unlockLockoutUntil = 0;
                  isLocked = false;
                  vescThrottleLockDesired = false;
                  autoLockKick = true;
                  requestSettingsSaveNow();
                  notifyLockBleStatus(LOCK_STATUS_UNLOCKED);
                  currentScreen = SCREEN_HOME;
                  if (!hasRunStartupAnim) {
                    hasRunStartupAnim = true;
                    animationDone = true;
                    precisionTimer = millis();
                  }
                } else {
                  inputPin = "";
                  ++failedUnlockCount;
                  unsigned long delayMs = (unsigned long)failedUnlockCount * 3000UL;
                  if (delayMs > 9000UL) delayMs = 9000UL;
                  unlockLockoutUntil = millis() + delayMs;
                  screenNeedsRedraw  = true;
                }
              }

              if (currentScreen != SCREEN_NUMPAD) screenNeedsRedraw = true;
              else drawNumpadInputArea();
            }
          }
        }
      }
    }
    else if (currentScreen == SCREEN_OTA) {
      if (otaActive) {
        if (inRect(touchX, touchY, 106, 135, 108, 35)) {
          stopOTA();
          currentScreen = SCREEN_SETTINGS;
          screenNeedsRedraw = true;
        }
      } else if (otaConnectState == OTA_STATE_CONNECTING ||
                 otaConnectState == OTA_STATE_FAILED) {
        if (inRect(touchX, touchY, 106, 135, 108, 35)) {
          if (otaConnectState == OTA_STATE_CONNECTING) {
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
          }
          otaConnectState = OTA_STATE_IDLE;
          currentScreen = SCREEN_SETTINGS;
          screenNeedsRedraw = true;
        }
      } else {
        if (inRect(touchX, touchY, 46, 135, 108, 35)) {
          screenNeedsRedraw = true;
          startOTA();
        }
        else if (inRect(touchX, touchY, 166, 135, 108, 35)) {
          currentScreen = SCREEN_SETTINGS;
          screenNeedsRedraw = true;
        }
      }
    }
    else if (currentScreen == SCREEN_RECORDS) {
      if (touchX > 250 && touchY < 70) {
        currentScreen = SCREEN_SETTINGS;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, 90, 208, 140, 28)) {
        currentScreen = SCREEN_RECORDS_CONFIRM;
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_RECORDS_CONFIRM) {
      if (inRect(touchX, touchY, 46, 134, 108, 44)) {
        if (savedPin.length() == 4) {
          currentScreen = SCREEN_NUMPAD;
          numpadMode = NUMPAD_CONFIRM_RESET;
          inputPin = "";
          screenNeedsRedraw = true;
        } else {
          resetRecordsAll();
          currentScreen = SCREEN_RECORDS;
          screenNeedsRedraw = true;
        }
      }
      else if (inRect(touchX, touchY, 166, 134, 108, 44)) {
        currentScreen = SCREEN_RECORDS;
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_CAN_ID) {
      if (inRect(touchX, touchY, 110, 145, 100, 35)) {
        currentScreen = SCREEN_SETTINGS_3;
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_AUTOLOCK) {
      using namespace autolock_layout;
      if (touchX > 250 && touchY < 70) {
        currentScreen = SCREEN_SETTINGS_3;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, TOGGLE_X, TOGGLE_Y, TOGGLE_W, TOGGLE_H)) {
        autoLockEnabled = !autoLockEnabled;
        autoLockKick = true;
        requestSettingsSave();
        drawAutoLockToggleBtn();
        drawAutoLockStatusLive(true);
      }
      else if (inRect(touchX, touchY, STEP_MINUS_X, STEP_Y, STEP_W, STEP_H)) {
        autoLockStepTimeout(-1);
        requestSettingsSave();
        drawAutoLockValues();
      }
      else if (inRect(touchX, touchY, STEP_PLUS_X, STEP_Y, STEP_W, STEP_H)) {
        autoLockStepTimeout(+1);
        requestSettingsSave();
        drawAutoLockValues();
      }
      else if (inRect(touchX, touchY, BACK_X, BACK_Y, BACK_W, BACK_H)) {
        currentScreen = SCREEN_SETTINGS_3;
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_SPEEDLIMIT) {
      using namespace autolock_layout;
      if (touchX > 250 && touchY < 70) {
        currentScreen = SCREEN_SETTINGS_3;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, TOGGLE_X, TOGGLE_Y, TOGGLE_W, TOGGLE_H)) {
        speedLimitEnabled = !speedLimitEnabled;
        requestSettingsSave();
        drawSpeedLimitToggleBtn();
        drawSpeedLimitStatusLive(true);
      }
      else if (inRect(touchX, touchY, STEP_MINUS_X, STEP_Y, STEP_W, STEP_H)) {
        speedLimitStepKmh(-1);
        requestSettingsSave();
        drawSpeedLimitValues();
      }
      else if (inRect(touchX, touchY, STEP_PLUS_X, STEP_Y, STEP_W, STEP_H)) {
        speedLimitStepKmh(+1);
        requestSettingsSave();
        drawSpeedLimitValues();
      }
      else if (inRect(touchX, touchY, BACK_X, BACK_Y, BACK_W, BACK_H)) {
        currentScreen = SCREEN_SETTINGS_3;
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_MOTOR_CFG) {
      using namespace motor_cfg_layout;
      if (touchX > 250 && touchY < 70) {
        currentScreen = SCREEN_SETTINGS;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, STEP_MINUS_X, STEP_Y, STEP_W, STEP_H)) {
        motorCfgStep(motorCfgField, -1);
        drawMotorCfgValues();
      }
      else if (inRect(touchX, touchY, STEP_PLUS_X, STEP_Y, STEP_W, STEP_H)) {
        motorCfgStep(motorCfgField, +1);
        drawMotorCfgValues();
      }
      else if (inRect(touchX, touchY, FIELD_BTN_X, FIELD_BTN_Y, FIELD_BTN_W, FIELD_BTN_H)) {
        motorCfgField = (motorCfgField + 1) % 3;
        drawMotorCfgValues();
        drawMotorCfgFieldBtn();
      }
      else if (inRect(touchX, touchY, CARD_X + 8, CARD_Y + 26, CARD_W - 16, ROW_H)) {
        if (motorCfgField != MCF_POLES) {
          motorCfgField = MCF_POLES;
          drawMotorCfgValues();
          drawMotorCfgFieldBtn();
        }
      }
      else if (inRect(touchX, touchY, CARD_X + 8, CARD_Y + 26 + ROW_H, CARD_W - 16, ROW_H)) {
        if (motorCfgField != MCF_WHEEL) {
          motorCfgField = MCF_WHEEL;
          drawMotorCfgValues();
          drawMotorCfgFieldBtn();
        }
      }
      else if (inRect(touchX, touchY, CARD_X + 8, CARD_Y + 26 + ROW_H * 2, CARD_W - 16, ROW_H)) {
        if (motorCfgField != MCF_RATIO) {
          motorCfgField = MCF_RATIO;
          drawMotorCfgValues();
          drawMotorCfgFieldBtn();
        }
      }
      else if (inRect(touchX, touchY, SAVE_X, SAVE_Y, SAVE_W, SAVE_H)) {
        motorCfgCommit();
        currentScreen = SCREEN_SETTINGS;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, CANCEL_X, CANCEL_Y, CANCEL_W, CANCEL_H)) {
        currentScreen = SCREEN_SETTINGS;
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_CELLS) {
      if (inRect(touchX, touchY, cells_layout::CANCEL_X, cells_layout::CANCEL_Y,
                 cells_layout::CANCEL_W, cells_layout::CANCEL_H)) {
        currentScreen = SCREEN_SETTINGS_2;
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_WIFI_CFG) {
      using namespace wifi_cfg_layout;
      if (touchX > 250 && touchY < 70) {
        currentScreen = SCREEN_SETTINGS_2;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, BTN_X, SSID_Y, BTN_W, BTN_H)) {
        kbTarget = KB_TARGET_SSID;
        kbBuffer = userWifiSsid;
        kbShift = false;
        kbSymbols = false;
        currentScreen = SCREEN_WIFI_KEYBOARD;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, BTN_X, PASS_Y, BTN_W, BTN_H)) {
        kbTarget = KB_TARGET_PASS;
        kbBuffer = userWifiPass;
        kbShift = false;
        kbSymbols = false;
        currentScreen = SCREEN_WIFI_KEYBOARD;
        screenNeedsRedraw = true;
      }
      else if (inRect(touchX, touchY, CLEAR_X, CLEAR_Y, CLEAR_W, CLEAR_H)) {
        userWifiSsid = "";
        userWifiPass = "";
        saveWifiCredentialsToPrefs();
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_WIFI_KEYBOARD) {
      using namespace kb_layout;

      if (touchY >= ACT_Y && touchY <= ACT_Y + ACT_H) {
        if (inRect(touchX, touchY, 6, ACT_Y, 46, ACT_H)) {
          kbShift = !kbShift;
          screenNeedsRedraw = true;
        }
        else if (inRect(touchX, touchY, 56, ACT_Y, 40, ACT_H)) {
          kbSymbols = !kbSymbols;
          screenNeedsRedraw = true;
        }
        else if (inRect(touchX, touchY, 100, ACT_Y, 80, ACT_H)) {
          if (kbBuffer.length() < kbMaxLen()) {
            kbBuffer += ' ';
            drawKeyboardInputField();
          }
        }
        else if (inRect(touchX, touchY, 184, ACT_Y, 40, ACT_H)) {
          if (kbBuffer.length() > 0) {
            kbBuffer.remove(kbBuffer.length() - 1);
            drawKeyboardInputField();
          }
        }
        else if (inRect(touchX, touchY, 228, ACT_Y, 40, ACT_H)) {
          if (kbTarget == KB_TARGET_SSID) userWifiSsid = kbBuffer;
          else                            userWifiPass = kbBuffer;
          saveWifiCredentialsToPrefs();
          currentScreen = SCREEN_WIFI_CFG;
          screenNeedsRedraw = true;
        }
        else if (inRect(touchX, touchY, 272, ACT_Y, 42, ACT_H)) {
          currentScreen = SCREEN_WIFI_CFG;
          screenNeedsRedraw = true;
        }
      }
      else {
        bool handled = false;
        for (int r = 0; r < ROWS && !handled; ++r) {
          int ky = ORIGIN_Y + r * (KEY_H + GAP_Y);
          if (touchY < ky || touchY >= ky + KEY_H) continue;
          for (int c = 0; c < COLS && !handled; ++c) {
            int kx = ORIGIN_X + c * (KEY_W + GAP_X);
            if (touchX < kx || touchX >= kx + KEY_W) continue;
            char ch = kbKeyAt(r, c);
            if (ch != 0 && kbBuffer.length() < kbMaxLen()) {
              kbBuffer += ch;
              drawKeyboardInputField();
            }
            handled = true;
          }
        }
      }
    }
    else if (currentScreen == SCREEN_BT_BRIDGE) {
      using namespace bt_layout;
      if (inRect(touchX, touchY, EXIT_X, EXIT_Y, EXIT_W, EXIT_H)) {
        stopBtBridge();
        currentScreen = SCREEN_SETTINGS;
        screenNeedsRedraw = true;
      }
    }
    else if (currentScreen == SCREEN_WEB) {
      using namespace web_layout;
      if (inRect(touchX, touchY, BACK_X, BTN_Y, BACK_W, BTN_H)) {
        // Выход с экрана веб-даша всегда глушит сервер/Wi-Fi AP,
        // иначе активный AP съедает RAM и спрайты батареи/мощности
        // на главном экране не пересоздаются (колонки заряда и ватт пропадают).
        if (webDashActive) stopWebDash();
        currentScreen = SCREEN_SETTINGS_2;
        screenNeedsRedraw = true;
      }
    }
  }
}

static void drawMusicScreen() {
  using namespace cockpit;
  drawCockpitHeader("MUSIC");
  drawHomeIcon(300, 20);

  const int n      = MUSIC_COUNT;
  const int btnW   = 248;
  const int btnX   = (320 - btnW) / 2;
  const int btnH   = 26;
  const int gap    = 6;
  const int top    = 42;
  const int bottom = 236;
  const int total  = n * btnH + (n - 1) * gap;
  int y0 = top + ((bottom - top) - total) / 2;
  if (y0 < top) y0 = top;

  for (int i = 0; i < n; i++) {
    drawCockpitActionBtn(btnX, y0 + (btnH + gap) * i, btnW, btnH,
                         MELODIES[i].name, 0, CPI_BOLT);
  }
}

static void renderCurrentScreen() {
  if (!(screenNeedsRedraw || currentScreen != lastScreen)) return;

  if (!firstRunAfterAnim) tft.fillScreen(BG_COLOR);

  switch (currentScreen) {
    case SCREEN_HOME:
      drawUIStaticByTheme();
      drawMainValuesByTheme(true);
      break;
    case SCREEN_STATS:
      drawStatsStatic();
      lastStatsUpdate = 0;
      updateStatsValues();
      break;
    case SCREEN_SETTINGS:
      drawSettingsStatic();
      break;
    case SCREEN_LOCK_CONFIRM:
      drawPopupConfirmLock();
      break;
    case SCREEN_NUMPAD:
      drawNumpad();
      break;
    case SCREEN_OTA:
      drawPopupConfirmUpdate();
      break;
    case SCREEN_RECORDS:
      drawRecordsScreen();
      break;
    case SCREEN_RECORDS_CONFIRM:
      drawRecordsConfirmScreen();
      break;
    case SCREEN_SETTINGS_2:
      drawSettingsStatic2();
      break;
    case SCREEN_SETTINGS_3:
      drawSettingsStatic3();
      break;
    case SCREEN_DRAGY:
      drawDragyStatic();
      break;
    case SCREEN_DIAG:
      drawPopupDiagStatic();
      updatePopupDiagValues(true);
      break;
    case SCREEN_CAN_ID:
      drawPopupCanId();
      break;
    case SCREEN_CELLS:
      drawPopupCells();
      break;
    case SCREEN_MOTOR_CFG:
      drawMotorCfgScreen();
      break;
    case SCREEN_WIFI_CFG:
      drawWifiCfgScreen();
      break;
    case SCREEN_WIFI_KEYBOARD:
      drawKeyboardScreen();
      break;
    case SCREEN_BT_BRIDGE:
      drawBtBridgeScreen();
      break;
    case SCREEN_WEB:
      drawWebDashScreen();
      break;
    case SCREEN_AUTOLOCK:
      drawAutoLockScreen();
      break;
    case SCREEN_SPEEDLIMIT:
      drawSpeedLimitScreen();
      break;
    case SCREEN_MUSIC:
      drawMusicScreen();
      break;
  }

  if (firstRunAfterAnim) firstRunAfterAnim = false;

  lastScreen = currentScreen;
  screenNeedsRedraw = false;
}

static void handleVescTimeout() {
  if (millis() - lastVescUpdate > timing::VESC_TIMEOUT) {
    vesc_online = false;
    if (telemetryMutex && xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
      telemetry.vesc_online = false;
      xSemaphoreGive(telemetryMutex);
    }
  }
}

static void updateDiagAutoRefresh() {
  if (currentScreen != SCREEN_DIAG) {
    lastDiagRefresh = 0;
    diagStaticDrawn = false;
    diagCachePrimaryMotor[0] = '\0';
    diagCachePrimaryFet[0] = '\0';
    diagCachePrimaryFault[0] = '\0';
    diagCacheSecondaryMotor[0] = '\0';
    diagCacheSecondaryFet[0] = '\0';
    diagCacheSecondaryFault[0] = '\0';
    diagCacheEsc2Header[0] = '\0';
    if (diagCellInit) { diagCellSpr.deleteSprite(); diagCellInit = false; }
    return;
  }

  if (!diagStaticDrawn) {
    drawPopupDiagStatic();
    updatePopupDiagValues(true);
    lastDiagRefresh = millis();
    return;
  }

  if (millis() - lastDiagRefresh >= timing::DIAG_REFRESH) {
    lastDiagRefresh = millis();
    updatePopupDiagValues(false);
  }
}

static void logMissingVescData() {
  static unsigned long lastNoDataLog = 0;

  if (!vesc_online && millis() - lastNoDataLog > timing::NO_DATA_LOG_PERIOD) {
    lastNoDataLog = millis();
    Serial.println("VESC: no data on UART1 (check TX/RX crossed, GND common, 115200, ADC+UART enabled)");
  }
}

static void updateBtBridgeStatus() {
  if (currentScreen != SCREEN_BT_BRIDGE) return;

  static bool lastShown = false;
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 400) return;
  lastCheck = millis();

  const bool now = btClientConnected;
  if (now == lastShown) return;
  lastShown = now;

  using namespace cockpit;
  const int statusY = 178;
  const uint16_t accent = now ? NEON_LIME : NEON_BLUE;
  const char* label = now ? "CONNECTED" : "WAITING FOR CLIENT";

  tft.fillRect(20, statusY - 8, 280, 18, BG_COLOR);

  tft.setFreeFont(GLCD);
  const int textW = tft.textWidth(label);
  const int totalW = 6 + 4 + textW;
  const int dotX = (320 - totalW) / 2 + 3;
  drawCockpitStatusDot(dotX, statusY, accent, label);
}

static void updateUnlockLockoutUi() {
  if (unlockLockoutUntil == 0) return;
  if (currentScreen != SCREEN_NUMPAD || numpadMode != NUMPAD_UNLOCK) return;

  const unsigned long now = millis();
  if (now >= unlockLockoutUntil) {
    unlockLockoutUntil = 0;
    screenNeedsRedraw  = true;
    return;
  }

  const unsigned long remMs  = unlockLockoutUntil - now;
  const unsigned long remSec = (remMs + 999) / 1000;

  static unsigned long lastShownSec = 0xFFFFFFFF;
  if (remSec == lastShownSec) return;
  lastShownSec = remSec;

  char buf[40];
  snprintf(buf, sizeof(buf), "LOCKED - TRY AGAIN IN %lus", remSec);
  tft.setFreeFont(GLCD);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(cockpit::NEON_RED, BG_COLOR);
  tft.drawString(buf, 160, 38);
}

static void updateCellsLiveVoltage() {
  if (currentScreen != SCREEN_CELLS) return;
  using namespace cockpit;
  using namespace cells_layout;
  static unsigned long lastMs = 0;
  if (millis() - lastMs < 400) return;
  lastMs = millis();

  char buf[24];
  if (vesc_online) snprintf(buf, sizeof(buf), "VESC: %.1f V", disp_voltage);
  else             snprintf(buf, sizeof(buf), "VESC: -- V");

  static char lastBuf[24] = "";
  if (strcmp(buf, lastBuf) == 0) return;
  snprintf(lastBuf, sizeof(lastBuf), "%s", buf);

  tft.fillRect(LIVE_X, LIVE_Y - 7, LIVE_W, 14, BG_COLOR);
  tft.setFreeFont(GLCD);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(NEON_LIME, BG_COLOR);
  tft.drawString(buf, 160, LIVE_Y);
}

static void handleBackgroundTasks() {
  processLockBleCommand();
  syncTelemetryToGlobals();
  updateDisplaySmoothing();
  handleVescTimeout();
  updateDiagAutoRefresh();
  updateBtBridgeStatus();
  updateWebDashStatus();
  if (currentScreen == SCREEN_AUTOLOCK) drawAutoLockStatusLive(false);
  handleWebServer();
  updateCellsLiveVoltage();
  updateUnlockLockoutUi();
  pollOtaConnect();
  handleOtaFailedCleanup();
  updateOtaMarqueeBar();
  logMissingVescData();
  saveRecordStatsIfNeeded();
  saveSettingsIfNeeded();
  saveOdometerIfNeeded();
}

void loop() {
  if (otaActive) ArduinoOTA.handle();

  updateUptimeClock();
  enforceLockScreen();
  handleTouchEvent();
  renderCurrentScreen();
  handleBackgroundTasks();

  if (!isLocked) {
    if (currentScreen == SCREEN_HOME) {
      drawMainValuesByTheme(false);
      updateGaugeBatteryAnimation();
      updateGaugePowerAnimation();
    }
    else if (currentScreen == SCREEN_STATS) updateStatsValues();
    else if (currentScreen == SCREEN_DRAGY) {
      updateDragy();
      renderDragyDynamic(false);
    }
  }

  delay(20);
}