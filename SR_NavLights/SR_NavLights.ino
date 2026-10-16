// =====================================================================
//  SR Nav Lights  -  ESP32-C3 SuperMini LED light controller
// =====================================================================
//  v2.0.0  FUNCTIONS: up to 5 light functions (e.g. Nav, Landing,
//          Instrument, Cockpit, Front). Each function has its own LEDs
//          (name, GPIO, invert, dimmable, offset), its own steps (up to
//          12: per LED a pattern + brightness) and its own timing.
//          6-pos switch: each position = a function, WiFi, or nothing.
//          Toggle in a function position: short press = next step
//          (on release), hold 3 s = that function off, press again =
//          back on at the same step. WiFi position: hold 6 s = WiFi
//          on / off. All functions keep running when the switch moves.
//          Invert ON by default (pin LOW = LED on). Pin lists only offer
//          free pins. v1.0.x settings are carried over.
//  Inputs  (receiver PWM 1000-2000 us, internal pull-up): default
//          GPIO0 = 6-position switch, GPIO1 = toggle
//  GPIO8 = onboard blue LED = status (double blink: not activated,
//          fast blink: no receiver signal, slow blink: WiFi on)
//  Settings: WiFi "SR_NavLights", open http://192.168.4.1
//  USB commands for SR Nav Factory (same as v1.0.4):
//    "ID"          -> SRNAV ID=<board id> VER=<x> ACT=0|1|OFF PK=<key start>
//    "ACT <code>"  -> SRNAV ACT=OK  /  SRNAV ACT=FAIL <reason>
//
//  v1.0.0-v1.0.9: see SR_NavLights v1.0.9 (nav steps, sequence, activation,
//          USB commands, assignable pins, GitHub build fixes)
// =====================================================================

#define FW_VERSION "2.0.0"

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include "esp_timer.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "esp_mac.h"
#include "src/ed25519/Ed25519.h"
#include "nav_public_key.h"

static_assert(sizeof(NAV_PUBLIC_KEY_HEX) == 65, "NAV_PUBLIC_KEY_HEX must be 64 hex characters");

// ------------------------------ sizes / codes ------------------------
#define MAX_LED    11         // LED slots (13 usable GPIOs - 2 inputs)
#define N_FN       5          // light functions
#define N_STEP     12         // steps per function
#define FN_NONE    255        // LED not used by any function
#define PIN_NONE   255
#define MAX_PWM    6          // ESP32-C3 has 6 PWM (LEDC) channels
#define LED_STATUS 8          // onboard LED, active LOW
// pins the user may choose (GPIO18/19 = USB, never offered; 11-17 = flash)
static const uint8_t PIN_OK[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21};

// 6-pos position: 0 = nothing, 1..5 = function 1..5, 6 = WiFi (hold)
#define PF_NONE 0
#define PF_WIFI 6

enum { P_OFF = 0, P_STEADY, P_SINGLE, P_DOUBLE, P_TRIPLE, P_FADE, P_FAST, P_ALTA, P_ALTB,
       P_ADA, P_ADB, P_COUNT };
enum { TT_MOM_HIGH = 0, TT_MOM_LOW, TT_2POS, TT_COUNT };

#define CFG_MAGIC 0x53524E4CUL
#define CFG_VER   5
#define ST_MAGIC  0x53524E35UL
#define TEST_TIMEOUT_MS 120000UL

// flash pattern timing: [0]=single [1]=double [2]=triple
struct Flash { uint16_t on, gap, per; };

struct LedC {
  char     name[17];
  uint8_t  fn;            // function 0..4, FN_NONE = not used
  uint8_t  pin;           // GPIO or PIN_NONE
  uint8_t  inv;           // 1 = LED on when pin LOW
  uint8_t  dim;           // 1 = PWM (brightness / fade)
  uint16_t offs;          // pattern offset (ms)
};

struct StepC {
  char    name[13];
  uint8_t pat[MAX_LED];   // pattern per LED slot (only this function's LEDs are used)
  uint8_t br[MAX_LED];    // brightness % per LED slot
};

struct FuncC {
  char     name[17];
  uint8_t  nSteps;        // 1..12: toggle goes 1..nSteps, then back to 1
  Flash    fl[3];
  uint16_t beaconPer, fastPer, altPer;
  StepC    st[N_STEP];
};

struct Cfg {
  uint32_t magic;
  uint16_t ver;
  uint16_t posUs[6];      // value of each 6-pos position
  uint8_t  posFn[6];      // PF_NONE / 1..5 / PF_WIFI
  uint16_t togHi, togLo;
  uint16_t posStableMs;
  uint8_t  togType;
  uint8_t  offHoldS;      // function position: hold = function off (s)
  uint8_t  wifiHoldS;     // WiFi position: hold = WiFi on/off (s)
  uint16_t wifiBootS;
  uint8_t  wifiIdleMin;
  uint8_t  inPin[2];      // 6-pos switch, toggle
  LedC     led[MAX_LED];
  FuncC    fn[N_FN];
  uint16_t pwmHz;
  uint8_t  remember;
  uint8_t  lowPwr;
  char     ssid[24];
  char     pass[24];
};

// ---- v1.0.2 - v1.0.9 layout, only used to carry settings over ----
struct NavStepV4 {
  char    name[13];
  uint8_t pat[6];
  uint8_t br[6];
};

struct CfgV4 {
  uint32_t magic;
  uint16_t ver;
  uint16_t posUs[6];
  uint8_t  posFn[6];
  uint16_t togHi, togLo;
  uint16_t posStableMs;
  uint8_t  togType;
  uint8_t  wifiHoldS;
  uint16_t wifiBootS;
  uint8_t  wifiIdleMin;
  uint8_t  navHoldS;      // nav position: hold = nav off (s)
  NavStepV4 nav[12];
  uint8_t  seq[12];   // step index per slot, SEQ_END = loop back
  uint8_t  grpSteps[3];
  Flash    fl[3];
  uint16_t beaconPer, fastPer, altPer;
  uint16_t offs[6];
  uint8_t  maxPct[9];
  uint8_t  inv[9];
  uint16_t pwmHz;
  uint8_t  remember;
  uint8_t  lowPwr;
  char     ssid[24];
  char     pass[24];
  // ---- v1.0.5 (CFG_VER 4): pin map, appended so v1.0.2-v1.0.4 settings load as they are
  uint8_t  outPinC[9];
  uint8_t  outPwmC[9];
  uint8_t  inPinC[2];
};

struct State {
  uint32_t magic;
  uint8_t  step[N_FN];    // current step of each function
  uint8_t  off[N_FN];     // 1 = function off (3 s hold)
};

static Cfg cfg;
static State st;
static Preferences prefs;
static WebServer server(80);

// pin map in use (set once at power-up from the settings)
static uint8_t ledPin[MAX_LED];
static bool    ledPwm[MAX_LED];
static volatile uint8_t inPin[2];
static bool    statusLedFree = true;
static bool    pinsDefault = false;

// ------------------------------ inputs -------------------------------
static volatile uint32_t riseUs[2], widthUs[2], lastUs[2];
static volatile bool     gotPulse[2];

static void IRAM_ATTR edge(uint8_t ch, uint8_t pin) {
  uint32_t t = (uint32_t)esp_timer_get_time();
  if ((REG_READ(GPIO_IN_REG) >> pin) & 1) {
    riseUs[ch] = t;
  } else {
    uint32_t w = t - riseUs[ch];
    if (w >= 800 && w <= 2200) { widthUs[ch] = w; lastUs[ch] = t; gotPulse[ch] = true; }
  }
}
static void IRAM_ATTR isrPos() { edge(0, inPin[0]); }
static void IRAM_ATTR isrTog() { edge(1, inPin[1]); }

static bool     sigOk[2];
static uint16_t sigW[2];
static int8_t   posCur = -1, posCand = -1;
static uint32_t posCandT;
static int8_t   togState = -1, togRaw = -1;
static uint32_t togRawT;
static bool     pressed = false, holdDone = false;
static uint32_t pressT = 0;

// ------------------------------ misc state ---------------------------
static bool     wifiOn = false, serverUp = false;
static bool     bootWifiPending = false;
static uint32_t lastClientMs = 0, lastWifiPoll = 0;
static bool     testMode = false;
static uint16_t testMask = 0;
static uint32_t testLastMs = 0;
static bool     stDirty = false;
static uint32_t stDirtyT = 0;
static int32_t  lastDuty[MAX_LED];

// ------------------------------ defaults -----------------------------
static void setName(char* dst, size_t n, const char* src) {
  strncpy(dst, src, n - 1);
  dst[n - 1] = 0;
}

static void setLed(Cfg& c, int i, const char* name, uint8_t fn, uint8_t pin, uint8_t dim, uint16_t offs) {
  LedC& L = c.led[i];
  setName(L.name, sizeof(L.name), name);
  L.fn = fn; L.pin = pin; L.inv = 1; L.dim = dim; L.offs = offs;
}

// one step: n LEDs (slot numbers in leds[]) with their pattern and brightness
static void setStep(Cfg& c, int f, int k, const char* name, int n,
                    const uint8_t* leds, const uint8_t* pats, const uint8_t* brs) {
  StepC& S = c.fn[f].st[k];
  setName(S.name, sizeof(S.name), name);
  for (int i = 0; i < n; i++) { S.pat[leds[i]] = pats[i]; S.br[leds[i]] = brs ? brs[i] : 100; }
}

static void defaultTiming(FuncC& F) {
  for (int k = 0; k < 3; k++) { F.fl[k].on = 50; F.fl[k].gap = 80; F.fl[k].per = 1200; }
  F.beaconPer = 1000; F.fastPer = 250; F.altPer = 800;
}

// dimmer function: Off, then 20 % ... 100 % (starts off at the first power-up)
static void dimmerSteps(Cfg& c, int f, uint8_t led, int levels) {
  uint8_t p = P_OFF;
  setStep(c, f, 0, "Off", 1, &led, &p, NULL);
  for (int k = 0; k < levels; k++) {
    char n[13]; uint8_t b = (uint8_t)((k + 1) * 100 / levels);
    snprintf(n, sizeof(n), "%d %%", b);
    p = P_STEADY;
    setStep(c, f, k + 1, n, 1, &led, &p, &b);
  }
  c.fn[f].nSteps = levels + 1;
}

static void defaults(Cfg& c) {
  memset(&c, 0, sizeof(c));
  c.magic = CFG_MAGIC; c.ver = CFG_VER;
  const uint16_t pu[6] = {1000, 1193, 1398, 1602, 1807, 2000};
  for (int i = 0; i < 6; i++) { c.posUs[i] = pu[i]; c.posFn[i] = (i < 5) ? i + 1 : PF_WIFI; }
  c.togHi = 1650; c.togLo = 1350; c.posStableMs = 200;
  c.togType = TT_MOM_HIGH; c.offHoldS = 3; c.wifiHoldS = 6; c.wifiBootS = 60; c.wifiIdleMin = 0;
  c.inPin[0] = 0; c.inPin[1] = 1;

  //                   name            fn  pin dim offs
  setLed(c, 0, "Left red",       0,  5, 0, 0);
  setLed(c, 1, "Right green",    0,  6, 0, 0);
  setLed(c, 2, "Top beacon",     0,  7, 1, 0);
  setLed(c, 3, "Bottom strobe",  0, 10, 0, 0);
  setLed(c, 4, "Tail strobe",    0, 21, 1, 600);
  setLed(c, 5, "Landing",        1, 20, 1, 0);
  setLed(c, 6, "Instrument",     2,  2, 1, 0);
  setLed(c, 7, "Cockpit",        3,  4, 1, 0);
  setLed(c, 8, "Front light",    4,  3, 1, 0);
  setLed(c, 9, "LED 10",   FN_NONE, PIN_NONE, 0, 0);
  setLed(c, 10, "LED 11",  FN_NONE, PIN_NONE, 0, 0);

  const char* FN_NAMES[N_FN] = {"Nav lights", "Landing", "Instrument", "Cockpit", "Front light"};
  for (int f = 0; f < N_FN; f++) {
    setName(c.fn[f].name, sizeof(c.fn[f].name), FN_NAMES[f]);
    defaultTiming(c.fn[f]);
    for (int k = 0; k < N_STEP; k++) {
      char n[13]; snprintf(n, sizeof(n), "Step %d", k + 1);
      setName(c.fn[f].st[k].name, sizeof(c.fn[f].st[k].name), n);
      for (int i = 0; i < MAX_LED; i++) c.fn[f].st[k].br[i] = 100;
    }
  }
  // 1 Nav lights: L red, R green, beacon, bottom, tail
  const uint8_t NV[5] = {0, 1, 2, 3, 4};
  const uint8_t s1[5] = {P_STEADY, P_STEADY, P_FADE,   P_SINGLE, P_SINGLE};
  const uint8_t s2[5] = {P_STEADY, P_STEADY, P_FADE,   P_OFF,    P_STEADY};
  const uint8_t s3[5] = {P_STEADY, P_STEADY, P_FADE,   P_DOUBLE, P_DOUBLE};
  const uint8_t s4[5] = {P_DOUBLE, P_DOUBLE, P_FADE,   P_SINGLE, P_TRIPLE};
  const uint8_t s5[5] = {P_ADA,    P_ADB,    P_FADE,   P_SINGLE, P_TRIPLE};
  const uint8_t s6[5] = {P_ALTA,   P_ALTB,   P_FAST,   P_TRIPLE, P_ALTB};
  const uint8_t s7[5] = {P_FAST,   P_FAST,   P_FAST,   P_FAST,   P_FAST};
  const uint8_t s8[5] = {P_OFF,    P_OFF,    P_OFF,    P_OFF,    P_OFF};
  setStep(c, 0, 0, "Normal",       5, NV, s1, NULL);
  setStep(c, 0, 1, "Taxi",         5, NV, s2, NULL);
  setStep(c, 0, 2, "Takeoff/Land", 5, NV, s3, NULL);
  setStep(c, 0, 3, "Double flash", 5, NV, s4, NULL);
  setStep(c, 0, 4, "Alt double",   5, NV, s5, NULL);
  setStep(c, 0, 5, "Wig-wag",      5, NV, s6, NULL);
  setStep(c, 0, 6, "Full flash",   5, NV, s7, NULL);
  setStep(c, 0, 7, "Off",          5, NV, s8, NULL);
  c.fn[0].nSteps = 8;
  // 2 Landing: Off, On, Taxi 40 %, Wig-wag
  const uint8_t LD = 5;
  uint8_t p, b;
  p = P_OFF;    b = 100; setStep(c, 1, 0, "Off",      1, &LD, &p, &b);
  p = P_STEADY; b = 100; setStep(c, 1, 1, "On",       1, &LD, &p, &b);
  p = P_STEADY; b = 40;  setStep(c, 1, 2, "Taxi 40%", 1, &LD, &p, &b);
  p = P_ALTA;   b = 100; setStep(c, 1, 3, "Wig-wag",  1, &LD, &p, &b);
  c.fn[1].nSteps = 4;
  // 3 Instrument, 4 Cockpit, 5 Front light: Off, 20 % ... 100 %
  dimmerSteps(c, 2, 6, 5);
  dimmerSteps(c, 3, 7, 5);
  dimmerSteps(c, 4, 8, 5);

  c.pwmHz = 1000; c.remember = 1; c.lowPwr = 1;
  strcpy(c.ssid, "SR_NavLights");
  strcpy(c.pass, "12345678");
}

static void saveCfg() { prefs.putBytes("cfg", &cfg, sizeof(Cfg)); }

// v1.0.2 - v1.0.9 settings -> functions (nav steps follow the old sequence)
static void migrateV4(const CfgV4& o, bool hasPins) {
  defaults(cfg);
  memcpy(cfg.posUs, o.posUs, sizeof(o.posUs));
  cfg.togHi = o.togHi; cfg.togLo = o.togLo; cfg.posStableMs = o.posStableMs;
  cfg.togType = o.togType < TT_COUNT ? o.togType : (uint8_t)TT_MOM_HIGH;
  cfg.offHoldS = o.navHoldS ? o.navHoldS : 3;
  cfg.wifiHoldS = o.wifiHoldS ? o.wifiHoldS : 6;
  cfg.wifiBootS = o.wifiBootS; cfg.wifiIdleMin = o.wifiIdleMin;
  cfg.pwmHz = o.pwmHz; cfg.remember = o.remember; cfg.lowPwr = o.lowPwr;
  memcpy(cfg.ssid, o.ssid, sizeof(o.ssid));
  memcpy(cfg.pass, o.pass, sizeof(o.pass));
  // old outputs 0..8 -> LED slots (old order: L red, R green, beacon, bottom, landing, tail, instr, cockpit, front)
  const uint8_t MAP[9] = {0, 1, 2, 3, 5, 4, 6, 7, 8};
  for (int i = 0; i < 9; i++) {
    LedC& L = cfg.led[MAP[i]];
    L.inv = o.inv[i] ? 1 : 0;
    if (hasPins) { L.pin = o.outPinC[i]; L.dim = o.outPwmC[i] ? 1 : 0; }
    if (i < 6) L.offs = o.offs[i];
  }
  if (hasPins) { cfg.inPin[0] = o.inPinC[0]; cfg.inPin[1] = o.inPinC[1]; }
  FuncC& N = cfg.fn[0];
  memcpy(N.fl, o.fl, sizeof(o.fl));
  N.beaconPer = o.beaconPer; N.fastPer = o.fastPer; N.altPer = o.altPer;
  // nav steps in the order of the old sequence (old pattern index 0,1,2,3,5 -> LED 0..4)
  const uint8_t OLDI[5] = {0, 1, 2, 3, 5};
  int n = 0;
  for (int k = 0; k < 12 && o.seq[k] < 12; k++) {
    const NavStepV4& os = o.nav[o.seq[k]];
    StepC& S = N.st[n];
    memcpy(S.name, os.name, sizeof(S.name));
    S.name[sizeof(S.name) - 1] = 0;
    for (int j = 0; j < 5; j++) {
      S.pat[j] = os.pat[OLDI[j]] < P_COUNT ? os.pat[OLDI[j]] : (uint8_t)P_OFF;
      S.br[j] = os.br[OLDI[j]] <= 100 ? os.br[OLDI[j]] : 100;
    }
    n++;
  }
  if (n) N.nSteps = n;
  // old dimmer groups (instrument, cockpit, front) -> functions 3, 4, 5
  const uint8_t GLED[3] = {6, 7, 8};
  for (int g = 0; g < 3; g++) {
    int lv = o.grpSteps[g];
    if (lv < 1 || lv > 10) lv = 5;
    for (int k = 0; k < N_STEP; k++) cfg.fn[2 + g].st[k].pat[GLED[g]] = P_OFF;
    dimmerSteps(cfg, 2 + g, GLED[g], lv);
  }
}

static void loadCfg() {
  size_t len = prefs.getBytesLength("cfg");
  defaults(cfg);
  if (len < 6 || len > sizeof(Cfg) + sizeof(CfgV4)) return;
  uint8_t* buf = (uint8_t*)malloc(len);
  if (!buf) return;
  prefs.getBytes("cfg", buf, len);
  uint32_t magic; uint16_t ver;
  memcpy(&magic, buf, 4); memcpy(&ver, buf + 4, 2);
  if (magic == CFG_MAGIC) {
    if (ver == CFG_VER && len == sizeof(Cfg)) {
      memcpy(&cfg, buf, sizeof(Cfg));
    } else if (ver == 4 && len == sizeof(CfgV4)) {                  // v1.0.5 - v1.0.9
      CfgV4* o = (CfgV4*)calloc(1, sizeof(CfgV4));
      if (o) { memcpy(o, buf, sizeof(CfgV4)); migrateV4(*o, true); saveCfg(); free(o); }
    } else if (ver == 3 && len >= offsetof(CfgV4, outPinC) && len < sizeof(CfgV4)) {  // v1.0.2 - v1.0.4
      CfgV4* o = (CfgV4*)calloc(1, sizeof(CfgV4));
      if (o) { memcpy(o, buf, offsetof(CfgV4, outPinC)); migrateV4(*o, false); saveCfg(); free(o); }
    }
  }
  free(buf);
}

static void fixState() {
  for (int f = 0; f < N_FN; f++) {
    if (cfg.fn[f].nSteps < 1 || cfg.fn[f].nSteps > N_STEP) cfg.fn[f].nSteps = 1;
    if (st.step[f] >= cfg.fn[f].nSteps) st.step[f] = 0;
    if (st.off[f] > 1) st.off[f] = 0;
  }
}

static void loadState() {
  memset(&st, 0, sizeof(st));
  st.magic = ST_MAGIC;
  if (cfg.remember && prefs.getBytesLength("st") == sizeof(State)) {
    State t; prefs.getBytes("st", &t, sizeof(State));
    if (t.magic == ST_MAGIC) st = t;
  }
  fixState();
}
static void markState() { stDirty = true; stDirtyT = millis(); }

// ------------------------------ pin map ------------------------------
static bool pinAllowed(uint8_t p) {
  for (uint8_t a : PIN_OK) if (a == p) return true;
  return false;
}

// checks the pin map of a settings set; "" = good, otherwise the reason
static String pinMapError(const Cfg& c) {
  uint32_t used = 0;
  int pwm = 0;
  for (int k = 0; k < 2; k++) {
    if (!pinAllowed(c.inPin[k])) return String(k ? "Toggle" : "6-pos switch") + " input needs a GPIO";
    if (used & (1UL << c.inPin[k])) return "6-pos switch and toggle are on the same GPIO";
    used |= 1UL << c.inPin[k];
  }
  for (int i = 0; i < MAX_LED; i++) {
    const LedC& L = c.led[i];
    if (L.fn >= N_FN || L.pin == PIN_NONE) continue;
    if (!pinAllowed(L.pin)) return String(L.name) + ": GPIO not allowed";
    if (used & (1UL << L.pin)) return String(L.name) + ": GPIO" + L.pin + " is already used";
    used |= 1UL << L.pin;
    if (L.dim) pwm++;
  }
  if (pwm > MAX_PWM) return String("Too many dimmable LEDs (") + pwm + "), max " + MAX_PWM;
  return "";
}

// takes the pin map from the settings (default map if it is bad) - power-up only
static void applyPins() {
  const Cfg* src = &cfg;
  Cfg* def = NULL;
  if (pinMapError(cfg).length()) {
    pinsDefault = true;
    def = (Cfg*)malloc(sizeof(Cfg));
    if (def) { defaults(*def); src = def; }
  }
  for (int i = 0; i < MAX_LED; i++) {
    const LedC& L = src->led[i];
    bool used = (L.fn < N_FN) && pinAllowed(L.pin);
    ledPin[i] = used ? L.pin : PIN_NONE;
    ledPwm[i] = used && L.dim;
    if (def) { cfg.led[i].pin = def->led[i].pin; cfg.led[i].dim = def->led[i].dim; cfg.led[i].fn = def->led[i].fn; }
  }
  inPin[0] = src->inPin[0]; inPin[1] = src->inPin[1];
  if (def) { cfg.inPin[0] = def->inPin[0]; cfg.inPin[1] = def->inPin[1]; free(def); }
  statusLedFree = inPin[0] != LED_STATUS && inPin[1] != LED_STATUS;
  for (int i = 0; i < MAX_LED; i++) if (ledPin[i] == LED_STATUS) statusLedFree = false;
}

// ------------------------------ activation ---------------------------
//  code = Ed25519 signature of "SRNAV1:" + board ID (12 hex, factory MAC)
#define AUTH_PREFIX "SRNAV1:"
static Preferences aprefs;              // own area: factory reset does not touch it
static char     boardId[13];
static uint8_t  navPub[32];
static bool     authOff = false;        // no public key in this build = no check
static bool     authKeyBad = false;     // public key text is not valid hex
static bool     authOk = false;

static int hexNib(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
static bool parseHex(const char* h, uint8_t* out, size_t n) {
  for (size_t i = 0; i < n; i++) {
    int a = hexNib(h[2 * i]), b = (a < 0) ? -1 : hexNib(h[2 * i + 1]);
    if (a < 0 || b < 0) return false;
    out[i] = (uint8_t)(a * 16 + b);
  }
  return true;
}
static bool checkCode(const uint8_t* sig) {
  if (authKeyBad) return false;
  char msg[24];
  snprintf(msg, sizeof(msg), "%s%s", AUTH_PREFIX, boardId);
  return Ed25519::verify(sig, navPub, msg, strlen(msg));
}
static void authInit() {
  uint8_t mac[6] = {0};
  esp_efuse_mac_get_default(mac);
  snprintf(boardId, sizeof(boardId), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  bool zero = true;
  for (size_t i = 0; i < 64; i++) if (NAV_PUBLIC_KEY_HEX[i] != '0') zero = false;
  if (zero) { authOff = true; authOk = true; return; }
  authKeyBad = !parseHex(NAV_PUBLIC_KEY_HEX, navPub, 32);
  aprefs.begin("srauth", false);
  if (aprefs.getBytesLength("sig") == 64) {
    uint8_t sig[64];
    aprefs.getBytes("sig", sig, 64);
    authOk = checkCode(sig);
  }
}

// ------------------------------ WiFi ---------------------------------
static void wifiStart() {
  if (wifiOn) return;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(cfg.ssid, cfg.pass);
  if (cfg.lowPwr) WiFi.setTxPower(WIFI_POWER_8_5dBm);   // SuperMini antenna fix
  if (!serverUp) { server.begin(); serverUp = true; }
  wifiOn = true;
  lastClientMs = millis();
}

static void endTest() { testMode = false; testMask = 0; }

static void wifiStop() {
  if (!wifiOn) return;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiOn = false;
  endTest();
}

// ------------------------------ switch actions -----------------------
static int fnOfPos(int pos) {             // function of a 6-pos position, -1 = none
  if (pos < 0) return -1;
  uint8_t p = cfg.posFn[pos];
  return (p >= 1 && p <= N_FN) ? p - 1 : -1;
}
static bool posIsWifi() { return posCur >= 0 && cfg.posFn[posCur] == PF_WIFI; }

static void fnNext(int f) {
  if (f < 0) return;
  if (st.off[f]) st.off[f] = 0;                       // back on at the same step
  else st.step[f] = (st.step[f] + 1) % cfg.fn[f].nSteps;
  markState();
}

static uint8_t nearestPos(uint16_t w) {
  uint8_t b = 0; int bd = 100000;
  for (uint8_t i = 0; i < 6; i++) {
    int d = abs((int)w - (int)cfg.posUs[i]);
    if (d < bd) { bd = d; b = i; }
  }
  return b;
}

static bool isPressed(int8_t s) {
  if (s < 0) return false;
  if (cfg.togType == TT_MOM_LOW) return s == 0;
  return s == 1;                                // momentary HIGH and 2-position: HIGH = pressed
}

static void readInputs(uint32_t ms) {
  uint32_t nowUs = (uint32_t)esp_timer_get_time();
  for (int ch = 0; ch < 2; ch++) {
    sigOk[ch] = gotPulse[ch] && (uint32_t)(nowUs - lastUs[ch]) < 200000UL;
    sigW[ch]  = (uint16_t)widthUs[ch];
  }

  // 6-position switch: must stay on a new position posStableMs
  if (sigOk[0]) {
    int8_t p = nearestPos(sigW[0]);
    if (p != posCand) { posCand = p; posCandT = ms; }
    else if (p != posCur && ms - posCandT >= cfg.posStableMs) {
      if (posCur >= 0) bootWifiPending = false;       // switch moved: no power-up WiFi
      posCur = p;
      holdDone = true;                                // a press must start in the new position
    }
  }

  // toggle: HIGH above togHi, LOW below togLo, between = no change
  bool mom = cfg.togType != TT_2POS;
  int f = fnOfPos(posCur);
  if (sigOk[1]) {
    int8_t r = togRaw;
    if (sigW[1] > cfg.togHi) r = 1;
    else if (sigW[1] < cfg.togLo) r = 0;
    if (r != togRaw) { togRaw = r; togRawT = ms; }
    if (togRaw >= 0 && togRaw != togState && ms - togRawT >= 40) {
      int8_t old = togState;
      togState = togRaw;
      bool p = isPressed(togState);
      if (old >= 0) {                                 // first reading = no action
        bootWifiPending = false;                      // switch moved: no power-up WiFi
        if (!mom) fnNext(f);                                         // 2-position: every flip
        else if (!p && pressed && !holdDone) fnNext(f);              // short press, on release
      }
      if (p && !pressed) { pressT = ms; holdDone = (old < 0); }      // held at power-up: no action
      if (!p) holdDone = false;
      pressed = p;
    }
  } else {
    togState = -1; togRaw = -1; pressed = false;      // signal lost: re-learn
  }

  // long holds
  if (pressed && !holdDone && posCur >= 0) {
    uint32_t held = ms - pressT;
    if (posIsWifi() && held >= (uint32_t)cfg.wifiHoldS * 1000UL) {
      holdDone = true;
      bootWifiPending = false;
      if (wifiOn && authOk) wifiStop(); else wifiStart();   // not activated: WiFi stays on
    } else if (mom && f >= 0 && held >= (uint32_t)cfg.offHoldS * 1000UL) {
      holdDone = true;                                // release after this does nothing
      st.off[f] = 1;
      markState();
    }
  }
}

// ------------------------------ patterns -----------------------------
static bool flashOn(const Flash& fl, uint8_t n, uint32_t t) {
  uint32_t per = fl.per ? fl.per : 1;
  uint32_t ph = t % per;
  uint32_t cyc = (uint32_t)fl.on + fl.gap;
  for (uint8_t k = 0; k < n; k++) {
    uint32_t s = k * cyc;
    if (ph >= s && ph < s + fl.on) return true;
  }
  return false;
}

static float patVal(uint8_t p, const FuncC& F, uint16_t offs, uint32_t ms, bool pwm) {
  uint32_t t = ms + offs;                       // offset for strobe / beacon patterns
  switch (p) {
    case P_STEADY: return 1.0f;
    case P_SINGLE: return flashOn(F.fl[0], 1, t) ? 1.0f : 0.0f;
    case P_DOUBLE: return flashOn(F.fl[1], 2, t) ? 1.0f : 0.0f;
    case P_TRIPLE: return flashOn(F.fl[2], 3, t) ? 1.0f : 0.0f;
    case P_ADA:    return flashOn(F.fl[1], 2, ms) ? 1.0f : 0.0f;                     // no offset:
    case P_ADB:    return flashOn(F.fl[1], 2, ms + F.fl[1].per / 2) ? 1.0f : 0.0f;   // A and B stay half apart
    case P_FADE: {
      uint32_t per = F.beaconPer ? F.beaconPer : 1;
      uint32_t ph = t % per;
      if (!pwm) return (ph < per / 5) ? 1.0f : 0.0f;  // on/off LED: short flash
      float x = (float)ph / per;
      return x < 0.5f ? x * 2.0f : (1.0f - x) * 2.0f;
    }
    case P_FAST: { uint32_t per = F.fastPer ? F.fastPer : 1; return (ms % per) < per / 2 ? 1.0f : 0.0f; }
    case P_ALTA: { uint32_t per = F.altPer  ? F.altPer  : 1; return (ms % per) < per / 2 ? 1.0f : 0.0f; }
    case P_ALTB: { uint32_t per = F.altPer  ? F.altPer  : 1; return (ms % per) >= per / 2 ? 1.0f : 0.0f; }
    default: return 0.0f;
  }
}

static void writeDuty(uint8_t i, int32_t d) {
  if (ledPin[i] == PIN_NONE || d == lastDuty[i]) return;
  if (ledPwm[i]) ledcWrite(ledPin[i], d);
  else digitalWrite(ledPin[i], d ? HIGH : LOW);
  lastDuty[i] = d;
}

// v = LED brightness 0..1 (LED view); invert is applied here
static void writeOut(uint8_t i, float v) {
  if (v < 0) v = 0;
  if (v > 1) v = 1;
  int32_t d;
  if (ledPwm[i]) {
    d = (int32_t)(v * v * 1023.0f + 0.5f);      // square curve = even brightness steps
    if (cfg.led[i].inv) d = 1023 - d;
  } else {
    d = (v > 0.001f) ? 1 : 0;
    if (cfg.led[i].inv) d ^= 1;
  }
  writeDuty(i, d);
}

static void updateOutputs(uint32_t ms) {
  for (int i = 0; i < MAX_LED; i++) {
    float v = 0;
    if (authOk) {
      if (testMode) {
        v = (testMask & (1 << i)) ? 1.0f : 0.0f;   // GPIO test: LED full on / off
      } else {
        int f = cfg.led[i].fn;
        if (f < N_FN && !st.off[f]) {
          const FuncC& F = cfg.fn[f];
          const StepC& S = F.st[st.step[f] < F.nSteps ? st.step[f] : 0];
          v = patVal(S.pat[i], F, cfg.led[i].offs, ms, ledPwm[i]) * S.br[i] / 100.0f;
        }
      }
    }
    writeOut(i, v);
  }
}

static void statusLed(uint32_t ms) {
  bool on = false;
  uint32_t a = ms % 1500;
  if (!authOk) on = a < 100 || (a >= 250 && a < 350);                  // not activated: double blink
  else if (ms > 1500 && (!sigOk[0] || !sigOk[1])) on = (ms % 200) < 100;   // no signal
  else if (wifiOn) on = (ms % 1000) < 100;                              // WiFi on
  if (statusLedFree) digitalWrite(LED_STATUS, on ? LOW : HIGH);
}

// ------------------------------ web page -----------------------------
#include "web_page.h"   // the settings page (PAGE)

// ------------------------------ web helpers --------------------------
static void jNum(String& j, const char* k, long v) { j += '"'; j += k; j += "\":"; j += v; j += ','; }
static void jArr8(String& j, const char* k, const uint8_t* a, int n) {
  j += '"'; j += k; j += "\":[";
  for (int i = 0; i < n; i++) { if (i) j += ','; j += a[i]; }
  j += "],";
}
static void jStr(String& j, const char* k, const char* v) { j += '"'; j += k; j += "\":\""; j += v; j += "\","; }

static void handleCfg() {
  String j; j.reserve(10000);
  j = "{\"ver\":\"" FW_VERSION "\",";
  j += "\"pu\":["; for (int i = 0; i < 6; i++) { if (i) j += ','; j += cfg.posUs[i]; } j += "],";
  jArr8(j, "pf", cfg.posFn, 6);
  jNum(j, "th", cfg.togHi); jNum(j, "tl", cfg.togLo); jNum(j, "ps", cfg.posStableMs);
  jNum(j, "tt", cfg.togType); jNum(j, "oh", cfg.offHoldS); jNum(j, "wh", cfg.wifiHoldS);
  jNum(j, "wb", cfg.wifiBootS); jNum(j, "wi", cfg.wifiIdleMin);
  jArr8(j, "ip", cfg.inPin, 2);
  j += "\"led\":[";
  for (int i = 0; i < MAX_LED; i++) {
    const LedC& L = cfg.led[i];
    if (i) j += ',';
    j += "{"; jStr(j, "n", L.name); jNum(j, "f", L.fn); jNum(j, "p", L.pin); jNum(j, "i", L.inv);
    jNum(j, "d", L.dim); j += "\"o\":"; j += L.offs; j += "}";
  }
  j += "],\"fn\":[";
  for (int f = 0; f < N_FN; f++) {
    const FuncC& F = cfg.fn[f];
    if (f) j += ',';
    j += "{"; jStr(j, "n", F.name); jNum(j, "s", F.nSteps);
    j += "\"t\":[";
    for (int k = 0; k < 3; k++) { if (k) j += ','; j += F.fl[k].on; j += ','; j += F.fl[k].gap; j += ','; j += F.fl[k].per; }
    j += ','; j += F.beaconPer; j += ','; j += F.fastPer; j += ','; j += F.altPer; j += "],\"st\":[";
    for (int k = 0; k < N_STEP; k++) {
      if (k) j += ',';
      j += "{"; jStr(j, "n", F.st[k].name); jArr8(j, "p", F.st[k].pat, MAX_LED);
      j += "\"b\":["; for (int i = 0; i < MAX_LED; i++) { if (i) j += ','; j += F.st[k].br[i]; } j += "]}";
    }
    j += "]}";
  }
  j += "],";
  jArr8(j, "rp", ledPin, MAX_LED);           // pins in use now
  j += "\"ri\":["; j += inPin[0]; j += ','; j += inPin[1]; j += "],";
  jNum(j, "pd", pinsDefault);
  jNum(j, "hz", cfg.pwmHz); jNum(j, "rm", cfg.remember); jNum(j, "lp", cfg.lowPwr);
  j += "\"ss\":\""; j += cfg.ssid; j += "\"}";
  server.send(200, "application/json", j);
}

static void handleLive() {
  String j; j.reserve(400);
  j = "{";
  jNum(j, "s0", sigOk[0]); jNum(j, "w0", sigW[0]);
  jNum(j, "s1", sigOk[1]); jNum(j, "w1", sigW[1]);
  jNum(j, "pos", posCur + 1); jNum(j, "tog", togState); jNum(j, "pr", pressed);
  jArr8(j, "fs", st.step, N_FN);
  jArr8(j, "fo", st.off, N_FN);
  jNum(j, "wf", wifiOn);
  jNum(j, "tm", testMode); jNum(j, "tb", testMask);
  jNum(j, "au", authOk); jNum(j, "ao", authOff); jNum(j, "kb", authKeyBad);
  j += "\"id\":\""; j += boardId; j += "\"}";
  server.send(200, "application/json", j);
}

static long argNum(const char* k, long lo, long hi, long cur) {
  if (!server.hasArg(k)) return cur;
  String s = server.arg(k); s.trim();
  if (!s.length()) return cur;
  long v = s.toInt();
  return v < lo ? lo : (v > hi ? hi : v);
}

// copies text, keeping only safe characters; returns length
static int cleanText(const char* src, char* dst, size_t n) {
  size_t j = 0;
  for (size_t i = 0; src[i] && j < n - 1; i++) {
    char c = src[i];
    if (c < 32 || c > 126 || c == '"' || c == '\\' || c == '<' || c == '>' || c == '&' || c == '|') continue;
    dst[j++] = c;
  }
  dst[j] = 0;
  return (int)j;
}
// copies a text field; returns length (-1 = not sent)
static int argText(const char* k, char* dst, size_t n) {
  if (!server.hasArg(k)) return -1;
  return cleanText(server.arg(k).c_str(), dst, n);
}

// "1,2,3|name": reads up to max numbers before '|'; returns count, *rest = text after '|'
static int parseList(const char* s, long* out, int max, const char** rest) {
  int n = 0;
  const char* p = s;
  *rest = "";
  while (*p && *p != '|' && n < max) {
    char* e;
    long v = strtol(p, &e, 10);
    if (e == p) break;
    out[n++] = v;
    p = e;
    if (*p == ',') p++;
  }
  const char* bar = strchr(s, '|');
  if (bar) *rest = bar + 1;
  return n;
}
static long clampL(long v, long lo, long hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void handleSave() {
  Cfg* n = (Cfg*)malloc(sizeof(Cfg));
  if (!n) { server.send(500, "text/plain", "Not saved: out of memory"); return; }
  memcpy(n, &cfg, sizeof(Cfg));
  char key[12];
  for (int i = 0; i < 6; i++) {
    snprintf(key, sizeof(key), "pu%d", i); n->posUs[i] = argNum(key, 800, 2200, n->posUs[i]);
    snprintf(key, sizeof(key), "pf%d", i); n->posFn[i] = argNum(key, 0, PF_WIFI, n->posFn[i]);
  }
  n->togHi = argNum("th", 1000, 2000, n->togHi);
  n->togLo = argNum("tl", 1000, 2000, n->togLo);
  n->posStableMs = argNum("ps", 20, 2000, n->posStableMs);
  n->togType = argNum("tt", 0, TT_COUNT - 1, n->togType);
  n->offHoldS = argNum("oh", 1, 10, n->offHoldS);
  n->wifiHoldS = argNum("wh", 2, 30, n->wifiHoldS);
  n->wifiBootS = argNum("wb", 0, 600, n->wifiBootS);
  n->wifiIdleMin = argNum("wi", 0, 60, n->wifiIdleMin);
  n->inPin[0] = argNum("ip0", 0, 255, n->inPin[0]);
  n->inPin[1] = argNum("ip1", 0, 255, n->inPin[1]);
  long v[2 * MAX_LED + 4];
  const char* rest;
  for (int i = 0; i < MAX_LED; i++) {                     // L<i> = fn,pin,inv,dim,offs|name
    snprintf(key, sizeof(key), "L%d", i);
    if (!server.hasArg(key)) continue;
    String a = server.arg(key);
    if (parseList(a.c_str(), v, 5, &rest) != 5) continue;
    LedC& L = n->led[i];
    L.fn   = (v[0] >= 0 && v[0] < N_FN) ? v[0] : FN_NONE;
    L.pin  = (L.fn != FN_NONE && pinAllowed((uint8_t)clampL(v[1], 0, 255))) ? v[1] : PIN_NONE;
    L.inv  = v[2] ? 1 : 0;
    L.dim  = v[3] ? 1 : 0;
    L.offs = clampL(v[4], 0, 10000);
    char nm[sizeof(L.name)];
    if (cleanText(rest, nm, sizeof(nm)) > 0) strcpy(L.name, nm);
  }
  for (int f = 0; f < N_FN; f++) {                        // F<f> = steps,timing x12|name
    FuncC& F = n->fn[f];
    snprintf(key, sizeof(key), "F%d", f);
    if (server.hasArg(key)) {
      String a = server.arg(key);
      if (parseList(a.c_str(), v, 13, &rest) == 13) {
        F.nSteps = clampL(v[0], 1, N_STEP);
        for (int k = 0; k < 3; k++) {
          F.fl[k].on  = clampL(v[1 + 3 * k], 10, 5000);
          F.fl[k].gap = clampL(v[2 + 3 * k], 10, 5000);
          F.fl[k].per = clampL(v[3 + 3 * k], 50, 10000);
        }
        F.beaconPer = clampL(v[10], 50, 10000);
        F.fastPer   = clampL(v[11], 20, 10000);
        F.altPer    = clampL(v[12], 20, 10000);
        char nm[sizeof(F.name)];
        if (cleanText(rest, nm, sizeof(nm)) > 0) strcpy(F.name, nm);
      }
    }
    for (int k = 0; k < N_STEP; k++) {                    // S<f>_<k> = p0,b0,p1,b1,...|name
      snprintf(key, sizeof(key), "S%d_%d", f, k);
      if (!server.hasArg(key)) continue;
      String a = server.arg(key);
      if (parseList(a.c_str(), v, 2 * MAX_LED, &rest) != 2 * MAX_LED) continue;
      StepC& S = F.st[k];
      for (int i = 0; i < MAX_LED; i++) {
        S.pat[i] = clampL(v[2 * i], 0, P_COUNT - 1);
        S.br[i]  = clampL(v[2 * i + 1], 0, 100);
      }
      char nm[13];
      if (cleanText(rest, nm, sizeof(nm)) > 0) strcpy(S.name, nm);
    }
  }
  n->remember = argNum("rm", 0, 1, n->remember);
  n->lowPwr   = argNum("lp", 0, 1, n->lowPwr);
  n->pwmHz    = argNum("hz", 100, 20000, n->pwmHz);
  char tmp[24];
  if (argText("ss", tmp, sizeof(tmp)) > 0) strcpy(n->ssid, tmp);
  int pl = argText("pw", tmp, sizeof(tmp));

  String err;
  if (n->togLo >= n->togHi) err = "toggle LOW must be below HIGH";
  else if (pl > 0 && pl < 8) err = "WiFi password needs 8+ characters";
  else err = pinMapError(*n);
  if (err.length()) { free(n); server.send(400, "text/plain", "Not saved: " + err); return; }
  if (pl >= 8) strcpy(n->pass, tmp);

  bool pinsChanged = n->inPin[0] != cfg.inPin[0] || n->inPin[1] != cfg.inPin[1];
  for (int i = 0; i < MAX_LED; i++) {
    const LedC& a = n->led[i];
    const LedC& b = cfg.led[i];
    bool ua = a.fn < N_FN && a.pin != PIN_NONE, ub = b.fn < N_FN && b.pin != PIN_NONE;
    if (ua != ub || (ua && (a.pin != b.pin || a.dim != b.dim))) pinsChanged = true;
  }
  bool needRestart = (n->pwmHz != cfg.pwmHz) || strcmp(n->ssid, cfg.ssid) || strcmp(n->pass, cfg.pass);
  memcpy(&cfg, n, sizeof(Cfg));
  free(n);
  saveCfg();
  fixState();
  markState();
  if (pinsChanged) {
    if (cfg.remember) prefs.putBytes("st", &st, sizeof(State));
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "Saved - restarting to use the new pins (reconnect WiFi if needed)");
    delay(500);
    ESP.restart();
  }
  server.send(200, "text/plain", needRestart ? "Saved. Restart the board to use the new WiFi / PWM settings" : "Saved");
}

static void handleGpio() {
  if (!authOk) { server.send(403, "text/plain", "Not activated"); return; }
  if (server.hasArg("end")) { endTest(); server.send(200, "text/plain", "Test ended"); return; }
  long o = argNum("o", -1, MAX_LED - 1, -1);
  if (o < 0) { server.send(400, "text/plain", "Bad LED"); return; }
  if (!testMode) { testMode = true; testMask = 0; }
  testMask ^= (1 << o);
  testLastMs = millis();
  server.send(200, "text/plain", "OK");
}

// checks a pasted / sent code; on success saves it and turns the lights on
static bool activateText(const String& s, String& err) {
  if (authOff) { err = "activation is not used in this firmware"; return false; }
  char hx[129]; int n = 0;
  for (int i = s.length() - 1; i >= 0 && n < 128; i--)       // last 128 hex digits (accepts ID:CODE too)
    if (hexNib(s[i]) >= 0) hx[127 - n++] = s[i];
  if (n < 128) { err = "code too short, it must be 128 characters"; return false; }
  hx[128] = 0;
  uint8_t sig[64];
  if (!parseHex(hx, sig, 64) || !checkCode(sig)) { err = String("code does not match this board (ID ") + boardId + ")"; return false; }
  aprefs.putBytes("sig", sig, 64);
  authOk = true;
  return true;
}

static void handleActivate() {
  String err;
  if (activateText(server.arg("code"), err)) server.send(200, "text/plain", "Activated - lights are on");
  else server.send(400, "text/plain", "Not activated: " + err);
}

// ------------------------------ USB commands (SR Nav Factory) --------
static char    serBuf[220];
static uint8_t serLen = 0;

static void serialCmd(char* c) {
  while (*c == ' ') c++;
  if (!strcasecmp(c, "ID")) {
    Serial.printf("SRNAV ID=%s VER=%s ACT=%s PK=%.8s\n", boardId, FW_VERSION,
                  authOff ? "OFF" : (authOk ? "1" : "0"), NAV_PUBLIC_KEY_HEX);
  } else if (!strncasecmp(c, "ACT ", 4)) {
    String err;
    if (activateText(String(c + 4), err)) Serial.println("SRNAV ACT=OK");
    else Serial.printf("SRNAV ACT=FAIL %s\n", err.c_str());
  }
}

static void serialPoll() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n') {
      serBuf[serLen] = 0;
      if (serLen) serialCmd(serBuf);
      serLen = 0;
    } else if (serLen < sizeof(serBuf) - 1) {
      serBuf[serLen++] = ch;
    } else {
      serLen = 0;                                // too long: drop the line
    }
  }
}

static bool otaFirst = false, otaOk = false;
static String otaErr;

static void handleUpdateDone() {
  server.sendHeader("Connection", "close");
  if (otaOk) {
    server.send(200, "text/plain", "Update OK, restarting...");
    delay(500);
    ESP.restart();
  } else {
    server.send(500, "text/plain", "Update FAILED: " + otaErr);
  }
}

static void handleUpdateUpload() {
  HTTPUpload& u = server.upload();
  if (u.status == UPLOAD_FILE_START) {
    otaOk = false; otaErr = ""; otaFirst = true;
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) otaErr = Update.errorString();
  } else if (u.status == UPLOAD_FILE_WRITE) {
    if (otaErr.length()) return;
    if (otaFirst) {
      otaFirst = false;
      // app image: 0xE9 header, app description magic 0xABCD5432 at byte 32
      if (u.currentSize < 36 || u.buf[0] != 0xE9 ||
          u.buf[32] != 0x32 || u.buf[33] != 0x54 || u.buf[34] != 0xCD || u.buf[35] != 0xAB) {
        otaErr = "this is not a _WiFi.bin update file (the _USB.bin file is only for USB)";
        Update.abort();
        return;
      }
    }
    if (Update.write(u.buf, u.currentSize) != u.currentSize) otaErr = Update.errorString();
  } else if (u.status == UPLOAD_FILE_END) {
    if (!otaErr.length()) {
      if (Update.end(true)) otaOk = true;
      else otaErr = Update.errorString();
    }
  } else if (u.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    otaErr = "upload stopped";
  }
}

// ------------------------------ setup / loop -------------------------
void setup() {
  Serial.begin(115200);
  prefs.begin("srnav", false);
  loadCfg();
  loadState();
  authInit();

  applyPins();
  for (int i = 0; i < MAX_LED; i++) {
    lastDuty[i] = -1;
    if (ledPin[i] == PIN_NONE) continue;
    if (ledPwm[i] && !ledcAttach(ledPin[i], cfg.pwmHz, 10)) ledPwm[i] = false;  // no channel left: on/off
    if (!ledPwm[i]) pinMode(ledPin[i], OUTPUT);
    writeOut(i, 0);
  }
  if (statusLedFree) { pinMode(LED_STATUS, OUTPUT); digitalWrite(LED_STATUS, HIGH); }

  pinMode(inPin[0], INPUT_PULLUP);
  pinMode(inPin[1], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(inPin[0]), isrPos, CHANGE);
  attachInterrupt(digitalPinToInterrupt(inPin[1]), isrTog, CHANGE);

  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", PAGE); });
  server.on("/cfg", HTTP_GET, handleCfg);
  server.on("/live", HTTP_GET, handleLive);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/gpio", HTTP_POST, handleGpio);
  server.on("/activate", HTTP_POST, handleActivate);
  server.on("/reboot", HTTP_POST, []() { server.send(200, "text/plain", "Restarting"); delay(300); ESP.restart(); });
  server.on("/reset", HTTP_POST, []() { prefs.clear(); server.send(200, "text/plain", "Reset done"); delay(300); ESP.restart(); });
  server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  server.onNotFound([]() { server.sendHeader("Location", "/"); server.send(302, "text/plain", ""); });

  bootWifiPending = cfg.wifiBootS > 0;
  if (!authOk) { bootWifiPending = false; wifiStart(); }   // not activated: WiFi on to activate
  Serial.printf("SR Nav Lights v%s ready\n", FW_VERSION);
  Serial.printf("Board ID %s - %s\n", boardId, authOff ? "activation off" : (authOk ? "activated" : "NOT ACTIVATED"));
}

void loop() {
  static uint32_t lastOutMs = 0;
  uint32_t ms = millis();

  readInputs(ms);
  serialPoll();

  // power-up WiFi: only if no switch moved during wifiBootS
  if (bootWifiPending && ms >= (uint32_t)cfg.wifiBootS * 1000UL) {
    bootWifiPending = false;
    wifiStart();
  }

  if (testMode && ms - testLastMs > TEST_TIMEOUT_MS) endTest();

  if (ms - lastOutMs >= 5) { lastOutMs = ms; updateOutputs(ms); }
  statusLed(ms);

  if (wifiOn) {
    server.handleClient();
    if (ms - lastWifiPoll >= 1000) {
      lastWifiPoll = ms;
      if (WiFi.softAPgetStationNum() > 0) lastClientMs = ms;
      else if (testMode) endTest();                   // phone gone: stop GPIO test
      if (authOk && cfg.wifiIdleMin && ms - lastClientMs > (uint32_t)cfg.wifiIdleMin * 60000UL) wifiStop();
    }
  }

  if (stDirty && ms - stDirtyT > 3000) {
    stDirty = false;
    if (cfg.remember) prefs.putBytes("st", &st, sizeof(State));
  }
  delay(1);
}
