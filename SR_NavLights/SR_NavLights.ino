// =====================================================================
//  SR Nav Lights  -  ESP32-C3 SuperMini LED navigation light controller
// =====================================================================
//  Inputs  (receiver PWM 1000-2000 us, internal pull-up):
//    GPIO0  6-position switch  -> selects the MODE
//    GPIO1  toggle switch      -> press steps the current mode
//  Outputs:
//    GPIO5  Left red nav          GPIO20 Landing light  (PWM)
//    GPIO6  Right green nav       GPIO21 Tail strobe    (PWM)
//    GPIO7  Top beacon   (PWM)    GPIO2  Instrument     (PWM)
//    GPIO10 Bottom strobe         GPIO4  Cockpit        (PWM)
//                                 GPIO3  Front light    (PWM)
//    GPIO8  onboard blue LED = status (fast blink: no signal,
//           slow blink: WiFi on)
//  Nav position: short press = next slot of the sequence
//                (or nav back on), hold 3 s = all nav LEDs off.
//  WiFi: on 60 s after power-up only if no switch moved.
//        WiFi position + hold toggle 6 s = WiFi on / off.
//  Settings: WiFi "SR_NavLights", open http://192.168.4.1
//
//  v1.0.1  momentary toggle, WiFi hold rule, timing per pattern,
//          GPIO test, backup / restore
//  v1.0.2  12 nav steps + settable sequence (end = loop to slot 1),
//          Alt double A/B patterns, new default steps,
//          nav position: 3 s hold = nav off, step on release
//  v1.0.3  GPIO test follows Invert (sink / source wiring),
//          board activation: Ed25519 code for the chip ID, checked at
//          power-up; not activated = lights off + WiFi page to activate
//  v1.0.5  every light and both switch inputs assignable to any GPIO
//          (WiFi page -> Pins), dimmable per light (max 6 = PWM channels),
//          pin change = automatic restart, bad pin map = default pins
//  v1.0.4  USB commands for SR Nav Factory (flash + activate in one go):
//            "ID"          -> SRNAV ID=<board id> VER=<x> ACT=0|1|OFF PK=<key start>
//            "ACT <code>"  -> SRNAV ACT=OK  /  SRNAV ACT=FAIL <reason>
// =====================================================================

#define FW_VERSION "1.0.5"

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <initializer_list>
#include "esp_timer.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "esp_mac.h"
#include "src/ed25519/Ed25519.h"
#include "nav_public_key.h"

static_assert(sizeof(NAV_PUBLIC_KEY_HEX) == 65, "NAV_PUBLIC_KEY_HEX must be 64 hex characters");

// ------------------------------ pins ---------------------------------
#define N_OUT 9
// default pin map (v1.0.0 - v1.0.4 wiring); the real map is in the settings
static const uint8_t DEF_OUT_PIN[N_OUT] = {5, 6, 7, 10, 20, 21, 2, 4, 3};
static const uint8_t DEF_OUT_PWM[N_OUT] = {0, 0, 1, 0, 1, 1, 1, 1, 1};
static const uint8_t DEF_IN_PIN[2]      = {0, 1};   // 6-pos switch, toggle
// pins the user may choose (GPIO18/19 = USB, never offered; 11-17 = flash)
static const uint8_t PIN_OK[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21};
#define PIN_NONE   255
#define MAX_PWM    6          // ESP32-C3 has 6 PWM (LEDC) channels
#define LED_STATUS 8          // onboard LED, active LOW

// pin map in use (set once at power-up from the settings)
static uint8_t outPin[N_OUT];
static bool    outPwm[N_OUT];
static volatile uint8_t inPin[2];
static bool    statusLedFree = true;  // GPIO8 not used by a light / input
static bool    pinsDefault = false;   // saved map was bad -> default pins

enum { G_INST = 0, G_COCK = 1, G_FRONT = 2 };
enum { FN_NONE = 0, FN_NAV, FN_INST, FN_COCK, FN_FRONT, FN_MASTER, FN_WIFI, FN_COUNT };
enum { P_OFF = 0, P_STEADY, P_SINGLE, P_DOUBLE, P_TRIPLE, P_FADE, P_FAST, P_ALTA, P_ALTB,
       P_ADA, P_ADB, P_COUNT };
enum { TT_MOM_HIGH = 0, TT_MOM_LOW, TT_2POS, TT_COUNT };

#define NAV_STEPS 12
#define SEQ_END   255
#define CFG_MAGIC 0x53524E4CUL
#define CFG_VER   4
#define ST_MAGIC  0x53524E33UL
#define TEST_TIMEOUT_MS 120000UL

struct NavStep {
  char    name[13];
  uint8_t pat[6];
  uint8_t br[6];
};

// flash pattern timing: [0]=single [1]=double [2]=triple
struct Flash { uint16_t on, gap, per; };

struct Cfg {
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
  NavStep  nav[NAV_STEPS];
  uint8_t  seq[NAV_STEPS];// step index per slot, SEQ_END = loop back
  uint8_t  grpSteps[3];
  Flash    fl[3];
  uint16_t beaconPer, fastPer, altPer;
  uint16_t offs[6];
  uint8_t  maxPct[N_OUT];
  uint8_t  inv[N_OUT];
  uint16_t pwmHz;
  uint8_t  remember;
  uint8_t  lowPwr;
  char     ssid[24];
  char     pass[24];
  // ---- v1.0.5 (CFG_VER 4): pin map, appended so v1.0.2-v1.0.4 settings load as they are
  uint8_t  outPinC[N_OUT];
  uint8_t  outPwmC[N_OUT];
  uint8_t  inPinC[2];
};

// ---- older layouts, only used to carry settings over ----
struct NavStepOld { char name[13]; uint8_t en; uint8_t pat[6]; uint8_t br[6]; };
struct CfgV1 {            // v1.0.0
  uint32_t magic; uint16_t ver;
  uint16_t posUs[6]; uint8_t posFn[6];
  uint16_t togHi, togLo; uint16_t posStableMs;
  NavStepOld nav[8];
  uint8_t  grpSteps[3];
  uint16_t flashOn, flashGap, strobePer, beaconPer, fastPer, altPer;
  uint16_t offs[6];
  uint8_t  maxPct[N_OUT]; uint8_t inv[N_OUT];
  uint16_t pwmHz; uint8_t remember; uint8_t wifiBootMin; uint8_t lowPwr;
  char     ssid[24]; char pass[24];
};
struct CfgV2 {            // v1.0.1
  uint32_t magic; uint16_t ver;
  uint16_t posUs[6]; uint8_t posFn[6];
  uint16_t togHi, togLo; uint16_t posStableMs;
  uint8_t  togType; uint8_t wifiHoldS; uint16_t wifiBootS; uint8_t wifiIdleMin;
  NavStepOld nav[8];
  uint8_t  grpSteps[3];
  Flash    fl[3];
  uint16_t beaconPer, fastPer, altPer;
  uint16_t offs[6];
  uint8_t  maxPct[N_OUT]; uint8_t inv[N_OUT];
  uint16_t pwmHz; uint8_t remember; uint8_t lowPwr;
  char     ssid[24]; char pass[24];
};

struct State {
  uint32_t magic;
  uint8_t  slot;          // current slot in the sequence
  uint8_t  lvl[3];
  uint8_t  master;
  uint8_t  navOff;        // 1 = nav LEDs off (3 s hold)
};

static Cfg cfg;
static State st;
static Preferences prefs;
static WebServer server(80);

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
static int32_t  lastDuty[N_OUT];

// ------------------------------ defaults -----------------------------
static void setStep(int i, const char* n,
                    std::initializer_list<uint8_t> p, std::initializer_list<uint8_t> b) {
  NavStep& s = cfg.nav[i];
  strncpy(s.name, n, sizeof(s.name) - 1);
  s.name[sizeof(s.name) - 1] = 0;
  int k = 0; for (uint8_t v : p) if (k < 6) s.pat[k++] = v;
  k = 0;     for (uint8_t v : b) if (k < 6) s.br[k++] = v;
}

static void defaultSteps() {
  //            name           L-red     R-green   Beacon    Bottom    Landing   Tail
  setStep(0, "Normal",       {P_STEADY, P_STEADY, P_FADE,   P_SINGLE, P_OFF,    P_SINGLE}, {100,100,100,100,100,100});
  setStep(1, "Taxi",         {P_STEADY, P_STEADY, P_FADE,   P_OFF,    P_STEADY, P_STEADY}, {100,100,100,100, 40,100});
  setStep(2, "Takeoff/Land", {P_STEADY, P_STEADY, P_FADE,   P_DOUBLE, P_STEADY, P_DOUBLE}, {100,100,100,100,100,100});
  setStep(3, "Double flash", {P_DOUBLE, P_DOUBLE, P_FADE,   P_SINGLE, P_OFF,    P_TRIPLE}, {100,100,100,100,100,100});
  setStep(4, "Alt double",   {P_ADA,    P_ADB,    P_FADE,   P_SINGLE, P_OFF,    P_TRIPLE}, {100,100,100,100,100,100});
  setStep(5, "Wig-wag",      {P_ALTA,   P_ALTB,   P_FAST,   P_TRIPLE, P_ALTA,   P_ALTB},   {100,100,100,100,100,100});
  setStep(6, "Full flash",   {P_FAST,   P_FAST,   P_FAST,   P_FAST,   P_FAST,   P_FAST},   {100,100,100,100,100,100});
  setStep(7, "Off",          {P_OFF,    P_OFF,    P_OFF,    P_OFF,    P_OFF,    P_OFF},    {100,100,100,100,100,100});
  for (int i = 8; i < NAV_STEPS; i++) {
    char n[13]; snprintf(n, sizeof(n), "Spare %d", i + 1);
    setStep(i, n, {P_OFF, P_OFF, P_OFF, P_OFF, P_OFF, P_OFF}, {100,100,100,100,100,100});
  }
  for (int i = 0; i < NAV_STEPS; i++) cfg.seq[i] = (i < 8) ? i : SEQ_END;
}

static void defaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = CFG_MAGIC; cfg.ver = CFG_VER;
  const uint16_t pu[6] = {1000, 1193, 1398, 1602, 1807, 2000};
  const uint8_t  pf[6] = {FN_NAV, FN_INST, FN_FRONT, FN_COCK, FN_MASTER, FN_WIFI};
  for (int i = 0; i < 6; i++) { cfg.posUs[i] = pu[i]; cfg.posFn[i] = pf[i]; }
  cfg.togHi = 1650; cfg.togLo = 1350; cfg.posStableMs = 200;
  cfg.togType = TT_MOM_HIGH; cfg.wifiHoldS = 6; cfg.wifiBootS = 60; cfg.wifiIdleMin = 0;
  cfg.navHoldS = 3;
  defaultSteps();
  for (int g = 0; g < 3; g++) cfg.grpSteps[g] = 5;
  for (int f = 0; f < 3; f++) { cfg.fl[f].on = 50; cfg.fl[f].gap = 80; cfg.fl[f].per = 1200; }
  cfg.beaconPer = 1000; cfg.fastPer = 250; cfg.altPer = 800;
  const uint16_t of[6] = {0, 0, 0, 0, 0, 600};
  for (int o = 0; o < 6; o++) cfg.offs[o] = of[o];
  for (int o = 0; o < N_OUT; o++) { cfg.maxPct[o] = 100; cfg.inv[o] = 0; }
  cfg.pwmHz = 1000; cfg.remember = 1; cfg.lowPwr = 1;
  strcpy(cfg.ssid, "SR_NavLights");
  strcpy(cfg.pass, "12345678");
  memcpy(cfg.outPinC, DEF_OUT_PIN, N_OUT);
  memcpy(cfg.outPwmC, DEF_OUT_PWM, N_OUT);
  memcpy(cfg.inPinC, DEF_IN_PIN, 2);
}

static void saveCfg() { prefs.putBytes("cfg", &cfg, sizeof(Cfg)); }

// copies the parts that are the same in every version (nav steps get the new defaults)
template <typename T> static void carryCommon(const T& o) {
  memcpy(cfg.posUs, o.posUs, sizeof(o.posUs));
  memcpy(cfg.posFn, o.posFn, sizeof(o.posFn));
  cfg.togHi = o.togHi; cfg.togLo = o.togLo; cfg.posStableMs = o.posStableMs;
  memcpy(cfg.grpSteps, o.grpSteps, sizeof(o.grpSteps));
  cfg.beaconPer = o.beaconPer; cfg.fastPer = o.fastPer; cfg.altPer = o.altPer;
  memcpy(cfg.offs, o.offs, sizeof(o.offs));
  memcpy(cfg.maxPct, o.maxPct, sizeof(o.maxPct));
  memcpy(cfg.inv, o.inv, sizeof(o.inv));
  cfg.pwmHz = o.pwmHz; cfg.remember = o.remember; cfg.lowPwr = o.lowPwr;
  memcpy(cfg.ssid, o.ssid, sizeof(o.ssid));
  memcpy(cfg.pass, o.pass, sizeof(o.pass));
}

static void loadCfg() {
  size_t len = prefs.getBytesLength("cfg");
  static uint8_t buf[sizeof(Cfg) > sizeof(CfgV2) ? sizeof(Cfg) : sizeof(CfgV2)];
  defaults();
  if (len < 6 || len > sizeof(buf)) return;
  prefs.getBytes("cfg", buf, len);
  uint32_t magic; uint16_t ver;
  memcpy(&magic, buf, 4); memcpy(&ver, buf + 4, 2);
  if (magic != CFG_MAGIC) return;
  if (ver == CFG_VER && len == sizeof(Cfg)) { memcpy(&cfg, buf, sizeof(Cfg)); return; }
  if (ver == 3 && len >= offsetof(Cfg, outPinC) && len < sizeof(Cfg)) {   // from v1.0.2 - v1.0.4
    memcpy(&cfg, buf, offsetof(Cfg, outPinC));    // same fields, same place; pins keep the defaults
    cfg.ver = CFG_VER;
    saveCfg();
    return;
  }
  if (ver == 2 && len == sizeof(CfgV2)) {          // from v1.0.1
    CfgV2 o; memcpy(&o, buf, sizeof(o));
    carryCommon(o);
    cfg.togType = o.togType; cfg.wifiHoldS = o.wifiHoldS;
    cfg.wifiBootS = o.wifiBootS; cfg.wifiIdleMin = o.wifiIdleMin;
    memcpy(cfg.fl, o.fl, sizeof(o.fl));
    saveCfg();
  } else if (ver == 1 && len == sizeof(CfgV1)) {   // from v1.0.0
    CfgV1 o; memcpy(&o, buf, sizeof(o));
    carryCommon(o);
    for (int f = 0; f < 3; f++) { cfg.fl[f].on = o.flashOn; cfg.fl[f].gap = o.flashGap; cfg.fl[f].per = o.strobePer; }
    saveCfg();
  }
}

static uint8_t seqLen() {
  uint8_t n = 0;
  while (n < NAV_STEPS && cfg.seq[n] < NAV_STEPS) n++;
  return n;
}

static void fixState() {
  if (st.slot >= seqLen()) st.slot = 0;
  for (int g = 0; g < 3; g++) if (st.lvl[g] > cfg.grpSteps[g]) st.lvl[g] = cfg.grpSteps[g];
  if (st.master > 1) st.master = 0;
  if (st.navOff > 1) st.navOff = 0;
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

// checks a pin map; returns "" when good, otherwise the reason
static String pinMapError(const uint8_t* op, const uint8_t* pw, const uint8_t* ip) {
  static const char* const NAMES[N_OUT] = {"Left red", "Right green", "Top beacon", "Bottom strobe", "Landing",
                                           "Tail strobe", "Instrument", "Cockpit", "Front light"};
  uint32_t used = 0;
  int pwm = 0;
  for (int c = 0; c < 2; c++) {
    if (!pinAllowed(ip[c])) return String(c ? "Toggle" : "6-pos switch") + " input needs a GPIO";
    if (used & (1UL << ip[c])) return "6-pos switch and toggle are on the same GPIO";
    used |= 1UL << ip[c];
  }
  for (int i = 0; i < N_OUT; i++) {
    if (op[i] == PIN_NONE) continue;
    if (!pinAllowed(op[i])) return String(NAMES[i]) + ": GPIO not allowed";
    if (used & (1UL << op[i])) return String(NAMES[i]) + ": GPIO" + op[i] + " is already used";
    used |= 1UL << op[i];
    if (pw[i]) pwm++;
  }
  if (pwm > MAX_PWM) return String("Too many dimmable lights (") + pwm + "), max " + MAX_PWM;
  return "";
}

// takes the pin map from the settings (default map if it is bad) - power-up only
static void applyPins() {
  if (pinMapError(cfg.outPinC, cfg.outPwmC, cfg.inPinC).length()) {
    pinsDefault = true;
    memcpy(outPin, DEF_OUT_PIN, N_OUT);
    for (int i = 0; i < N_OUT; i++) outPwm[i] = DEF_OUT_PWM[i];
    inPin[0] = DEF_IN_PIN[0]; inPin[1] = DEF_IN_PIN[1];
  } else {
    memcpy(outPin, cfg.outPinC, N_OUT);
    for (int i = 0; i < N_OUT; i++) outPwm[i] = cfg.outPwmC[i] != 0;
    inPin[0] = cfg.inPinC[0]; inPin[1] = cfg.inPinC[1];
  }
  statusLedFree = inPin[0] != LED_STATUS && inPin[1] != LED_STATUS;
  for (int i = 0; i < N_OUT; i++) if (outPin[i] == LED_STATUS) statusLedFree = false;
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
static void navNext() {
  if (st.navOff) { st.navOff = 0; markState(); return; }     // back on at the same slot
  uint8_t n = seqLen();
  if (n) st.slot = (st.slot + 1) % n;
  markState();
}

static void doStep(uint8_t pos) {
  switch (cfg.posFn[pos]) {
    case FN_NAV:   navNext(); return;
    case FN_INST:  st.lvl[G_INST]  = (st.lvl[G_INST]  + 1) % (cfg.grpSteps[G_INST]  + 1); break;
    case FN_COCK:  st.lvl[G_COCK]  = (st.lvl[G_COCK]  + 1) % (cfg.grpSteps[G_COCK]  + 1); break;
    case FN_FRONT: st.lvl[G_FRONT] = (st.lvl[G_FRONT] + 1) % (cfg.grpSteps[G_FRONT] + 1); break;
    case FN_MASTER: st.master ^= 1; break;
    default: return;                            // WiFi position: only the long hold counts
  }
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

static bool posIs(uint8_t fn) { return posCur >= 0 && cfg.posFn[posCur] == fn; }

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
        if (posCur >= 0) {
          if (!mom) doStep(posCur);                   // 2-position: every flip counts
          else if (p && !posIs(FN_NAV)) doStep(posCur);              // press
          else if (!p && pressed && !holdDone && posIs(FN_NAV)) doStep(posCur); // nav: short press, on release
        }
      }
      if (p && !pressed) { pressT = ms; holdDone = (old < 0); }  // held at power-up: no action
      if (!p) holdDone = false;
      pressed = p;
    }
  } else {
    togState = -1; togRaw = -1; pressed = false;      // signal lost: re-learn
  }

  // long holds
  if (pressed && !holdDone && posCur >= 0) {
    uint32_t held = ms - pressT;
    if (posIs(FN_WIFI) && held >= (uint32_t)cfg.wifiHoldS * 1000UL) {
      holdDone = true;
      bootWifiPending = false;
      if (wifiOn && authOk) wifiStop(); else wifiStart();   // not activated: WiFi stays on
    } else if (mom && posIs(FN_NAV) && held >= (uint32_t)cfg.navHoldS * 1000UL) {
      holdDone = true;                                // release after this does nothing
      st.navOff = 1;
      markState();
    }
  }
}

// ------------------------------ patterns -----------------------------
static bool flashOn(const Flash& f, uint8_t n, uint32_t t) {
  uint32_t per = f.per ? f.per : 1;
  uint32_t ph = t % per;
  uint32_t cyc = (uint32_t)f.on + f.gap;
  for (uint8_t k = 0; k < n; k++) {
    uint32_t s = k * cyc;
    if (ph >= s && ph < s + f.on) return true;
  }
  return false;
}

static float patVal(uint8_t p, uint8_t o, uint32_t ms, bool pwm) {
  uint32_t t = ms + cfg.offs[o];                // offset for strobe/beacon patterns
  switch (p) {
    case P_STEADY: return 1.0f;
    case P_SINGLE: return flashOn(cfg.fl[0], 1, t) ? 1.0f : 0.0f;
    case P_DOUBLE: return flashOn(cfg.fl[1], 2, t) ? 1.0f : 0.0f;
    case P_TRIPLE: return flashOn(cfg.fl[2], 3, t) ? 1.0f : 0.0f;
    case P_ADA:    return flashOn(cfg.fl[1], 2, ms) ? 1.0f : 0.0f;                    // no offset:
    case P_ADB:    return flashOn(cfg.fl[1], 2, ms + cfg.fl[1].per / 2) ? 1.0f : 0.0f; // A and B stay half apart
    case P_FADE: {
      uint32_t per = cfg.beaconPer ? cfg.beaconPer : 1;
      uint32_t ph = t % per;
      if (!pwm) return (ph < per / 5) ? 1.0f : 0.0f;  // on/off output: short flash
      float x = (float)ph / per;
      return x < 0.5f ? x * 2.0f : (1.0f - x) * 2.0f;
    }
    case P_FAST: { uint32_t per = cfg.fastPer ? cfg.fastPer : 1; return (ms % per) < per / 2 ? 1.0f : 0.0f; }
    case P_ALTA: { uint32_t per = cfg.altPer  ? cfg.altPer  : 1; return (ms % per) < per / 2 ? 1.0f : 0.0f; }
    case P_ALTB: { uint32_t per = cfg.altPer  ? cfg.altPer  : 1; return (ms % per) >= per / 2 ? 1.0f : 0.0f; }
    default: return 0.0f;
  }
}

static void writeDuty(uint8_t i, int32_t d) {
  if (outPin[i] == PIN_NONE || d == lastDuty[i]) return;
  if (outPwm[i]) ledcWrite(outPin[i], d);
  else digitalWrite(outPin[i], d ? HIGH : LOW);
  lastDuty[i] = d;
}

static void writeOut(uint8_t i, float v) {
  if (v < 0) v = 0;
  if (v > 1) v = 1;
  int32_t d;
  if (outPwm[i]) {
    d = (int32_t)(v * v * 1023.0f + 0.5f);      // square curve = even brightness steps
    if (cfg.inv[i]) d = 1023 - d;
  } else {
    d = (v > 0.001f) ? 1 : 0;
    if (cfg.inv[i]) d ^= 1;
  }
  writeDuty(i, d);
}

static void updateOutputs(uint32_t ms) {
  if (!authOk) {                                // not activated: every LED off
    for (int i = 0; i < N_OUT; i++) writeOut(i, 0);
    return;
  }
  if (testMode) {                               // GPIO test: LED full on / off, Invert followed
    for (int i = 0; i < N_OUT; i++) {
      bool on = testMask & (1 << i);
      int32_t d = outPwm[i] ? (on ? 1023 : 0) : (on ? 1 : 0);
      if (cfg.inv[i]) d = outPwm[i] ? 1023 - d : (d ^ 1);
      writeDuty(i, d);
    }
    return;
  }
  float v[N_OUT];
  for (int i = 0; i < N_OUT; i++) v[i] = 0;
  if (!st.master) {
    uint8_t n = seqLen();
    if (!st.navOff && n) {
      const NavStep& s = cfg.nav[cfg.seq[st.slot < n ? st.slot : 0]];
      for (int o = 0; o < 6; o++) v[o] = patVal(s.pat[o], o, ms, outPwm[o]) * s.br[o] / 100.0f;
    }
    for (int g = 0; g < 3; g++)
      v[6 + g] = st.lvl[g] ? (float)st.lvl[g] / cfg.grpSteps[g] : 0.0f;
  }
  for (int i = 0; i < N_OUT; i++) writeOut(i, v[i] * cfg.maxPct[i] / 100.0f);
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
static const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>SR Nav Lights</title>
<style>
body{background:#000;color:#fff;font-family:system-ui,Arial,sans-serif;margin:0 auto;max-width:720px;padding:10px 10px 90px;font-size:16px}
h1{color:#ffd000;font-size:22px;margin:4px 0 8px}
h2{color:#ffd000;font-size:18px;border-bottom:2px solid #ffd000;margin:22px 0 8px;padding-bottom:2px}
.live{border:2px solid #fff;border-radius:6px;padding:8px;font-weight:bold;line-height:1.6}
table{border-collapse:collapse;width:100%}td{padding:4px;border-bottom:1px solid #444;vertical-align:middle}
input,select,button{font-size:16px;background:#000;color:#fff;border:1px solid #aaa;border-radius:4px;padding:5px}
input[type=number]{width:74px}input[type=checkbox]{width:22px;height:22px}
button{color:#ffd000;border:2px solid #ffd000;font-weight:bold;padding:6px 12px}
button.on{background:#ffd000;color:#000}
button:focus,input:focus,select:focus{outline:3px solid #fff}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:8px}
.grid button{padding:10px 6px;text-align:left}
.seq{display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:6px}
.seq label{display:flex;gap:6px;align-items:center}
details{border:1px solid #666;border-radius:6px;margin:6px 0;padding:6px}summary{font-weight:bold;cursor:pointer}
.bar{position:fixed;left:0;right:0;bottom:0;background:#000;border-top:2px solid #ffd000;padding:10px;display:flex;gap:12px;align-items:center;justify-content:center}
.ok{color:#3f3}.bad{color:#f55}.note{color:#ccc;font-size:14px}
.warn{border:3px solid #f55;border-radius:6px;padding:10px;margin-top:10px;line-height:1.6}
.warn textarea{width:100%;font:15px ui-monospace,Menlo,Consolas,monospace;background:#000;color:#fff;border:1px solid #aaa;border-radius:4px;padding:6px;margin:6px 0}
.yel{color:#ffd000}
.idbox{font:bold 18px ui-monospace,Menlo,Consolas,monospace;width:11em;color:#ffd000}
</style></head><body>
<h1>SR Nav Lights <span id="ver"></span></h1>
<div class="live" id="live">Connecting...</div>
<div id="auth"></div>
<h2>1. Modes (6-position switch)</h2>
<table id="modes"></table>
<table id="tog"></table>
<h2>2. Steps</h2>
<p><b>Nav sequence</b> <span class="note">(each short press goes to the next slot; at "end" it goes back to slot 1)</span></p>
<div class="seq" id="seq"></div>
<p><b>Nav steps</b> <span class="note">(what each step does)</span></p>
<div id="nav"></div>
<p><b>Lights</b> <span class="note">(1 step = on/off, 5 steps = 20% to 100% then off)</span></p>
<table id="grp"></table>
<h2>3. Timing (ms)</h2>
<table id="tim"></table>
<p class="note">Alt double A / B use the double flash timing; B flashes half a repeat period after A.</p>
<p><b>Offsets (ms)</b> <span class="note">shift single/double/triple flash and beacon so lights don't flash together</span></p>
<table id="off"></table>
<h2>4. Pins</h2>
<p class="note">Choose the GPIO each light and switch is wired to. Saving a pin change restarts the board.
Dimmable = brightness / fade (max 6). Not dimmable = on/off only.</p>
<div id="pinwarn"></div>
<table id="pins"></table>
<div id="pinnote" class="note"></div>
<h2>5. Other</h2>
<table id="outs"></table><br>
<table id="oth"></table>
<p><button onclick="bk()">Backup settings</button> <button onclick="$('rf').click()">Restore settings</button>
<input type="file" id="rf" accept=".json" style="display:none" onchange="rs(this)"></p>
<p><button onclick="rb()">Restart board</button> <button onclick="fr()">Factory reset</button></p>
<h2>GPIO test</h2>
<p class="note">Press = LED ON, press again = LED OFF (Invert is followed, the pin level is shown). Normal lights stop while testing. Ends by itself after 2 minutes or when the phone disconnects.</p>
<div class="grid" id="gt"></div>
<p><button onclick="gEnd()">End test</button></p>
<h2>Firmware update</h2>
<p>Choose the <b>_WiFi.bin</b> file <input type="file" id="fw" accept=".bin"><br><br>
<button onclick="up()">Upload firmware</button> <span id="upm"></span></p>
<div class="bar"><button onclick="save()">Save settings</button><span id="msg"></span></div>
<script>
const PN=["Off","Steady","Single flash","Double flash","Triple flash","Beacon fade","Fast flash","Alternate A","Alternate B","Alt double A","Alt double B"];
const FN=["None","Nav lights","Instrument","Cockpit","Front light","Master off","WiFi (hold)"];
const TT=["Momentary, pressed = HIGH","Momentary, pressed = LOW","2-position (every flip)"];
const ON=["Left red","Right green","Top beacon","Bottom strobe","Landing","Tail strobe","Instrument","Cockpit","Front light"];
const PINS=[0,1,2,3,4,5,6,7,8,9,10,20,21];
const PWARN={2:'boot pin - check the board still starts',8:'boot pin + onboard status LED (status blink stops)',9:'BOOT button pin - must not be LOW at power-up',
 20:'may flicker at power-up (boot text)',21:'may flicker at power-up (boot text)'};
let GP=[5,6,7,10,20,21,2,4,3];
function psel(id,v,none){let s='<select id="'+id+'" onchange="pchk()">'+(none?'<option value="255"'+(v==255?' selected':'')+'>None</option>':'');
 PINS.forEach(p=>s+='<option value="'+p+'"'+(p==v?' selected':'')+'>GPIO'+p+'</option>');return s+'</select>';}
function pchk(){if(!$('ip0'))return true;const used={},err=[],warn=[];let pwm=0;
 const add=(name,p)=>{if(p==255)return;if(used[p]!==undefined)err.push('GPIO'+p+' used twice: '+used[p]+' and '+name);else used[p]=name;
  if(PWARN[p])warn.push(name+' on GPIO'+p+': '+PWARN[p]);};
 add('6-pos switch',+$('ip0').value);add('Toggle',+$('ip1').value);
 for(let o=0;o<9;o++){add(ON[o],+$('op'+o).value);if(+$('op'+o).value!=255&&$('pw'+o).checked)pwm++;}
 if(pwm>6)err.push('Too many dimmable lights: '+pwm+' ticked, max 6');
 $('pinwarn').innerHTML=err.map(e=>'<div class="bad">✖ '+e+'</div>').join('')+warn.map(w=>'<div class="yel">⚠ '+w+'</div>').join('');
 return err.length==0;}
let C=null,L={};const $=i=>document.getElementById(i);
const esc=s=>String(s).replace(/[&<>"]/g,'');
function sel(id,o,v){let s='<select id="'+id+'">';o.forEach((t,i)=>s+='<option value="'+i+'"'+(i==v?' selected':'')+'>'+t+'</option>');return s+'</select>';}
function num(id,v,a,b){return '<input type="number" id="'+id+'" value="'+v+'" min="'+a+'" max="'+b+'">';}
function chk(id,v){return '<input type="checkbox" id="'+id+'"'+(v?' checked':'')+'>';}
function row(a,b){return '<tr><td>'+a+'</td><td>'+b+'</td></tr>';}
function build(){
 $('ver').textContent='v'+C.ver;
 let h='<tr><td><b>Pos</b></td><td><b>Value µs</b></td><td></td><td><b>Function</b></td></tr>';
 for(let i=0;i<6;i++)h+='<tr><td>'+(i+1)+'</td><td>'+num('pu'+i,C.pu[i],800,2200)+'</td><td><button onclick="learn('+i+')">Learn</button></td><td>'+sel('pf'+i,FN,C.pf[i])+'</td></tr>';
 $('modes').innerHTML=h;
 $('tog').innerHTML=row('Toggle switch type',sel('tt',TT,C.tt))+row('Toggle HIGH above (µs)',num('th',C.th,1000,2000))+row('Toggle LOW below (µs)',num('tl',C.tl,1000,2000))
 +row('New 6-pos position counts after (ms)',num('ps',C.ps,20,2000))
 +row('Nav position: hold toggle = nav LEDs off (s)',num('nh',C.nh,1,10))
 +row('WiFi position: hold toggle = WiFi on/off (s)',num('wh',C.wh,2,30));
 const SN=C.nav.map((s,i)=>(i+1)+' '+esc(s.n));
 h='';for(let k=0;k<12;k++){h+='<label>'+(k+1)+'. <select id="q'+k+'"><option value="255"'+(C.seq[k]>11?' selected':'')+'>- end -</option>';
  SN.forEach((n,i)=>h+='<option value="'+i+'"'+(C.seq[k]==i?' selected':'')+'>'+n+'</option>');h+='</select></label>';}
 $('seq').innerHTML=h;
 h='';C.nav.forEach((s,i)=>{h+='<details><summary>Step '+(i+1)+': '+esc(s.n)+'</summary><p>Name <input id="n'+i+'n" maxlength="12" size="12" value="'+esc(s.n)+'"></p><table>';
  for(let o=0;o<6;o++)h+='<tr><td>'+ON[o]+'</td><td>'+sel('n'+i+'p'+o,PN,s.p[o])+'</td><td>'+num('n'+i+'b'+o,s.b[o],0,100)+' %</td></tr>';
  h+='</table></details>';});$('nav').innerHTML=h;
 h='';for(let g=0;g<3;g++)h+=row(ON[6+g],num('g'+g,C.g[g],1,10)+' steps');$('grp').innerHTML=h;
 h='<tr><td></td><td><b>On</b></td><td><b>Gap</b></td><td><b>Repeat</b></td></tr>';
 ['Single flash','Double flash','Triple flash'].forEach((n,f)=>{h+='<tr><td>'+n+'</td><td>'+num('f'+f+'o',C.fl[f][0],10,5000)+'</td><td>'+(f?num('f'+f+'g',C.fl[f][1],10,5000):'')+'</td><td>'+num('f'+f+'p',C.fl[f][2],50,10000)+'</td></tr>';});
 h+='<tr><td>Beacon fade</td><td></td><td></td><td>'+num('bp',C.bp,50,10000)+'</td></tr>';
 h+='<tr><td>Fast flash</td><td></td><td></td><td>'+num('fp',C.fp,20,10000)+'</td></tr>';
 h+='<tr><td>Wig-wag (Alternate)</td><td></td><td></td><td>'+num('ap',C.ap,20,10000)+'</td></tr>';
 $('tim').innerHTML=h;
 h='';for(let o=0;o<6;o++)h+=row(ON[o],num('of'+o,C.of[o],0,10000));$('off').innerHTML=h;
 GP=C.rp||GP;
 h='<tr><td><b>Light / switch</b></td><td><b>GPIO</b></td><td><b>Dimmable</b></td></tr>';
 h+='<tr><td>6-pos switch input</td><td>'+psel('ip0',C.ip[0],0)+'</td><td></td></tr>';
 h+='<tr><td>Toggle input</td><td>'+psel('ip1',C.ip[1],0)+'</td><td></td></tr>';
 for(let o=0;o<9;o++)h+='<tr><td>'+ON[o]+'</td><td>'+psel('op'+o,C.op[o],1)+'</td><td>'+chk('pw'+o,C.pw[o]).replace('<input','<input onchange="pchk()"')+'</td></tr>';
 $('pins').innerHTML=h;
 $('pinnote').innerHTML=C.pd?'<span class="bad">The saved pin map was not valid - the board is using the default pins now.</span>':'';
 pchk();
 h='<tr><td><b>Output</b></td><td><b>Max %</b></td><td><b>Invert</b></td></tr>';
 for(let o=0;o<9;o++)h+='<tr><td>'+ON[o]+' ('+(GP[o]==255?'not used':'GPIO'+GP[o])+')</td><td>'+num('mx'+o,C.mx[o],0,100)+'</td><td>'+chk('iv'+o,C.iv[o])+'</td></tr>';
 $('outs').innerHTML=h;
 $('oth').innerHTML=row('Remember lights after power-off',chk('rm',C.rm))
 +row('WiFi on after power-up if no switch moved (s, 0 = never)',num('wb',C.wb,0,600))
 +row('WiFi off when no phone connected (min, 0 = never)',num('wi',C.wi,0,60))
 +row('WiFi low power (SuperMini fix)',chk('lp',C.lp))
 +row('WiFi name','<input id="ss" maxlength="23" value="'+esc(C.ss)+'">')
 +row('New WiFi password (8+ characters, empty = keep)','<input id="pw" maxlength="23">')
 +row('PWM frequency Hz (used after restart)',num('hz',C.hz,100,20000));
 h='';for(let o=0;o<9;o++)h+='<button id="gb'+o+'" onclick="gT('+o+')"></button>';$('gt').innerHTML=h;gLab(0,0);
}
function msg(t,ok){$('msg').textContent=t;$('msg').className=ok?'ok':'bad';}
function post(u,b){return fetch(u,{method:'POST',body:b}).then(r=>r.text().then(t=>({ok:r.ok,t})));}
function learn(i){if(L.s0){$('pu'+i).value=L.w0;msg('Position '+(i+1)+' = '+L.w0+' µs, press Save',1);}else msg('No signal on the 6-pos input',0);}
function save(){if(!pchk()){msg('Not saved: fix the pins first',0);return;}const p=new URLSearchParams();document.querySelectorAll('input,select').forEach(e=>{if(!e.id||e.type=='file')return;p.append(e.id,e.type=='checkbox'?(e.checked?1:0):e.value);});
 post('/save',p).then(r=>{msg(r.t,r.ok);if(r.ok)setTimeout(load,/restarting/.test(r.t)?6000:0);}).catch(()=>msg('Not saved: no connection',0));}
function gT(o){post('/gpio',new URLSearchParams({o:o})).then(r=>{if(!r.ok)msg(r.t,0);}).catch(()=>msg('No connection',0));}
function gEnd(){post('/gpio',new URLSearchParams({end:1})).then(()=>msg('Test ended',1));}
function bk(){const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify(C,null,1)],{type:'application/json'}));a.download='SR_NavLights_settings.json';a.click();msg('Backup saved',1);}
function rs(inp){const f=inp.files[0];if(!f)return;f.text().then(t=>{const j=JSON.parse(t);delete j.ver;C=Object.assign({},C,j);build();save();}).catch(()=>msg('Not a settings file',0));inp.value='';}
function rb(){if(confirm('Restart the board?'))post('/reboot',new URLSearchParams()).then(()=>msg('Restarting',1));}
function fr(){if(confirm('Reset ALL settings to factory?'))post('/reset',new URLSearchParams()).then(()=>msg('Reset done, restarting',1));}
function up(){const f=$('fw').files[0];if(!f){$('upm').textContent='Choose a file first';return;}
 if(!/_(wifi|ota)\.bin$/i.test(f.name)&&!confirm('The file name does not end with _WiFi.bin. Upload anyway?'))return;
 const d=new FormData();d.append('fw',f,f.name);$('upm').textContent='Uploading, do not power off...';
 fetch('/update',{method:'POST',body:d}).then(r=>r.text()).then(t=>$('upm').textContent=t).catch(()=>$('upm').textContent='Upload failed');}
function lv(i){return L.lv[i]?L.lv[i]+'/'+L.g[i]:'OFF';}
function gLab(tm,tb){if(!C)return;for(let o=0;o<9;o++){const b=$('gb'+o);if(!b)continue;const on=!!(tm&&(tb>>o&1)),hi=on!==!!C.iv[o];
 if(GP[o]==255){b.className='';b.disabled=true;b.innerHTML=ON[o]+'<br>not used';continue;}b.disabled=false;
 b.className=on?'on':'';b.innerHTML='GPIO'+GP[o]+' '+ON[o]+'<br>LED '+(on?'ON':'OFF')+' · pin '+(hi?'HIGH':'LOW');}}
let AK='';
function authBox(j){const k=j.au+'|'+j.ao+'|'+j.kb+'|'+j.id;if(k===AK)return;AK=k;
 if(j.au){$('auth').innerHTML='<p class="note">'+(j.ao?'Activation not set up in this firmware':'Activated')+' · Board ID '+j.id+'</p>';return;}
 $('auth').innerHTML='<div class="warn"><b class="bad">NOT ACTIVATED — all lights are off</b>'+(j.kb?'<br><span class="bad">Public key in this firmware is not valid</span>':'')
 +'<br>Board ID: <input class="idbox" readonly value="'+j.id+'" onclick="this.select()"> <span class="note">send this to SrRcTech</span>'
 +'<br><textarea id="ac" rows="3" placeholder="Paste the activation code here"></textarea><button onclick="act()">Activate</button></div>';}
function act(){const c=$('ac').value;post('/activate',new URLSearchParams({code:c})).then(r=>{msg(r.t,r.ok);if(r.ok){AK='';}}).catch(()=>msg('No connection',0));}
function live(){fetch('/live').then(r=>r.json()).then(j=>{L=j;
 const t=j.tog<0?'-':(j.pr?'PRESSED':'released')+(j.tog?' HIGH':' LOW');
 const nav=j.sn==0?'no steps in sequence':(j.no?'<span class="bad">OFF</span> (slot '+(j.sl+1)+')':'Slot '+(j.sl+1)+'/'+j.sn+': Step '+(j.si+1)+' '+esc(j.nn));
 $('live').innerHTML='Position: <span class="'+(j.s0?'ok':'bad')+'">'+(j.s0?(j.pos||'-')+' ('+j.w0+' µs)':'NO SIGNAL')+'</span> &nbsp; Toggle: <span class="'+(j.s1?'ok':'bad')+'">'+(j.s1?t+' ('+j.w1+' µs)':'NO SIGNAL')+'</span><br>'
 +(j.tm?'<span class="bad">GPIO TEST ON</span>':'Nav: '+nav+'<br>Instrument: '+lv(0)+' &nbsp; Cockpit: '+lv(1)+' &nbsp; Front: '+lv(2)+(j.m?' &nbsp; <span class="bad">MASTER OFF</span>':''));
 gLab(j.tm,j.tb);authBox(j);
}).catch(()=>{$('live').textContent='No connection';}).finally(()=>setTimeout(live,500));}
function load(){fetch('/cfg').then(r=>r.json()).then(j=>{C=j;build();}).catch(()=>setTimeout(load,2000));}
load();live();
</script></body></html>)HTML";

// ------------------------------ web helpers --------------------------
static void jArr16(String& j, const char* k, const uint16_t* a, int n) {
  j += '"'; j += k; j += "\":[";
  for (int i = 0; i < n; i++) { if (i) j += ','; j += a[i]; }
  j += "],";
}
static void jArr8(String& j, const char* k, const uint8_t* a, int n) {
  j += '"'; j += k; j += "\":[";
  for (int i = 0; i < n; i++) { if (i) j += ','; j += a[i]; }
  j += "],";
}
static void jNum(String& j, const char* k, long v) { j += '"'; j += k; j += "\":"; j += v; j += ','; }

static void handleCfg() {
  String j; j.reserve(4000);
  j = "{\"ver\":\"" FW_VERSION "\",";
  jArr16(j, "pu", cfg.posUs, 6);
  jArr8(j, "pf", cfg.posFn, 6);
  jNum(j, "th", cfg.togHi); jNum(j, "tl", cfg.togLo); jNum(j, "ps", cfg.posStableMs);
  jNum(j, "tt", cfg.togType); jNum(j, "wh", cfg.wifiHoldS); jNum(j, "nh", cfg.navHoldS);
  jNum(j, "wb", cfg.wifiBootS); jNum(j, "wi", cfg.wifiIdleMin);
  j += "\"nav\":[";
  for (int i = 0; i < NAV_STEPS; i++) {
    if (i) j += ',';
    j += "{\"n\":\""; j += cfg.nav[i].name; j += "\",";
    jArr8(j, "p", cfg.nav[i].pat, 6);
    j += "\"b\":[";
    for (int o = 0; o < 6; o++) { if (o) j += ','; j += cfg.nav[i].br[o]; }
    j += "]}";
  }
  j += "],";
  jArr8(j, "seq", cfg.seq, NAV_STEPS);
  jArr8(j, "g", cfg.grpSteps, 3);
  j += "\"fl\":[";
  for (int f = 0; f < 3; f++) {
    if (f) j += ',';
    j += '['; j += cfg.fl[f].on; j += ','; j += cfg.fl[f].gap; j += ','; j += cfg.fl[f].per; j += ']';
  }
  j += "],";
  jNum(j, "bp", cfg.beaconPer); jNum(j, "fp", cfg.fastPer); jNum(j, "ap", cfg.altPer);
  jArr16(j, "of", cfg.offs, 6);
  jArr8(j, "mx", cfg.maxPct, N_OUT);
  jArr8(j, "iv", cfg.inv, N_OUT);
  jArr8(j, "op", cfg.outPinC, N_OUT);
  jArr8(j, "pw", cfg.outPwmC, N_OUT);
  jArr8(j, "ip", cfg.inPinC, 2);
  jArr8(j, "rp", outPin, N_OUT);           // pins in use now
  j += "\"ri\":["; j += inPin[0]; j += ','; j += inPin[1]; j += "],";
  jNum(j, "pd", pinsDefault);
  jNum(j, "hz", cfg.pwmHz); jNum(j, "rm", cfg.remember); jNum(j, "lp", cfg.lowPwr);
  j += "\"ss\":\""; j += cfg.ssid; j += "\"}";
  server.send(200, "application/json", j);
}

static void handleLive() {
  String j; j.reserve(360);
  uint8_t n = seqLen();
  uint8_t si = n ? cfg.seq[st.slot < n ? st.slot : 0] : 0;
  j = "{";
  jNum(j, "s0", sigOk[0]); jNum(j, "w0", sigW[0]);
  jNum(j, "s1", sigOk[1]); jNum(j, "w1", sigW[1]);
  jNum(j, "pos", posCur + 1); jNum(j, "tog", togState); jNum(j, "pr", pressed);
  jNum(j, "sl", st.slot); jNum(j, "sn", n); jNum(j, "si", si); jNum(j, "no", st.navOff);
  jArr8(j, "lv", st.lvl, 3);
  jArr8(j, "g", cfg.grpSteps, 3);
  jNum(j, "m", st.master);
  jNum(j, "tm", testMode); jNum(j, "tb", testMask);
  jNum(j, "au", authOk); jNum(j, "ao", authOff); jNum(j, "kb", authKeyBad);
  j += "\"id\":\""; j += boardId; j += "\",";
  j += "\"nn\":\""; j += cfg.nav[si].name; j += "\"}";
  server.send(200, "application/json", j);
}

static long argNum(const char* k, long lo, long hi, long cur) {
  if (!server.hasArg(k)) return cur;
  String s = server.arg(k); s.trim();
  if (!s.length()) return cur;
  long v = s.toInt();
  return v < lo ? lo : (v > hi ? hi : v);
}

// copies a text field, keeping only safe characters; returns length (-1 = not sent)
static int argText(const char* k, char* dst, size_t n) {
  if (!server.hasArg(k)) return -1;
  String s = server.arg(k);
  size_t j = 0;
  for (size_t i = 0; i < s.length() && j < n - 1; i++) {
    char c = s[i];
    if (c < 32 || c > 126 || c == '"' || c == '\\' || c == '<' || c == '>' || c == '&') continue;
    dst[j++] = c;
  }
  dst[j] = 0;
  return (int)j;
}

static void handleSave() {
  Cfg n = cfg;
  char key[12];
  for (int i = 0; i < 6; i++) {
    snprintf(key, sizeof(key), "pu%d", i); n.posUs[i] = argNum(key, 800, 2200, n.posUs[i]);
    snprintf(key, sizeof(key), "pf%d", i); n.posFn[i] = argNum(key, 0, FN_COUNT - 1, n.posFn[i]);
  }
  n.togHi = argNum("th", 1000, 2000, n.togHi);
  n.togLo = argNum("tl", 1000, 2000, n.togLo);
  n.posStableMs = argNum("ps", 20, 2000, n.posStableMs);
  n.togType = argNum("tt", 0, TT_COUNT - 1, n.togType);
  n.wifiHoldS = argNum("wh", 2, 30, n.wifiHoldS);
  n.navHoldS = argNum("nh", 1, 10, n.navHoldS);
  n.wifiBootS = argNum("wb", 0, 600, n.wifiBootS);
  n.wifiIdleMin = argNum("wi", 0, 60, n.wifiIdleMin);
  if (n.togLo >= n.togHi) { server.send(400, "text/plain", "Not saved: toggle LOW must be below HIGH"); return; }
  for (int i = 0; i < NAV_STEPS; i++) {
    NavStep& s = n.nav[i];
    char nm[13];
    snprintf(key, sizeof(key), "n%dn", i);
    int l = argText(key, nm, sizeof(nm));
    if (l > 0) strcpy(s.name, nm); else if (l == 0) snprintf(s.name, sizeof(s.name), "Step %d", i + 1);
    for (int o = 0; o < 6; o++) {
      snprintf(key, sizeof(key), "n%dp%d", i, o); s.pat[o] = argNum(key, 0, P_COUNT - 1, s.pat[o]);
      snprintf(key, sizeof(key), "n%db%d", i, o); s.br[o]  = argNum(key, 0, 100, s.br[o]);
    }
  }
  bool ended = false;
  for (int k = 0; k < NAV_STEPS; k++) {
    snprintf(key, sizeof(key), "q%d", k);
    long v = argNum(key, 0, 255, n.seq[k]);
    if (v >= NAV_STEPS) v = SEQ_END;
    if (ended) v = SEQ_END;                     // everything after the first end is end
    if (v == SEQ_END) ended = true;
    n.seq[k] = v;
  }
  if (n.seq[0] == SEQ_END) { server.send(400, "text/plain", "Not saved: nav sequence slot 1 needs a step"); return; }
  for (int g = 0; g < 3; g++) { snprintf(key, sizeof(key), "g%d", g); n.grpSteps[g] = argNum(key, 1, 10, n.grpSteps[g]); }
  for (int f = 0; f < 3; f++) {
    snprintf(key, sizeof(key), "f%do", f); n.fl[f].on  = argNum(key, 10, 5000, n.fl[f].on);
    snprintf(key, sizeof(key), "f%dg", f); n.fl[f].gap = argNum(key, 10, 5000, n.fl[f].gap);
    snprintf(key, sizeof(key), "f%dp", f); n.fl[f].per = argNum(key, 50, 10000, n.fl[f].per);
  }
  n.beaconPer = argNum("bp", 50, 10000, n.beaconPer);
  n.fastPer   = argNum("fp", 20, 10000, n.fastPer);
  n.altPer    = argNum("ap", 20, 10000, n.altPer);
  for (int o = 0; o < 6; o++) { snprintf(key, sizeof(key), "of%d", o); n.offs[o] = argNum(key, 0, 10000, n.offs[o]); }
  for (int o = 0; o < N_OUT; o++) {
    snprintf(key, sizeof(key), "mx%d", o); n.maxPct[o] = argNum(key, 0, 100, n.maxPct[o]);
    snprintf(key, sizeof(key), "iv%d", o); n.inv[o]    = argNum(key, 0, 1, n.inv[o]);
    snprintf(key, sizeof(key), "op%d", o); n.outPinC[o] = argNum(key, 0, 255, n.outPinC[o]);
    snprintf(key, sizeof(key), "pw%d", o); n.outPwmC[o] = argNum(key, 0, 1, n.outPwmC[o]);
  }
  n.remember = argNum("rm", 0, 1, n.remember);
  n.lowPwr   = argNum("lp", 0, 1, n.lowPwr);
  n.pwmHz    = argNum("hz", 100, 20000, n.pwmHz);
  char tmp[24];
  if (argText("ss", tmp, sizeof(tmp)) > 0) strcpy(n.ssid, tmp);
  int pl = argText("pw", tmp, sizeof(tmp));
  if (pl > 0 && pl < 8) { server.send(400, "text/plain", "Not saved: WiFi password needs 8+ characters"); return; }
  if (pl >= 8) strcpy(n.pass, tmp);

  for (int c = 0; c < 2; c++) { snprintf(key, sizeof(key), "ip%d", c); n.inPinC[c] = argNum(key, 0, 255, n.inPinC[c]); }
  String pe = pinMapError(n.outPinC, n.outPwmC, n.inPinC);
  if (pe.length()) { server.send(400, "text/plain", "Not saved: " + pe); return; }
  bool pinsChanged = memcmp(n.outPinC, cfg.outPinC, N_OUT) || memcmp(n.outPwmC, cfg.outPwmC, N_OUT) ||
                     memcmp(n.inPinC, cfg.inPinC, 2);
  bool needRestart = (n.pwmHz != cfg.pwmHz) || strcmp(n.ssid, cfg.ssid) || strcmp(n.pass, cfg.pass);
  cfg = n;
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
  long o = argNum("o", -1, N_OUT - 1, -1);
  if (o < 0) { server.send(400, "text/plain", "Bad output"); return; }
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
  for (int i = 0; i < N_OUT; i++) {
    lastDuty[i] = -1;
    if (outPin[i] == PIN_NONE) continue;
    if (outPwm[i] && !ledcAttach(outPin[i], cfg.pwmHz, 10)) outPwm[i] = false;  // no channel left: on/off
    if (!outPwm[i]) pinMode(outPin[i], OUTPUT);
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
