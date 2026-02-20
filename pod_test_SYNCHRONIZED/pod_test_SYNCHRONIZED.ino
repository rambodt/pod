#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>

// ------------------- USER CONFIG -------------------
#define SD_CS        10          // Adalogger FeatherWing default on many Feathers
#define GPS_BAUD     9600
#define NMEA_MAX     128
#define FLUSH_MS     1000        // flush SD at least every 1s
#define LED_PULSE_MS 30

// SYNCHRONIZED CAPTURE CONFIG
#define TRIGGER_PIN  7           // D5 (GPIO7) - the pin next to SCL
#define TRIGGER_MODE FALLING     // Button press pulls pin LOW
// ---------------------------------------------------

RTC_PCF8523 rtc;

File rawFile;
File procFile;

static char nmeaLine[NMEA_MAX];
static uint16_t nmeaIdx = 0;

static uint32_t lastFlushMs = 0;
static uint32_t lastLedMs   = 0;

static bool rtcIsRunning = false;
static bool rtcSetFromGps = false;

// ------------------- SYNCHRONIZED CAPTURE -------------------
volatile bool captureTriggered = false;
volatile uint32_t triggerMicros = 0;
uint32_t captureCount = 0;

// Interrupt Service Routine - MUST be fast!
void triggerISR() {
  captureTriggered = true;
  triggerMicros = micros();  // Capture exact timing
}
// -----------------------------------------------------------

// ------------------- Parsed state -------------------
struct {
  // time/date (UTC) from RMC
  bool   hasDate = false;
  bool   hasTime = false;
  int    yy=0, mm=0, dd=0;
  int    hh=0, min=0, ss=0;

  // position
  bool   hasFix = false;
  bool   rmcValid = false;       // RMC status = 'A'
  int    fixQuality = 0;         // from GGA
  int    satsUsed   = 0;         // from GGA
  int    satsInView = 0;         // from GSV
  double lat = 0.0;
  double lon = 0.0;
  double alt_m = 0.0;            // from GGA

  // speed
  double speed_knots = 0.0;      // from RMC
  double speed_kmh   = 0.0;      // calculated from knots

  // DOP
  double hdop_gga = NAN;         // from GGA
  double pdop = NAN;             // from GSA
  double hdop = NAN;             // from GSA
  double vdop = NAN;             // from GSA

  // per-sat SNR table (from GSV)
  int satId[64];
  int satSnr[64];
  int satCount = 0;

  // "new solution" latch
  bool updatedThisSecond = false;
  
  // Track last written UTC second to prevent duplicates
  int lastWrittenHour = -1;
  int lastWrittenMin = -1;
  int lastWrittenSec = -1;
} gps;

// ------------------- helpers -------------------
static bool isDigitStr(const char* s) {
  if (!s || !*s) return false;
  for (; *s; s++) if (*s < '0' || *s > '9') return false;
  return true;
}

static uint8_t hexNibble(char c) {
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
  if (c >= 'A' && c <= 'F') return (uint8_t)(10 + c - 'A');
  if (c >= 'a' && c <= 'f') return (uint8_t)(10 + c - 'a');
  return 0;
}

static bool nmeaChecksumOK(const char* line) {
  const char* p = line;
  if (!p || *p != '$') return false;
  const char* star = nullptr;
  for (; *p; p++) if (*p == '*') { star = p; break; }
  if (!star) return false;
  if (!star[1] || !star[2]) return false;

  uint8_t cs = 0;
  for (const char* q = line + 1; q < star; q++) cs ^= (uint8_t)(*q);

  uint8_t want = (hexNibble(star[1]) << 4) | hexNibble(star[2]);
  return cs == want;
}

static void splitFields(char* s, char** out, int maxOut) {
  int n = 0;
  out[n++] = s;

  for (char* p = s; *p && n < maxOut; p++) {
    if (*p == '*') { *p = 0; break; }
    if (*p == ',') { *p = 0; out[n++] = p + 1; }
  }
  for (; n < maxOut; n++) out[n] = nullptr;
}

static double parseLatLon(const char* ddmm, const char* hemi) {
  if (!ddmm || !*ddmm || !hemi || !*hemi) return NAN;

  double v = atof(ddmm);
  int deg = (int)(v / 100.0);
  double minutes = v - (deg * 100.0);
  double dec = (double)deg + minutes / 60.0;

  char h = hemi[0];
  if (h == 'S' || h == 'W') dec = -dec;
  return dec;
}

static int computeYear(int yy) {
  if (yy >= 80) {
    return 1900 + yy;  // 80-99 → 1980-1999
  } else {
    return 2000 + yy;  // 00-79 → 2000-2079
  }
}

static void maybeSetRtcFromGps() {
  if (rtcSetFromGps) return;
  if (!gps.hasDate || !gps.hasTime) return;
  if (!gps.rmcValid) return;

  int year = computeYear(gps.yy);
  if (gps.mm < 1 || gps.mm > 12) return;
  if (gps.dd < 1 || gps.dd > 31) return;
  if (gps.hh < 0 || gps.hh > 23) return;
  if (gps.min < 0 || gps.min > 59) return;
  if (gps.ss < 0 || gps.ss > 59) return;

  rtc.adjust(DateTime(year, gps.mm, gps.dd, gps.hh, gps.min, gps.ss));
  rtcSetFromGps = true;
}

static DateTime nowUtc() {
  // CRITICAL: Always use GPS time directly
  if (gps.hasDate && gps.hasTime) {
    int year = computeYear(gps.yy);
    return DateTime(year, gps.mm, gps.dd, gps.hh, gps.min, gps.ss);
  }

  // Fallback to RTC if GPS not ready
  if (rtcIsRunning && rtcSetFromGps) {
    return rtc.now();
  }

  // Last resort
  uint32_t t = millis() / 1000;
  return DateTime(2000, 1, 1, 0, 0, 0) + TimeSpan((int32_t)t);
}

static void formatTimestamp(const DateTime& t, char* buf, size_t n) {
  snprintf(buf, n, "%04d-%02d-%02dT%02d:%02d:%02dZ",
           t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
}

static void satSnrList(char* buf, size_t n) {
  buf[0] = 0;
  size_t used = 0;
  for (int i = 0; i < gps.satCount; i++) {
    if (gps.satId[i] <= 0) continue;
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%d:%d", gps.satId[i], gps.satSnr[i]);
    size_t need = strlen(tmp) + (used ? 1 : 0);
    if (used + need + 1 >= n) break;
    if (used) buf[used++] = '|';
    strcpy(&buf[used], tmp);
    used += strlen(tmp);
    buf[used] = 0;
  }
}

static void openNewLogFiles() {
  DateTime t = nowUtc();

  char rawName[32];
  char procName[32];

  snprintf(rawName, sizeof(rawName),  "RAW_%04d%02d%02d_%02d%02d%02d.TXT",
           t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
  snprintf(procName, sizeof(procName), "PROC_%04d%02d%02d_%02d%02d%02d.CSV",
           t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());

  rawFile = SD.open(rawName, FILE_WRITE);
  procFile = SD.open(procName, FILE_WRITE);

  if (!rawFile || !procFile) {
    // slow blink forever if file open fails
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH); delay(300);
      digitalWrite(LED_BUILTIN, LOW);  delay(300);
    }
  }

  // headers
  rawFile.println("# RAW NMEA log");
  rawFile.flush();

  // CSV header with capture_id and trigger_micros for synchronized mode
  procFile.println("capture_id,trigger_micros,ts_utc,lat,lon,alt_m,speed_knots,speed_kmh,fix_quality,sats_used,sats_in_view,hdop_gga,pdop,hdop,vdop,per_sat_snr");
  procFile.flush();

  lastFlushMs = millis();
}

// ------------------- NMEA parsers -------------------
static void updateSatSnr(int id, int snr) {
  if (id <= 0) return;
  for (int i = 0; i < gps.satCount; i++) {
    if (gps.satId[i] == id) {
      gps.satSnr[i] = snr;
      return;
    }
  }
  if (gps.satCount < 64) {
    gps.satId[gps.satCount] = id;
    gps.satSnr[gps.satCount] = snr;
    gps.satCount++;
  }
}

static void parseGGA(char** f) {
  if (f[1] && strlen(f[1]) >= 6) {
    gps.hh  = (f[1][0]-'0')*10 + (f[1][1]-'0');
    gps.min = (f[1][2]-'0')*10 + (f[1][3]-'0');
    gps.ss  = (f[1][4]-'0')*10 + (f[1][5]-'0');
    gps.hasTime = true;
  }

  double lat = parseLatLon(f[2], f[3]);
  double lon = parseLatLon(f[4], f[5]);
  if (!isnan(lat) && !isnan(lon)) {
    gps.lat = lat;
    gps.lon = lon;
  }

  gps.fixQuality = f[6] ? atoi(f[6]) : 0;
  gps.satsUsed   = f[7] ? atoi(f[7]) : 0;
  gps.hdop_gga   = (f[8] && *f[8]) ? atof(f[8]) : NAN;
  gps.alt_m      = (f[9] && *f[9]) ? atof(f[9]) : NAN;

  gps.hasFix = (gps.fixQuality > 0);
  gps.updatedThisSecond = true;
}

static void parseRMC(char** f) {
  if (f[1] && strlen(f[1]) >= 6) {
    gps.hh  = (f[1][0]-'0')*10 + (f[1][1]-'0');
    gps.min = (f[1][2]-'0')*10 + (f[1][3]-'0');
    gps.ss  = (f[1][4]-'0')*10 + (f[1][5]-'0');
    gps.hasTime = true;
  }

  gps.rmcValid = (f[2] && f[2][0] == 'A');
  if (gps.rmcValid) gps.hasFix = true;

  double lat = parseLatLon(f[3], f[4]);
  double lon = parseLatLon(f[5], f[6]);
  if (!isnan(lat) && !isnan(lon)) {
    gps.lat = lat;
    gps.lon = lon;
  }

  gps.speed_knots = (f[7] && *f[7]) ? atof(f[7]) : gps.speed_knots;

  if (f[9] && strlen(f[9]) == 6) {
    gps.dd = (f[9][0]-'0')*10 + (f[9][1]-'0');
    gps.mm = (f[9][2]-'0')*10 + (f[9][3]-'0');
    gps.yy = (f[9][4]-'0')*10 + (f[9][5]-'0');
    gps.hasDate = true;
  }

  gps.updatedThisSecond = true;
}

static void parseGSA(char** f, int nf) {
  if (nf < 6) return;
  const char* pd = f[nf-3];
  const char* hd = f[nf-2];
  const char* vd = f[nf-1];

  gps.pdop = (pd && *pd) ? atof(pd) : gps.pdop;
  gps.hdop = (hd && *hd) ? atof(hd) : gps.hdop;
  gps.vdop = (vd && *vd) ? atof(vd) : gps.vdop;

  gps.updatedThisSecond = true;
}

static void parseGSV(char** f) {
  static int gpgsv_sats = 0;
  static int glgsv_sats = 0;
  
  const char* talker = f[0];
  bool isGP = (talker && talker[0] == 'G' && talker[1] == 'P');
  bool isGL = (talker && talker[0] == 'G' && talker[1] == 'L');
  
  if (f[3] && *f[3]) {
    int sats = atoi(f[3]);
    if (isGP) gpgsv_sats = sats;
    if (isGL) glgsv_sats = sats;
  }
  
  gps.satsInView = gpgsv_sats + glgsv_sats;

  for (int i = 4; i + 3 < 32; i += 4) {
    if (!f[i] || !*f[i]) break;
    int id = atoi(f[i]);
    int snr = (f[i+3] && *f[i+3]) ? atoi(f[i+3]) : -1;
    updateSatSnr(id, snr);
  }

  gps.updatedThisSecond = true;
}

static void parseVTG(char** f) {
  if (f[9] && *f[9]) {
    char mode = f[9][0];
    if (mode != 'A' && mode != 'D' && mode != 'E') {
      return;
    }
  } else {
    return;
  }
  
  if (f[5] && *f[5]) gps.speed_knots = atof(f[5]);
  if (f[7] && *f[7]) gps.speed_kmh   = atof(f[7]);
}

static void parseNmeaSentence(char* line) {
  if (strchr(line, '*')) {
    if (!nmeaChecksumOK(line)) return;
  }

  char s[NMEA_MAX];
  strncpy(s, line, sizeof(s));
  s[sizeof(s)-1] = 0;

  char* start = s;
  if (*start == '$') start++;

  char* f[32];
  for (int i = 0; i < 32; i++) f[i] = nullptr;
  splitFields(start, f, 32);

  if (!f[0]) return;

  const char* t = f[0];
  size_t L = strlen(t);
  if (L < 3) return;
  const char* tail = t + (L - 3);

  int nf = 0; 
  while (nf < 32 && f[nf] != nullptr) nf++;

  if (!strcmp(tail, "GGA")) parseGGA(f);
  else if (!strcmp(tail, "RMC")) parseRMC(f);
  else if (!strcmp(tail, "GSA")) parseGSA(f, nf);
  else if (!strcmp(tail, "GSV")) parseGSV(f);
  else if (!strcmp(tail, "VTG")) parseVTG(f);
}

// ------------------- SYNCHRONIZED CAPTURE -------------------
static void writeSynchronizedSnapshot() {
  // Only write if we have valid GPS fix
  if (!gps.rmcValid || gps.fixQuality == 0) {
    Serial.println("⚠ Trigger received but no valid GPS fix!");
    return;
  }

  captureCount++;
  
  DateTime t = nowUtc();
  char ts[32];
  formatTimestamp(t, ts, sizeof(ts));

  char snrBuf[512];
  satSnrList(snrBuf, sizeof(snrBuf));

  // Write CSV row with capture_id and trigger timing
  procFile.print(captureCount); procFile.print(",");
  procFile.print(triggerMicros); procFile.print(",");
  procFile.print(ts); procFile.print(",");

  // lat/lon
  if (!isnan(gps.lat)) procFile.print(gps.lat, 8);
  procFile.print(",");

  if (!isnan(gps.lon)) procFile.print(gps.lon, 8);
  procFile.print(",");

  if (!isnan(gps.alt_m)) procFile.print(gps.alt_m, 3);
  procFile.print(",");

  procFile.print(gps.speed_knots, 3); procFile.print(",");
  
  // Calculate km/h from knots
  double speed_kmh_calculated = gps.speed_knots * 1.852;
  procFile.print(speed_kmh_calculated, 3); procFile.print(",");

  procFile.print(gps.fixQuality); procFile.print(",");
  procFile.print(gps.satsUsed);   procFile.print(",");
  procFile.print(gps.satsInView); procFile.print(",");

  if (!isnan(gps.hdop_gga)) procFile.print(gps.hdop_gga, 2);
  procFile.print(",");

  if (!isnan(gps.pdop)) procFile.print(gps.pdop, 2);
  procFile.print(",");

  if (!isnan(gps.hdop)) procFile.print(gps.hdop, 2);
  procFile.print(",");

  if (!isnan(gps.vdop)) procFile.print(gps.vdop, 2);
  procFile.print(",");

  procFile.println(snrBuf);
  procFile.flush();  // Flush immediately to ensure data is written

  // LED pulse on capture
  digitalWrite(LED_BUILTIN, HIGH);
  lastLedMs = millis();

  // Serial feedback
  Serial.print("✓ Snapshot #");
  Serial.print(captureCount);
  Serial.print(" captured at ");
  Serial.print(ts);
  Serial.print(" (trigger_micros=");
  Serial.print(triggerMicros);
  Serial.println(")");
}
// -----------------------------------------------------------

// ------------------- setup/loop -------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  delay(200);
  
  Serial.println("======================================");
  Serial.println("GPS POD - SYNCHRONIZED CAPTURE MODE");
  Serial.println("======================================");

  // RTC init
  Wire.begin();
  rtcIsRunning = rtc.begin();

  // SD init
  if (!SD.begin(SD_CS)) {
    Serial.println("❌ SD card failed!");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH); delay(100);
      digitalWrite(LED_BUILTIN, LOW);  delay(100);
    }
  }
  Serial.println("✓ SD card ready");

  // GPS serial
  Serial1.begin(GPS_BAUD);
  Serial.println("✓ GPS serial started");

  // Configure GPS for all data at 1 Hz
  Serial1.print("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
  Serial1.print("$PMTK220,1000*1F\r\n");

  openNewLogFiles();
  Serial.println("✓ Log files created");

  // Setup interrupt trigger pin
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), triggerISR, TRIGGER_MODE);
  Serial.println("✓ Trigger interrupt configured on D5 (GPIO7)");
  
  Serial.println("\nWaiting for GPS lock...");
  Serial.println("Press trigger button to capture synchronized snapshots");
  Serial.println("======================================\n");

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // LED auto-off after pulse
  if (digitalRead(LED_BUILTIN) && (millis() - lastLedMs) > LED_PULSE_MS) {
    digitalWrite(LED_BUILTIN, LOW);
  }

  // Read GPS bytes and continuously update state
  while (Serial1.available()) {
    char c = (char)Serial1.read();

    if (c == '\r') continue;

    if (c == '\n') {
      nmeaLine[nmeaIdx] = 0;

      if (nmeaIdx > 0) {
        // Always write to RAW file
        rawFile.println(nmeaLine);
        
        // Parse NMEA to update GPS state
        parseNmeaSentence(nmeaLine);

        // Update RTC if needed
        if (rtcIsRunning) {
          maybeSetRtcFromGps();
        }
      }

      nmeaIdx = 0;
    } else {
      if (nmeaIdx < (NMEA_MAX - 1)) {
        nmeaLine[nmeaIdx++] = c;
      } else {
        nmeaIdx = 0;
      }
    }
  }

  // Check for trigger interrupt
  if (captureTriggered) {
    captureTriggered = false;  // Clear flag
    
    // Write synchronized snapshot
    writeSynchronizedSnapshot();
  }

  // Periodic flush of RAW file
  uint32_t now = millis();
  if (now - lastFlushMs >= FLUSH_MS) {
    rawFile.flush();
    lastFlushMs = now;
  }
}
