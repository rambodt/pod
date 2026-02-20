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

// ------------------- Parsed state -------------------
struct {
  // time/date (UTC) from RMC
  bool   hasDate = false;
  bool   hasTime = false;
  int    yy=0, mm=0, dd=0;
  int    hh=0, min=0, ss=0;

  // position
  bool   hasFix = false;
  bool   rmcValid = false;       // RMC status = 'A' (ADDED)
  int    fixQuality = 0;         // from GGA
  int    satsUsed   = 0;         // from GGA
  int    satsInView = 0;         // from GSV
  double lat = 0.0;
  double lon = 0.0;
  double alt_m = 0.0;            // from GGA

  // speed
  double speed_knots = 0.0;      // from RMC
  double speed_kmh   = 0.0;      // from VTG if present

  // DOP
  double hdop_gga = NAN;         // from GGA
  double pdop = NAN;             // from GSA
  double hdop = NAN;             // from GSA
  double vdop = NAN;             // from GSA

  // per-sat SNR table (from GSV)
  // store up to 64 sats: id + snr (cn0)
  int satId[64];
  int satSnr[64];
  int satCount = 0;

  // "new solution" latch
  bool updatedThisSecond = false;
  
  // Track last written UTC second to prevent duplicates (ADDED)
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
  // line like: $.....*HH
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
  // splits in-place on ',' and also terminates at '*'
  int n = 0;
  out[n++] = s;

  for (char* p = s; *p && n < maxOut; p++) {
    if (*p == '*') { *p = 0; break; }
    if (*p == ',') { *p = 0; out[n++] = p + 1; }
  }
  // fill remainder with nullptr
  for (; n < maxOut; n++) out[n] = nullptr;
}

static double parseLatLon(const char* ddmm, const char* hemi) {
  // ddmm.mmmm (lat) or dddmm.mmmm (lon)
  if (!ddmm || !*ddmm || !hemi || !*hemi) return NAN;

  double v = atof(ddmm);
  int deg = (int)(v / 100.0);
  double minutes = v - (deg * 100.0);
  double dec = (double)deg + minutes / 60.0;

  char h = hemi[0];
  if (h == 'S' || h == 'W') dec = -dec;
  return dec;
}

// ------------------- FIXED YEAR CALCULATION -------------------
static int computeYear(int yy) {
  // GPS epoch is 1980. NMEA RMC date is 2-digit year.
  // If yy >= 80, it's 1980-1999
  // If yy < 80, it's 2000-2079
  if (yy >= 80) {
    return 1900 + yy;  // 80-99 → 1980-1999
  } else {
    return 2000 + yy;  // 00-79 → 2000-2079
  }
}

static void maybeSetRtcFromGps() {
  if (rtcSetFromGps) return;
  if (!gps.hasDate || !gps.hasTime) return;
  if (!gps.rmcValid) return;  // Only set RTC from valid fix (ADDED)

  // Basic sanity
  int year = computeYear(gps.yy);  // FIXED
  if (gps.mm < 1 || gps.mm > 12) return;
  if (gps.dd < 1 || gps.dd > 31) return;
  if (gps.hh < 0 || gps.hh > 23) return;
  if (gps.min < 0 || gps.min > 59) return;
  if (gps.ss < 0 || gps.ss > 59) return;

  rtc.adjust(DateTime(year, gps.mm, gps.dd, gps.hh, gps.min, gps.ss));
  rtcSetFromGps = true;
}

static DateTime nowUtc() {
  // CRITICAL: Always use GPS time directly for exact timestamps
  // DO NOT use RTC - it causes ~1 second offset
  // This is essential for distributed array analysis where exact timing matters
  
  if (gps.hasDate && gps.hasTime) {
    int year = computeYear(gps.yy);
    return DateTime(year, gps.mm, gps.dd, gps.hh, gps.min, gps.ss);
  }

  // Fallback only if GPS hasn't provided time yet
  // This should only happen during initial startup
  if (rtcIsRunning && rtcSetFromGps) {
    return rtc.now();
  }

  // last resort: "fake epoch" with millis
  uint32_t t = millis() / 1000;
  return DateTime(2000, 1, 1, 0, 0, 0) + TimeSpan((int32_t)t);
}

static void formatTimestamp(const DateTime& t, char* buf, size_t n) {
  // UTC ISO-like: YYYY-MM-DDTHH:MM:SSZ
  snprintf(buf, n, "%04d-%02d-%02dT%02d:%02d:%02dZ",
           t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
}

static void satSnrList(char* buf, size_t n) {
  // "id:snr|id:snr|..."
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

  // RAW_YYYYMMDD_HHMMSS.TXT  and  PROC_YYYYMMDD_HHMMSS.CSV
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

  procFile.println("ts_utc,lat,lon,alt_m,speed_knots,speed_kmh,fix_quality,sats_used,sats_in_view,hdop_gga,pdop,hdop,vdop,per_sat_snr");
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
  // f[0] is like "$GPGGA" or "GNGGA" depending on split usage
  // fields: 1 time, 2 lat,3 N/S, 4 lon,5 E/W, 6 fixq, 7 numsats, 8 hdop, 9 alt, 10 M ...
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
  // fields: 1 time, 2 status, 3 lat,4 N/S, 5 lon,6 E/W, 7 speed(knots), 8 course, 9 date(ddmmyy)
  if (f[1] && strlen(f[1]) >= 6) {
    gps.hh  = (f[1][0]-'0')*10 + (f[1][1]-'0');
    gps.min = (f[1][2]-'0')*10 + (f[1][3]-'0');
    gps.ss  = (f[1][4]-'0')*10 + (f[1][5]-'0');
    gps.hasTime = true;
  }

  // status A=valid, V=void (FIXED - now stored properly)
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
  // NMEA GSA last fields are PDOP, HDOP, VDOP (often f[n-3], f[n-2], f[n-1])
  // Works for both $GPGSA and $GNGSA
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
  // fields: 1 total_msgs, 2 msg_num, 3 sats_in_view,
  // then repeating: sat_id, elev, az, snr
  
  // FIX: Accumulate sats_in_view from multiple GSV sentences (GPGSV + GLGSV)
  // Each constellation reports its own count, we need to sum them
  static int gpgsv_sats = 0;
  static int glgsv_sats = 0;
  
  // Detect which constellation this GSV is from
  const char* talker = f[0];  // e.g., "GPGSV" or "GLGSV"
  bool isGP = (talker && talker[0] == 'G' && talker[1] == 'P');
  bool isGL = (talker && talker[0] == 'G' && talker[1] == 'L');
  
  if (f[3] && *f[3]) {
    int sats = atoi(f[3]);
    if (isGP) gpgsv_sats = sats;
    if (isGL) glgsv_sats = sats;
  }
  
  // Update total (GPS + GLONASS + any other constellations)
  gps.satsInView = gpgsv_sats + glgsv_sats;

  // sat blocks start at f[4]
  for (int i = 4; i + 3 < 32; i += 4) {
    if (!f[i] || !*f[i]) break;
    int id = atoi(f[i]);
    int snr = (f[i+3] && *f[i+3]) ? atoi(f[i+3]) : -1;
    updateSatSnr(id, snr);
  }

  gps.updatedThisSecond = true;
}

static void parseVTG(char** f) {
  // VTG provides speed in km/h, but it arrives AFTER GGA/RMC
  // This causes timing issues. Instead, we'll calculate km/h from knots in RMC.
  // Keeping this parser for future use, but not using it for CSV logging.
  
  // Format: courseT, T, courseM, M, speedKnots, N, speedKmh, K, mode
  // Mode indicator (f[9]): A=autonomous, D=differential, E=estimated, N=not valid
  
  if (f[9] && *f[9]) {
    char mode = f[9][0];
    if (mode != 'A' && mode != 'D' && mode != 'E') {
      return;  // Skip invalid VTG sentence
    }
  } else {
    return;
  }
  
  if (f[5] && *f[5]) gps.speed_knots = atof(f[5]);
  if (f[7] && *f[7]) gps.speed_kmh   = atof(f[7]);
  // Do NOT set updatedThisSecond to avoid duplicate CSV writes
}

static void parseNmeaSentence(char* line) {
  // Validate checksum if present
  if (strchr(line, '*')) {
    if (!nmeaChecksumOK(line)) return; // drop corrupted sentence
  }

  // Make a working copy to split (in-place)
  char s[NMEA_MAX];
  strncpy(s, line, sizeof(s));
  s[sizeof(s)-1] = 0;

  // Strip leading '$'
  char* start = s;
  if (*start == '$') start++;

  // split
  char* f[32];
  for (int i = 0; i < 32; i++) f[i] = nullptr;
  splitFields(start, f, 32);

  if (!f[0]) return;

  // detect sentence type by last 3 chars: GGA/RMC/GSA/GSV/VTG
  const char* t = f[0];
  size_t L = strlen(t);
  if (L < 3) return;
  const char* tail = t + (L - 3);

  // count how many fields were produced (until nullptr)
  int nf = 0; while (nf < 32 && f[nf] != nullptr) nf++;

  if (!strcmp(tail, "GGA")) parseGGA(f);
  else if (!strcmp(tail, "RMC")) parseRMC(f);
  else if (!strcmp(tail, "GSA")) parseGSA(f, nf);
  else if (!strcmp(tail, "GSV")) parseGSV(f);
  else if (!strcmp(tail, "VTG")) parseVTG(f);
}

// ------------------- FIXED CSV WRITING -------------------
static void writeProcessedRowIfReady() {
  if (!gps.updatedThisSecond) return;
  gps.updatedThisSecond = false;

  // CRITICAL FIX 1: Only write if we have a valid fix
  // A fix is valid when:
  // - RMC status = 'A' (valid)
  // - AND GGA fix quality > 0
  if (!gps.rmcValid || gps.fixQuality == 0) {
    return;  // Skip invalid fixes
  }

  // CRITICAL FIX 2: Only write once per UTC second
  // Check if we've already written for this second
  if (gps.hh == gps.lastWrittenHour && 
      gps.min == gps.lastWrittenMin && 
      gps.ss == gps.lastWrittenSec) {
    return;  // Already wrote for this second
  }

  // Update the last written time
  gps.lastWrittenHour = gps.hh;
  gps.lastWrittenMin = gps.min;
  gps.lastWrittenSec = gps.ss;

  DateTime t = nowUtc();
  char ts[32];
  formatTimestamp(t, ts, sizeof(ts));

  char snrBuf[512];
  satSnrList(snrBuf, sizeof(snrBuf));

  // Write CSV row
  procFile.print(ts); procFile.print(",");

  // lat/lon
  if (!isnan(gps.lat)) procFile.print(gps.lat, 8);
  procFile.print(",");

  if (!isnan(gps.lon)) procFile.print(gps.lon, 8);
  procFile.print(",");

  if (!isnan(gps.alt_m)) procFile.print(gps.alt_m, 3);
  procFile.print(",");

  procFile.print(gps.speed_knots, 3); procFile.print(",");
  // Calculate km/h from knots (1 knot = 1.852 km/h)
  // This avoids timing issues with VTG which arrives after GGA/RMC
  double speed_kmh_calculated = gps.speed_knots * 1.852;
  procFile.print(speed_kmh_calculated, 3);   procFile.print(",");

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

  // LED pulse on each processed row
  digitalWrite(LED_BUILTIN, HIGH);
  lastLedMs = millis();
}

// ------------------- setup/loop -------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  delay(200);

  // RTC init
  Wire.begin();
  rtcIsRunning = rtc.begin();
  if (rtcIsRunning) {
    // If not initialized it still "begins"; we only use it once GPS sets it.
    // (rtc.initialized() exists for DS3231; PCF8523 uses rtc.initialized() too in RTClib)
    // We'll be conservative:
    // rtcSetFromGps remains false until GPS gives date/time.
  }

  // SD init
  if (!SD.begin(SD_CS)) {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH); delay(100);
      digitalWrite(LED_BUILTIN, LOW);  delay(100);
    }
  }

  // GPS serial
  Serial1.begin(GPS_BAUD);

  // Ask GPS to output "all data" at 1 Hz.
  // (If your module ignores these, it's okay — RAW logging still works.)
  Serial1.print("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"); // GGA,GSA,GSV,RMC,VTG + more
  Serial1.print("$PMTK220,1000*1F\r\n"); // 1Hz

  openNewLogFiles();

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // LED auto-off after pulse
  if (digitalRead(LED_BUILTIN) && (millis() - lastLedMs) > LED_PULSE_MS) {
    digitalWrite(LED_BUILTIN, LOW);
  }

  // Read GPS bytes
  while (Serial1.available()) {
    char c = (char)Serial1.read();

    // Build line
    if (c == '\r') continue;

    if (c == '\n') {
      nmeaLine[nmeaIdx] = 0;

      // Write RAW
      if (nmeaIdx > 0) {
        rawFile.println(nmeaLine);
        parseNmeaSentence(nmeaLine);

        // If we now have good date/time, set RTC once
        if (rtcIsRunning) {
          maybeSetRtcFromGps();
        }

        // Write processed row opportunistically
        writeProcessedRowIfReady();
      }

      nmeaIdx = 0;
    } else {
      if (nmeaIdx < (NMEA_MAX - 1)) {
        nmeaLine[nmeaIdx++] = c;
      } else {
        // overflow: drop line
        nmeaIdx = 0;
      }
    }
  }

  // periodic flush
  uint32_t now = millis();
  if (now - lastFlushMs >= FLUSH_MS) {
    rawFile.flush();
    procFile.flush();
    lastFlushMs = now;
  }
}
