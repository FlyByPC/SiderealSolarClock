/*
  UTC + Sidereal Clock for ESP32 LilyGo T-Display (240x135 ST7789) + I2S Beeps

  - Connects to WiFi, syncs UTC via NTP.
  - Computes GMST (sidereal) from UTC.
  - Displays UTC and GMST as HH:MM:SS, each updating on its own second edge.
  - If NTP fails, falls back to DEFAULT_* constants (UTC).
  - I2S audio mixer (background task) plays:
        Tick()  -> Sidereal second (GMST)  [default 550 Hz]
        Tock()  -> Solar second  (UTC)     [default 440 Hz]
    Overlaps are mixed (summed) in mono; separated L/R in stereo.
  - Stubs left for GPS and DS3231 RTC.

  Uses Bodmer/TFT_eSPI: select the T-Display setup (e.g. Setup25_TTGO_T_Display.h).
*/

#include <WiFi.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <time.h>
#include "driver/i2s.h"
#include <math.h>

// ---------------------- WIFI CREDENTIALS ----------------------
// (Fill these in)
const char* WIFI_SSID = "NETWORK";
const char* WIFI_PASS = "PASSWORD";

// ---------------------- NTP & FALLBACK ----------------------
const char* NTP_SERVER = "pool.ntp.org";
const long   NTP_GMT_OFFSET_SEC = 0;   // UTC
const int    NTP_DAYLIGHT_OFFSET = 0;  // no DST in UTC
const uint32_t NTP_WAIT_MS = 10000;    // give NTP up to ~10s

// Fallback default time if NTP fails (UTC)
const int DEFAULT_YEAR   = 2025;
const int DEFAULT_MONTH  = 8;    // 1..12
const int DEFAULT_DAY    = 1;    // 1..31
const int DEFAULT_HOUR   = 0;    // 0..23
const int DEFAULT_MINUTE = 0;    // 0..59
const int DEFAULT_SECOND = 0;    // 0..59

// Optional: longitude for Local Sidereal Time later
const double OBS_LONGITUDE_DEG = 0.0;   // GMST for now

// Optional GPS UART (skeleton)
#define GPS_UART       Serial1
const int PIN_GPS_TX = 26;   // ESP32 TX -> GPS RX
const int PIN_GPS_RX = 27;   // ESP32 RX <- GPS TX
const uint32_t GPS_BAUD = 9600;

// I2C pins for DS3231/AT24C32 (typical ESP32 defaults)
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// TFT colors (565)
#define COL_BG    TFT_BLACK
#define COL_LABEL TFT_CYAN
#define COL_TIME  TFT_BLUE     // UTC
#define COL_SID   TFT_GREEN    // GMST

// Some T-Display boards want the backlight driven HIGH
#ifndef TFT_BL
  #define TFT_BL 4
#endif
#ifndef TFT_BACKLIGHT_ON
  #define TFT_BACKLIGHT_ON HIGH
#endif

// ---------------------- I2S AUDIO CONFIG ----------------------
// Enable/disable audio quickly (no hardware needed to leave this ON)
#define ENABLE_I2S 1

// I2S pins (avoid TFT pins 18/19/23; avoid I2C 21/22; avoid GPS 26/27)
const int I2S_BCLK = 32;   // BCK
const int I2S_LRCK = 25;   // LRCK/WS
const int I2S_DOUT = 33;   // Data out

// Sample rate & DMA
const int I2S_SAMPLE_RATE   = 48000;  // 44_100 also fine
const int I2S_DMA_BUF_COUNT = 6;      // number of DMA buffers
const int I2S_DMA_BUF_LEN   = 256;    // frames per buffer (stereo frames)

// Stereo mode: 0 = mono (identical L/R, mixed overlap); 1 = stereo (Tick->L, Tock->R)
int   AUDIO_STEREO    = 0;

// Master/output level (0.0..1.0)
float AUDIO_MASTER_VOL = 0.60f;

// Event-specific levels (0.0..1.0)
// ***Mapping per request: Tick = Sidereal; Tock = Solar (UTC)***
float TICK_VOL = 0.80f;   // Sidereal (Tick)
float TOCK_VOL = 0.80f;   // Solar   (Tock)

// Frequencies (Hz) and durations (ms)
int   TICK_FREQ_HZ = 550; // Sidereal (Tick) = higher
int   TOCK_FREQ_HZ = 440; // Solar (Tock)    = lower
int   TICK_MS      = 20;
int   TOCK_MS      = 20;

// Fade at start/end (ms) to guarantee near-zero crossings & avoid clicks
const int I2S_FADE_MS = 2;

// Small headroom to reduce clipping when mixing
const float MIX_HEADROOM = 0.9f;

// ---------------------- GLOBALS ----------------------

TFT_eSPI tft = TFT_eSPI();

// Monotonic UTC epoch tracking (float seconds) based on NTP + millis()
double   utcEpochBase  = 0.0;       // epoch seconds at baseline
uint32_t utcMillisBase = 0;         // millis() at baseline

// Last displayed integer seconds for each clock
long lastUtcIntSec = -1;
long lastSidIntSec = -1;

// Tracking: did we get NTP?
bool ntpSynced = false;

// ---------------------- UTIL: Time helpers ----------------------

time_t makeEpochUTC(int yr, int mon, int day, int hh, int mm, int ss) {
  struct tm t = {};
  t.tm_year = yr - 1900;
  t.tm_mon  = mon - 1;
  t.tm_mday = day;
  t.tm_hour = hh;
  t.tm_min  = mm;
  t.tm_sec  = ss;
  char *oldTZ = getenv("TZ");
  setenv("TZ", "UTC0", 1);
  tzset();
  time_t epoch = mktime(&t);
  if (oldTZ) setenv("TZ", oldTZ, 1); else unsetenv("TZ");
  tzset();
  return epoch;
}

double currentUtcEpoch() {
  uint32_t nowMs = millis();
  double delta = (nowMs - utcMillisBase) / 1000.0;
  return utcEpochBase + delta;
}

void formatHMS(long secOfDay, char* out) {
  if (secOfDay < 0) secOfDay = (secOfDay % 86400 + 86400) % 86400;
  int hh = (secOfDay / 3600) % 24;
  int mm = (secOfDay % 3600) / 60;
  int ss = secOfDay % 60;
  sprintf(out, "%02d:%02d:%02d", hh, mm, ss);
}

// ---------------------- Sidereal Time ----------------------

double gmstHoursFromUtcEpoch(double utcEpochSeconds) {
  const double SEC_PER_DAY = 86400.0;
  double JD  = utcEpochSeconds / SEC_PER_DAY + 2440587.5;  // Unix->JD
  double JDa = JD - 0.5;
  double JD0 = floor(JDa) + 0.5;                           // 0h UT for this date
  double H   = (JD - JD0) * 24.0;
  double D   = JD  - 2451545.0;
  double D0  = JD0 - 2451545.0;
  double T   = D / 36525.0;

  double GMST = 6.697374558
              + 0.06570982441908 * D0
              + 1.00273790935    * H
              + 0.000026         * T * T;

  GMST = fmod(GMST, 24.0);
  if (GMST < 0) GMST += 24.0;
  return GMST;
}

long gmstSecondsOfDay(double gmstHours) {
  long sec = (long)floor(gmstHours * 3600.0 + 0.5 /* round to nearest */);
  sec %= 86400;
  if (sec < 0) sec += 86400;
  return sec;
}

// ---------------------- Display ----------------------

void drawStaticUI() {
  tft.fillScreen(COL_BG);
  tft.setSwapBytes(true);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(3);

  tft.setTextColor(COL_LABEL, COL_BG);
  tft.setFreeFont(NULL);

  tft.setCursor(2, 5);
  tft.println(" UTC:");

  tft.setCursor(2, 70);
  tft.setTextColor(COL_LABEL, COL_BG);
  tft.println("GMST:");
}

void printBigTime(int x, int y, uint16_t color, const char* hhmmss) {
  tft.setTextColor(color, COL_BG);
  tft.setTextSize(3);
  tft.setCursor(x, y);
  tft.println(hhmmss);
  tft.setTextSize(1);
}

void updateUtcDisplay(const char* hhmmss) {
  tft.fillRect(80, 33, 232, 32, COL_BG);
  printBigTime(80, 33, COL_TIME, hhmmss);
}

void updateSiderealDisplay(const char* hhmmss) {
  tft.fillRect(80, 98, 232, 32, COL_BG);
  printBigTime(80, 98, COL_SID, hhmmss);
}

// ---------------------- Audio: background mixer ----------------------

static inline int16_t clamp_i16(float x) {
  if (x > 32767.0f) return 32767;
  if (x < -32768.0f) return -32768;
  return (int16_t)x;
}

struct ToneState {
  volatile bool  active;
  float  freq;
  float  phase;
  int    totalSamples;
  int    samplesElapsed;
  int    fadeSamples;
  float  gainL;   // per-event linear gain (0..1), not including master
  float  gainR;
};

static ToneState s_tick = {};  // Sidereal (Tick)
static ToneState s_tock = {};  // Solar    (Tock)

static bool g_i2sReady = false;
static bool g_audioTaskStarted = false;

void startTickTone(float freqHz, int durMs, float volL, float volR) {
#if ENABLE_I2S
  if (!g_i2sReady) return;
  if (AUDIO_MASTER_VOL <= 0.0f) return;
  if (freqHz <= 0.0f || durMs <= 0) return;

  const int sr = I2S_SAMPLE_RATE;
  float durSec = durMs / 1000.0f;
  long  cycles = lroundf(freqHz * durSec);
  if (cycles < 1) cycles = 1;
  int   totalS = (int)lroundf((cycles * (float)sr) / freqHz);
  int   fadeS  = (int)lroundf((I2S_FADE_MS / 1000.0f) * sr);
  if (fadeS * 2 > totalS) fadeS = totalS / 2;

  s_tick.freq           = freqHz;
  s_tick.phase          = 0.0f;
  s_tick.totalSamples   = totalS;
  s_tick.samplesElapsed = 0;
  s_tick.fadeSamples    = fadeS;
  s_tick.gainL          = volL;
  s_tick.gainR          = volR;
  s_tick.active         = true;
#endif
}

void startTockTone(float freqHz, int durMs, float volL, float volR) {
#if ENABLE_I2S
  if (!g_i2sReady) return;
  if (AUDIO_MASTER_VOL <= 0.0f) return;
  if (freqHz <= 0.0f || durMs <= 0) return;

  const int sr = I2S_SAMPLE_RATE;
  float durSec = durMs / 1000.0f;
  long  cycles = lroundf(freqHz * durSec);
  if (cycles < 1) cycles = 1;
  int   totalS = (int)lroundf((cycles * (float)sr) / freqHz);
  int   fadeS  = (int)lroundf((I2S_FADE_MS / 1000.0f) * sr);
  if (fadeS * 2 > totalS) fadeS = totalS / 2;

  s_tock.freq           = freqHz;
  s_tock.phase          = 0.0f;
  s_tock.totalSamples   = totalS;
  s_tock.samplesElapsed = 0;
  s_tock.fadeSamples    = fadeS;
  s_tock.gainL          = volL;
  s_tock.gainR          = volR;
  s_tock.active         = true;
#endif
}

void audioTask(void* pv) {
#if ENABLE_I2S
  const int sr = I2S_SAMPLE_RATE;
  const float twoPiOverSR = 2.0f * (float)M_PI / (float)sr;
  const int FRAMES = I2S_DMA_BUF_LEN;
  static int16_t buf[FRAMES * 2];  // interleaved L,R

  while (true) {
    for (int i = 0; i < FRAMES; ++i) {
      float l = 0.0f, r = 0.0f;

      // Mix Tick (sidereal)
      if (s_tick.active) {
        int k = s_tick.samplesElapsed;
        float env = 1.0f;
        if (k < s_tick.fadeSamples) {
          env = 0.5f * (1.0f - cosf((float)M_PI * (float)k / (float)s_tick.fadeSamples));
        } else if (k >= (s_tick.totalSamples - s_tick.fadeSamples)) {
          int idx = s_tick.totalSamples - 1 - k;
          env = 0.5f * (1.0f - cosf((float)M_PI * (float)idx / (float)s_tick.fadeSamples));
        }
        float s = sinf(s_tick.phase) * env;
        s_tick.phase += twoPiOverSR * s_tick.freq;
        if (s_tick.phase > 2.0f * (float)M_PI) s_tick.phase -= 2.0f * (float)M_PI;

        l += s * s_tick.gainL;
        r += s * s_tick.gainR;

        if (++s_tick.samplesElapsed >= s_tick.totalSamples) {
          s_tick.active = false;
        }
      }

      // Mix Tock (solar)
      if (s_tock.active) {
        int k = s_tock.samplesElapsed;
        float env = 1.0f;
        if (k < s_tock.fadeSamples) {
          env = 0.5f * (1.0f - cosf((float)M_PI * (float)k / (float)s_tock.fadeSamples));
        } else if (k >= (s_tock.totalSamples - s_tock.fadeSamples)) {
          int idx = s_tock.totalSamples - 1 - k;
          env = 0.5f * (1.0f - cosf((float)M_PI * (float)idx / (float)s_tock.fadeSamples));
        }
        float s = sinf(s_tock.phase) * env;
        s_tock.phase += twoPiOverSR * s_tock.freq;
        if (s_tock.phase > 2.0f * (float)M_PI) s_tock.phase -= 2.0f * (float)M_PI;

        l += s * s_tock.gainL;
        r += s * s_tock.gainR;

        if (++s_tock.samplesElapsed >= s_tock.totalSamples) {
          s_tock.active = false;
        }
      }

      // Apply master volume & headroom, convert to i16
      float A = 32767.0f * AUDIO_MASTER_VOL * MIX_HEADROOM;
      buf[2*i + 0] = clamp_i16(l * A);
      buf[2*i + 1] = clamp_i16(r * A);
    }

    size_t bytesToWrite = FRAMES * 2 * sizeof(int16_t);
    size_t written = 0;
    i2s_write(I2S_NUM_0, (const char*)buf, bytesToWrite, &written, portMAX_DELAY);
  }
#else
  vTaskDelete(NULL);
#endif
}

bool initI2S() {
#if ENABLE_I2S
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = I2S_DMA_BUF_COUNT,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pins = {
    .bck_io_num   = I2S_BCLK,
    .ws_io_num    = I2S_LRCK,
    .data_out_num = I2S_DOUT,
    .data_in_num  = I2S_PIN_NO_CHANGE
  };

  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr) != ESP_OK) {
    Serial.println("I2S: driver install failed");
    g_i2sReady = false;
    return false;
  }
  if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) {
    Serial.println("I2S: set pin failed");
    g_i2sReady = false;
    return false;
  }
  i2s_set_clk(I2S_NUM_0, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  i2s_zero_dma_buffer(I2S_NUM_0);
  g_i2sReady = true;

  if (!g_audioTaskStarted) {
    xTaskCreatePinnedToCore(audioTask, "audioTask", 4096, nullptr, 1, nullptr, 0);
    g_audioTaskStarted = true;
  }

  Serial.println("I2S: ready");
  return true;
#else
  g_i2sReady = false;
  return false;
#endif
}

// Public event calls (mapping per request):
//   Tick() = Sidereal second (GMST)
//   Tock() = Solar second (UTC)
void Tick() {
#if ENABLE_I2S
  if (!g_i2sReady) return;
  if (AUDIO_STEREO == 1) {
    // Sidereal on LEFT
    startTickTone((float)TICK_FREQ_HZ, TICK_MS, TICK_VOL, 0.0f);
  } else {
    // Mono: both channels; mixer sums overlaps
    startTickTone((float)TICK_FREQ_HZ, TICK_MS, TICK_VOL, TICK_VOL);
  }
#endif
}

void Tock() {
#if ENABLE_I2S
  if (!g_i2sReady) return;
  if (AUDIO_STEREO == 1) {
    // Solar on RIGHT
    startTockTone((float)TOCK_FREQ_HZ, TOCK_MS, 0.0f, TOCK_VOL);
  } else {
    startTockTone((float)TOCK_FREQ_HZ, TOCK_MS, TOCK_VOL, TOCK_VOL);
  }
#endif
}

// ---------------------- GPS time (skeleton) ----------------------

bool GPSTime(time_t &outEpochUtc) {
  // TODO: Read NMEA from GPS_UART, parse UTC date/time.
  // GPS_UART.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  // Set outEpochUtc and return true when valid.
  return false;
}

// ---------------------- DS3231 RTC (skeleton) ----------------------

bool loadTime(time_t &outEpochUtc) {
  // TODO: Read DS3231 via I2C; convert to epoch; return true if successful.
  return false;
}

bool saveTime(time_t epochUtc) {
  // TODO: Write DS3231 via I2C.
  return false;
}

// ---------------------- WiFi + NTP ----------------------

bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(200);
  }
  return WiFi.status() == WL_CONNECTED;
}

bool syncNTP() {
  configTime(NTP_GMT_OFFSET_SEC, NTP_DAYLIGHT_OFFSET, NTP_SERVER);

  uint32_t start = millis();
  time_t now = 0;
  do {
    delay(200);
    time(&now);
  } while (now < 100000 && millis() - start < NTP_WAIT_MS);

  if (now >= 100000) {
    utcEpochBase  = (double)now;
    utcMillisBase = millis();
    ntpSynced = true;
    return true;
  }
  return false;
}

void setFallbackUtcBaseline() {
  time_t epoch;
  if (GPSTime(epoch)) {
    utcEpochBase = (double)epoch;
  } else if (loadTime(epoch)) {
    utcEpochBase = (double)epoch;
  } else {
    epoch = makeEpochUTC(DEFAULT_YEAR, DEFAULT_MONTH, DEFAULT_DAY,
                         DEFAULT_HOUR, DEFAULT_MINUTE, DEFAULT_SECOND);
    utcEpochBase = (double)epoch;
  }
  utcMillisBase = millis();
  ntpSynced = false;
}

// ---------------------- Arduino setup/loop ----------------------

void setup() {
  Serial.begin(115200);
  delay(100);

  // I2C (for future RTC use)
  Wire.begin(I2C_SDA, I2C_SCL);

  // TFT init
  tft.init();
  tft.setRotation(1); // landscape
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);
  tft.fillScreen(COL_BG);
  drawStaticUI();

  // WiFi + NTP
  bool wifiOK = connectWiFi();
  if (wifiOK) {
    if (!syncNTP()) {
      Serial.println("NTP failed; using fallback default/RTC/GPS time.");
      setFallbackUtcBaseline();
    } else {
      Serial.println("NTP sync OK.");
    }
  } else {
    Serial.println("WiFi connect failed; using fallback default/RTC/GPS time.");
    setFallbackUtcBaseline();
  }

  // Audio
  initI2S();  // safe if no I2S DAC attached

  // Force initial draw
  lastUtcIntSec = -1;
  lastSidIntSec = -1;
}

void loop() {
  // Current UTC epoch with sub-second precision based on our baseline
  double utcNow = currentUtcEpoch();

  // Solar (UTC) seconds of day
  long utcSecOfDay = ((long)floor(fmod(utcNow, 86400.0) + 86400.0)) % 86400;

  // Sidereal: compute GMST hours from utcNow, then seconds of day
  double gmstHours = gmstHoursFromUtcEpoch(utcNow);
  long sidSecOfDay = gmstSecondsOfDay(gmstHours);

  // If integer UTC second changed, update display & play TOCK (Solar)
  if (utcSecOfDay != lastUtcIntSec) {
    char buf[16];
    formatHMS(utcSecOfDay, buf);
    updateUtcDisplay(buf);
    Tock();  // Solar = Tock
    lastUtcIntSec = utcSecOfDay;
  }

  // If integer sidereal second changed, update display & play TICK (Sidereal)
  if (sidSecOfDay != lastSidIntSec) {
    char buf[16];
    formatHMS(sidSecOfDay, buf);
    updateSiderealDisplay(buf);
    Tick();  // Sidereal = Tick
    lastSidIntSec = sidSecOfDay;
  }

  delay(10);
}
