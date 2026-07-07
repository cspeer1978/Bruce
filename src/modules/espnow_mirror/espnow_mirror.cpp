// espnow_mirror.cpp  --  standalone screen-mirror sender for Bruce
//
// Periodically grabs the current screen as Bruce's draw-command binlog
// (tft.getBinLog) and streams it to an ESP32-S3-Nano over ESP-NOW.
//
// Integration:
//   1) Put espnow_mirror.h / .cpp under src/modules/espnow_mirror/
//   2) #include "modules/espnow_mirror/espnow_mirror.h" in main.cpp
//   3) call espnowMirrorBegin(); once in setup() AFTER the tft is up.
//
// Notes / caveats:
//   - Forces Wi-Fi to STA + channel 1. If Bruce is using Wi-Fi for something
//     else at the same time, channel handling will fight; this is meant for
//     an otherwise-idle-radio mirror. Refine later if needed.
//   - getBinLog is read from a task while the UI writes the log from other
//     contexts; a transient race can garble one frame, but we resend full
//     frames continuously so it self-heals. Good enough for first light.

#include "espnow_mirror.h"
#include <globals.h>          // for `tft` (tft_logger)
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---- broadcast: any single active receiver on channel 1 picks up the stream ----
// (Only ever run ONE receiver at a time, per the project design.)
static uint8_t RX_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static const uint8_t  CH        = 1;
static const uint8_t  MSG       = 0xE5;   // must match receiver
static const int      CHUNK     = 240;    // <= 250 - 4 header bytes
static const uint32_t PERIOD_MS = 60;     // ~16 fps poll; snappy navigation
static const size_t   BIN_MAX   = 9216;

static TaskHandle_t s_task = nullptr;
static bool         s_run  = false;
static volatile uint32_t s_okCount  = 0;
static volatile uint32_t s_failCount = 0;
static volatile uint32_t s_lastSz    = 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onSent(const wifi_tx_info_t*, esp_now_send_status_t st) {
#else
static void onSent(const uint8_t*, esp_now_send_status_t st) {
#endif
  if (st == ESP_NOW_SEND_SUCCESS) s_okCount++; else s_failCount++;
}

// Re-assert ESP-NOW after a Wi-Fi state change. Bruce's "Turn Off WiFi"
// deinitializes ESP-NOW entirely (sends fail with "esp now not init") and can
// leave the radio on channel 0. So fully rebuild: revive the STA + channel,
// then esp_now_init + re-register + re-add peer. Safe to call repeatedly.
static void reassertPeer() {
  uint8_t primaryCh = 0; wifi_second_chan_t sc;
  esp_wifi_get_channel(&primaryCh, &sc);

  // Make sure the radio is alive on a valid channel.
  if (primaryCh == 0 || primaryCh > 13) {
    WiFi.mode(WIFI_STA);
    esp_wifi_start();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
  }

  // Rebuild ESP-NOW from scratch (deinit is a no-op / harmless if not inited).
  esp_now_deinit();
  if (esp_now_init() != ESP_OK) return;
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, RX_MAC, 6);
  p.channel = 0;                 // home channel -> no mismatch
  p.encrypt = false;
  p.ifidx   = WIFI_IF_STA;
  esp_now_add_peer(&p);
}

static void mirrorTask(void*) {
  static uint8_t bin[BIN_MAX];
  static uint8_t lastGood[BIN_MAX];
  static size_t  lastGoodSz = 0;
  static uint8_t pkt[4 + CHUNK];
  uint8_t   seq = 0;
  uint32_t  beat = 0;
  uint32_t  lastSendMs = 0;
  size_t    lastSentSz = 0;
  uint32_t  lastOk = 0, lastFail = 0, lastReassertMs = 0;
  const uint32_t IDLE_RESEND_MS = 1000;   // true-idle keepalive cadence

  for (;;) {
    if (!s_run) break;

    size_t sz = 0;
    tft.getBinLog(bin, sz);

    uint8_t* src = nullptr;
    size_t   srcSz = 0;
    uint32_t now = millis();

    if (sz > 8 && sz <= BIN_MAX) {
      // We have real content. Cache it as last-good, and send it right away if
      // it differs from what we last sent (navigation) OR enough time passed.
      bool changed = (sz != lastSentSz) || (memcmp(bin, lastGood, sz) != 0);
      memcpy(lastGood, bin, sz);
      lastGoodSz = sz;
      if (changed || (now - lastSendMs) >= IDLE_RESEND_MS) {
        src = lastGood; srcSz = sz;
      }
    } else if (lastGoodSz > 0 && (now - lastSendMs) >= IDLE_RESEND_MS) {
      // Log momentarily empty (e.g. just after fillScreen) or truly idle:
      // resend last good so the receiver holds the correct screen.
      src = lastGood; srcSz = lastGoodSz;
    }

    if (src && srcSz > 0) {
      uint8_t frags = (uint8_t)((srcSz + CHUNK - 1) / CHUNK);
      if (frags == 0) frags = 1;
      for (uint8_t f = 0; f < frags; f++) {
        size_t off = (size_t)f * CHUNK;
        size_t clen = (srcSz - off) < CHUNK ? (srcSz - off) : CHUNK;
        pkt[0] = MSG; pkt[1] = seq; pkt[2] = f; pkt[3] = frags;
        memcpy(pkt + 4, src + off, clen);
        esp_now_send(RX_MAC, pkt, 4 + clen);
        vTaskDelay(pdMS_TO_TICKS(3));
      }
      seq++;
      lastSendMs = now;
      lastSentSz = srcSz;
    }

    if (++beat % 13 == 0) s_lastSz = sz;

    // Self-heal after Wi-Fi state changes. Bruce's Turn-Off-WiFi deinitializes
    // ESP-NOW and can leave the radio on channel 0. Detect the dead state
    // (bad channel, sends failing, or ok count not advancing) and fully rebuild
    // ESP-NOW until sends succeed again.
    uint32_t nowc = millis();
    if (nowc - lastReassertMs > 1000) {
      uint8_t ch=0; wifi_second_chan_t sc2; esp_wifi_get_channel(&ch,&sc2);
      bool okStalled   = (s_okCount == lastOk);
      bool failGrowing = (s_failCount != lastFail);
      bool weAreSending = (lastSendMs != 0 && nowc - lastSendMs < 3000);
      bool badChannel  = (ch == 0 || ch > 13);
      if (badChannel || (weAreSending && (failGrowing || okStalled))) {
        reassertPeer();
      }
      lastOk = s_okCount; lastFail = s_failCount; lastReassertMs = nowc;
    }

    vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
  }
  s_task = nullptr;
  vTaskDelete(nullptr);
}

void espnowMirrorBegin() {
  if (s_run) return;

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(CH, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) { log_e("espnow_mirror: esp_now_init failed"); return; }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, RX_MAC, 6);
  p.channel = 0;            // 0 = transmit on the CURRENT radio channel, so
                           // ESP-NOW follows the radio when Bruce joins Wi-Fi
  p.encrypt = false;
  p.ifidx   = WIFI_IF_STA;
  if (esp_now_add_peer(&p) != ESP_OK) { log_e("espnow_mirror: add_peer failed"); return; }

  tft.setLogging(true);                   // start capturing draw ops

  s_run = true;
  xTaskCreatePinnedToCore(mirrorTask, "espnow_mirror", 8192, nullptr, 1, &s_task, 0);
  log_i("espnow_mirror: started");
}

void espnowMirrorEnd() {
  s_run = false;
  tft.setLogging(false);
}
