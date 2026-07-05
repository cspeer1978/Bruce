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

// ---- target: S3-Nano STA MAC + shared channel ----
static uint8_t RX_MAC[6] = { 0x3C, 0x0F, 0x02, 0xD4, 0xAF, 0xDC };
static const uint8_t  CH        = 1;
static const uint8_t  MSG       = 0xE5;   // must match receiver
static const int      CHUNK     = 240;    // <= 250 - 4 header bytes
static const uint32_t PERIOD_MS = 150;    // ~6-7 fps; plenty for static UI
static const size_t   BIN_MAX   = 8192;

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

static void mirrorTask(void*) {
  static uint8_t bin[BIN_MAX];
  static uint8_t lastGood[BIN_MAX];
  static size_t  lastGoodSz = 0;
  static uint8_t pkt[4 + CHUNK];
  uint8_t   seq = 0;
  uint32_t  beat = 0;
  uint32_t  lastSendMs = 0;
  const uint32_t IDLE_RESEND_MS = 1000;   // idle keepalive cadence

  for (;;) {
    if (!s_run) break;

    size_t sz = 0;
    tft.getBinLog(bin, sz);

    uint8_t* src = nullptr;
    size_t   srcSz = 0;
    uint32_t now = millis();

    if (sz > 8 && sz <= BIN_MAX) {          // fresh content -> send immediately
      memcpy(lastGood, bin, sz);
      lastGoodSz = sz;
      src = lastGood; srcSz = sz;
    } else if (lastGoodSz > 0 && (now - lastSendMs) >= IDLE_RESEND_MS) {
      src = lastGood; srcSz = lastGoodSz;   // idle: resend last good ~1/sec only
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
    }

    if (++beat % 13 == 0) s_lastSz = sz;

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
  p.channel = CH;
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
