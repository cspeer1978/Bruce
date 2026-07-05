// espnow_mirror.h  --  standalone screen-mirror sender for Bruce
// Drops the current screen (tft.getBinLog) to an ESP32-S3-Nano over ESP-NOW.
// Add to Bruce: place under src/modules/espnow_mirror/ and call
// espnowMirrorBegin() once from setup() (after tft is initialized).

#ifndef __ESPNOW_MIRROR_H__
#define __ESPNOW_MIRROR_H__

// Start the mirror: enables tft logging + spawns the sender task.
void espnowMirrorBegin();

// Stop mirroring (optional).
void espnowMirrorEnd();

#endif
