#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Adafruit_NeoPixel.h>
#include "driver/gpio.h"
extern "C"
{
#include "driver/i2s_std.h"
}

/***** ============ USER CONFIG ============ *****/

// I2S pins (SPH0645 SEL -> 3V => RIGHT channel)
#define PIN_I2S_LRCLK GPIO_NUM_5 // Feather D5  → LRCLK/WS
#define PIN_I2S_BCLK GPIO_NUM_6  // Feather D6  → BCLK
#define PIN_I2S_DIN GPIO_NUM_9   // Feather D9  → DOUT

// NeoPixel (onboard)
#define NEOPIXEL_PIN 33
#define NEOPIXEL_NUM 1

// Audio capture config
#define SAMPLE_RATE_HZ 16000 // 16 kHz, speech
// STREAM_MS was used for fixed 5s capture; now unused (continuous until STOP)
// #define STREAM_MS 5000
#define CAPTURE_SAMPLES 512 // samples per I2S read (32-bit containers)

// BLE batching config
#define NOTIFY_SAMPLES 120 // target samples per notify (fits MTU)
#define BLE_PCM_BYTES_PER_NOTIFY (NOTIFY_SAMPLES * sizeof(int16_t))
#define NOTIFY_INTERVAL_MS 15 // ~15 ms between notifies

// Ring buffer capacity: enough for ~6 seconds of audio (safety margin)
// 6 s * 16000 samples/s = 96000 samples
#define RING_CAP_SAMPLES (SAMPLE_RATE_HZ * 6)

// BLE names/UUIDs
static const char *BLE_DEVICE_NAME = "ESP32S3-Audio";
static const char *UUID_AUDIO_SERVICE = "3c3b0001-8c5a-4b78-9c3a-1d5a6db3a001";
static const char *UUID_CONTROL_CHAR = "3c3b0002-8c5a-4b78-9c3a-1d5a6db3a001"; // Write
static const char *UUID_AUDIO_CHAR = "3c3b0003-8c5a-4b78-9c3a-1d5a6db3a001";   // Notify

/***** ============ GLOBALS ============ *****/

// NeoPixel
Adafruit_NeoPixel pixel(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
static inline void setPixel(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = 40)
{
  pixel.setBrightness(brightness);
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

// BLE
static NimBLEServer *g_server = nullptr;
static NimBLEService *g_service = nullptr;
static NimBLECharacteristic *g_controlChar = nullptr;
static NimBLECharacteristic *g_audioChar = nullptr;

static volatile bool g_isConnected = false;
static volatile bool g_wantStart = false; // set only from callback
static volatile bool g_wantStop = false;  // NEW: STOP flag set only from callback

static bool g_streaming = false;     // overall session active (ring draining + BLE)
static bool g_captureActive = false; // still pulling from I2S into ring

static uint32_t g_streamStartMs = 0;
static uint32_t g_nextNotifyMs = 0;
static portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;

// I2S (STD driver)
static i2s_chan_handle_t g_i2sRx = nullptr;
static bool g_i2sReady = false;

// Buffers
static int32_t g_i2sBuf32[CAPTURE_SAMPLES];             // 32-bit raw samples
static int16_t g_pcm16Buf[CAPTURE_SAMPLES];             // converted 16-bit samples
static uint8_t g_bleFrag[4 + BLE_PCM_BYTES_PER_NOTIFY]; // BLE packet buffer
static uint16_t g_seq = 0;                              // BLE sequence counter

// Ring buffer for PCM16
static int16_t g_ringBuf[RING_CAP_SAMPLES];
static uint32_t g_ringWriteIdx = 0;
static uint32_t g_ringReadIdx = 0;
static uint32_t g_ringCount = 0;      // samples currently in ring
static uint32_t g_droppedSamples = 0; // if ring overflows

// Debug counters
static uint32_t g_totalSamplesSent = 0; // total int16 samples sent over BLE
static uint32_t g_totalNotifies = 0;    // total BLE notifications sent
static uint32_t g_totalI2SReads = 0;    // total I2S read calls

/***** ============ RING BUFFER HELPERS ============ *****/

static inline void ringClear()
{
  g_ringWriteIdx = 0;
  g_ringReadIdx = 0;
  g_ringCount = 0;
  g_droppedSamples = 0;
}

// Push n samples into ring (dropping if full)
static void ringPushSamples(const int16_t *src, int n)
{
  for (int i = 0; i < n; ++i)
  {
    if (g_ringCount >= RING_CAP_SAMPLES)
    {
      // ring full – drop sample
      g_droppedSamples++;
      continue;
    }
    g_ringBuf[g_ringWriteIdx] = src[i];
    g_ringWriteIdx = (g_ringWriteIdx + 1) % RING_CAP_SAMPLES;
    g_ringCount++;
  }
}

// Pop up to maxSamples into dst, return actual count popped
static int ringPopSamples(int16_t *dst, int maxSamples)
{
  int n = 0;
  while (n < maxSamples && g_ringCount > 0)
  {
    dst[n++] = g_ringBuf[g_ringReadIdx];
    g_ringReadIdx = (g_ringReadIdx + 1) % RING_CAP_SAMPLES;
    g_ringCount--;
  }
  return n;
}

/***** ============ I2S (IDF 5.x STD) ============ *****/

static void i2sInitStd()
{
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 8;
  chan_cfg.dma_frame_num = CAPTURE_SAMPLES;
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &g_i2sRx));

  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ),
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                  I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED,
          .bclk = PIN_I2S_BCLK,
          .ws = PIN_I2S_LRCLK,
          .dout = I2S_GPIO_UNUSED,
          .din = PIN_I2S_DIN,
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv = false}}};
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT; // mic on right channel

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(g_i2sRx, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(g_i2sRx));

  // Warm-up read (non-blocking)
  size_t bytes_read = 0;
  (void)i2s_channel_read(g_i2sRx, g_i2sBuf32, sizeof(g_i2sBuf32), &bytes_read, 10);

  g_i2sReady = true;
  Serial.println("[I2S] STD ready.");
}

static void i2sDeinitStd()
{
  if (g_i2sRx)
  {
    i2s_channel_disable(g_i2sRx);
    i2s_del_channel(g_i2sRx);
    g_i2sRx = nullptr;
  }
  g_i2sReady = false;
}

/***** ============ SAMPLE CONVERSION ============ *****/

static inline void convert32to16(const int32_t *in32, int16_t *out16, int n)
{
  for (int i = 0; i < n; ++i)
  {
    int32_t s = in32[i] >> 14; // downshift to ~18-bit range
    if (s > 32767)
      s = 32767;
    else if (s < -32768)
      s = -32768;
    out16[i] = (int16_t)s;
  }
}

/***** ============ BLE ============ *****/

class ServerCB : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *, ble_gap_conn_desc *) override
  {
    g_isConnected = true;
    setPixel(0, 255, 0);
    Serial.println("[BLE] Central connected.");
  }

  void onDisconnect(NimBLEServer *) override
  {
    g_isConnected = false;

    portENTER_CRITICAL(&g_stateMux);
    g_wantStart = false;
    g_wantStop = false; // clear STOP flag on disconnect
    portEXIT_CRITICAL(&g_stateMux);

    g_streaming = false;
    g_captureActive = false;
    i2sDeinitStd();
    ringClear();
    setPixel(0, 0, 255);
    Serial.println("[BLE] Central disconnected, advertising...");
    NimBLEDevice::startAdvertising();
  }
};

class ControlWriteCB : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *c) override
  {
    std::string val = c->getValue();
    for (char &ch : val)
    {
      ch = toupper((unsigned char)ch);
    }

    if (val.find("START") != std::string::npos)
    {
      portENTER_CRITICAL(&g_stateMux);
      g_wantStart = true;
      portEXIT_CRITICAL(&g_stateMux);

      Serial.println("[DEBUG] [CTRL] START command received via BLE write.");
    }

    // NEW: STOP handling
    if (val.find("STOP") != std::string::npos)
    {
      portENTER_CRITICAL(&g_stateMux);
      g_wantStop = true;
      portEXIT_CRITICAL(&g_stateMux);

      Serial.println("[DEBUG] [CTRL] STOP command received via BLE write.");
    }
  }
};

static void bleInit()
{
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setMTU(247);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(new ServerCB());

  g_service = g_server->createService(UUID_AUDIO_SERVICE);

  g_controlChar = g_service->createCharacteristic(
      UUID_CONTROL_CHAR,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  g_controlChar->setCallbacks(new ControlWriteCB());

  g_audioChar = g_service->createCharacteristic(
      UUID_AUDIO_CHAR,
      NIMBLE_PROPERTY::NOTIFY);

  g_service->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(UUID_AUDIO_SERVICE);
  adv->setScanResponse(true);
  adv->start();

  Serial.println("[DEBUG] BLE initialized, advertising started.");
}

/***** ============ BLE SEND (BATCHED) ============ *****/

static void bleSendNextChunk()
{
  if (!g_isConnected)
    return;
  if (g_audioChar->getSubscribedCount() == 0)
    return;
  if (g_ringCount == 0)
    return;

  // Pop up to NOTIFY_SAMPLES from ring
  int16_t localBuf[NOTIFY_SAMPLES];
  int samplesToSend = ringPopSamples(localBuf, NOTIFY_SAMPLES);
  if (samplesToSend <= 0)
    return;

  uint16_t seq = g_seq++;
  uint16_t nSamp = (uint16_t)samplesToSend;
  uint16_t payloadBytes = nSamp * sizeof(int16_t);

  memcpy(g_bleFrag, &seq, sizeof(seq));
  memcpy(g_bleFrag + 2, &nSamp, sizeof(nSamp));
  memcpy(g_bleFrag + 4, localBuf, payloadBytes);

  g_audioChar->setValue(g_bleFrag, 4 + payloadBytes);
  g_audioChar->notify();

  g_totalSamplesSent += nSamp;
  g_totalNotifies++;

  if ((g_totalNotifies % 100) == 0)
  {
    float secondsApprox = (float)g_totalSamplesSent / (float)SAMPLE_RATE_HZ;
    Serial.printf("[DEBUG] BLE sent %lu notifies, %lu samples (~%.2f s)\n",
                  (unsigned long)g_totalNotifies,
                  (unsigned long)g_totalSamplesSent,
                  secondsApprox);
  }
}

/***** ============ ARDUINO SETUP/LOOP ============ *****/

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }

  pixel.begin();
  setPixel(0, 0, 255); // BLUE: waiting/advertising

  Serial.println("[BOOT] Starting BLE advertising; waiting for connection...");
  bleInit();
}

void loop()
{
  if (!g_isConnected)
  {
    // idle when not connected
    delay(20);
    return;
  }

  // Handle START command → begin new capture+stream session
  if (!g_streaming && g_wantStart)
  {
    portENTER_CRITICAL(&g_stateMux);
    g_wantStart = false;
    g_wantStop = false; // clear any stale STOP
    portEXIT_CRITICAL(&g_stateMux);

    if (!g_i2sReady)
    {
      i2sInitStd();
    }

    // Reset ring + counters
    ringClear();
    g_seq = 0;
    g_totalSamplesSent = 0;
    g_totalNotifies = 0;
    g_totalI2SReads = 0;

    g_streaming = true;     // session active
    g_captureActive = true; // actively pulling from I2S
    g_streamStartMs = millis();
    g_nextNotifyMs = g_streamStartMs; // first notify can go out ASAP

    setPixel(255, 0, 0); // RED: active capture/stream
    Serial.println("[CTRL] START received; beginning continuous capture (until STOP)...");
  }

  if (!g_streaming)
  {
    // connected but idle
    delay(10);
    return;
  }

  uint32_t now = millis();

  /***** 1) CAPTURE: read from I2S into ring while captureActive *****/
  if (g_captureActive)
  {
    // NEW: STOP-based termination instead of time-based
    if (g_wantStop)
    {
      portENTER_CRITICAL(&g_stateMux);
      g_wantStop = false;
      portEXIT_CRITICAL(&g_stateMux);

      g_captureActive = false;
      i2sDeinitStd();

      Serial.println("[I2S] STOP command received; stopping I2S capture.");
      if (g_droppedSamples > 0)
      {
        Serial.printf("[DEBUG] Ring overflow during capture, dropped %lu samples.\n",
                      (unsigned long)g_droppedSamples);
      }
    }
    else
    {
      // Non-blocking-ish read with small timeout (5 ms)
      size_t bytesRead = 0;
      esp_err_t ret = i2s_channel_read(g_i2sRx,
                                       g_i2sBuf32,
                                       sizeof(g_i2sBuf32),
                                       &bytesRead,
                                       5);
      if (ret == ESP_OK && bytesRead > 0)
      {
        g_totalI2SReads++;
        int nSamples = bytesRead / (int)sizeof(int32_t);
        convert32to16(g_i2sBuf32, g_pcm16Buf, nSamples);
        ringPushSamples(g_pcm16Buf, nSamples);

        if ((g_totalI2SReads % 20) == 0)
        {
          Serial.printf("[DEBUG] I2S read #%lu: bytesRead=%u (nSamples=%d), ringCount=%lu\n",
                        (unsigned long)g_totalI2SReads,
                        (unsigned int)bytesRead,
                        nSamples,
                        (unsigned long)g_ringCount);
        }
      }
      // else: timeout or error; we just skip this iteration
    }
  }

  /***** 2) SEND: paced BLE notifications from ring *****/
  if (g_isConnected &&
      g_audioChar->getSubscribedCount() > 0 &&
      g_ringCount > 0 &&
      (int32_t)(now - g_nextNotifyMs) >= 0)
  {
    bleSendNextChunk();
    g_nextNotifyMs = now + NOTIFY_INTERVAL_MS;
  }

  /***** 3) END OF STREAM: when capture done AND ring drained *****/
  if (!g_captureActive && g_ringCount == 0)
  {
    g_streaming = false;
    setPixel(0, 255, 0); // GREEN: finished, idle

    uint32_t endMs = millis();
    uint32_t sessionMs = endMs - g_streamStartMs;
    float secondsApprox = (float)g_totalSamplesSent / (float)SAMPLE_RATE_HZ;

    Serial.println("[STREAM] Finished streaming; back to idle.");
    Serial.printf("[DEBUG] Stream summary:\n");
    Serial.printf("        sessionDurationMs = %lu ms\n", (unsigned long)sessionMs);
    Serial.printf("        totalI2SReads     = %lu\n", (unsigned long)g_totalI2SReads);
    Serial.printf("        totalSamplesSent  = %lu (~%.2f s at %d Hz)\n",
                  (unsigned long)g_totalSamplesSent,
                  secondsApprox,
                  SAMPLE_RATE_HZ);
    Serial.printf("        totalNotifies     = %lu\n", (unsigned long)g_totalNotifies);
    Serial.printf("        ringOverflowDrops = %lu\n", (unsigned long)g_droppedSamples);
  }

  // Small delay to avoid a too-tight loop
  delay(1);
}
