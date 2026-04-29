/*
  doppler-cart-n800-spectrum-2k.ino
  ESP32 (Arduino Nano ESP32) Goertzel 周波数センサ — フルスペクトル送信版 (2 kHz)
  doppler-cart-n800-spectrum.ino をベースに、音源を 2 kHz に変更したバリアント。
  Goertzel バンドを 1960–2040 Hz に変更。それ以外は同一。

  【概要】
    マイク信号を analogRead で N=800 サンプル取得し、
    Goertzel 法で 1960–2040 Hz の振幅スペクトルを評価して周波数を推定。
    推定 freq/amplitude に加え、81 点全部の Goertzel 振幅を 1 通知で送信。
    1 ループ ≈ 55 ms。NimBLE-Arduino / Hanning 窓 / 放物線補間 / 実測 fs。

  【送信パケット形式】(178 バイト, little-endian, 1 notify で完結)
    offset  size  type         field
    ------  ----  -----------  ----------------------------------------
    0       4     float32      t_esp_s (窓中点 micros 秒換算)
    4       4     float32      freq_hz (-1.0 = 無効)
    8       4     float32      amplitude (= mag[bestB] / (N/2))
    12      4     float32      mag_scale (mag_q[] の 1 LSB が表す mag 値)
    16      162   uint16[81]   mag_q[b] = round(mag[b] / mag_scale)

    HTML 側で mag[b] = mag_q[b] * mag_scale により復元。
    mag_scale は毎フレーム max(mag) / 65535 にして
    uint16 のダイナミックレンジを最大限に使う。

  【ハードウェア / 品質ゲート / 振幅の意味 / タイムスタンプ】
    n800 と同一。amplitude = mag[bestB] / (N/2)。

  【BLE】
    Service:        Environmental Sensing (0x181A)
    Characteristic: Analog               (0x2A58), Notify
    Device name:    "Doppler Spec Sensor 2k"
    MTU:            247 を要求（payload 244 B 上限。178 B パケットが収まる）

  Copyright (c) 2026 一般社団法人 国際物理オリンピック2023記念協会
  Licensed under CC BY-NC 4.0
*/

#include <NimBLEDevice.h>
#include <math.h>
#include <string.h>

// ─── ユーザ設定 ───────────────────────────────────────
#define MIC_PIN    A0
#define N          800
#define MIN_PP          20
#define AMPLITUDE_MIN  10.0f
#define DEBUG_SERIAL 0
#define FREQ_LOW   1960.0f
#define FREQ_HIGH  2040.0f
#define FREQ_STEP  1.0f
// ─────────────────────────────────────────────────────

#define N_BINS  ((int)((FREQ_HIGH - FREQ_LOW) / FREQ_STEP) + 1)  // 81

#define SERVICE_UUID        "181A"
#define CHARACTERISTIC_UUID "2A58"

#define PACKET_HEADER_BYTES  16
#define PACKET_TOTAL_BYTES   (PACKET_HEADER_BYTES + N_BINS * 2)  // 178

static NimBLEServer*         pServer = nullptr;
static NimBLECharacteristic* pChar   = nullptr;
static bool wasConnected = false;

static uint16_t raw[N];
static float    hann[N];
static float    xw[N];
static float    mag[N_BINS];
static uint8_t  packet[PACKET_TOTAL_BYTES];

// ─── Goertzel ────────────────────────────────────────
static float goertzel_mag(float freq, float fs) {
  const float omega = 2.0f * M_PI * freq / fs;
  const float coeff = 2.0f * cosf(omega);
  float s1 = 0.0f, s2 = 0.0f;
  for (int i = 0; i < N; i++) {
    float s0 = xw[i] + coeff * s1 - s2;
    s2 = s1; s1 = s0;
  }
  return sqrtf(fmaxf(s1 * s1 + s2 * s2 - coeff * s1 * s2, 0.0f));
}

// ─── setup ───────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("=== doppler-cart n800 spectrum 2k (NimBLE binary) ===");

  analogReadResolution(12);

  for (int i = 0; i < N; i++) {
    hann[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (N - 1)));
  }

  NimBLEDevice::init("Doppler Spec Sensor 2k");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(247);  // Web Bluetooth 側の MTU 上限交渉に備える
  pServer = NimBLEDevice::createServer();
  NimBLEService* pService = pServer->createService(SERVICE_UUID);
  pChar = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  pService->start();
  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->start();

  Serial.printf("BLE started.  Goertzel: %.0f–%.0f Hz / %.1f Hz step / N=%d\n",
                FREQ_LOW, FREQ_HIGH, FREQ_STEP, N);
  Serial.printf("N_BINS = %d  packet = %d bytes\n", N_BINS, PACKET_TOTAL_BYTES);
}

// ─── loop ────────────────────────────────────────────
void loop() {
  const bool connected = (pServer->getConnectedCount() > 0);
  if (!connected && wasConnected) NimBLEDevice::startAdvertising();
  wasConnected = connected;

  // ── 1. サンプリング（最高速ループ＋実測 fs）────────
  const unsigned long t0 = micros();
  for (int i = 0; i < N; i++) raw[i] = analogRead(MIC_PIN);
  const unsigned long t1 = micros();
  const float actualFs = (float)N * 1e6f / (float)(t1 - t0);

  // ── 2. 統計（平均・pp）────────────────────────────
  long long sumV = 0;
  uint16_t vMin = raw[0], vMax = raw[0];
  for (int i = 0; i < N; i++) {
    sumV += raw[i];
    if (raw[i] < vMin) vMin = raw[i];
    if (raw[i] > vMax) vMax = raw[i];
  }
  const float    meanV = (float)sumV / N;
  const uint16_t pp    = vMax - vMin;

  // ── 3. 平均除去 + Hanning 窓 ─────────────────────
  for (int i = 0; i < N; i++) {
    xw[i] = ((float)raw[i] - meanV) * hann[i];
  }

  // ── 4. Goertzel ─────────────────────────────────
  for (int b = 0; b < N_BINS; b++) {
    mag[b] = goertzel_mag(FREQ_LOW + b * FREQ_STEP, actualFs);
  }

  int bestB = 0;
  for (int b = 1; b < N_BINS; b++) {
    if (mag[b] > mag[bestB]) bestB = b;
  }
  const float amplitude = mag[bestB] / (N / 2.0f);

  // ── 5. 周波数推定 ───────────────────────────────
  float freq_hz = -1.0f;
  if (pp >= MIN_PP && amplitude >= AMPLITUDE_MIN) {
    if (bestB > 0 && bestB < N_BINS - 1) {
      float delta = 0.0f;
      const float y1  = mag[bestB - 1];
      const float y2  = mag[bestB];
      const float y3  = mag[bestB + 1];
      const float den = y1 - 2.0f * y2 + y3;
      if (den < -1e-10f) {
        delta = 0.5f * (y1 - y3) / den;
        if (delta >  0.5f) delta =  0.5f;
        if (delta < -0.5f) delta = -0.5f;
      }
      freq_hz = FREQ_LOW + bestB * FREQ_STEP + delta * FREQ_STEP;
    }
  }

  // ── 6. パケット組み立て（バイナリ）──────────────
  const float t_esp_s = (t0 + (t1 - t0) / 2) * 1e-6f;

  // mag_scale = max(mag) / 65535（uint16 のレンジを最大限活用）
  // freq_hz が無効でもスペクトル形状を保持して送る（外れ値解析の主目的）
  float mag_max = mag[0];
  for (int b = 1; b < N_BINS; b++) {
    if (mag[b] > mag_max) mag_max = mag[b];
  }
  const float mag_scale = (mag_max > 0.0f) ? (mag_max / 65535.0f) : 1.0f;

  // ESP32 はリトルエンディアンなので memcpy で OK
  memcpy(&packet[0],  &t_esp_s,   4);
  memcpy(&packet[4],  &freq_hz,   4);
  memcpy(&packet[8],  &amplitude, 4);
  memcpy(&packet[12], &mag_scale, 4);

  for (int b = 0; b < N_BINS; b++) {
    float v = mag[b] / mag_scale + 0.5f;
    if (v < 0.0f)        v = 0.0f;
    if (v > 65535.0f)    v = 65535.0f;
    const uint16_t q = (uint16_t)v;
    packet[PACKET_HEADER_BYTES + b * 2]     = (uint8_t)(q & 0xFF);
    packet[PACKET_HEADER_BYTES + b * 2 + 1] = (uint8_t)((q >> 8) & 0xFF);
  }

  if (connected) {
    pChar->setValue(packet, PACKET_TOTAL_BYTES);
    pChar->notify();
  }
#if DEBUG_SERIAL
  Serial.printf("t=%.4f freq=%.2f amp=%.2f mag_scale=%.4f bestB=%d\n",
                t_esp_s, freq_hz, amplitude, mag_scale, bestB);
#endif
}
