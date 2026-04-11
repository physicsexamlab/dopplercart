/*
  doppler-cart.ino  (v5 — MAX9814 マイクアンプ対応)
  ESP32 (Arduino Nano ESP32) Goertzel 周波数センサ

  【v5 変更点】
    - MAX9814 マイクアンプモジュール対応
        VDD→3.3V / GND→GND / OUT→A0
        GAIN→VDD(3.3V) で 60dB 固定 / GAIN→open で 40dB
    - AMPLITUDE_MIN を追加: pp≥MIN_PP でも音源なし時に freq が出る問題を解消
      MAX9814 接続後の実測 amplitude を見て値を調整すること
    - Serial 出力を有効化（接続確認・キャリブレーション用）

  【旧コードから継承（動作実績あり）】
    NimBLE-Arduino / analogRead 高速ループ / 実測 fs / Hanning 窓 / 放物線補間

  出力フォーマット (BLE notify / Serial):
    "t_esp_s,freq_hz,amplitude"
    freq_hz < 0 は品質不足 (pp < MIN_PP または amplitude < AMPLITUDE_MIN)
    amplitude = mag_peak / (N/2) ≈ ADC ピーク振幅 [counts]（Hanning 窓係数 0.5 分を含む）

  BLE:
    Service:        Environmental Sensing (0x181A)
    Characteristic: Analog               (0x2A58), Notify

  Copyright (c) 2026 一般社団法人 国際物理オリンピック2023記念協会
  Licensed under CC BY-NC 4.0
*/

#include <NimBLEDevice.h>
#include <math.h>

// ─── ユーザ設定 ───────────────────────────────────────
#define MIC_PIN    A0       // アナログ入力（Arduino Nano ESP32）
#define N          1600     // サンプル数（≈110ms @ ~14.5kHz）
#define MIN_PP          200      // 品質閾値: peak-to-peak [ADC counts]
#define AMPLITUDE_MIN   30.0f    // Goertzel amplitude 閾値 [counts] — MAX9814 接続後に要較正
                                 // 音源 OFF 時の amplitude を確認し、その 5〜10 倍に設定する
#define FREQ_LOW   960.0f
#define FREQ_HIGH  1040.0f
#define FREQ_STEP  1.0f     // Goertzel 評価間隔 [Hz]
// ─────────────────────────────────────────────────────

#define N_BINS  ((int)((FREQ_HIGH - FREQ_LOW) / FREQ_STEP) + 1)  // 81

#define SERVICE_UUID        "181A"
#define CHARACTERISTIC_UUID "2A58"

static NimBLEServer*         pServer = nullptr;
static NimBLECharacteristic* pChar   = nullptr;
static bool wasConnected = false;

// サンプルバッファ（グローバルでスタック節約）
static uint16_t raw[N];
static float    hann[N];
static float    xw[N];
static float    mag[N_BINS];

// ─── Goertzel ────────────────────────────────────────
// xw[] に窓掛け済み信号が入った状態で呼ぶ
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
  Serial.println("=== doppler-cart v4 (NimBLE + analogRead) ===");

  analogReadResolution(12);

  // Hanning 窓を事前計算
  for (int i = 0; i < N; i++) {
    hann[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (N - 1)));
  }

  NimBLEDevice::init("Doppler Freq Sensor");
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
  Serial.printf("N_BINS = %d\n", N_BINS);
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

  // ── 4. Goertzel（常に実行して amplitude を取得）─────
  for (int b = 0; b < N_BINS; b++) {
    mag[b] = goertzel_mag(FREQ_LOW + b * FREQ_STEP, actualFs);
  }

  int bestB = 0;
  for (int b = 1; b < N_BINS; b++) {
    if (mag[b] > mag[bestB]) bestB = b;
  }

  // amplitude = Goertzel ピーク / (N/2)
  // Hanning 窓の coherent gain = 0.5 が含まれるため、
  // 真の ADC ピーク振幅は amplitude * 2 に相当する
  const float amplitude = mag[bestB] / (N / 2.0f);

  // ── 5. 周波数推定（pp と amplitude が両方閾値以上のときのみ）──
  float freq_hz = -1.0f;
  if (pp >= MIN_PP && amplitude >= AMPLITUDE_MIN) {
    float delta = 0.0f;
    if (bestB > 0 && bestB < N_BINS - 1) {
      const float y1  = mag[bestB - 1];
      const float y2  = mag[bestB];
      const float y3  = mag[bestB + 1];
      const float den = y1 - 2.0f * y2 + y3;
      if (den < -1e-10f) {
        delta = 0.5f * (y1 - y3) / den;
        if (delta >  0.5f) delta =  0.5f;
        if (delta < -0.5f) delta = -0.5f;
      }
    }
    freq_hz = FREQ_LOW + bestB * FREQ_STEP + delta * FREQ_STEP;
  }

  // ── 6. 送信 ──────────────────────────────────────
  const float t_esp_s = micros() * 1e-6f;
  char msg[64];
  snprintf(msg, sizeof(msg), "%.4f,%.2f,%.2f", t_esp_s, freq_hz, amplitude);

  if (connected) {
    pChar->setValue(msg);
    pChar->notify();
  }
  Serial.println(msg);
}
