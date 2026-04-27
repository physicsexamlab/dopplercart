/*
  doppler-cart-n800.ino  (v6 — MAX9814 / N=800 / 約55ms周期)
  ESP32 (Arduino Nano ESP32) Goertzel 周波数センサ — 現行運用版

  【概要】
    マイク信号を analogRead で N=800 サンプル取得し、
    Goertzel 法で 960–1040 Hz の振幅スペクトルを評価して周波数を BLE で送信する。
    1 ループ ≈ 55 ms。NimBLE-Arduino / Hanning 窓 / 放物線補間 / 実測 fs。

  【ハードウェア】
    MAX9814 マイクアンプモジュール
      VDD→3.3V / GND→GND / OUT→A0
      GAIN ピン設定（ボード変種により挙動が異なるため要実機確認）:
        - Adafruit/SparkFun 系の MAX9814 ボード: open=60dB(最大)/GND=50dB/VDD=40dB
        - MAX9814 生チップ datasheet 準拠       : open=50dB/GND=60dB(最大)/VDD=40dB
      静止時 amplitude が想定（≈880）と大きく外れたら配線/ボード変種を再確認。

  【品質ゲート】
    pp ≥ MIN_PP かつ amplitude ≥ AMPLITUDE_MIN の両方を満たす窓のみ周波数を出力。
    満たさない場合は freq_hz = -1 として送信（HTML 側で除外可）。

  【amplitude の意味】
    amplitude = goertzel_peak / (N/2)
    Hanning 窓の coherent gain = 0.5 が分母に残るため、
    真の ADC ピーク振幅 A_peak ≈ amplitude × 2 に相当する。
    AMPLITUDE_MIN=10 は ADC ピーク約 20 counts に相当し、ノイズフロア（≈2〜28）の上限付近。

  【タイムスタンプ】
    t_esp_s = サンプリング窓の中点 micros 値（秒）。
    HTML 側は BLE 受信時刻から「窓中点→受信」遅延
    （窓半分 27.5 + Goertzel 2 + BLE ≈ 10 → 約 37 ms）を差し引いて
    位置補間用の時刻に変換する。

  【BLE】
    Service:        Environmental Sensing (0x181A)
    Characteristic: Analog               (0x2A58), Notify
    送信フォーマット: "t_esp_s,freq_hz,amplitude"

  Copyright (c) 2026 一般社団法人 国際物理オリンピック2023記念協会
  Licensed under CC BY-NC 4.0
*/

#include <NimBLEDevice.h>
#include <math.h>

// ─── ユーザ設定 ───────────────────────────────────────
#define MIC_PIN    A0       // アナログ入力（Arduino Nano ESP32）
#define N          800      // サンプル数（≈55ms @ ~14.5kHz）
#define MIN_PP          20      // 品質閾値: peak-to-peak [ADC counts]
#define AMPLITUDE_MIN  10.0f    // Goertzel amplitude 閾値（A_peak/2 換算で約20counts相当）
                                 //   ノイズフロア(MAX9814最大ゲイン): ≈2〜28
                                 //   音源あり(BTスピーカー)         : ≈880
                                 //   amp<100 の窓は反射・S/N不足で外れ値になりやすい
#define DEBUG_SERIAL 0           // 1 にするとシリアル出力を有効化（BLE と併用可）
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
  Serial.println("=== doppler-cart n800 (NimBLE + analogRead) ===");

  analogReadResolution(12);

  // Hanning 窓を事前計算
  for (int i = 0; i < N; i++) {
    hann[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (N - 1)));
  }

  NimBLEDevice::init("Doppler Freq Sensor");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // TX power +9 dBm（最大）
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

  // amplitude = goertzel_peak / (N/2) = A_peak / 2 （Hanning coherent gain 0.5 を含む）
  const float amplitude = mag[bestB] / (N / 2.0f);

  // ── 5. 周波数推定（pp と amplitude が両方閾値以上のときのみ）──
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
    // bestB が境界 (0 or N_BINS-1) の場合は freq_hz = -1 のまま（推定不能）
  }

  // ── 6. 送信 ──────────────────────────────────────
  // タイムスタンプ = サンプリング窓中点 (t0+t1)/2 の micros 値（秒換算）
  // HTML 側で BLE 受信時刻から「窓中点→受信」遅延（≈37ms）を差し引く
  const float t_esp_s = (t0 + (t1 - t0) / 2) * 1e-6f;
  char msg[64];
  snprintf(msg, sizeof(msg), "%.4f,%.2f,%.2f", t_esp_s, freq_hz, amplitude);

  if (connected) {
    pChar->setValue(msg);
    pChar->notify();
  }
#if DEBUG_SERIAL
  Serial.println(msg);
#endif
}
