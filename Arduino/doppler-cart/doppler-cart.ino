/*
  doppler-cart-iq.ino  ── Stage 2a (Stage 2a-r1: drop-aware)
  ESP32 (Arduino Nano ESP32) I/Q Goertzel 位相 Doppler センサ — 2 kHz 版

  【Stage 0(現行)からの変更点】
    ・ADC取得を analogRead × 800 から adc_continuous_* (DMA) + リングバッファに変更
    ・サンプリング周波数 FS = 16000 Hz をハードウェアで厳密に保証
    ・キャリア位相補正に dt_us(実測) ではなく DT_BLOCK = 50ms 定数を使用
    ・Goertzel に渡す fs を実測値ではなく 16000.0 定数化
    ・t_esp_s をブロック番号から決定論的に計算
    ・dt_us は診断値(BLE notify実測間隔)として送信、位相補正には未使用
    ・起動時の DMA 損失を防ぐため max_store_buf_size を増量
    ・シリアルから 'd' でブロックダンプ機能を追加

  【Stage 2a-r1 (2026-05-11) ドロップ耐性】
    ・total_dropped / total_unexpected を毎ブロックで前回値と比較
    ・ドロップ発生時(=サンプル列に時間ギャップ)は theta_valid=false + freq_hz=-1
      ── theta_prev と次の theta_raw が 50ms 離れていない位相差分の破綻を予防
    ・dt_us フィールドの上位 16bit にブロック内累積 drop_delta を埋め込み、
      HTML 側で motion 中のドロップ有無を可視化可能にする (下位 16bit は dt_us & 0xFFFF)

  【BLEパケット形式 (20バイト, little-endian)】
    offset  size  type     field
    0       4     float32  t_esp_s    (窓中点 [秒], 決定論的計算)
    4       4     float32  freq_hz    (-1.0 = 無効, ドロップ or 振幅不足)
    8       4     float32  amplitude
    12      4     float32  theta_raw  (常に有効, ドロップ後でも値を持つ)
    16      4     uint32   diag       (下位16bit = dt_us mod 65536,
                                       上位16bit = drop_delta このブロック)

  【BLE設定 (Stage 0 と完全互換)】
    Service:        Environmental Sensing (0x181A)
    Characteristic: Analog               (0x2A58), Notify
    Device name:    "Doppler IQ Sensor"

  【動作環境】
    - Arduino IDE
    - Board package: esp32 by Espressif Systems 3.3.8
    - Board: Arduino Nano ESP32 (esp32 グループ側)
    - Library: NimBLE-Arduino 2.5.0

  Copyright (c) 2026 一般社団法人 国際物理オリンピック2023記念協会
  Licensed under CC BY-NC 4.0
*/

#include <NimBLEDevice.h>
#include <math.h>
#include <string.h>

extern "C" {
  #include "esp_adc/adc_continuous.h"
}

// ─── ユーザ設定 ───────────────────────────────────────
#define MIC_PIN              A0
#define N                    800
#define MIN_PP               20
#define AMPLITUDE_MIN        10.0f
#define DEBUG_SERIAL         0
#define F0                   2000.0f       // キャリア周波数 [Hz]
constexpr uint32_t FS_HZ        = 16000;   // サンプリング周波数 [Hz] (DMA厳密)
constexpr float    FS_F         = 16000.0f;
constexpr float    DT_BLOCK     = (float)N / FS_F;        // = 0.050 秒 (定数)
constexpr uint32_t DT_BLOCK_US  = 50000;                  // = 50000 µs (定数)
// ─────────────────────────────────────────────────────

// ─── ADC continuous 設定 ──────────────────────────────
constexpr adc_channel_t  ADC_CHAN     = ADC_CHANNEL_0;    // GPIO1 = A0 = ADC1_CH0
constexpr adc_unit_t     ADC_UNIT_X   = ADC_UNIT_1;
constexpr uint32_t       CONV_FRAME_BYTES = 256;          // 64 samples/frame
constexpr uint32_t       MAX_STORE_BYTES  = 16384;        // 4096 samples (256ms分の余裕)

// ─── リングバッファ ──────────────────────────────────
constexpr size_t   FIFO_SIZE  = 4096;
constexpr size_t   FIFO_MASK  = FIFO_SIZE - 1;
static_assert((FIFO_SIZE & FIFO_MASK) == 0, "FIFO_SIZE must be power of 2");

// ─── BLE 設定 ────────────────────────────────────────
#define SERVICE_UUID         "181A"
#define CHARACTERISTIC_UUID  "2A58"
#define PACKET_TOTAL_BYTES   20

static NimBLEServer*         pServer = nullptr;
static NimBLECharacteristic* pChar   = nullptr;
static bool wasConnected = false;

// ─── ADC ハンドル & FIFO 状態 ─────────────────────────
static adc_continuous_handle_t adc_handle = nullptr;
static volatile uint16_t fifo[FIFO_SIZE];
static volatile size_t   fifo_head = 0;
static          size_t   fifo_tail = 0;
static volatile uint32_t total_dropped   = 0;
static volatile uint32_t total_unexpected = 0;
static uint8_t           raw_buf[CONV_FRAME_BYTES];

// ─── 信号処理 状態 ───────────────────────────────────
static uint16_t raw[N];                // ブロック取り出し用
static float    hann[N];
static float    xw[N];
static uint8_t  packet[PACKET_TOTAL_BYTES];

static float    theta_prev      = 0.0f;
static bool     theta_valid     = false;
static uint32_t block_count     = 0;
static uint32_t start_us        = 0;   // 起動時刻基準 (t_esp_s 計算用)
static uint32_t prev_notify_us  = 0;   // 前回 notify 時刻 (dt_us 計算用)
static uint32_t prev_dropped_total    = 0;  // 前ブロック時点での total_dropped (drop_delta 計算用)
static uint32_t prev_unexpected_total = 0;  // 同 total_unexpected

// 起動時に1回だけ計算するキャリア位相補正定数
static float    DTHETA_CARRIER  = 0.0f;

// ─── DUMP 機能 ───────────────────────────────────────
static bool     dump_request    = false;

// ─── Goertzel 複素出力 (FS 固定版) ───────────────────
static void goertzel_complex(float *re, float *im) {
  const float omega = 2.0f * M_PI * F0 / FS_F;   // 起動時に最適化されて定数畳み込み
  const float cosw  = cosf(omega);
  const float sinw  = sinf(omega);
  const float coeff = 2.0f * cosw;
  float s1 = 0.0f, s2 = 0.0f;
  for (int i = 0; i < N; i++) {
    float s0 = xw[i] + coeff * s1 - s2;
    s2 = s1; s1 = s0;
  }
  *re = s1 - s2 * cosw;
  *im = s2 * sinw;
}

// ─── ADC 連続サンプリング初期化 ───────────────────────
static bool setup_adc_continuous() {
  adc_continuous_handle_cfg_t handle_cfg = {};
  handle_cfg.max_store_buf_size = MAX_STORE_BYTES;
  handle_cfg.conv_frame_size    = CONV_FRAME_BYTES;
  handle_cfg.flags.flush_pool   = 0;

  esp_err_t err = adc_continuous_new_handle(&handle_cfg, &adc_handle);
  if (err != ESP_OK) {
    Serial.printf("ERROR: adc_continuous_new_handle (0x%x)\n", err);
    return false;
  }

  adc_digi_pattern_config_t pattern[1] = {};
  pattern[0].atten     = ADC_ATTEN_DB_11;
  pattern[0].channel   = ADC_CHAN;
  pattern[0].unit      = ADC_UNIT_X;
  pattern[0].bit_width = ADC_BITWIDTH_12;

  adc_continuous_config_t adc_config = {};
  adc_config.pattern_num    = 1;
  adc_config.adc_pattern    = pattern;
  adc_config.sample_freq_hz = FS_HZ;
  adc_config.conv_mode      = ADC_CONV_SINGLE_UNIT_1;
  adc_config.format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2;

  err = adc_continuous_config(adc_handle, &adc_config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: adc_continuous_config (0x%x)\n", err);
    return false;
  }
  err = adc_continuous_start(adc_handle);
  if (err != ESP_OK) {
    Serial.printf("ERROR: adc_continuous_start (0x%x)\n", err);
    return false;
  }
  return true;
}

// ─── ADC → FIFO ドレイン ─────────────────────────────
static esp_err_t drain_adc_to_fifo() {
  uint32_t bytes_read = 0;
  esp_err_t err = adc_continuous_read(adc_handle, raw_buf, sizeof(raw_buf),
                                      &bytes_read, 0);
  if (err != ESP_OK) return err;

  for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= bytes_read;
       i += SOC_ADC_DIGI_RESULT_BYTES) {
    adc_digi_output_data_t* p = (adc_digi_output_data_t*)&raw_buf[i];
    if (p->type2.channel != ADC_CHAN || p->type2.unit != ADC_UNIT_X) {
      total_unexpected++;
      continue;
    }
    size_t head = fifo_head;
    size_t next = (head + 1) & FIFO_MASK;
    if (next == fifo_tail) {
      total_dropped++;
    } else {
      fifo[head] = (uint16_t)p->type2.data;
      fifo_head = next;
    }
  }
  return ESP_OK;
}

// ─── FIFO から N サンプル取り出し ─────────────────────
static bool try_pop_block() {
  size_t avail = (fifo_head - fifo_tail) & FIFO_MASK;
  if (avail < (size_t)N) return false;
  for (int i = 0; i < N; i++) {
    raw[i] = fifo[fifo_tail];
    fifo_tail = (fifo_tail + 1) & FIFO_MASK;
  }
  return true;
}

// ─── setup ───────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("=== doppler-cart Stage 2a (DMA + NimBLE 2.5.0) ===");

  // Hann 窓
  for (int i = 0; i < N; i++) {
    hann[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (N - 1)));
  }

  // キャリア位相補正定数 (起動時に1回計算)
  DTHETA_CARRIER = fmodf(F0 * DT_BLOCK, 1.0f) * 2.0f * M_PI;
  if (DTHETA_CARRIER >  M_PI) DTHETA_CARRIER -= 2.0f * M_PI;
  Serial.printf("DT_BLOCK=%.6f s  DTHETA_CARRIER=%.6f rad\n",
                DT_BLOCK, DTHETA_CARRIER);

  // BLE 初期化
  NimBLEDevice::init("Doppler IQ Sensor");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(247);
  pServer = NimBLEDevice::createServer();
  NimBLEService* pService = pServer->createService(SERVICE_UUID);
  pChar = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  pService->start();
  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->enableScanResponse(true);
  pAdv->start();
  Serial.printf("BLE started. f0=%.0f Hz  packet=%d bytes\n", F0, PACKET_TOTAL_BYTES);

  // ADC 連続サンプリング開始
  if (!setup_adc_continuous()) {
    Serial.println("ADC setup failed. Halting.");
    while (1) delay(1000);
  }
  start_us       = micros();
  prev_notify_us = start_us;
  Serial.println("ADC continuous started.");
  Serial.println("Send 'd' to dump next block (800 raw samples).");
}

// ─── loop ────────────────────────────────────────────
void loop() {
  const bool connected = (pServer->getConnectedCount() > 0);
  if (!connected && wasConnected) NimBLEDevice::startAdvertising();
  wasConnected = connected;

  // ── 1. ADC → FIFO ドレイン (空になるまで) ────────
  while (true) {
    esp_err_t err = drain_adc_to_fifo();
    if (err == ESP_OK) continue;
    if (err == ESP_ERR_TIMEOUT) break;
    break;
  }

  // ── 2. 800 サンプル揃ったらブロック処理 ─────────
  if (!try_pop_block()) return;

  // ── 3. シリアル入力 ('d' でダンプ要求) ───────────
  if (Serial.available()) {
    int c = Serial.read();
    if (c == 'd' || c == 'D') {
      dump_request = true;
    }
  }

  // ── 4. 決定論的タイムスタンプ ────────────────────
  // 窓中点の絶対時刻 (起動時刻基準)
  const uint32_t t_mid_us = start_us
                          + (uint32_t)((double)block_count * DT_BLOCK_US)
                          + (DT_BLOCK_US / 2);
  const float t_esp_s = t_mid_us * 1e-6f;

  // ── 5. 統計 (平均・pp) ───────────────────────────
  long long sumV = 0;
  uint16_t vMin = raw[0], vMax = raw[0];
  for (int i = 0; i < N; i++) {
    sumV += raw[i];
    if (raw[i] < vMin) vMin = raw[i];
    if (raw[i] > vMax) vMax = raw[i];
  }
  const float    meanV = (float)sumV / N;
  const uint16_t pp    = vMax - vMin;

  // ── 6. 平均除去 + Hann 窓 ────────────────────────
  for (int i = 0; i < N; i++) {
    xw[i] = ((float)raw[i] - meanV) * hann[i];
  }

  // ── 7. f0 ビン単体 複素 Goertzel (FS = 16000 固定) ─
  float re0, im0;
  goertzel_complex(&re0, &im0);

  const float amplitude = sqrtf(re0 * re0 + im0 * im0) / (N / 2.0f);
  const float theta_raw = atan2f(im0, re0);

  // ── 8a. DMA ドロップ検出 ─────────────────────────
  // 前ブロック処理時点からの drop_delta を計算。drop が発生していれば、
  // theta_prev (前ブロック中心) と theta_raw (本ブロック中心) は
  // 実時間で 50ms ちょうど離れておらず、位相差分式が破綻する。
  // → theta_valid=false とし、本ブロックでは freq_hz=-1 を返す
  //   (theta_raw 自体は記録のため packet に含めて送る)
  const uint32_t cur_dropped_total    = total_dropped;
  const uint32_t cur_unexpected_total = total_unexpected;
  const uint32_t drop_delta = (cur_dropped_total    - prev_dropped_total)
                            + (cur_unexpected_total - prev_unexpected_total);
  prev_dropped_total    = cur_dropped_total;
  prev_unexpected_total = cur_unexpected_total;
  if (drop_delta > 0) {
    theta_valid = false;
  }

  // ── 8b. I/Q 位相差分で周波数推定 (DT_BLOCK 固定) ──
  float    freq_hz   = -1.0f;

  if (pp >= MIN_PP && amplitude >= AMPLITUDE_MIN) {
    if (theta_valid) {
      float dtheta = theta_raw - theta_prev;
      if (dtheta >  M_PI) dtheta -= 2.0f * M_PI;
      if (dtheta < -M_PI) dtheta += 2.0f * M_PI;

      // キャリア位相補正 (定数, 起動時に計算済み)
      float dtheta_doppler = dtheta - DTHETA_CARRIER;
      if (dtheta_doppler >  M_PI) dtheta_doppler -= 2.0f * M_PI;
      if (dtheta_doppler < -M_PI) dtheta_doppler += 2.0f * M_PI;
      freq_hz = F0 + dtheta_doppler / (2.0f * M_PI * DT_BLOCK);
    }
    theta_prev  = theta_raw;
    theta_valid = true;
  } else {
    theta_valid = false;
  }

  // ── 9. dt_us = 前回 notify からの実測経過時間 ────
  // packet の 16-19 バイト目に下位16bit=dt_us, 上位16bit=drop_delta を埋め込む
  // (dt_us は <= 約 65 ms の範囲なので 16 bit に収まる, drop_delta が
  //  64K を超える病的ケースは飽和して送る)
  const uint32_t now_us   = micros();
  const uint32_t dt_us_val = now_us - prev_notify_us;
  prev_notify_us = now_us;
  const uint16_t dt_us_low = (uint16_t)(dt_us_val > 0xFFFF ? 0xFFFF : dt_us_val);
  const uint16_t drop_hi   = (uint16_t)(drop_delta > 0xFFFF ? 0xFFFF : drop_delta);
  const uint32_t diag_word = ((uint32_t)drop_hi << 16) | (uint32_t)dt_us_low;

  // ── 10. パケット組み立て & BLE notify ────────────
  memcpy(&packet[0],  &t_esp_s,    4);
  memcpy(&packet[4],  &freq_hz,    4);
  memcpy(&packet[8],  &amplitude,  4);
  memcpy(&packet[12], &theta_raw,  4);
  memcpy(&packet[16], &diag_word,  4);

  if (connected) {
    pChar->setValue(packet, PACKET_TOTAL_BYTES);
    pChar->notify();
  }

  // ── 11. ブロックカウンタ更新 ──────────────────────
  block_count++;

  // ── 12. ダンプ要求対応 (該当ブロック取得済み後) ──
  if (dump_request) {
    dump_request = false;
    Serial.printf("# DUMP block=%lu  mean=%.1f  min=%u  max=%u  pp=%u  amp=%.2f\n",
                  (unsigned long)block_count, meanV, vMin, vMax, pp, amplitude);
    for (int i = 0; i < N; i++) Serial.println(raw[i]);
    Serial.println("# DUMP_END");
  }

#if DEBUG_SERIAL
  Serial.printf("blk=%lu t=%.4f freq=%.2f amp=%.4f theta=%.4f dt_us=%lu drop_d=%lu drop_tot=%lu\n",
                (unsigned long)block_count, t_esp_s, freq_hz, amplitude,
                theta_raw, (unsigned long)dt_us_val,
                (unsigned long)drop_delta, (unsigned long)total_dropped);
#endif
}