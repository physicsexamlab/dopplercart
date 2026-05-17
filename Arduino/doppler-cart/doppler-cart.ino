/*
  doppler-cart.ino  ── Stage 2a (drop-aware 撤回版, 2026-05-15)
  ESP32 (Arduino Nano ESP32) I/Q Goertzel 位相 Doppler センサ — 2 kHz 版

  【Stage 0(analogRead 版)からの変更点】
    ・ADC取得を analogRead × 800 から adc_continuous_* (DMA) + リングバッファに変更
    ・サンプリング周波数 FS = 16000 Hz をハードウェアで厳密に保証
    ・キャリア位相補正に dt_us(実測) ではなく DT_BLOCK = 50ms 定数を使用
    ・Goertzel に渡す fs を実測値ではなく 16000.0 定数化
    ・t_esp_s をブロック番号から決定論的に計算
    ・dt_us は診断値(BLE notify実測間隔)として送信、位相補正には未使用
    ・起動時の DMA 損失を防ぐため max_store_buf_size を増量
    ・シリアルから 'd' でブロックダンプ機能を追加

  【Stage 2a-r1 撤回 (2026-05-15)】
    Stage 2a-r1 (drop-aware) で導入した「drop_delta>0 → theta_valid=false +
    freq_hz=-1」のブロック無効化は、HTML 側の tracking_break を誘発して 2-5
    frame 連続 null クラスタを作り、高 v 域の高レバレッジ点を喪失させ Doppler
    slope を圧縮していた (5/15 観測, slope 4.10 vs 5/13 5.50)。drop 中も
    freq_hz を出力し、HTML 側 PLL unwrap + outlier ガード (FREQ_GAP_GATE_HZ=8)
    で処理する元の方針 (pre-r1 Stage 2a) に戻した。

  【デバイス名の一意化 (2026-05-17)】
    BLE デバイス名を固定 "Doppler IQ Sensor" から、Bluetooth MAC 末尾 2
    バイトを付加した "Doppler IQ Sensor XXXX" に変更。同一教室内で複数台
    を運用しても Web Bluetooth の選択ダイアログで各台を区別できるように
    する。XXXX は基板固有(efuse 由来)で再起動しても不変。起動時に
    シリアルへ名前を出力するので、各台に同じ4文字のラベルを貼って運用
    する。HTML 側は service UUID フィルタのみで名前に依存しないため変更
    不要。
    なお NimBLE 2.5.0 は init() の GAP 名を広告へ自動挿入しない。
    さらに macOS CoreBluetooth は名前を scan response のみに置いても
    旧 GAP 名のキャッシュを優先し、Mac 再起動でも消えない。よって
    Complete Local Name は広告本体(primary PDU)へ明示的に載せ、
    16bit Service UUID を scan response へ退避する構成にした
    (setAdvertisementData/setScanResponseData を明示使用)。
    加えて NimBLE on ESP32-S3 は既定で非解決ランダムアドレスを使う。
    旧ファームでそのランダムアドレスに macOS が旧名を紐づけて永続
    キャッシュすると、名前を直しても再起動でも表示が更新されない。
    setOwnAddrType(BLE_OWN_ADDR_PUBLIC) でチップ固有の公開 MAC を
    広告アドレスにし、OS にキャッシュの無い「新規デバイス」として
    名前を取得し直させる。起動時シリアルに addr= も出力して検証可能。

  【BLEパケット形式 (20バイト, little-endian)】
    offset  size  type     field
    0       4     float32  t_esp_s    (窓中点 [秒], 決定論的計算)
    4       4     float32  freq_hz    (-1.0 = 無効, 振幅不足のみ)
    8       4     float32  amplitude
    12      4     float32  theta_raw  (常に有効)
    16      4     uint32   dt_us      (前回 notify からの実測経過 µs, 診断用)

  【BLE設定 (Stage 0 と完全互換)】
    Service:        Environmental Sensing (0x181A)
    Characteristic: Analog               (0x2A58), Notify
    Device name:    "Doppler IQ Sensor XXXX"  (XXXX = BT MAC 末尾2バイト)

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
  #include "esp_mac.h"
}

// ─── ファームウェア識別 ───────────────────────────────
// 起動時シリアルに一意の刻印を出力し、書き込んだ版が正しいことを
// 目視確認できるようにする。FW_REV は変更ごとに手で更新。
// __DATE__/__TIME__ は再コンパイルの度に必ず変わるので、
// 「いまコンパイルした版が動いているか」を確実に判定できる。
#define FW_REV               "r6-pubaddr-namefix"
#define FW_BUILD             __DATE__ " " __TIME__

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
  Serial.println("####################################################");
  Serial.printf ("### FW_REV=%s\n", FW_REV);
  Serial.printf ("### BUILD =%s\n", FW_BUILD);
  Serial.println("####################################################");

  // Hann 窓
  for (int i = 0; i < N; i++) {
    hann[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (N - 1)));
  }

  // キャリア位相補正定数 (起動時に1回計算)
  DTHETA_CARRIER = fmodf(F0 * DT_BLOCK, 1.0f) * 2.0f * M_PI;
  if (DTHETA_CARRIER >  M_PI) DTHETA_CARRIER -= 2.0f * M_PI;
  Serial.printf("DT_BLOCK=%.6f s  DTHETA_CARRIER=%.6f rad\n",
                DT_BLOCK, DTHETA_CARRIER);

  // BLE 初期化 — デバイス名に BT MAC 末尾2バイトを付加し教室内で一意化
  uint8_t bleMac[6];
  esp_read_mac(bleMac, ESP_MAC_BT);
  char devName[28];
  snprintf(devName, sizeof(devName), "Doppler IQ Sensor %02X%02X",
           bleMac[4], bleMac[5]);
  NimBLEDevice::init(devName);
  // 公開アドレス(チップ固有の BT MAC)で広告させる。NimBLE on ESP32-S3
  // は既定で非解決ランダムアドレスを生成するため、旧ファーム時代に
  // macOS がそのランダムアドレスへ紐づけて保存した旧名キャッシュが
  // 再起動でも消えず、Chrome 選択ダイアログに接尾辞が出ない原因になる。
  // 公開アドレスは基板固有かつ macOS が未キャッシュなので、OS は名前を
  // 取得し直し、教室運用でもアドレスが安定一意になる。
  NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
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
  // Complete Local Name を「広告本体(primary PDU)」に載せる。
  // scan response だけに名前を置くと macOS CoreBluetooth は旧 GAP 名の
  // キャッシュを優先表示し、Chrome の選択ダイアログに接尾辞が出ない。
  // primary PDU に Local Name があれば各 central はそれを名前に採用する。
  //   flags(3) + Complete Name "Doppler IQ Sensor XXXX"(2+22=24) = 27 / 31
  // 16bit Service UUID(4B) は scan response 側へ退避（HTML の service
  // フィルタは scan response でも一致するため動作に影響なし）。
  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.setName(devName);
  NimBLEAdvertisementData scanData;
  scanData.addServiceUUID(SERVICE_UUID);
  pAdv->enableScanResponse(true);          // scannable 化（m_advDataSet をリセット）
  pAdv->setScanResponseData(scanData);
  pAdv->setAdvertisementData(advData);     // 最後に呼ぶ（m_advDataSet=true を確定）
  pAdv->start();
  Serial.printf("BLE started. name=\"%s\"  addr=%s  f0=%.0f Hz  packet=%d bytes\n",
                devName, NimBLEDevice::getAddress().toString().c_str(),
                F0, PACKET_TOTAL_BYTES);

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

  // ── 8. I/Q 位相差分で周波数推定 (DT_BLOCK 固定) ──
  // 注: Stage 2a-r1 で導入した DMA ドロップ検出 (drop_delta>0 → theta_valid=false)
  //     は 2026-05-15 に撤回した。BLE バーストや CPU 負荷で発生する稀な drop に
  //     対して、HTML 側 tracking_break (polyBuffer flush + amp≥ENTRY 再エントリー)
  //     を誘発し、高 v 域フレームを 2-5 連続で失って Doppler slope を圧縮していた。
  //     drop 中も freq_hz は出力し、HTML 側の PLL unwrap・amp ゲート・outlier
  //     ガード (FREQ_GAP_GATE_HZ=8) で処理する方が高レバレッジ点を保持できる。
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
  // packet の 16-19 バイト目は dt_us (uint32 µs)。drop_delta の上位 16bit 埋め込みは
  // Stage 2a-r1 撤回に伴い廃止。dt_us は 50ms 公称 (≈ 50000 µs, 0xC350)。
  const uint32_t now_us   = micros();
  const uint32_t dt_us_val = now_us - prev_notify_us;
  prev_notify_us = now_us;
  const uint32_t diag_word = dt_us_val;

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
  Serial.printf("blk=%lu t=%.4f freq=%.2f amp=%.4f theta=%.4f dt_us=%lu drop_tot=%lu\n",
                (unsigned long)block_count, t_esp_s, freq_hz, amplitude,
                theta_raw, (unsigned long)dt_us_val,
                (unsigned long)total_dropped);
#endif
}