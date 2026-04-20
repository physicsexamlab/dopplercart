/*
  doppler-cart-phase.ino  (v1 — Goertzel I/Q 位相追跡法)
  ESP32 (Arduino Nano ESP32) ・ N=800 (≈55ms)

  【背景】
    音源1000Hz・速度1m/s付近のドップラー実験では、反射波・定在波によって
    マイクへの音圧が急激に変動（フェージング）する。
    これにより振幅変調 → スペクトル広がり → Goertzel振幅ピーク法が不安定化する。

    位相追跡法はこれを根本的に解決する：
      1. Goertzel の I（実部）・Q（虚部）から位相 φ = atan2(Q, I) を取得
      2. 連続ブロック間の位相差 Δφ を計算
      3. 静止時キャリブレーションで求めた Δφ₀（キャリア定常進み）を除去
      4. f = fc + δΔφ · (fs/N) / (2π)  ← 振幅変動に依存しない

  【N=800 を選択した理由（N=1600との比較）】
    ○ 55ms 更新率でカート位置サンプリングと同期しやすい（散布図精度↑）
    ○ ドップラー位相進み 2π×3Hz×0.055s ≈ 1.04rad < π → アンラップ安全
    ○ 短窓の方が深いフェード区間を丸ごと含む確率が低く振幅ゲートが効く
    △ S/N は N=1600 より √2 倍（≈3dB）劣るが、振幅ゲートで補完可能
    N=1600 は位相 S/N で有利だが時間分解能が劣り散布図の位置合わせが悪化する

  【キャリア周波数 fc の重要性】
    ESP32 のクロック誤差により実際の音源周波数は 1000Hz からズレる（例: 998Hz）。
    fc を固定 1000Hz にすると位相に系統的なバイアスが生じる。
    起動時キャリブレーション（スキャン法の中央値）で fc を実測し、
    位相キャリブレーションで Δφ₀ を決定することで、クロック誤差・個体差・
    温度ドリフトを自動吸収する。

  【動作フロー】
    SCANNING  (N_CAL_SCAN 有効ブロック):
      81点スキャン → 周波数中央値 → fc 決定
    CAL_PHASE (N_CAL_PHASE 有効ブロック):
      fc 点の I/Q → 連続ブロック Δφ 中央値 → Δφ₀ 決定
    TRACKING:
      81点スキャン法（比較用）と位相差法を並行実行して両方送信

  【BLE 出力フォーマット（7フィールド）】
    "t_esp_s,freq_scan,freq_phase,amplitude,status,fc,cal_progress"
    t_esp_s    : ESP32 内部時刻 [s]
    freq_scan  : スキャン法（放物線補間）[Hz]。品質不足 → -1
    freq_phase : 位相差法 [Hz]。TRACKING且つ振幅OK → 有効値。それ以外 → -1
    amplitude  : スキャンピーク正規化振幅 [counts]（Hanning coherent gain 0.5 込み）
    status     : 0=SCANNING, 1=CAL_PHASE, 2=TRACKING
    fc         : 現在のキャリア周波数 [Hz]
    cal_progress: SCANNING中=fc蓄積数, CAL_PHASE中=Δφ蓄積数, TRACKING中=0

  【BLE コマンド受信 (Char 0x2A59, WRITE)】
    "RECAL"      → キャリブレーション再開（SCANNING へ）
    "FC:998.3"   → fc を手動設定して CAL_PHASE へ移行

  【BLE】
    Service UUID: 0x181A (Environmental Sensing)
    Char 0x2A58 Notify : データ出力
    Char 0x2A59 Write  : コマンド受信

  Copyright (c) 2026 一般社団法人 国際物理オリンピック2023記念協会
  Licensed under CC BY-NC 4.0
*/

#include <NimBLEDevice.h>
#include <math.h>
#include <string.h>

// ─── ユーザ設定 ───────────────────────────────────────
#define MIC_PIN       A0
#define N             800         // サンプル数（≈55ms @ ~14.5kHz）
#define MIN_PP        200         // 品質閾値: peak-to-peak [ADC counts]
#define AMPLITUDE_MIN 80.0f       // 正規化振幅閾値 [counts]
                                  //   ノイズフロア(MAX9814 60dB): ≈2〜28
                                  //   音源あり: ≈460〜610 → 余裕十分
                                  //   50→80: 50〜79は偽値を出力しうるため除外

#define FREQ_LOW      960.0f
#define FREQ_HIGH     1040.0f
#define FREQ_STEP     1.0f

#define FREQ_CONT_HZ     8.0f    // 連続性閾値: 1フレームで8Hz超変化は偽ピーク棄却
#define FREQ_WATCHDOG_HZ 5.0f    // 位相スリップ番犬: scan vs phase の最大許容差

#define N_CAL_SCAN    50          // fc 決定に必要な有効ブロック数（≈2.75s）
#define N_CAL_PHASE   20          // Δφ₀ 決定に必要な有効ブロック数（≈1.1s）
// ──────────────────────────────────────────────────────

#define N_BINS  ((int)((FREQ_HIGH - FREQ_LOW) / FREQ_STEP) + 1)  // 81

#define SERVICE_UUID  "181A"
#define CHAR_NOTIFY   "2A58"   // Notify: データ出力
#define CHAR_CMD      "2A59"   // Write:  コマンド受信

// ─── BLE ──────────────────────────────────────────────
static NimBLEServer*         pServer  = nullptr;
static NimBLECharacteristic* pNotify  = nullptr;
static NimBLECharacteristic* pCmd     = nullptr;
static bool wasConnected = false;

// ─── バッファ ──────────────────────────────────────────
static uint16_t raw[N];
static float    hann[N];
static float    xw[N];
static float    mag[N_BINS];

// ─── キャリブレーション状態 ────────────────────────────
enum CalState : uint8_t { SCANNING = 0, CAL_PHASE = 1, TRACKING = 2 };
static CalState calState        = SCANNING;
static float    fc              = 1000.0f;  // キャリア周波数（スキャン後更新）
static float    dphi0           = 0.0f;     // 静止時のΔφ（キャリア定常進み）
static float    prev_phi        = 0.0f;
static bool     prev_phi_valid  = false;

static float    fc_cands[N_CAL_SCAN];
static int      fc_cand_count   = 0;

static float    dphi_cands[N_CAL_PHASE];
static int      dphi_cand_count = 0;

// 連続性チェック用（偽ピーク除去）
static float    last_freq_scan  = -1.0f;
static int      scan_miss_count = 0;

// ─── ユーティリティ ────────────────────────────────────

// 位相差を (-π, π] にラップ
static float wrap_pi(float d) {
  while (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
  while (d < -(float)M_PI) d += 2.0f * (float)M_PI;
  return d;
}

// 小配列メディアン（最大 N_CAL_SCAN 要素対応）
static float median_f(const float *arr, int n) {
  static float tmp[N_CAL_SCAN];
  if (n <= 0) return 0.0f;
  const int len = (n <= N_CAL_SCAN) ? n : N_CAL_SCAN;
  memcpy(tmp, arr, len * sizeof(float));
  for (int i = 0; i < len - 1; i++)
    for (int j = 0; j < len - i - 1; j++)
      if (tmp[j] > tmp[j + 1]) {
        float t = tmp[j]; tmp[j] = tmp[j + 1]; tmp[j + 1] = t;
      }
  return (len % 2) ? tmp[len / 2] : 0.5f * (tmp[len / 2 - 1] + tmp[len / 2]);
}

// ─── Goertzel（振幅のみ）─────────────────────────────
// 事前に xw[] へ窓掛け済み信号を入れておくこと
static float goertzel_mag(float freq, float fs) {
  const float omega = 2.0f * (float)M_PI * freq / fs;
  const float coeff = 2.0f * cosf(omega);
  float s1 = 0.0f, s2 = 0.0f;
  for (int i = 0; i < N; i++) {
    float s0 = xw[i] + coeff * s1 - s2;
    s2 = s1; s1 = s0;
  }
  return sqrtf(fmaxf(s1 * s1 + s2 * s2 - coeff * s1 * s2, 0.0f));
}

// ─── Goertzel（I/Q, N/2 正規化済み）─────────────────
// sqrt(I²+Q²) = goertzel_mag(freq,fs) / (N/2) が成立する
// 位相 φ = atan2(Q, I)
static void goertzel_iq(float freq, float fs, float *I_out, float *Q_out) {
  const float omega = 2.0f * (float)M_PI * freq / fs;
  const float coeff = 2.0f * cosf(omega);
  float s1 = 0.0f, s2 = 0.0f;
  for (int i = 0; i < N; i++) {
    float s0 = xw[i] + coeff * s1 - s2;
    s2 = s1; s1 = s0;
  }
  const float hN = N / 2.0f;
  *I_out = (s1 - s2 * cosf(omega)) / hN;
  *Q_out = (s2 * sinf(omega))       / hN;
}

// ─── キャリブレーション初期化 ─────────────────────────
static void start_recal() {
  calState        = SCANNING;
  fc              = 1000.0f;
  dphi0           = 0.0f;
  fc_cand_count   = 0;
  dphi_cand_count = 0;
  prev_phi_valid  = false;
  last_freq_scan  = -1.0f;
  scan_miss_count = 0;
}

// ─── BLE コマンドコールバック ──────────────────────────
class CmdCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pC) override {
    const char *s = pC->getValue().c_str();
    if (strcmp(s, "RECAL") == 0) {
      start_recal();
      Serial.println("[CMD] RECAL → SCANNING");
    } else if (strncmp(s, "FC:", 3) == 0) {
      const float f = atof(s + 3);
      if (f > 800.0f && f < 1200.0f) {
        fc              = f;
        calState        = CAL_PHASE;
        dphi_cand_count = 0;
        prev_phi_valid  = false;
        Serial.printf("[CMD] FC → %.3f Hz\n", fc);
      }
    }
  }
};

// ─── setup ───────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("=== doppler-cart-phase v1 (Goertzel I/Q 位相追跡) ===");
  Serial.printf("N=%d  N_CAL_SCAN=%d  N_CAL_PHASE=%d  N_BINS=%d\n",
                N, N_CAL_SCAN, N_CAL_PHASE, N_BINS);

  analogReadResolution(12);

  // Hanning 窓を事前計算
  for (int i = 0; i < N; i++)
    hann[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (N - 1)));

  // MTU を拡大してから init（46byte パケットに必要）
  NimBLEDevice::init("Doppler Phase Sensor");
  NimBLEDevice::setMTU(512);

  pServer = NimBLEDevice::createServer();

  NimBLEService *pSvc = pServer->createService(SERVICE_UUID);

  pNotify = pSvc->createCharacteristic(
    CHAR_NOTIFY, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  pCmd = pSvc->createCharacteristic(
    CHAR_CMD, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pCmd->setCallbacks(new CmdCallbacks());

  pSvc->start();

  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->start();

  Serial.println("BLE advertising started. Waiting for connection...");
}

// ─── loop ────────────────────────────────────────────
void loop() {
  const bool connected = (pServer->getConnectedCount() > 0);
  if (!connected && wasConnected) NimBLEDevice::startAdvertising();
  wasConnected = connected;

  // ── 1. サンプリング（実測 fs）────────────────────────
  const unsigned long t0 = micros();
  for (int i = 0; i < N; i++) raw[i] = analogRead(MIC_PIN);
  const unsigned long t1 = micros();
  const float fs = (float)N * 1e6f / (float)(t1 - t0);

  // ── 2. 統計（平均・pp）──────────────────────────────
  long long sumV = 0;
  uint16_t  vMin = raw[0], vMax = raw[0];
  for (int i = 0; i < N; i++) {
    sumV += raw[i];
    if (raw[i] < vMin) vMin = raw[i];
    if (raw[i] > vMax) vMax = raw[i];
  }
  const float    meanV = (float)sumV / N;
  const uint16_t pp    = vMax - vMin;

  // ── 3. 平均除去 + Hanning 窓 ─────────────────────────
  for (int i = 0; i < N; i++)
    xw[i] = ((float)raw[i] - meanV) * hann[i];

  // ── 4. 81点スキャン（全ステート共通・比較用）──────────
  for (int b = 0; b < N_BINS; b++)
    mag[b] = goertzel_mag(FREQ_LOW + b * FREQ_STEP, fs);

  int bestB = 0;
  for (int b = 1; b < N_BINS; b++)
    if (mag[b] > mag[bestB]) bestB = b;

  const float amplitude = mag[bestB] / (N / 2.0f);

  float freq_scan = -1.0f;
  if (pp >= MIN_PP && amplitude >= AMPLITUDE_MIN) {
    float delta = 0.0f;
    if (bestB > 0 && bestB < N_BINS - 1) {
      const float y1 = mag[bestB - 1], y2 = mag[bestB], y3 = mag[bestB + 1];
      const float den = y1 - 2.0f * y2 + y3;
      if (den < -1e-10f) {
        delta = 0.5f * (y1 - y3) / den;
        if (delta >  0.5f) delta =  0.5f;
        if (delta < -0.5f) delta = -0.5f;
      }
    }
    freq_scan = FREQ_LOW + bestB * FREQ_STEP + delta * FREQ_STEP;
  }

  // 連続性チェック（1フレームで FREQ_CONT_HZ 超の跳躍は偽ピークとして棄却）
  if (freq_scan > 0.0f) {
    if (last_freq_scan > 0.0f && fabsf(freq_scan - last_freq_scan) > FREQ_CONT_HZ) {
      freq_scan = -1.0f;                    // 棄却
    } else {
      last_freq_scan  = freq_scan;
      scan_miss_count = 0;
    }
  }
  if (freq_scan < 0.0f) {
    if (++scan_miss_count > 3) last_freq_scan = -1.0f;  // 3連続ミスで参照値リセット
  }

  // ── 5. キャリブレーション / 位相追跡 ─────────────────

  // SCANNING: 有効な freq_scan を蓄積し中央値から fc を決定
  if (calState == SCANNING) {
    if (freq_scan > 0.0f && fc_cand_count < N_CAL_SCAN)
      fc_cands[fc_cand_count++] = freq_scan;

    if (fc_cand_count >= N_CAL_SCAN) {
      fc              = median_f(fc_cands, N_CAL_SCAN);
      calState        = CAL_PHASE;
      dphi_cand_count = 0;
      prev_phi_valid  = false;
      Serial.printf("[CAL] fc = %.3f Hz  (ESP32 offset = %+.3f Hz)\n",
                    fc, fc - 1000.0f);
    }
  }

  // CAL_PHASE / TRACKING: fc 点の I/Q を取得して位相を追跡
  float freq_phase = -1.0f;

  if (calState == CAL_PHASE || calState == TRACKING) {
    float I_fc, Q_fc;
    goertzel_iq(fc, fs, &I_fc, &Q_fc);
    const float amp_fc = sqrtf(I_fc * I_fc + Q_fc * Q_fc);

    if (amp_fc >= AMPLITUDE_MIN) {
      const float phi = atan2f(Q_fc, I_fc);

      if (calState == CAL_PHASE) {
        // Δφ = wrap_pi(φ_new - φ_prev) を蓄積
        if (prev_phi_valid && dphi_cand_count < N_CAL_PHASE)
          dphi_cands[dphi_cand_count++] = wrap_pi(phi - prev_phi);

        prev_phi       = phi;
        prev_phi_valid = true;

        if (dphi_cand_count >= N_CAL_PHASE) {
          dphi0    = median_f(dphi_cands, N_CAL_PHASE);
          calState = TRACKING;
          Serial.printf("[CAL] Δφ₀ = %.5f rad  (%.3f Hz)\n",
                        dphi0, dphi0 * (fs / N) / (2.0f * (float)M_PI));
        }

      } else {  // TRACKING
        if (prev_phi_valid) {
          // キャリア定常進みを除去してドップラー成分のみ取り出す
          const float dDphi = wrap_pi(wrap_pi(phi - prev_phi) - dphi0);
          freq_phase = fc + dDphi * (fs / N) / (2.0f * (float)M_PI);
        }
        prev_phi       = phi;
        prev_phi_valid = true;
      }

    } else {
      // フェード（amp < 閾値）→ 次回の位相差を無効化
      prev_phi_valid = false;
    }
  }

  // 位相スリップ番犬（freq_scan を参照して freq_phase を検証）
  // 両者が有効かつ FREQ_WATCHDOG_HZ 超の乖離 → スリップと判定してリセット
  if (freq_phase > 0.0f && freq_scan > 0.0f) {
    if (fabsf(freq_phase - freq_scan) > FREQ_WATCHDOG_HZ) {
      Serial.printf("[WARN] phase slip: scan=%.2f phase=%.2f → reset\n",
                    freq_scan, freq_phase);
      freq_phase     = -1.0f;
      prev_phi_valid = false;
    }
  }

  // ── 6. BLE 送信 ──────────────────────────────────────
  const float t_s    = micros() * 1e-6f;
  const int   stat   = (int)calState;
  const int   cprog  = (calState == SCANNING)  ? fc_cand_count  :
                       (calState == CAL_PHASE)  ? dphi_cand_count : 0;

  char msg[96];
  snprintf(msg, sizeof(msg), "%.4f,%.2f,%.2f,%.2f,%d,%.2f,%d",
           t_s, freq_scan, freq_phase, amplitude, stat, fc, cprog);

  if (connected) {
    pNotify->setValue(msg);
    pNotify->notify();
  }
  Serial.println(msg);
}
