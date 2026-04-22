/*
  doppler-cart-phase.ino  (v3 — 真の PLL: Goertzel を f_est で計算)
  ESP32 (Arduino Nano ESP32) ・ N=800 (≈55ms)

  【背景】
    音源1000Hz・速度1m/s付近のドップラー実験では、反射波・定在波によって
    マイクへの音圧が急激に変動（フェージング）する。
    これにより振幅変調 → スペクトル広がり → Goertzel振幅ピーク法が不安定化する。

  【v1→v2→v3 の改善経緯】
    v1: Goertzel を fc で計算。dphi0 を引いて Doppler 成分を取得。
        → fs 変動と wrap_pi の折り返しが重なり、freq_phase が fc±9Hz に二値化。

    v2: dphi0 廃止。T_elapsed ベースの整数サイクル補完でアンラップ。
        → アルゴリズムは正しいが、「周波数が急変しない」制約を生かしていない。
        → fc で測った位相差は依然として ≈ ±1 rad と大きく、ノイズの影響を受ける。

    v3（本版）: Goertzel を fc ではなく f_est（現在の推定値）で計算。
        → PLL がロックしていれば信号は f_est ≈ f_signal → 位相差 ≈ 0
        → 整数サイクル補完不要（wrap_pi が常に正しい）
        → ノイズ由来の位相ゆらぎが f_est に与える影響が最小化される
        → 「周波数は急激には変化しない」という物理的制約を直接アルゴリズムに組み込む

  【真の PLL の動作原理】
    各フレーム:
      1. Goertzel を f_est（前フレームの推定値）で計算 → I, Q
      2. phi = atan2(Q, I)
      3. raw_dphi = wrap_pi(phi - prev_phi)
            ≈ 2π × (f_signal - f_est) × T_el  [ロック時 ≈ 0]
      4. freq_error = raw_dphi / (2π × T_el)  [Hz, 小さい]
      5. freq_phase = f_est + freq_error       [単フレーム推定値]
      6. f_est ← (1-α)×f_est + α×freq_phase  [IIR = f_est + α×freq_error]

    ロック時の位相差の上界:
      |raw_dphi| ≤ 2π × (max_accel × T_el / (c/fc)) × T_el
                 = 2π × (1m/s² × 0.055s / (343/996)) × 0.055
                 ≈ 0.06 rad → wrap_pi で折り返しの心配なし

  【収束特性】
    時定数 τ = T_el / α = 0.055 / 0.2 ≈ 275ms
    ドップラー加速度 2.9 Hz/s に対するトラッキング遅れ ≈ 0.8 Hz
    最大捕捉範囲: |f_signal - f_est| < 1/(2×T_el) ≈ 9Hz（= v < 3 m/s）

  【動作フロー】
    SCANNING  (N_CAL_SCAN 有効ブロック):
      81点スキャン → 周波数中央値 → fc 決定 → TRACKING へ即移行
    TRACKING:
      Goertzel を f_est で計算。freq_scan = -1、amplitude は IQ 振幅で送信。
      初期フレーム（f_est 未確定）は fc を probe として使用。

  【デバッグ出力】
    #define DEBUG_PHASE を有効にすると TRACKING 中に毎フレーム出力：
      [PHZ] T=0.0551 probe=993.01 raw=0.0041 err=0.012 fp=993.02 amp=812.3
      raw が小さければ PLL がロックしている証拠。

  【BLE 出力フォーマット（8フィールド、v1互換）】
    "t_esp_s,freq_scan,freq_phase,amplitude,status,fc,cal_progress,freq_pll"
    status: 0=SCANNING, 2=TRACKING（CAL_PHASE=1 は廃止）

  【BLE コマンド受信 (Char 0x2A59, WRITE)】
    "RECAL"      → キャリブレーション再開（SCANNING へ）
    "FC:998.3"   → fc を手動設定して TRACKING へ移行

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
#define AMPLITUDE_MIN_SCAN  80.0f  // スキャン法の閾値: 低SNRで偽ピーク誤検出を防ぐ
#define AMPLITUDE_MIN_PHASE 10.0f  // 位相法の閾値
                                   //   ノイズフロア(MAX9814 60dB): ≈2〜28
#define PLL_ALPHA   0.20f           // IIR 更新係数（時定数 τ = T_el/α ≈ 275ms）
#define HOLD_MAX    10              // 連続ホールド上限 ≈ 550ms でロック喪失判定

// DEBUG_PHASE: 有効にすると TRACKING 中に毎フレームシリアルデバッグ出力
#define DEBUG_PHASE

#define FREQ_LOW      900.0f
#define FREQ_HIGH     1040.0f
#define FREQ_STEP     1.0f

#define N_CAL_SCAN    50          // fc 決定に必要な有効ブロック数（≈2.75s）
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
enum CalState : uint8_t { SCANNING = 0, TRACKING = 2 };  // CAL_PHASE(1)廃止
static CalState calState       = SCANNING;
static float    fc             = 1000.0f;  // キャリア周波数（スキャン後更新）
static float    prev_phi       = 0.0f;
static bool     prev_phi_valid = false;

// フレーム開始時刻（T_elapsed 計算用）
static unsigned long prev_t0_us    = 0;
static bool          prev_t0_valid = false;

static float    f_est       = 0.0f;   // PLL 推定周波数 [Hz]（probe に使用）
static bool     f_est_valid = false;  // f_est 初期化済みフラグ
static int      hold_count  = 0;      // 連続ホールドフレーム数
static float    nco_phase   = 0.0f;   // 連続 NCO 位相 [rad]
static bool     nco_valid   = false;

static float    fc_cands[N_CAL_SCAN];
static int      fc_cand_count = 0;


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

// ─── 連続 NCO による IQ 検波（TRACKING専用）────────────
// ブロック境界をまたいで位相基準を連続に保つことで、
// 停止中に PLL が自己ドリフトするのを防ぐ。
static void pll_iq_continuous(float freq, float fs, float *phase_io,
                              float *I_out, float *Q_out) {
  const float omega = 2.0f * (float)M_PI * freq / fs;
  float phase = *phase_io;
  float I = 0.0f, Q = 0.0f;

  for (int i = 0; i < N; i++) {
    I += xw[i] * cosf(phase);
    Q += xw[i] * sinf(phase);
    phase += omega;
    if (phase >  (float)M_PI) phase -= 2.0f * (float)M_PI;
    if (phase < -(float)M_PI) phase += 2.0f * (float)M_PI;
  }

  const float hN = N / 2.0f;
  *I_out = I / hN;
  *Q_out = Q / hN;
  *phase_io = phase;
}

// ─── キャリブレーション初期化 ─────────────────────────
static void start_recal() {
  calState       = SCANNING;
  fc             = 1000.0f;
  fc_cand_count  = 0;
  prev_phi_valid = false;
  prev_t0_valid  = false;
  f_est_valid    = false;
  hold_count     = 0;
  nco_phase      = 0.0f;
  nco_valid      = false;
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
        fc             = f;
        calState       = TRACKING;
        prev_phi_valid = false;
        prev_t0_valid  = false;
        f_est          = f;    // probe を fc で初期化（stale 値を排除）
        f_est_valid    = true;
        hold_count     = 0;
        nco_phase      = 0.0f;
        nco_valid      = false;
        Serial.printf("[CMD] FC → %.3f Hz  → TRACKING, f_est=fc\n", fc);
      }
    }
  }
};

// ─── setup ───────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("=== doppler-cart-phase v3 (真の PLL: Goertzel at f_est) ===");
  Serial.printf("N=%d  N_CAL_SCAN=%d  N_BINS=%d  PLL_ALPHA=%.2f\n",
                N, N_CAL_SCAN, N_BINS, PLL_ALPHA);
#ifdef DEBUG_PHASE
  Serial.println("[DBG] DEBUG_PHASE 有効: TRACKING中に毎フレームデバッグ出力");
  Serial.println("[DBG] フォーマット: T probe phi p_hz i_hz fp amp");
#endif

  analogReadResolution(12);

  // Hanning 窓を事前計算
  for (int i = 0; i < N; i++)
    hann[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (N - 1)));

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

  // ── 1. サンプリング（実測 fs・フレーム開始時刻）──────
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

  // ── 4. 81点スキャン（SCANNING のみ）─────────────────
  float freq_scan = -1.0f;
  float amplitude  = -1.0f;

  if (calState == SCANNING) {
    for (int b = 0; b < N_BINS; b++)
      mag[b] = goertzel_mag(FREQ_LOW + b * FREQ_STEP, fs);

    int bestB = 0;
    for (int b = 1; b < N_BINS; b++)
      if (mag[b] > mag[bestB]) bestB = b;

    amplitude = mag[bestB] / (N / 2.0f);

    if (pp >= MIN_PP && amplitude >= AMPLITUDE_MIN_SCAN) {
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
  }

  // ── 5. fc 決定 / PLL 追跡 ─────────────────────────────

  // SCANNING: 有効な freq_scan を蓄積し中央値から fc を決定
  if (calState == SCANNING) {
    if (freq_scan > 0.0f && fc_cand_count < N_CAL_SCAN)
      fc_cands[fc_cand_count++] = freq_scan;

    if (fc_cand_count >= N_CAL_SCAN) {
      fc             = median_f(fc_cands, N_CAL_SCAN);
      calState       = TRACKING;
      prev_phi_valid = false;
      prev_t0_valid  = false;
      f_est          = fc;    // probe を fc で初期化（stale 値を排除）
      f_est_valid    = true;
      hold_count     = 0;
      nco_phase      = 0.0f;
      nco_valid      = false;
      Serial.printf("[CAL] fc = %.3f Hz  (ESP32 offset = %+.3f Hz) → TRACKING, f_est=fc\n",
                    fc, fc - 1000.0f);
    }
  }

  // TRACKING: Goertzel を f_est で計算する真の PLL
  float freq_phase = -1.0f;

  if (calState == TRACKING) {
    // probe = 現在の推定値（未確定なら fc を初期値として使用）
    const float probe = f_est_valid ? f_est : fc;

    float I_fc, Q_fc;
    if (!nco_valid) {
      nco_phase = 0.0f;
      nco_valid = true;
    }
    pll_iq_continuous(probe, fs, &nco_phase, &I_fc, &Q_fc);
    const float amp_fc = sqrtf(I_fc * I_fc + Q_fc * Q_fc);
    amplitude = amp_fc;

    if (amp_fc >= AMPLITUDE_MIN_PHASE) {
      // 連続 NCO 基準に対する位相誤差。停止中は 0 近傍に留まるべき。
      const float phi = atan2f(Q_fc, I_fc);

      if (prev_phi_valid && prev_t0_valid) {
        const float T_el = (t0 - prev_t0_us) * 1e-6f;

        if (T_el >= 0.01f && T_el <= 0.5f) {
          // dphi = フレーム間の位相差 ≈ 2π×(f_signal - probe)×T_el
          const float dphi = wrap_pi(phi - prev_phi);
          const float freq_error = dphi / (2.0f * (float)M_PI * T_el);

          freq_phase = probe + freq_error;
          f_est = probe + PLL_ALPHA * freq_error;  // IIR: f_est += α×freq_error
          f_est_valid = true;
          hold_count = 0;

#ifdef DEBUG_PHASE
          Serial.printf("[PHZ] T=%.4f probe=%.3f dphi=%.4f err=%.3f fp=%.3f amp=%.1f\n",
                        T_el, probe, dphi, freq_error, freq_phase, amp_fc);
#endif
        } else {
          // 異常なフレーム間隔（BLE 処理遅延など）→ チェーンリセット
          Serial.printf("[PLL] abnormal T_el=%.4f → skip\n", T_el);
          prev_phi_valid = false;
          prev_t0_valid  = false;
          nco_valid      = false;
          hold_count++;
        }
      } else {
        // チェーン初期フレーム → ホールド
        hold_count++;
      }

      prev_phi       = phi;
      prev_phi_valid = true;

    } else {
      // フェード（amp < 閾値）→ チェーン破断
      prev_phi_valid = false;
      nco_valid      = false;
      hold_count++;
    }

    // HOLD_MAX 超過 → φ チェーンリセット、f_est はホールド継続
    if (hold_count >= HOLD_MAX) {
      prev_phi_valid = false;
      prev_t0_valid  = false;
      nco_valid      = false;
      hold_count     = 0;
      Serial.println("[PLL] fade: reset phi chain, hold f_est");
    }

    // フレーム開始時刻を次フレーム用に保存
    prev_t0_us    = t0;
    prev_t0_valid = true;
  }

  // ── 6. BLE 送信 ──────────────────────────────────────
  const float t_s   = micros() * 1e-6f;
  const int   stat  = (int)calState;
  const int   cprog = (calState == SCANNING) ? fc_cand_count : 0;

  const float freq_pll = f_est_valid ? f_est : -1.0f;
  char msg[96];
  snprintf(msg, sizeof(msg), "%.4f,%.2f,%.2f,%.2f,%d,%.2f,%d,%.2f",
           t_s, freq_scan, freq_phase, amplitude, stat, fc, cprog, freq_pll);

  if (connected) {
    pNotify->setValue(msg);
    pNotify->notify();
  }
}
