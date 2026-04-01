# HANDOFF: Smart Cart Position Monitor

**作成日**: 2026-04-01
**プロジェクト**: Physics Exam Lab — Doppler Cart
**公開 URL**: https://physicsexamlab.github.io/dopplercart/
**リポジトリ**: https://github.com/physicsexamlab/dopplercart
**ローカルパス**: `/Users/hayano/Documents/doppler-cart/`

---

## 現在の状態（動作確認済み）

PASCO Wireless Smart Cart（ME-1240/ME-1241）の**位置**をリアルタイムにグラフ表示する Web アプリが完成し、GitHub Pages で公開中。

- BLE 接続・ペアリング ✅
- 50ms ごとの位置サンプリング ✅
- Position [m] / Velocity [m/s] グラフ ✅
- CSV エクスポート（コピー＆ダウンロード） ✅

---

## ファイル構成

```
doppler-cart/
├── index.html          ← Smart Cart Position Monitor（メインアプリ）
├── ble_monitor2.html   ← 旧アプリ（超音波距離センサ＋マイク Doppler）
├── package/            ← pasco-ble npm パッケージ（解析用、本番不要）
└── HANDOFF_smartcart_position.md  ← このファイル
```

---

## 動作手順

1. Chrome で https://physicsexamlab.github.io/dopplercart/ を開く
2. Smart Cart の電源を入れる
3. **Connect Smart Cart** → ダイアログから `Smart Cart 742-993>5M` を選択
4. 接続後、**Record** → カートを動かす → グラフ更新
5. **Stop** → **CSV Export** でデータ保存
6. **Reset** で初期化

> **注意**: Web Bluetooth は Chrome / Edge のみ対応。HTTPS 必須（GitHub Pages は OK）。

---

## 技術的要点（詳細は `PASCO_BLE_PROTOCOL.md` 参照）

| 項目 | 値 |
|---|---|
| デバイス名フィルタ | `namePrefix: 'Smart Cart'` |
| Position サービス UUID | `4a5c0003-0000-0000-0000-5c1e741f1c00` |
| コマンド Char (0002) | `writeWithoutResponse` — リクエスト送信先 |
| レスポンス Char (0003) | `write, notify, indicate` — データ受信 |
| ACK Char (0005) | `writeWithoutResponse` — ACK 返送先 |
| デバイスサービス CMD | `4a5c0000-0002-0000-0000-5c1e741f1c00` |
| デバイスサービス RECV | `4a5c0000-0003-0000-0000-5c1e741f1c00` |
| 位置スケール | `count × 0.09895 / 816` [m] |
| サンプリング方式 | 50ms ごとに `[0x05, 0x02]` をコマンド Char に送信 |

---

## デバッグ方法

- Diagnostic Log パネル（画面内 `▶ DIAGNOSTIC LOG`）を開くと BLE 通信の詳細が表示される
- `chrome://bluetooth-internals` でデバイスのアドレスや RSSI を確認可能
- nRF Connect（iOS/Android）でサービス UUID を確認する場合は、先にMacのBluetooth接続を切断してからスキャン

---

## 次のステップ（将来実装予定）

1. **ドップラー効果測定**: マイクで音波周波数を検出し、Smart Cart の速度と対応させる
   → `ble_monitor2.html` の超音波・周波数計測ロジックと統合
2. **複数カート対応**: 2台の Smart Cart を同時接続して衝突実験
3. **加速度・力センサの追加**: Smart Cart の `4a5c0001`（Force）、`4a5c0002`（Acceleration）サービスも同様の手順で実装可能

---

## コピーライト

```
Physics Exam Lab - Doppler Cart
Copyright (c) 2026 一般社団法人 国際物理オリンピック2023記念協会
Licensed under the Creative Commons BY-NC 4.0 International License.
https://creativecommons.org/licenses/by-nc/4.0/
```
