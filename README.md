# Doppler Cart Monitor

PASCO Smart Cart と Arduino Nano ESP32 製の周波数センサを組み合わせ、
ドップラー効果を定量的に測定するブラウザアプリと周辺ファイルを公開します。

本システムは Physics Exam Lab 物理入試ラボシリーズの一環として開発されたプロトタイプです。

---

## 必要機器

| 機器 | 用途 |
|---|---|
| PASCO Smart Cart | 台車の位置・速度測定（BLE）|
| スマートフォン + phyphox | 2000 Hz 音源（[phyphox.org](https://phyphox.org/)）|
| Arduino Nano ESP32 + MAX9814 マイクアンプモジュール | 周波数センサ |
| PC（Mac / Windows / Chromebook）+ **Google Chrome** | 計測アプリの実行 |

> Safari・Firefox には Web Bluetooth / Web Serial API が実装されていないため使用できません。

---

## 実験の構成

音源（スマートフォン）を台車に載せる **Config A（音源が動く）** と、
周波数センサを台車に載せる **Config B（観測者が動く）** の 2 通りが可能です。

いずれの場合も低速近似

$$\Delta f \approx \frac{f_0}{v_s} \, v$$

が成り立ち（$f_0 = 2000\,\text{Hz}$, $v_s = 343\,\text{m/s}$）、
散布図上の実測点が理論直線に沿うことをリアルタイムで確認できます。

---

## 使い方

### 1. ファームウェアの書き込み（初回のみ）

Chrome で以下の URL を開き、Arduino Nano ESP32 を USB 接続して「書き込む」ボタンを押します。

**https://physicsexamlab.github.io/dopplercart/flash.html**

### 2. 計測アプリの起動

Chrome で以下の URL を開きます。

**https://physicsexamlab.github.io/dopplercart/doppler-cart.html**

1. スマートフォンで phyphox を起動し、2000 Hz を最大音量で再生する
2. **[Connect Smart Cart]** をクリックして台車を接続する
3. **[Connect IQ Sensor]** をクリックして周波数センサを接続する
4. **[Record]** を押した後、台車をマイク近傍で 30 秒以上静置する
   （この区間でキャリア周波数が自動校正されます）
5. 台車を 0.3〜0.5 m/s でゆっくり押して走行させ、壁で跳ね返らせる
6. 台車が戻ったら **[Stop]** を押す
7. **[CSV Export]** で計測データをダウンロードする

---

## 周波数センサの回路

MAX9814 マイクアンプモジュールと Arduino Nano ESP32 を次のように接続します。

| MAX9814 | Arduino Nano ESP32 |
|---|---|
| Out | A0（GPIO1）|
| GND | GND |
| Vdd | 3.3V |
| GAIN | フローティング（最大ゲイン 60 dB）|

USB-C ケーブルで電池パックに接続して動作します。

---

## リポジトリ構成

```
doppler-cart.html        計測アプリ（Chrome で開く）
flash.html               ファームウェア書き込みページ
Arduino/
  doppler-cart/
    doppler-cart.ino     ファームウェアソースコード
firmware/
  manifest.json          ESP Web Tools 用マニフェスト
  bootloader.bin
  partitions.bin
  boot_app0.bin
  firmware.bin           コンパイル済みバイナリ（Arduino Nano ESP32 用）
```

---

## ライセンス

Copyright (c) 2026 一般社団法人 国際物理オリンピック2023記念協会

Licensed under the [Creative Commons Attribution-NonCommercial 4.0 International License](https://creativecommons.org/licenses/by-nc/4.0/).
