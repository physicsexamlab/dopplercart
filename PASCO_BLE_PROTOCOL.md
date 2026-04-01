# PASCO Wireless Smart Cart — BLE プロトコル解析メモ

**対象機器**: PASCO Wireless Smart Cart ME-1240 (Red) / ME-1241 (Blue)
**調査日**: 2026-04-01
**調査手法**: nRF Connect (iOS), chrome://bluetooth-internals, pasco-ble npm パッケージ(v0.3.65) ソース解析
**公開状況**: 非公開（社内メモ）

---

## 1. UUID 体系

PASCO 独自の 128-bit UUID パターン:

```
4a5c000{S}-000{C}-0000-0000-5c1e741f1c00
```

- `{S}` = サービス番号（0〜9）
- `{C}` = キャラクタリスティック番号（1〜9）

### サービス一覧（Smart Cart 実機確認）

| S | サービス UUID | 役割 |
|---|---|---|
| 0 | `4a5c0000-0000-...` | Device Service（グローバル制御） |
| 1 | `4a5c0001-0000-...` | Force センサ |
| 2 | `4a5c0002-0000-...` | Acceleration センサ |
| 3 | `4a5c0003-0000-...` | **Position センサ（ロータリーエンコーダ）** |
| 4 | `4a5c0004-0000-...` | Gyro センサ |

> **注意**: ハンドオフ文書では UUID の第2グループが `0002` などと記載されていたが、
> 実機では第2グループは **`0000`** （サービス側）。
> `000{C}` はキャラクタリスティック側のみ使用される。

---

## 2. キャラクタリスティック一覧

各センササービス内のキャラクタリスティックは共通パターン:

| C | プロパティ | 役割 |
|---|---|---|
| 0002 | `writeWithoutResponse` | **コマンド送信** (SEND_CMD_CHAR) |
| 0003 | `write, notify, indicate` | **レスポンス受信** (RECV_CMD_CHAR) |
| 0004 | `read, notify` | データチャンネル（用途要調査） |
| 0005 | `writeWithoutResponse` | **ACK 送信** (SEND_ACK_CHAR) |

Device Service (S=0) のキャラクタリスティック:

| Char UUID | プロパティ | 役割 |
|---|---|---|
| `4a5c0000-0002-...` | `writeWithoutResponse` | グローバルコマンド送信 |
| `4a5c0000-0003-...` | `notify` など | グローバルレスポンス受信 |

---

## 3. 接続シーケンス

```
1. requestDevice({ filters: [{ namePrefix: 'Smart Cart' }],
                   optionalServices: [全PASCOのUUID一覧] })

2. device.gatt.connect()

3. devSvc = getPrimaryService('4a5c0000-0000-...')
   devCmd  = devSvc.getCharacteristic('4a5c0000-0002-...')
   devRecv = devSvc.getCharacteristic('4a5c0000-0003-...')
   devRecv.startNotifications()

4. devCmd.writeValueWithoutResponse([0x37, 0x01, 0x00])
   // WIRELESS_RMS_START コマンド
   // → devRecv から [0xC0, 0x00, 0x37] が返る（成功の ACK）

5. posSvc = getPrimaryService('4a5c0003-0000-...')
   posCmdChar  = posSvc.getCharacteristic('4a5c0003-0002-...')  // コマンド
   posRespChar = posSvc.getCharacteristic('4a5c0003-0003-...')  // レスポンス
   posAckChar  = posSvc.getCharacteristic('4a5c0003-0005-...')  // ACK
   posRespChar.startNotifications()
```

---

## 4. データ取得（One-shot Read）

### リクエスト

```javascript
posCmdChar.writeValueWithoutResponse(new Uint8Array([0x05, 0x02]))
// [GCMD_READ_ONE_SAMPLE=0x05, DataSize=0x02]
// DataSize = 2 (int16 = RawCountChange)
```

### レスポンス

**重要**: レスポンスは `posRespChar`（`4a5c0003-0003-...`）ではなく、
**`devRecv`（`4a5c0000-0003-...`）** に届く。

```
受信バイト列: [0xC0, 0x00, 0x05, lo, hi]
  0xC0 = GRSP_RESULT（成功レスポンス）
  0x00 = エラーコード 0（成功）
  0x05 = GCMD_READ_ONE_SAMPLE のエコー
  lo, hi = RawCountChange（int16, リトルエンディアン, 2の補数）
```

### デコード

```javascript
const rawChange = dataView.getInt16(3, true);  // リトルエンディアン signed int16
countAccumulated += rawChange;
position_m = countAccumulated * 0.09895 / 816;
// 0.09895 = ホイール円周[m]相当のスケール係数
// 816 = 1回転あたりのエンコーダカウント数
```

---

## 5. 周期データ（Periodic Data）モード

One-shot Read の他に、デバイスが自動送信する周期データモードも存在する。

```
受信バイト列: [seqNum, lo, hi]
  seqNum = パケット番号（0x00〜0x1F で循環）
  lo, hi = RawCountChange（int16, LE）
```

- `seqNum <= 0x1F` で判別
- 8パケットごとに ACK を返す必要あり:
  `posAckChar.writeValueWithoutResponse([seqNum])`
- ただし Smart Cart で周期データが自動送信されるトリガー条件は未調査

---

## 6. プロトコル定数（pasco-ble v0.3.65 より）

```javascript
SENSOR_SERVICE_ID      = 0       // Device Service の service_id
SEND_CMD_CHAR_ID       = 2       // コマンド Char の番号
RECV_CMD_CHAR_ID       = 3       // レスポンス Char の番号
SEND_ACK_CHAR_ID       = 5       // ACK Char の番号

GCMD_READ_ONE_SAMPLE   = 0x05   // 1サンプル要求コマンド
GCMD_XFER_BURST_RAM    = 0x0E
GCMD_CUSTOM_CMD        = 0x37

GRSP_RESULT            = 0xC0   // 成功レスポンスのプレフィックス
GEVT_SENSOR_ID         = 0x82   // センサ ID イベント

WIRELESS_RMS_START     = [0x37, 0x01, 0x00]  // ロータリーモーション開始
```

---

## 7. Position センサデータシート（Sensor ID 2027）

pasco-ble の `datasheets.js` より:

```javascript
2027: {
  tag: 'SmartCartPositionSensor',
  measurements: [
    { ID: 0, NameTag: 'RawCountChange', Type: 'RawDigital',
      DataSize: 2, TwosComp: '1', Internal: 1 },     // 生カウント変化量（内部用）
    { ID: 1, NameTag: 'Position', Type: 'RotaryPos',
      Inputs: '0', Params: '0.09895,816', UnitType: 'm' },
    { ID: 2, NameTag: 'Velocity', Type: 'Derivative',
      Inputs: '1', Params: '3', UnitType: 'mps' },
    { ID: 3, NameTag: 'Acceleration', Type: 'Derivative',
      Inputs: '2', UnitType: 'ms2' },
  ]
}
```

### RotaryPos 計算式

```
position += (RawCountChange × Params[0]) / Params[1]
          = (RawCountChange × 0.09895) / 816
```

### Derivative 計算式（速度・加速度）

```
velocity = (position_now - position_prev) / 2
```

（pasco-ble では差分を 2 で割る実装）

---

## 8. DataSize とパケットサイズの計算

`GCMD_READ_ONE_SAMPLE` の第2引数 `packetSize` は、
センサの **DataSize を持つ測定値の合計バイト数**。

Smart Cart Position の場合:
- `RawCountChange`: DataSize=2, Internal=1（唯一の生データ）
- `Position`, `Velocity`, `Acceleration`: 全て派生値（DataSize なし）

→ **packetSize = 2**

---

## 9. Web Bluetooth 実装上の注意点

### optionalServices の宣言

Web Bluetooth は `requestDevice` 時に `optionalServices` に列挙されていないサービス UUID にはアクセスできない。
ただし、サービス UUID を宣言すれば、その配下のキャラクタリスティック UUID は個別宣言不要。

```javascript
// 全 PASCO UUID パターンを事前宣言する方法
const ALL_PASCO_UUIDS = [];
for (let s = 0; s <= 9; s++) {
  ALL_PASCO_UUIDS.push(`4a5c000${s}-0000-0000-0000-5c1e741f1c00`);
  for (let c = 1; c <= 9; c++) {
    ALL_PASCO_UUIDS.push(`4a5c000${s}-000${c}-0000-0000-5c1e741f1c00`);
  }
}
```

### getPrimaryServices() の挙動

`server.getPrimaryServices()`（引数なし）は `optionalServices` に宣言済みの UUID のみ返す。
→ 宣言漏れがあると「No Services found」エラーになる。

### BluetoothCharacteristicProperties の読み方

```javascript
// NG: Object.keys() では取れない
const props = Object.keys(c.properties).filter(k => c.properties[k]);

// OK: 明示的にプロパティ名を指定する
const PROP_NAMES = ['broadcast','read','writeWithoutResponse','write','notify','indicate'];
const props = PROP_NAMES.filter(p => c.properties[p]);
```

---

## 10. 未解決事項

- `[dev recv] 85 7d 0f 5d 00 00` の意味（接続直後に複数回届く）
  → `0x85` は未知のイベントコード。おそらく Interface ID 通知と思われる
- `4a5c0003-0004-...`（read, notify）の用途
- Force/Acceleration サービスの DataSize と packetSize（未計測）
- 2台同時接続時の挙動
