# ntripclient_G5-P3H

**M5Atom Lite を使った、Septentrio mosaic-go G5 P3H rover 向けの最小 NTRIP クライアント。**

NTRIP で拾った RTCM3 補正を Grove UART(3.3V TTL)経由で mosaic の COM1 に流し込み
RTK 測位させ、**NMEA GGA @38400 は mosaic の COM2 が直接トラクタへ**出力する構成。
Atom はヘッドレスの純フォワーダ（表示・cut/fill は持たない）。

[NTRIP-client-for-Arduino](https://github.com/yasunorioi/NTRIP-client-for-Arduino)
の `M5Atom_Eniwa_Custom` を土台に、受信機レグを RS232 Base から **G26/G32 TTL 直結**へ
振り替えたもの。ライブラリ本体は lib_deps でそのまま流用している。

## データフロー

```
 caster (rtk.toiso.fit:2101/eniwa-bd982, anonymous)
     │  RTCM3 over WiFi (NTRIP)
     ▼
 M5Atom Lite ── Grove UART G26/G32 (3.3V TTL) ──►  mosaic-go COM1  (RTCM3 IN → RTK fixed)

 mosaic-go COM2 ──► NMEA GGA @38400 ──► トラクタ・ガイダンス   (mosaic が直接出力・Atom は非関与)
```

## 配線

Atom の Grove(HY2.0) → mosaic COM1。**両側とも 3.3V LVTTL なので、このレグにレベル変換は不要**（3本）。

| Atom (Grove) | 向き | mosaic COM1 |
|---|---|---|
| G26 (Serial2 TX) | ──► | COM1 RX |
| G32 (Serial2 RX) | ◄── | COM1 TX（本フォワーダでは未使用・任意） |
| GND | ── | GND |

> RTK fix が出ないときは **G26/G32(TX/RX)の入れ違い**をまず疑う（`src/main.cpp` の
> `MOSAIC_TX_PIN`/`MOSAIC_RX_PIN` を入れ替え）。

**COM2 → トラクタ**は別レグ。mosaic COM2 も LVTTL、トラクタ入力が RS232 なら
その脚だけ RS232 トランシーバ(在庫品)を噛ませる。**Atom はこのレグに関与しない。**

## mosaic-go 側の設定（プロビジョニングの実際）

起動時、Atom は受信機の状態を見て2通りに振る舞う（**非ブロッキング**・転送は解決まで保留）：

1. **すでに GGA が来ている＝設定済み** → 何もせず即転送開始（保存済み boot config で動作）。
   → **実機の P3H はこれ**。数秒で RTK-FIX。
2. **GGA が来ない＝未設定（工場出荷など）** → COM1 の Septentrio コマンドで自己構成：
   - `setCOMSettings, COM2, baud<com2_baud>` → `setNMEAOutput, Stream1, COM2, <com2_nmea>, sec1`（トラクタ）
   - `setNMEAOutput, Stream2, COM1, GGA, sec1`（Atom の fix-LED 用）
   `$R:` ack までリトライ、~30s で諦めて転送（ベストエフォート）。

> ⚠ **重要な実機知見**：この P3H の **COM1 は ASCII コマンドに応答しない**（`$R:` も
> `$R?` も返らない＝input が RTCMv3 専用でコマンド解釈しない）。RTCM3 入力・GGA 出力は
> 保存設定で完動するが、**Atom からコマンドで COM2 等を再構成することはできない**。
> コマンド駆動の自己構成／後述の COM2 変更を効かせたい場合は、受信機の
> **COM1 input を `auto` に**しておく（RxTools/USB で一度）か、コマンドを受ける個体を使う。

前提：**COM1 は既定 115200**（`MOSAIC_COM1_BAUD`）。RTCM3 入力は Septentrio 既定の
auto 使用に依存（明示コマンド無しで補正が効く）。`PROVISION_MOSAIC 0` で自己構成を無効化。

### COM2（トラクタ出力）の baud / NMEA を選ぶ
Web UI で **COM2 baud** と **COM2 NMEA**（例 `GGA` / `GGA+VTG`）を設定できる（NVS 保存）。
これは上記②の自己構成コマンドに反映される。**コマンドを受け付ける受信機でのみ**受信機に
反映（この P3H のようにコマンド非対応の個体では、値は保存されるが受信機側は RxTools で設定）。

> この G5 P3H は rover 専用個体（permission に RTKBase 無し・RTCM3 は入力のみ）。
> 基準局には使えない＝補正は外部の BD982(`eniwa-bd982`)から取る、で正しい。

## ビルド / 焼き

PlatformIO。**M5 Atom Lite の CH9102F は 115200 固定**（230400+ はほぼ失敗）。

```sh
pio run                 # build
pio run -t upload       # flash (upload_speed=115200)
pio device monitor      # 115200
```

**WiFi 設定ポータル**：起動時に**本体ボタンを長押し**すると SSID `NTRIP-Client` の AP
ポータルが開き、WiFi creds＋NTRIP 設定を投入できる（web に到達する前段なので WiFi は
ここで）。押さなければ保存済みで自動接続。

**Web UI（液晶が無いのでこれが主 UI）**：STA 接続後、**http://ntrip-rover.local/**
（または表示された IP）で稼働中に**設定変更＋監視**ができる。実機確認済み：
- ライブ status（WiFi/RSSI、NTRIP 接続、RTCM バイト数、**fix 品質**、稼働時間）を 2 秒毎更新
- NTRIP ホスト/ポート/マウント/ユーザ/パスを変更→**Save & apply**（NVS 保存＋NTRIP を
  ホット再接続、再起動不要）。別現場・別基準局へ焼き直し無しで移動可
- Reboot / Forget WiFi（ポータルへ）ボタン
- 既定は `rtk.toiso.fit:2101/eniwa-bd982`（anonymous）
- 認証なし（個人 LAN 前提）。必要なら Basic auth 追加は容易

**再接続**：WiFi/NTRIP 断は**その場でリトライ**（再起動しない）。最後の砦として WiFi が
3分以上復帰しない時だけ再起動。

## LED

fix 状態は mosaic COM1 から返る GGA を読んで色分けする。

| 色 | 意味 |
|---|---|
| 青 | 設定ポータル |
| 黄 | WiFi 接続中 |
| マゼンタ | NTRIP 接続中 |
| **緑** | **RTK fixed**（GGA quality=4） |
| シアン | RTK float（quality=5） |
| 橙 | 補正は流れているが未 fix（DGPS/GPS/なし） |
| 薄白 | GGA まだ来ず（fix 不明） |
| レインボー | 補正ストール（>5s RTCM3 途絶） |
| 赤 | 起動直後 / WiFi 断（3分で再起動） |

## 流用元

- [NTRIP-client-for-Arduino](https://github.com/yasunorioi/NTRIP-client-for-Arduino)
  （`NTRIPClient` ライブラリ＝ lib_deps で参照）。GLAY-AK2 版の fork。
