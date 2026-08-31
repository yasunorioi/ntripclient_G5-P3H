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

## mosaic-go 側の一度きり設定（重要）

mosaic は「**SBF/出力ストリーム開始前の静かな窓**」でしか出力設定を適用しないため、
受信機側で**一度設定して boot config に保存**しておくのが堅い（Atom を毎起動で
provisioning させる必要がなくなり、フォワーダを単純に保てる）。web UI / RxTools で：

- **COM1** @115200: RTCM3 補正を差分ソースとして受理（`setRTCMInput` / 既定 auto）**かつ
  NMEA GGA を出力**（例 `setNMEAOutput, Stream2, COM1, GGA, sec1`）。GGA を COM1 に返すのは
  Atom が **fix 状態を読んで LED に反映**するため（下記 LED 参照）。COM1 baud は
  `MOSAIC_COM1_BAUD`(既定 115200) に合わせる。
- **COM2** @38400: **NMEA GGA** を出力（例 `setNMEAOutput, Stream1, COM2, GGA, sec1`）→ トラクタへ。
- 設定後 **boot config に保存**（`exeCopyConfigFile, Current, Boot`）。

> この G5 P3H は rover 専用個体（permission に RTKBase 無し・RTCM3 は入力のみ）。
> 基準局には使えない＝補正は外部の BD982(`eniwa-bd982`)から取る、で正しい。

## ビルド / 焼き

PlatformIO。**M5 Atom Lite の CH9102F は 115200 固定**（230400+ はほぼ失敗）。

```sh
pio run                 # build
pio run -t upload       # flash (upload_speed=115200)
pio device monitor      # 115200
```

**設定ポータル**：起動時に**本体ボタンを長押し**すると SSID `NTRIP-Client` のポータルが開き、
WiFi に加えて **NTRIP のホスト/ポート/マウント/ユーザ/パス**も設定できる（NVS 保存＝別現場・
別基準局へ焼き直し無しで移動可）。押さなければ保存済み設定で自動接続。既定は
`rtk.toiso.fit:2101/eniwa-bd982`（anonymous）。

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
