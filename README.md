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
             ◄── NMEA GGA (fix 状態・LED 用) ──   mosaic-go COM1

 mosaic-go COM2 ──► NMEA GGA @38400 ──► トラクタ・ガイダンス   (mosaic が直接出力・Atom は非関与)
```

## セットアップ（初回）

1. **配線する**（→[配線](#配線)）。Atom G26/G32 ↔ mosaic COM1（3.3V TTL 直結）、
   mosaic COM2 → トラクタ（RS232 なら在庫トランシーバ）、mosaic は別途 5V 給電。

2. **受信機（mosaic-go）を一度だけ準備する**。運用に応じて2択：

   - **A. Atom に任せる（工場出荷から・COM2 を Web で選びたいならこちら）**
     RxTools/web で **COM1 の input を `auto`** にして保存するだけ。以降 Atom が毎起動で
     COM1/COM2 の NMEA 出力を自己構成し、**COM2 の baud/NMEA は Web UI から選べる**。
     `auto` は RTCM3 補正の受理と ASCII コマンドの両方を通す。
     > ⚠ COM1 が RTCMv3 専用だと**コマンドを黙殺**する（実機の P3H がこれ）。その場合は B へ。

   - **B. 受信機側で作り込む（コマンド非対応の個体・割り切り運用）**
     RxTools で COM1＝RTCM3入力＋GGA出力、COM2＝希望 baud＋NMEA出力 を設定し
     **boot config に保存**。Atom は GGA を検知して転送＋監視に専念（自己構成しない）。

3. **焼く**（→[ビルド / 焼き](#ビルド--焼き)）。`pio run -t upload`。

4. **WiFi と NTRIP を設定する**。起動時に本体ボタン長押し → SSID `NTRIP-Client` の
   ポータルで WiFi creds＋NTRIP を投入。以降は稼働中に **http://ntrip-rover.local/** で変更可。

5. **確認する**。LED が**緑＝RTK fixed**。`http://ntrip-rover.local/` の status で
   NTRIP connected / RTCM バイト / fix=RTK-FIX を確認。

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

> ⚠ **重要な実機知見（結論）**：この P3H は、Atom が使う共有 UART（RTCM3 入力＋GGA 出力が
> 同時に流れる COM）上では **ASCII コマンドに `$R:` を返さない**。COM1 input を `auto` に
> 変えても（RxTools/USB で確認済み）**Atom からのコマンドは無応答のまま**だった
> （auto 検出が RTCM3 入力にロックしてコマンド解釈に切り替わらない挙動と推測）。
> → **Atom からの自己構成／後述の Web COM2 変更はこの個体では受信機に届かない。**
> **受信機の設定は RxTools/USB で行い boot config に保存する**のが確実（＝上記①の運用）。
> USB からのコマンド設定手順は本 README 末尾「受信機を USB で設定する」を参照。

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
- ライブ status（WiFi/RSSI、NTRIP 接続、RTCM バイト数、**fix 品質**、プロビジョニング
  状態 `Prov`（done/waiting/gave up）、稼働時間）を 2 秒毎更新
- NTRIP ホスト/ポート/マウント/ユーザ/パスを変更→**Save & apply**（NVS 保存＋NTRIP を
  ホット再接続、再起動不要）。別現場・別基準局へ焼き直し無しで移動可
- Reboot / Forget WiFi（ポータルへ）ボタン
- 既定は `rtk.toiso.fit:2101/eniwa-bd982`（anonymous）
- 認証なし（個人 LAN 前提）。必要なら Basic auth 追加は容易

**再接続**：WiFi/NTRIP 断は**その場でリトライ**（再起動しない）。最後の砦として WiFi が
3分以上復帰しない時だけ再起動。

## LED

稼働状態を色分けする（上ほど優先＝起動〜WiFi〜プロビジョニングを先に、解決後に fix 品質）。
fix 品質は mosaic COM1 から返る GGA を読んで判定する。

| 色 | 意味 |
|---|---|
| 赤 | 起動直後（初期化中・一瞬） |
| 青 | 設定ポータル（ボタン長押し） |
| 黄 | WiFi 接続中 / 再接続中（断が3分続くと再起動） |
| アンバー（濃橙） | 受信機プロビジョニング解決待ち（RTCM3 転送を保留中） |
| マゼンタ | NTRIP 接続中 |
| レインボー | 補正ストール（>5s RTCM3 途絶） |
| 薄白 | GGA まだ来ず（fix 不明） |
| 橙 | 補正は流れているが未 fix（DGPS/GPS/なし） |
| シアン | RTK float（quality=5） |
| **緑** | **RTK fixed**（GGA quality=4） |

## 受信機を USB で設定する（確実な方法）

mosaic-go を PC の USB に挿すと 2つの CDC ポートが出る（`/dev/ttyACM0`=USB1=コマンド港,
`/dev/ttyACM1`=USB2）。**ttyACM0 に Septentrio コマンドを送って設定し、boot config に保存**する。
実際にこの構成で動作確認した手順（115200・CR/LF 終端・応答は `$R:`）：

```
# COM1 = Atom（RTCM3 入力＋LED 用 GGA 出力, 115200）
setDataInOut,   COM1, auto                     # 入力を auto（RTCM3＋コマンド）
setNMEAOutput,  Stream1, COM1, GGA, sec1        # GGA を Atom へ
# COM2 = トラクタ（GGA 出力, 38400）
setNMEAOutput,  Stream2, COM2, GGA, sec1
setCOMSettings, COM2, baud38400
# 恒久化（電源再投入後も維持。これをしないと NMEA 設定は current-config で消える）
exeCopyConfigFile, Current, Boot
# 確認
getNMEAOutput / getCOMSettings / getDataInOut / getSBFOutput
```

> ⚠ **NMEA 出力は current-config だと電源再投入で消える**（実際に消えて fix が落ちた）。
> 必ず `exeCopyConfigFile, Current, Boot` で保存する。SBF 出力は COM1/COM2 では off に
> しておく（トラクタの 38400 を GGA だけにして圧迫させない）。COM2 の baud を変える前に、
> **Atom がどちらの COM に繋がっているか**を必ず確認（Atom=COM1@115200 前提。逆だと Atom の
> baud が合わず fix が出なくなる）。

## 流用元

- [NTRIP-client-for-Arduino](https://github.com/yasunorioi/NTRIP-client-for-Arduino)
  （`NTRIPClient` ライブラリ＝ lib_deps で参照）。GLAY-AK2 版の fork。
