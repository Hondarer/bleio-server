# bleio-server

bleio-server は、ESP32 で動作する BLE (Bluetooth Low Energy) ベースの GPIO 制御ファームウェアです。

このプロジェクトは [bleio](https://github.com/Hondarer/bleio) のサブモジュールとして使用されます。

## 概要

ESP32 を BLE ペリフェラル (周辺機器) として動作させ、PC などの BLE セントラル (中心機器) から GPIO を制御できるようにします。

NimBLE スタックを使用した軽量な実装で、GATT サービスを公開して GPIO のモード設定とデジタル入出力を提供します。

## 機能

**GPIO 制御**

- GPIO モード設定 (入力、出力、プルアップ付き入力)
- デジタル出力 (HIGH/LOW)
- デジタル入力読み取り

**BLE 通信**

- デバイス名: BLEIO-ESP32
- サービス UUID: `4fafc201-1fb5-459e-8fcc-c5c9c333914b`
- 書き込みキャラクタリスティック: `beb5483e-36e1-4688-b7f5-ea07361b26a8`
- 読み取りキャラクタリスティック: `1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e`

## ビルド環境

**プラットフォーム**

- ESP32 (espressif32)
- ボード: DOIT ESP32 DevKit V1

**フレームワーク**

- ESP-IDF
- NimBLE (BLE スタック)

**ビルドツール**

- PlatformIO

## ビルド方法

PlatformIO を使用してビルドします。

```bash
pio run
```

ESP32 に書き込みます。

```bash
pio run --target upload
```

シリアルモニターで動作を確認します。

```bash
pio device monitor
```

## ファイル構成

```text
server/
├── src/
│   ├── main.c           メインプログラム
│   └── CMakeLists.txt   ESP-IDF ビルド設定
├── include/             ヘッダファイル
├── lib/                 ライブラリ
├── test/                テストコード
├── platformio.ini       PlatformIO 設定
├── CMakeLists.txt       トップレベルビルド設定
└── README.md            このファイル
```

## プロトコル仕様

詳細なプロトコル仕様は [bleio の CLAUDE.md](https://github.com/Hondarer/bleio/blob/main/CLAUDE.md) を参照してください。

**GPIO 書き込みキャラクタリスティック**

2 バイトのデータを送信してモード設定と出力制御を行います。

| バイト | 内容 |
|-------|------|
| 0 | GPIO ピン番号 (2-39) |
| 1 | コマンド (0: INPUT, 1: OUTPUT, 2: INPUT_PULLUP, 10: LOW, 11: HIGH) |

**GPIO 読み取りキャラクタリスティック**

1 バイトのピン番号を書き込んでから読み取ることで、2 バイトのデータ (ピン番号 + 状態) を取得します。

## 対応 GPIO ピン

**デジタル入出力**

GPIO2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33

**入力専用**

GPIO34, 35, 36, 39

**使用禁止**

GPIO0 (ブート), GPIO1/3 (UART), GPIO6-11 (フラッシュ)

## ライセンス

MIT ライセンスです。詳細は [LICENSE](LICENSE) ファイルを参照してください。

## 関連リンク

- [bleio (親プロジェクト)](https://github.com/Hondarer/bleio)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [NimBLE Documentation](https://mynewt.apache.org/latest/network/docs/index.html)
