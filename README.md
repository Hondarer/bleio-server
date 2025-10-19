# bleio-server

bleio-server は、ESP32 で動作する BLE (Bluetooth Low Energy) ベースの GPIO 制御ファームウェアです。

このプロジェクトは [bleio](https://github.com/Hondarer/bleio) のサブモジュールとして使用されます。

## 概要

ESP32 を BLE ペリフェラル (周辺機器) として動作させ、PC などの BLE セントラル (中心機器) から GPIO を制御できるようにします。

NimBLE スタックを使用した軽量な実装で、GATT サービスを公開して GPIO のモード設定とデジタル入出力を提供します。

## ビルド環境

**プラットフォーム**

- ESP32 (espressif32)
- ボード: DOIT ESP32 DevKit V1

**フレームワーク**

- ESP-IDF
- NimBLE (BLE スタック)

**ビルドツール**

- PlatformIO

## プロトコル仕様

[BLEIO プロトコル仕様](https://github.com/Hondarer/bleio/blob/main/docs-src/protocol.md) を参照してください。

## ライセンス

[LICENSE](LICENSE) ファイルを参照してください。

## 関連リンク

- [bleio (親プロジェクト)](https://github.com/Hondarer/bleio)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [NimBLE Documentation](https://mynewt.apache.org/latest/network/docs/index.html)
