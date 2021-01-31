### Windowsサンプルの使用方法
CN29とUSB Type Cケーブルで接続し、
SW2を●、SW1を使用するサーボの通信方式に合わせて設定（写真はRS485設定）

[写真1を挿入]

gs2d-cpp/samples/windows_sample/saclib-v3/saclib-v3.slnをVisual Studioで開いて
WindowsSerial.h 29行目のCOMポート名と、main.cpp 31行目のサーボの種類を変更して実行

### Ubuntuサンプルの使用方法
CN29とUSB Type Cケーブルで接続し、
SW2を●、SW1を使用するサーボの通信方式に合わせて設定する（写真はRS485設定）

[写真1を挿入]

gs2d-cpp/samples/linux_sample/thread を任意の場所にコピー
LinuxSerial.h 56行目のCOMポート名と、main.cpp 48行目のサーボの種類を変更
以下のコマンドを実行してサーボを動かす
```
cd thread
mkdir build
cd build
cmake ..
make
./gs2d
```

### Arduino Leonardoサンプルの使用方法
Arduino用コネクタを使用しLeonardoと接続し、
SW2を〇、SW1を使用するサーボの通信方式に合わせて設定する（写真はRS485設定）

[写真2を挿入]

基板背面のジャンパをH側またはS側にショートする。
H側に接続するとCN27のハードウェアシリアルピン（RX : 1, TX : 2）
S側に接続するとCN27のソフトウェアシリアルピン（RX : 4, TX : 5）
が使用される。サンプルではハードウェアシリアルが使用されているため、H側へ接続する。

[写真3を挿入]

gs2d-cpp/samples/arduino_sample/sketch_apr21aを開き、Leonardo上で実行。

### その他マイコン等で使用する場合
SW2を〇、SW1を使用するサーボの通信方式に合わせて設定する（写真はRS485設定）

[写真2を挿入]

基板背面のジャンパをH側またはS側にショートする。
H側に接続するとCN27のハードウェアシリアルピン（RX : 1, TX : 2）
S側に接続するとCN27のソフトウェアシリアルピン（RX : 4, TX : 5）
が使用される。

[写真3を挿入]

CN27のTX, RX, DE、CN26のIOREF, GNDの合計5ピンを接続する。

[写真4を挿入]

gs2d-cpp/gs2d/template/TemplateSerial.hのUSER CODE内部に環境に合わせてUART操作を記述する。
記述が必要な関数は以下の6つ
* open
* close
* isConnected
* read
* write
* time

gs2d-cpp/gs2d/template/main.hのUSER CODE内部に環境に合わせて操作を記述する。
記述が必要な箇所は以下の2つ
* TimeoutCallback
* main

環境に合わせてビルドし、サーボが回転することが確認できれば完成。
