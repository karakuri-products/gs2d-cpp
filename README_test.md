## 使用方法
* crc16.h
* gs2d_b3m.h
* gs2d_base.h
* gs2d_buffer_type.h
* gs2d_futaba.h
* gs2d_krs.h
* gs2d_parameter.h
* gs2d_robotis_2_0.h
* TemplateSerial.h
以上9ファイルをプロジェクトに含める。
TemplateSerialのUSER CODE欄にハードウェアに合わせたコードを記入。
## 利用例

### ID1のサーボモーターを左右に動かす
```
#include <iostream.h>
#include <gs2d_robotis_2_0.h>
#include <TemplateSerial.h>

using namespace gs2d;

int main(void)
{
    ServoBase *servo = new RobotisP20<TemplateSerial>();

    // サーボ初期化
    servo->init();

    // ID1のトルクをON
    servo->writeTorque(1, 1);

    // ID1を90度の位置に移動
    servo->writeTargetPosition(1, 90.0);

    // ID1を-90度の位置に移動
    servo->writeTargetPosition(1, -90.0);
}
```
### ID1の現在温度の読み込み（同期）
```
#include <iostream.h>
#include <gs2d_robotis_2_0.h>
#include <TemplateSerial.h>

using namespace gs2d;

int main(void)
{
    ServoBase *servo = new RobotisP20<TemplateSerial>();

    // サーボ初期化
    servo->init();

    // ID1のサーボの温度を読み込み
    int temperature = servo->readTemperature(1);
    std::cout << temperature << std::endl;
}
```

### ID1の現在温度の読み込み（非同期）
```
#include <iostream.h>
#include <gs2d_robotis_2_0.h>
#include <TemplateSerial.h>

using namespace gs2d;

void TemperatureCallback(CallbackEventArgs e)
{
    std::cout << (float)e.data << std::endl;
}

int main(void)
{
    ServoBase *servo = new RobotisP20<TemplateSerial>();

    // サーボ初期化
    servo->init();

    // コールバック関数の登録
    servo->attachTemperatureCallback(TemperatureCallback);

    // ID1のサーボの温度を読み込み
    servo->readTemperature(1, true);

    // 受信待機
    while(1) servo->listener();
}
```

## API
### Type
コールバック関数の引数に使われる構造体です。
全てのコールバック関数がこの型で統一されています。
```
class CallbackEventArgs
{
        uint8_t id;
        int8_t status;
        uint16_t address;
        CallbackEventData data;
};
```
* メンバ変数
  * data : イベントの固有データ。代入先によってuint32_t型かfloat型どちらかで返される。
  * id : イベント発生元サーボのID
  * error : イベント発生時のエラー番号
  * address : イベント発生元サーボのROM/RAMアドレス
#### Baudrate
Baudrate設定時に使用される対応ボーレート列挙体です。
kr-SAC001では3Mbpsまで対応しています。
```
    enum BaudrateList {
        BAUDRATE_9600 = 9600,
        BAUDRATE_19200 = 19200,
        BAUDRATE_57600 = 57600,
        BAUDRATE_115200 = 115200,
        BAUDRATE_230400 = 230400,
        BAUDRATE_625000 = 625000,
        BAUDRATE_1000000 = 1000000,
        BAUDRATE_1250000 = 1250000,
        BAUDRATE_1500000 = 1500000,
        BAUDRATE_2000000 = 2000000,
        BAUDRATE_3000000 = 3000000,

        // Not Supported on SAC
        BAUDRATE_4000000 = 4000000,
        BAUDRATE_4500000 = 4500000
    };
```
#### CommResult
通信のエラー内容の列挙体です。
```
    enum CommResult {
        COMM_SUCCESS = 0,
        READ_SUCCESS = 1,
        WRITE_SUCCESS = 2,
        COMM_TIMEOUT = 3,
        CHECKSUM_ERROR = 4,
        BUFFER_IS_EMPTY = 5,
        BUFFER_IS_FULL = 6,
		FUNC_NOT_EXIST = 7,
        NO_EEPROM_DATA = 8,
	};
```
* CommSuccess : 異常なし
* ReadSuccess : 読み込み成功
* WriteSuccess : 書き込み成功
* CommTimeout : 通信タイムアウト
* CheckSumError : チェックサム不一致
* BufferIsEmpty : 送信バッファが空
* BufferIsFull : 送信待機バッファに空き無し
* FuncNotExist : サーボが関数の機能に対応していない
* NoEEPROMData : EEPROMが読み込まれていないためEEPROMの書き込みが出来ない（KRSシリーズ専用）
---

### Torque
<details>
<summary>readTorque</summary>

```
uint8_t readTorque(uint8_t id, bool async = false);
```
- 引数
  - id : サーボID
  - async : 非同期フラグ。trueの場合非同期通信モードで送信される
- 戻り値
  - トルク値。0 : Torque Off, 1 : Torque On。非同期モードの場合は常に0が返される。
</details>

<details>
<summary>writeTorque</summary>

```
CommResult writeTorque(uint8_t id, uint8_t torque, bool async = false);
```
* 引数
  * id : サーボID
  * torque : 0 : Torque Off, 1 : Torque On
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>


<details>
<summary>attachTorqueCallback</summary>

```
void attachTorqueCallback(callbackType func)
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
</details>

---

### Current Position
<details>
<summary>readPosition</summary>

```
float readPosition(uint8_t id, bool async = false);
```
- 引数
  - id : サーボID
  - async : 非同期フラグ。trueの場合非同期通信モードで送信される
- 戻り値
  - 角度（単位 : degree)。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>attachPositionCallback</summary>

```
void attachPositionCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
</details>

---

### Target Position
<details>
<summary>readTargetPosition</summary>

```
float readTargetPosition(uint8_t id, bool async = false);
```
- 引数
  - id : サーボID
  - async : 非同期フラグ。trueの場合非同期通信モードで送信される。
- 戻り値
  - 目標位置（単位 : degree）。非同期モードの場合は常に0が返される。

</details>
<details>
<summary>writeTargetPosition</summary>

```
CommResult writeTargetPosition(uint8_t id, float position, bool async = false);
```
* 引数
  * id : サーボID
  * position : 目標位置（単位 : degree）
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT

</details>
<details>
<summary>attachPositionCallback</summary>

```
void attachPositionCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
</details>

---

### Temperature
<details>
<summary>readTemperature</summary>

```
uint16_t readTemperature(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 現在の温度（単位 : degree）。非同期モードの場合は常に0が返される。

</details>
<details>
<summary>attachTemperatureCallback</summary>

```
void attachTemperatureCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
</details>

---

### Current

<details>
<summary>readCurrent</summary>

```
uint16_t readCurrent(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 負荷電流値（単位 : mA）。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>attachCurrentCallback</summary>

```
void attachCurrentCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
</details>

---

### Voltage

<details>
<summary>readVoltage</summary>

```
uint16_t readVoltage(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 入力電圧（単位 : mV）。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>attachVoltageCallback</summary>

```
void attachVoltageCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
</details>

---

### P Gain

<details>
<summary>readPGain</summary>

```
uint16_t readPGain(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 現在のPゲイン値。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>writePGain</summary>

```
CommResult writePGain(uint8_t id, uint16_t pGain, bool async = false);
```
* 引数
  * id : サーボID
  * pGain : Pゲイン値
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>
<details>
<summary>attachPGainCallback</summary>
```
void attachPGainCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
</details>

---

### I Gain

<details>
<summary>readIGain</summary>

```
uint16_t readIGain(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 現在のIゲイン値。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>writeIGain</summary>

```
CommResult writeIGain(uint8_t id, uint16_t iGain, bool async = false);
```
* 引数
  * id : サーボID
  * iGain : Iゲイン値
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>
<details>
<summary>attachIGainCallback</summary>

```
void attachIGainCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
</details>

---

### D Gain

<details>
<summary>readDGain</summary>

```
uint16_t readDGain(uint8_t id, bool async = false)
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 現在のDゲイン値。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>writeDGain</summary>

```
CommResult writeDGain(uint8_t id, uint16_t dGain, bool async = false);
```
* 引数
  * id : サーボID
  * dGain : Dゲイン値
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>
<details>
<summary>attachDGainCallback</summary>

```
void attachDGainCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
  </details>

---

### ID
<details>
<summary>readID</summary>

```
uint8_t readID(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 対象サーボのID。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>writeID</summary>

```
CommResult writeID(uint8_t id, uint8_t newid, bool async = false);
```
* 引数
  * id : サーボID
  * newID : 新しいサーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>
<details>
<summary>attachIDCallback</summary>

```
void attachIDCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
</details>

---

### Baudrate
<details>
<summary>readBaudrate</summary>

```
BaudrateList readBaudrate(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 対象サーボの通信ボーレート。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>writeBaudrate</summary>

```
CommResult writeBaudrate(uint8_t id, BaudrateList baudrate, bool async = false);
```
* 引数
  * id : サーボID
  * newID : 新しいサーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>
<details>
<summary>attachBaudrateCallback</summary>

```
void attachBaudrateCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
</details>

---

### Offset
<details>
<summary>readOffset</summary>

```
float readOffset(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * オフセット値。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>writeOffset</summary>

```
CommResult writeOffset(uint8_t id, float offset, bool async = false);
```
* 引数
  * id : サーボID
  * offset : オフセット値
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>
<details>
<summary>attachOffsetCallback</summary>

```
void attachOffsetCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
</details>

---

### Deadband
<details>
<summary>readDeadband</summary>

```
float readDeadband(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * デッドバンド値。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>writeDeadband</summary>

```
CommResult writeDeadband(uint8_t id, float deadband, bool async = false);
```
* 引数
  * id : サーボID
  * deadband : デッドバンド値
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>
<details>
<summary>attachDeadbandCallback</summary>

```
void attachDeadbandCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
</details>

---

### CW Position Limit 
<details>
<summary>readCWLimit</summary>

```
float readCWLimit(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 時計回り方向の回転角の制限。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>writeCWLimit</summary>

```
CommResult writeCWLimit(uint8_t id, float cwLimit, bool async = false);
```
* 引数
  * id : サーボID
  * cwLimit : 時計回り方向の回転角の制限
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>
<details>
<summary>attachCWLimitCallback</summary>

```
void attachCWLimitCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
</details>

---

### CCW Position Limit 
<details>
<summary>readCCWLimit</summary>

```
float readCCWLimit(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 反時計回り方向の回転角の制限。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>writeCCWLimit</summary>

```
CommResult writeCCWLimit(uint8_t id, float ccwLimit, bool async = false);
```
* 引数
  * id : サーボID
  * ccwLimit : 反時計回り方向の回転角の制限
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>
<details>
<summary>attachCCWLimitCallback</summary>

```
void attachCCWLimitCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
</details>

---

### Temperature Limit
<details>
<summary>readTemperatureLimit</summary>

```
uint16_t readTemperatureLimit(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 温度リミット値。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>writeTemperatureLimit</summary>

```
writeTemperatureLimit(uint8_t id, uint16_t tempLimit, bool async = false);
```
* 引数
  * id : サーボID
  * tempLimit : 温度リミット値
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>
<details>
<summary>attachTemperatureLimitCallback</summary>

```
void attachTemperatureLimitCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
  </details>

---

### Current Limit
<details>
<summary>readCurrentLimit</summary>

```
uint16_t readCurrentLimit(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 電流リミット値。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>writeCurrentLimit</summary>

```
CommResult writeCurrentLimit(uint8_t id, uint16_t currentLimit, bool async = false);
```
* 引数
  * id : サーボID
  * currentLimit : 電流リミット値
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>
<details>
<summary>attachCurrentLimitCallback</summary>

```
void attachCurrentLimitCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
  </details>

---

### Speed
<details>
<summary>readSpeed</summary>

```
uint16_t readSpeed(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 移動速度値。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>writeSpeed</summary>

```
CommResult writeSpeed(uint8_t id, uint16_t speed, bool async = false);
```
* 引数
  * id : サーボID
  * speed : 目標スピード値
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>
<details>
<summary>attachSpeedCallback</summary>

```
void attachSpeedCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
  </details>

---

### Acceleration
<details>
<summary>readAcceleration</summary>

```
uint16_t readAcceleration(uint8_t id, bool async = false);
```
* 引数
  * id : サーボID
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 加速度値。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>writeAcceleration</summary>

```
CommResult writeAcceleration(uint8_t id, uint16_t accel, bool async = false);
```
* 引数
  * id : サーボID
  * accel : 加速度値
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>
<details>
<summary>attachAccelCallback</summary>

```
void attachAccelCallback(callbackType func);
```
* 引数
  * func : コールバック関数
* 戻り値
  * なし
  </details>

---

### Burst R/W Position
複数のサーボのPositionを同時に読み書きする機能です。
Readの場合はCurrent Positionが、Writeの場合はTarget Positionが参照されます。
<details>
<summary>burstReadPosition</summary>

```
CommResult burstReadPosition(uint8_t* id_list, uint8_t num, float* position_list, bool async = false);
```
* 引数
  * id_list : サーボIDが格納された配列
  * num : 対象サーボ数
  * position_list : 位置データが格納される配列
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 0 or READ_SUCCESS or FUNC_NOT_EXIST

</details>
<details>
<summary>burstWritePosition</summary>

```
CommResult burstWriteTargetPosition(uint8_t* id_list, uint8_t num, float* position_list, bool async = false);
```
* 引数
  * id_list : サーボIDが格納された配列
  * num : 対象サーボ数
  * position_list : 位置データが格納された配列
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 0 or FUNC_NOT_EXIST
  </details>

---

### General Burst R/W
<details>
<summary>burstReadMemory</summary>

```
CommResult burstRead(uint8_t* id_list, uint16_t address, uint8_t num, uint8_t* data, bool async = false);
CommResult burstRead(uint8_t* id_list, uint16_t address, uint8_t num, uint16_t* data, bool async = false);
CommResult burstRead(uint8_t* id_list, uint16_t address, uint8_t num, uint32_t* data, bool async = false);
```
* 引数
  * id_list : サーボIDが格納された配列
  * address : 対象アドレス
  * num : 対象サーボ数
  * data : データが格納される配列
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 0 or READ_SUCCESS or FUNC_NOT_EXIST
</details>
<details>
<summary>burstWriteMemory</summary>

```
CommResult burstWrite(uint8_t* id_list, uint16_t address, uint8_t num, uint8_t* data, bool async = false);
CommResult burstWrite(uint8_t* id_list, uint16_t address, uint8_t num, uint16_t* data, bool async = false);
CommResult burstWrite(uint8_t* id_list, uint16_t address, uint8_t num, uint32_t* data, bool async = false);
```
* 引数
  * id_list : サーボIDが格納された配列
  * address : 対象アドレス
  * num : 対象サーボ数
  * data : データが格納された配列
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 0 or FUNC_NOT_EXIST
  </details>

---

### ROM
ROMの保存機能があるサーボのみ対応。RAM上に展開されたデータをROMに保存します。
<details>
<summary>saveRom</summary>

```
void saveRom(uint8_t id);
```
* 引数
  * id : サーボID
* 戻り値
  * なし
  </details>

---

### General
全サーボ共通の読み書き関数。アドレスは各社サーボモータのマニュアルを参照。
</details>
<details>
<summary>readMemory</summary>

```
uint32_t readMemory(uint8_t id, uint16_t address, uint8_t length, bool async = false);
```
* 引数
  * id : サーボID
  * address : 読み込み対象アドレス
  * length : 読み込みバイト数
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * 対象アドレスのデータ。非同期モードの場合は常に0が返される。
</details>
<details>
<summary>writeMemory</summary>

```
CommResult writeMemory(uint8_t id, uint16_t address, uint8_t length, uint32_t data, bool async = false);
```
* 引数
  * id : サーボID
  * address : 書き込み対象アドレス
  * length : 書き込みバイト数
  * data : 書き込みデータ
  * async : 非同期フラグ。trueの場合非同期通信モードで送信される。
* 戻り値
  * COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
</details>
<details>
<summary>attachCallback</summary>

```
void attachCallback(callbackType func);
```
readMemory関数で読み込んだデータのみこのコールバックが使用される。
</details>
<details>
<summary>attachWriteCallback</summary>

```
void attachWriteCallback(callbackType func);
```
タイムアウトが発生すると同期非同期関係なく使用される。
</details>
<details>
<summary>attachTimeoutCallback</summary>

```
void attachTimeoutCallback(callbackType func);
```
非同期でWriteを行った場合に使用される。
</details>

---

### System
<details>
<summary>listener</summary>

非同期読み込みをした際別スレッドなどで通信待機をする関数。
```
void listener(void);
```
* 引数
  * なし
* 戻り値
  * なし
  </details>

---

### init
シリアルポート等、サーボに必要な機能の初期化。
```
void init(void);
```
* 引数
  * なし
* 戻り値
  * なし
### close
シリアルポートを閉じる。
```
void close(void);
```
* 引数
  * なし
* 戻り値
  * なし
### changeTimeout
```
void changeTimeout(uint16_t ms)；
```
* 引数
  * ms : 通信のタイムアウト時間（単位 : ms）
* 戻り値
  * なし
## License
Generic Serial-bus Servo Driver library uses Apache License 2.0.
