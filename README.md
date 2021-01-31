## 使用方法
* crc16.h
* gs2d_driver.h
* gs2d_type.h
* gs2d_command.h
* gs2d_b3m.h
* gs2d_futaba.h
* gs2d_krs.h
* gs2d_robotis.h
以上8ファイルをプロジェクトに含める。
gs2d_serial.hの形式に合わせたクラスを用意。
## 利用例

### ID1のサーボモーターを左右に動かす
```
#include <iostream.h>
#include "gs2d_robotis_2_0.h"
#include "TemplateSerial.h"

using namespace gs2d;

int main(void)
{
    Driver *servo = new RobotisP20<TemplateSerial>();

    // ID1のトルクをON
    servo->writeTorqueEnable(id, 1);

    // ID1を90度の位置に移動
    servo->writeTargetPosition(1, 90.0);

    // ID1を-90度の位置に移動
    servo->writeTargetPosition(1, -90.0);
}
```
### ID1の現在温度の読み込み（同期）
```
#include <iostream.h>
#include "gs2d_robotis_2_0.h"
#include "TemplateSerial.h"

using namespace gs2d;

int main(void)
{
    Driver *servo = new RobotisP20<TemplateSerial>();

    // ID1のサーボの温度を読み込み
    int temperature = servo->readTemperature(1);
    std::cout << temperature << std::endl;
}
```

### ID1の現在温度の読み込み（非同期）
```
#include <iostream.h>
#include "gs2d_robotis_2_0.h"
#include "TemplateSerial.h"

using namespace gs2d;

void TemperatureCallback(CallbackEventArgs e)
{
    std::cout << (int32_t)e.data << std::endl;
}

int main(void)
{
    Driver *servo = new RobotisP20<TemplateSerial>();

    // ID1のサーボの温度を読み込み
    servo->readTemperature(1, TemperatureCallback);

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
        uint8_t status;
        EventDataType data;
};
```
* メンバ変数
  * data : イベントの固有データ。代入先によってuint32_t型かfloat型どちらかで返される。
  * id : イベント発生元サーボのID
  * status : イベント発生時のエラー状態

Generic Serial-bus Servo Driver library uses Apache License 2.0.
