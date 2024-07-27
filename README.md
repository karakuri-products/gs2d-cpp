## �g�p���@

* gs2d_util_crc16.h
* gs2d_driver.h
* gs2d_type.h
* gs2d_command.h
* gs2d_b3m.h
* gs2d_futaba.h
* gs2d_krs.h
* gs2d_robotis.h

�ȏ�8�t�@�C�����v���W�F�N�g�Ɋ܂߂�B
gs2d_serial.h�̌`���ɍ��킹���N���X��p�ӁB

## ���p��

### ID1�̃T�[�{���[�^�[�����E�ɓ�����
```cpp
#include <iostream.h>
#include "gs2d_robotis_2_0.h"
#include "TemplateSerial.h"

using namespace gs2d;

int main(void)
{
    Driver *servo = new RobotisP20<TemplateSerial>();

    // ID1�̃g���N��ON
    servo->writeTorqueEnable(id, 1);

    // ID1��90�x�̈ʒu�Ɉړ�
    servo->writeTargetPosition(1, 90.0);

    // ID1��-90�x�̈ʒu�Ɉړ�
    servo->writeTargetPosition(1, -90.0);
}
```

### ID1�̌��݉��x�̓ǂݍ��݁i�����j

```cpp
#include <iostream.h>
#include "gs2d_robotis_2_0.h"
#include "TemplateSerial.h"

using namespace gs2d;

int main(void)
{
    Driver *servo = new RobotisP20<TemplateSerial>();

    // ID1�̃T�[�{�̉��x��ǂݍ���
    int temperature = servo->readTemperature(1);
    std::cout << temperature << std::endl;
}
```

### ID1�̌��݉��x�̓ǂݍ��݁i�񓯊��j

```cpp
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

    // ID1�̃T�[�{�̉��x��ǂݍ���
    servo->readTemperature(1, TemperatureCallback);

    // ��M�ҋ@
    while(1) servo->listener();
}
```

## API
### Type
�R�[���o�b�N�֐��̈����Ɏg����\���̂ł��B
�S�ẴR�[���o�b�N�֐������̌^�œ��ꂳ��Ă��܂��B

```cpp
class CallbackEventArgs
{
        uint8_t id;
        uint8_t status;
        EventDataType data;
};
```

* �����o�ϐ�
  * data : �C�x���g�̌ŗL�f�[�^�B�����ɂ����uint32_t�^��float�^�ǂ��炩�ŕԂ����B
  * id : �C�x���g�������T�[�{��ID
  * status : �C�x���g�������̃G���[���

Generic Serial-bus Servo Driver library uses Apache License 2.0.
