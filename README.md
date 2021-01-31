## �g�p���@
* crc16.h
* gs2d_b3m.h
* gs2d_base.h
* gs2d_buffer_type.h
* gs2d_futaba.h
* gs2d_krs.h
* gs2d_parameter.h
* gs2d_robotis_2_0.h
* TemplateSerial.h
�ȏ�9�t�@�C�����v���W�F�N�g�Ɋ܂߂�B
TemplateSerial��USER CODE���Ƀn�[�h�E�F�A�ɍ��킹���R�[�h���L���B
## ���p��

### ID1�̃T�[�{���[�^�[�����E�ɓ�����
```
#include <iostream.h>
#include <gs2d_robotis_2_0.h>
#include <TemplateSerial.h>

using namespace gs2d;

int main(void)
{
    ServoBase *servo = new RobotisP20<TemplateSerial>();

    // �T�[�{������
    servo->init();

    // ID1�̃g���N��ON
    servo->writeTorque(1, 1);

    // ID1��90�x�̈ʒu�Ɉړ�
    servo->writeTargetPosition(1, 90.0);

    // ID1��-90�x�̈ʒu�Ɉړ�
    servo->writeTargetPosition(1, -90.0);
}
```
### ID1�̌��݉��x�̓ǂݍ��݁i�����j
```
#include <iostream.h>
#include <gs2d_robotis_2_0.h>
#include <TemplateSerial.h>

using namespace gs2d;

int main(void)
{
    ServoBase *servo = new RobotisP20<TemplateSerial>();

    // �T�[�{������
    servo->init();

    // ID1�̃T�[�{�̉��x��ǂݍ���
    int temperature = servo->readTemperature(1);
    std::cout << temperature << std::endl;
}
```

### ID1�̌��݉��x�̓ǂݍ��݁i�񓯊��j
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

    // �T�[�{������
    servo->init();

    // �R�[���o�b�N�֐��̓o�^
    servo->attachTemperatureCallback(TemperatureCallback);

    // ID1�̃T�[�{�̉��x��ǂݍ���
    servo->readTemperature(1, true);

    // ��M�ҋ@
    while(1) servo->listener();
}
```

## API
### Type
�R�[���o�b�N�֐��̈����Ɏg����\���̂ł��B
�S�ẴR�[���o�b�N�֐������̌^�œ��ꂳ��Ă��܂��B
```
class CallbackEventArgs
{
        uint8_t id;
        int8_t status;
        uint16_t address;
        CallbackEventData data;
};
```
* �����o�ϐ�
  * data : �C�x���g�̌ŗL�f�[�^�B�����ɂ����uint32_t�^��float�^�ǂ��炩�ŕԂ����B
  * id : �C�x���g�������T�[�{��ID
  * error : �C�x���g�������̃G���[�ԍ�
  * address : �C�x���g�������T�[�{��ROM/RAM�A�h���X
#### Baudrate
Baudrate�ݒ莞�Ɏg�p�����Ή��{�[���[�g�񋓑̂ł��B
kr-SAC001�ł�3Mbps�܂őΉ����Ă��܂��B
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
�ʐM�̃G���[���e�̗񋓑̂ł��B
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
* CommSuccess : �ُ�Ȃ�
* ReadSuccess : �ǂݍ��ݐ���
* WriteSuccess : �������ݐ���
* CommTimeout : �ʐM�^�C���A�E�g
* CheckSumError : �`�F�b�N�T���s��v
* BufferIsEmpty : ���M�o�b�t�@����
* BufferIsFull : ���M�ҋ@�o�b�t�@�ɋ󂫖���
* FuncNotExist : �T�[�{���֐��̋@�\�ɑΉ����Ă��Ȃ�
* NoEEPROMData : EEPROM���ǂݍ��܂�Ă��Ȃ�����EEPROM�̏������݂��o���Ȃ��iKRS�V���[�Y��p�j
### Torque
#### readTorque
```
uint8_t readTorque(uint8_t id, bool async = false);
```
- ����
  - id : �T�[�{ID
  - async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����
- �߂�l
  - �g���N�l�B0 : Torque Off, 1 : Torque On�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B

#### writeTorque
```
CommResult writeTorque(uint8_t id, uint8_t torque, bool async = false);
```
* ����
  * id : �T�[�{ID
  * torque : 0 : Torque Off, 1 : Torque On
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT

#### attachTorqueCallback
```
void attachTorqueCallback(callbackType func)
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�

### Current Position
#### readPosition
```
float readPosition(uint8_t id, bool async = false);
```
- ����
  - id : �T�[�{ID
  - async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����
- �߂�l
  - �p�x�i�P�� : degree)�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B

#### attachPositionCallback
```
void attachPositionCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�

### Target Position
#### readTargetPosition
```
float readTargetPosition(uint8_t id, bool async = false);
```
- ����
  - id : �T�[�{ID
  - async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
- �߂�l
  - �ڕW�ʒu�i�P�� : degree�j�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B

#### writeTargetPosition
```
CommResult writeTargetPosition(uint8_t id, float position, bool async = false);
```
* ����
  * id : �T�[�{ID
  * position : �ڕW�ʒu�i�P�� : degree�j
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT

#### attachPositionCallback
```
void attachPositionCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�

### Temperature
#### readTemperature
```
uint16_t readTemperature(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * ���݂̉��x�i�P�� : degree�j�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B

#### attachTemperatureCallback
```
void attachTemperatureCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�

### Current
#### readCurrent
```
uint16_t readCurrent(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * ���דd���l�i�P�� : mA�j�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### attachCurrentCallback
```
void attachCurrentCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### Voltage
#### readVoltage
```
uint16_t readVoltage(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * ���͓d���i�P�� : mV�j�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### attachVoltageCallback
```
void attachVoltageCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### P Gain
#### readPGain
```
uint16_t readPGain(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * ���݂�P�Q�C���l�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### writePGain
```
CommResult writePGain(uint8_t id, uint16_t pGain, bool async = false);
```
* ����
  * id : �T�[�{ID
  * pGain : P�Q�C���l
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
#### attachPGainCallback
```
void attachPGainCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### I Gain
#### readIGain
```
uint16_t readIGain(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * ���݂�I�Q�C���l�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### writeIGain
```
CommResult writeIGain(uint8_t id, uint16_t iGain, bool async = false);
```
* ����
  * id : �T�[�{ID
  * iGain : I�Q�C���l
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
#### attachIGainCallback
```
void attachIGainCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### D Gain
#### readDGain
```
uint16_t readDGain(uint8_t id, bool async = false)
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * ���݂�D�Q�C���l�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### writeDGain
```
CommResult writeDGain(uint8_t id, uint16_t dGain, bool async = false);
```
* ����
  * id : �T�[�{ID
  * dGain : D�Q�C���l
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
#### attachDGainCallback
```
void attachDGainCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### ID
#### readID
```
uint8_t readID(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * �ΏۃT�[�{��ID�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### writeID
```
CommResult writeID(uint8_t id, uint8_t newid, bool async = false);
```
* ����
  * id : �T�[�{ID
  * newID : �V�����T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
#### attachIDCallback
```
void attachIDCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### Baudrate
#### readBaudrate
```
BaudrateList readBaudrate(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * �ΏۃT�[�{�̒ʐM�{�[���[�g�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### writeBaudrate
```
CommResult writeBaudrate(uint8_t id, BaudrateList baudrate, bool async = false);
```
* ����
  * id : �T�[�{ID
  * newID : �V�����T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
#### attachBaudrateCallback
```
void attachBaudrateCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### Offset
#### readOffset
```
float readOffset(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * �I�t�Z�b�g�l�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### writeOffset
```
CommResult writeOffset(uint8_t id, float offset, bool async = false);
```
* ����
  * id : �T�[�{ID
  * offset : �I�t�Z�b�g�l
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
#### attachOffsetCallback
```
void attachOffsetCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### Deadband
#### readDeadband
```
float readDeadband(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * �f�b�h�o���h�l�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### writeDeadband
```
CommResult writeDeadband(uint8_t id, float deadband, bool async = false);
```
* ����
  * id : �T�[�{ID
  * deadband : �f�b�h�o���h�l
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
#### attachDeadbandCallback
```
void attachDeadbandCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### CW Position Limit 
#### readCWLimit
```
float readCWLimit(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * ���v�������̉�]�p�̐����B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### writeCWLimit
```
CommResult writeCWLimit(uint8_t id, float cwLimit, bool async = false);
```
* ����
  * id : �T�[�{ID
  * cwLimit : ���v�������̉�]�p�̐���
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
#### attachCWLimitCallback
```
void attachCWLimitCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### CCW Position Limit 
#### readCCWLimit
```
float readCCWLimit(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * �����v�������̉�]�p�̐����B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### writeCCWLimit
```
CommResult writeCCWLimit(uint8_t id, float ccwLimit, bool async = false);
```
* ����
  * id : �T�[�{ID
  * ccwLimit : �����v�������̉�]�p�̐���
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
#### attachCCWLimitCallback
```
void attachCCWLimitCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### Temperature Limit
#### readTemperatureLimit
```
uint16_t readTemperatureLimit(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * ���x���~�b�g�l�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### writeTemperatureLimit
```
writeTemperatureLimit(uint8_t id, uint16_t tempLimit, bool async = false);
```
* ����
  * id : �T�[�{ID
  * tempLimit : ���x���~�b�g�l
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
#### attachTemperatureLimitCallback
```
void attachTemperatureLimitCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### Current Limit
#### readCurrentLimit
```
uint16_t readCurrentLimit(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * �d�����~�b�g�l�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### writeCurrentLimit
```
CommResult writeCurrentLimit(uint8_t id, uint16_t currentLimit, bool async = false);
```
* ����
  * id : �T�[�{ID
  * currentLimit : �d�����~�b�g�l
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
#### attachCurrentLimitCallback
```
void attachCurrentLimitCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### Speed
#### readSpeed
```
uint16_t readSpeed(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * �ړ����x�l�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### writeSpeed
```
CommResult writeSpeed(uint8_t id, uint16_t speed, bool async = false);
```
* ����
  * id : �T�[�{ID
  * speed : �ڕW�X�s�[�h�l
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
#### attachSpeedCallback
```
void attachSpeedCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### Acceleration
#### readAcceleration
```
uint16_t readAcceleration(uint8_t id, bool async = false);
```
* ����
  * id : �T�[�{ID
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * �����x�l�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### writeAcceleration
```
CommResult writeAcceleration(uint8_t id, uint16_t accel, bool async = false);
```
* ����
  * id : �T�[�{ID
  * accel : �����x�l
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * FUNC_NOT_EXIST, COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
#### attachAccelCallback
```
void attachAccelCallback(callbackType func);
```
* ����
  * func : �R�[���o�b�N�֐�
* �߂�l
  * �Ȃ�
### Burst R/W Position
�����̃T�[�{��Position�𓯎��ɓǂݏ�������@�\�ł��B
Read�̏ꍇ��Current Position���AWrite�̏ꍇ��Target Position���Q�Ƃ���܂��B
#### burstReadPosition
```
CommResult burstReadPosition(uint8_t* id_list, uint8_t num, float* position_list, bool async = false);
```
* ����
  * id_list : �T�[�{ID���i�[���ꂽ�z��
  * num : �ΏۃT�[�{��
  * position_list : �ʒu�f�[�^���i�[�����z��
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * 0 or READ_SUCCESS or FUNC_NOT_EXIST

#### burstWritePosition
```
CommResult burstWriteTargetPosition(uint8_t* id_list, uint8_t num, float* position_list, bool async = false);
```
* ����
  * id_list : �T�[�{ID���i�[���ꂽ�z��
  * num : �ΏۃT�[�{��
  * position_list : �ʒu�f�[�^���i�[���ꂽ�z��
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * 0 or FUNC_NOT_EXIST
### General Burst R/W
#### burstReadMemory
```
CommResult burstRead(uint8_t* id_list, uint16_t address, uint8_t num, uint8_t* data, bool async = false);
CommResult burstRead(uint8_t* id_list, uint16_t address, uint8_t num, uint16_t* data, bool async = false);
CommResult burstRead(uint8_t* id_list, uint16_t address, uint8_t num, uint32_t* data, bool async = false);
```
* ����
  * id_list : �T�[�{ID���i�[���ꂽ�z��
  * address : �ΏۃA�h���X
  * num : �ΏۃT�[�{��
  * data : �f�[�^���i�[�����z��
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * 0 or READ_SUCCESS or FUNC_NOT_EXIST
#### burstWriteMemory
```
CommResult burstWrite(uint8_t* id_list, uint16_t address, uint8_t num, uint8_t* data, bool async = false);
CommResult burstWrite(uint8_t* id_list, uint16_t address, uint8_t num, uint16_t* data, bool async = false);
CommResult burstWrite(uint8_t* id_list, uint16_t address, uint8_t num, uint32_t* data, bool async = false);
```
* ����
  * id_list : �T�[�{ID���i�[���ꂽ�z��
  * address : �ΏۃA�h���X
  * num : �ΏۃT�[�{��
  * data : �f�[�^���i�[���ꂽ�z��
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * 0 or FUNC_NOT_EXIST

### ROM
ROM�̕ۑ��@�\������T�[�{�̂ݑΉ��BRAM��ɓW�J���ꂽ�f�[�^��ROM�ɕۑ����܂��B
#### saveRom
```
void saveRom(uint8_t id);
```
* ����
  * id : �T�[�{ID
* �߂�l
  * �Ȃ�
### General
�S�T�[�{���ʂ̓ǂݏ����֐��B�A�h���X�͊e�ЃT�[�{���[�^�̃}�j���A�����Q�ƁB
#### readMemory
```
uint32_t readMemory(uint8_t id, uint16_t address, uint8_t length, bool async = false);
```
* ����
  * id : �T�[�{ID
  * address : �ǂݍ��ݑΏۃA�h���X
  * length : �ǂݍ��݃o�C�g��
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * �ΏۃA�h���X�̃f�[�^�B�񓯊����[�h�̏ꍇ�͏��0���Ԃ����B
#### writeMemory
```
CommResult writeMemory(uint8_t id, uint16_t address, uint8_t length, uint32_t data, bool async = false);
```
* ����
  * id : �T�[�{ID
  * address : �������ݑΏۃA�h���X
  * length : �������݃o�C�g��
  * data : �������݃f�[�^
  * async : �񓯊��t���O�Btrue�̏ꍇ�񓯊��ʐM���[�h�ő��M�����B
* �߂�l
  * COMM_SUCCESS, WRITE_SUCCESS, COMM_TIMEOUT
#### attachCallback
```
void attachCallback(callbackType func);
```
readMemory�֐��œǂݍ��񂾃f�[�^�݂̂��̃R�[���o�b�N���g�p�����B
#### attachWriteCallback
```
void attachWriteCallback(callbackType func);
```
�^�C���A�E�g����������Ɠ����񓯊��֌W�Ȃ��g�p�����B
#### attachTimeoutCallback
```
void attachTimeoutCallback(callbackType func);
```
�񓯊���Write���s�����ꍇ�Ɏg�p�����B
### System
#### listener
�񓯊��ǂݍ��݂������ەʃX���b�h�ȂǂŒʐM�ҋ@������֐��B
```
void listener(void);
```
* ����
  * �Ȃ�
* �߂�l
  * �Ȃ�
### init
�V���A���|�[�g���A�T�[�{�ɕK�v�ȋ@�\�̏������B
```
void init(void);
```
* ����
  * �Ȃ�
* �߂�l
  * �Ȃ�
### close
�V���A���|�[�g�����B
```
void close(void);
```
* ����
  * �Ȃ�
* �߂�l
  * �Ȃ�
### changeTimeout
```
void changeTimeout(uint16_t ms)�G
```
* ����
  * ms : �ʐM�̃^�C���A�E�g���ԁi�P�� : ms�j
* �߂�l
  * �Ȃ�
## License
Generic Serial-bus Servo Driver library uses Apache License 2.0.
