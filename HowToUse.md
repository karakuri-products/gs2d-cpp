### Windows�T���v���̎g�p���@
CN29��USB Type C�P�[�u���Őڑ����A
SW2�����ASW1���g�p����T�[�{�̒ʐM�����ɍ��킹�Đݒ�i�ʐ^��RS485�ݒ�j

[�ʐ^1��}��]

gs2d-cpp/samples/windows_sample/saclib-v3/saclib-v3.sln��Visual Studio�ŊJ����
WindowsSerial.h 29�s�ڂ�COM�|�[�g���ƁAmain.cpp 31�s�ڂ̃T�[�{�̎�ނ�ύX���Ď��s

### Ubuntu�T���v���̎g�p���@
CN29��USB Type C�P�[�u���Őڑ����A
SW2�����ASW1���g�p����T�[�{�̒ʐM�����ɍ��킹�Đݒ肷��i�ʐ^��RS485�ݒ�j

[�ʐ^1��}��]

gs2d-cpp/samples/linux_sample/thread ��C�ӂ̏ꏊ�ɃR�s�[
LinuxSerial.h 56�s�ڂ�COM�|�[�g���ƁAmain.cpp 48�s�ڂ̃T�[�{�̎�ނ�ύX
�ȉ��̃R�}���h�����s���ăT�[�{�𓮂���
```
cd thread
mkdir build
cd build
cmake ..
make
./gs2d
```

### Arduino Leonardo�T���v���̎g�p���@
Arduino�p�R�l�N�^���g�p��Leonardo�Ɛڑ����A
SW2���Z�ASW1���g�p����T�[�{�̒ʐM�����ɍ��킹�Đݒ肷��i�ʐ^��RS485�ݒ�j

[�ʐ^2��}��]

��w�ʂ̃W�����p��H���܂���S���ɃV���[�g����B
H���ɐڑ������CN27�̃n�[�h�E�F�A�V���A���s���iRX : 1, TX : 2�j
S���ɐڑ������CN27�̃\�t�g�E�F�A�V���A���s���iRX : 4, TX : 5�j
���g�p�����B�T���v���ł̓n�[�h�E�F�A�V���A�����g�p����Ă��邽�߁AH���֐ڑ�����B

[�ʐ^3��}��]

gs2d-cpp/samples/arduino_sample/sketch_apr21a���J���ALeonardo��Ŏ��s�B

### ���̑��}�C�R�����Ŏg�p����ꍇ
SW2���Z�ASW1���g�p����T�[�{�̒ʐM�����ɍ��킹�Đݒ肷��i�ʐ^��RS485�ݒ�j

[�ʐ^2��}��]

��w�ʂ̃W�����p��H���܂���S���ɃV���[�g����B
H���ɐڑ������CN27�̃n�[�h�E�F�A�V���A���s���iRX : 1, TX : 2�j
S���ɐڑ������CN27�̃\�t�g�E�F�A�V���A���s���iRX : 4, TX : 5�j
���g�p�����B

[�ʐ^3��}��]

CN27��TX, RX, DE�ACN26��IOREF, GND�̍��v5�s����ڑ�����B

[�ʐ^4��}��]

gs2d-cpp/gs2d/template/TemplateSerial.h��USER CODE�����Ɋ��ɍ��킹��UART������L�q����B
�L�q���K�v�Ȋ֐��͈ȉ���6��
* open
* close
* isConnected
* read
* write
* time

gs2d-cpp/gs2d/template/main.h��USER CODE�����Ɋ��ɍ��킹�đ�����L�q����B
�L�q���K�v�ȉӏ��͈ȉ���2��
* TimeoutCallback
* main

���ɍ��킹�ăr���h���A�T�[�{����]���邱�Ƃ��m�F�ł���Ί����B
