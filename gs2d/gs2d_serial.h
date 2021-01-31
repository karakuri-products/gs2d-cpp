/*
* @file    gs2d_serial.h
* @author
* @date    2021/01/26
* @brief   Serial Base Class
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
/* USER INCLUDE CODE START */

// TODO : �V���A������ɕK�v�ȃt�@�C�����C���N���[�h

/* USER INCLUDE CODE END */

/* Variables -----------------------------------------------------------------*/
class GS2DSerial
{
public:
	GS2DSerial() {}
	~GS2DSerial() {}

	int open(void)
	{
		/* USER OPEN CODE START */

		// TODO : �|�[�g���J���BIO�̏�������
		// �߂�l�͌��ݎg���Ă��Ȃ��̂�0

		/* USER OPEN CODE END */
		return 0;
	}

	void close(void)
	{
		/* USER CLOSE CODE START */

		// �|�[�g����鏈��

		/* USER CLOSE CODE END */
		return;
	}

	int isConnected(void)
	{
		/* USER IS_CONNECTED CODE START */

		// ���݂̐ڑ���Ԃ�Ԃ��B
		// �߂�l -> �J����Ă���Ƃ� : 1, �����Ă��鎞 : 0

		/* USER IS_CONNECTED CODE END */
		return 0;
	}

	int read(void)
	{
		/* USER READ CODE START */

		// TODO : 1�o�C�g�ǂݍ��݁B�o�b�t�@�Ƀf�[�^�������ꍇ��-1��Ԃ��B

		/* USER READ CODE END */

		return -1;
	}

	int write(unsigned char* data, unsigned char size)
	{
		/* USER WRITE CODE START */

		// TODO : �w�肳�ꂽ�o�C�g���̏������݁BTXDEN�s���̐���
		// �߂�l�͎g���Ă��Ȃ�����0

		/* USER WRITE CODE END */
		return 0;
	}

	unsigned long long int time(void)
	{
		/* USER TIME CODE START */

		// TODO : �N�����Ă���̎��Ԃ�ms�P�ʂŕԂ��B
		// �s���m�ȏꍇ�^�C���A�E�g�����������삵�Ȃ�

		/* USER TIME CODE END */
		return 0;
	}
};
