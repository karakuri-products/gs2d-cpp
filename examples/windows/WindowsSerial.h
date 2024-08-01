/*
* @file    WindowsSerial.h
* @author
* @date    2019/12/26
* @brief   Windows Serial Base Class
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

/* Variables -----------------------------------------------------------------*/
namespace gs2d
{
	class WindowsSerial
	{
	private:
		HANDLE handler;
		bool connect_flag = false;

	public:
		WindowsSerial() {}
		~WindowsSerial() {}

		int open(void)
		{
			handler = CreateFileA("\\\\.\\COM3", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
			if (handler != INVALID_HANDLE_VALUE) {
				DCB dcbParam = { 0 };
				SetupComm(handler, 1024, 1024);

				if (GetCommState(handler, &dcbParam)) {
					dcbParam.BaudRate = CBR_115200;
					dcbParam.ByteSize = 8;
					dcbParam.StopBits = ONESTOPBIT;
					dcbParam.Parity = NOPARITY; 
//					dcbParam.Parity = EVENPARITY;

					dcbParam.fDtrControl = DTR_CONTROL_DISABLE;
					dcbParam.fRtsControl = RTS_CONTROL_DISABLE;
					dcbParam.fOutxDsrFlow = FALSE;
					dcbParam.fOutxCtsFlow = FALSE;
					dcbParam.fDsrSensitivity = FALSE;

					dcbParam.fBinary = TRUE;
					dcbParam.fNull = FALSE;
					dcbParam.fErrorChar = FALSE;
					dcbParam.fOutX = FALSE;
					dcbParam.fInX = FALSE;
				}


				if (SetCommState(handler, &dcbParam))
				{
					PurgeComm(handler, PURGE_RXCLEAR | PURGE_TXCLEAR);
					connect_flag = true;
				}
			}

			return 0;
		}

		int isConnected(void)
		{
			return connect_flag ? 1 : 0;
		}

		void close(void)
		{
			CloseHandle(handler);
		}

		int read(unsigned char* data, int size)
		{
			DWORD readbytes;
			unsigned int num = 0;
			COMSTAT status;
			DWORD error;

			ClearCommError(handler, &error, &status);

			if (status.cbInQue > 0) {
				if (status.cbInQue > size) num = size;
				else num = status.cbInQue;

				memset(data, 0, size);
				if (ReadFile(handler, data, num, &readbytes, NULL)) return readbytes;
			}

			return -1;
		}

		int read(void)
		{
			uint8_t data;
			if (read(&data, 1) != -1) return data;
			return -1;
		}

		int write(unsigned char* data, int size)
		{
			DWORD sendbytes;
			COMSTAT status;
			DWORD error;

			if (!WriteFile(handler, (void*)data, size, &sendbytes, 0)) {
				ClearCommError(handler, &error, &status);
				return 0;
			}
			return 1;
		}

		unsigned long long int time(void)
		{
			return GetTickCount();
		}
	};
}