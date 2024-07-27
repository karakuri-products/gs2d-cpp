/*
* @file    WindowsSerial.h
* @author
* @date    2020/04/05
* @brief   Arduino Serial Base Class
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>

/* Variables -----------------------------------------------------------------*/
	class ArduinoSerial
	{
	public:
		ArduinoSerial() {  }
		~ArduinoSerial() {}

		int open(void)
		{
      pinMode(2, OUTPUT);
      Serial1.begin(115200, SERIAL_8E1);
//      Serial1.begin(115200, SERIAL_8N1);
		}

		int isConnected(void)
		{
			return 1;
		}

		void close(void)
		{
		}
   
		int read(void)
		{
      if(Serial1.available())
      {
        return Serial1.read();        
      }else{
        return -1;
      }
		}

		int write(unsigned char* data, uint8_t size)
		{
      digitalWrite(2, HIGH);
		  int retval = Serial1.write(data, size);
      Serial1.flush();
      digitalWrite(2, LOW);
		}

		unsigned long long int time(void)
		{
			return millis();
		}
	};
