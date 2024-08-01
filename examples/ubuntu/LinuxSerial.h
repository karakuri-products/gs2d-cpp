/*
* @file    LinuxSerial.h
* @author
* @date    2019/12/26
* @brief   Linux Serial Base Class
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <dirent.h>
#include <sys/types.h>
#include <linux/serial.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <libgen.h>
#include <linux/limits.h>
#include <termios.h>

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <algorithm>

/* Variables -----------------------------------------------------------------*/
namespace gs2d
{
	int local_open(const char* file, int flag) { return open(file, flag); }
	int local_close(int fd) { return close(fd); }
	int local_read(int fd, void* buff, unsigned long size) 
	{
		int available_size = 0;
		ioctl(fd, FIONREAD, &available_size);

		if (available_size == 0) return -1;

		read(fd, buff, size);
		return 1;
	}
	int local_write(int fd, void* buff, unsigned long size) { return (int)write(fd, buff, size); }

	class LinuxSerial
	{
	private:
		struct termios tms;
		struct timeval start;
		bool connect_flag = false;
		int fd;

	public:
		LinuxSerial(){}
		int open()
		{ 
			fd = local_open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
			if (fd < 0) { return -1; }

			// old
			tcgetattr(fd, &tms);
			tms.c_iflag = 0;
			tms.c_oflag = 0;
			tms.c_lflag = 0;
			tms.c_cc[VMIN] = 1;
			tms.c_cc[VTIME] = 0;

			cfmakeraw(&tms);

			// baudrate
			cfsetispeed(&tms, 115200);
			cfsetospeed(&tms, 115200);

			tms.c_cflag += CREAD;

			// Parity
			tms.c_cflag &= ~PARENB;
			tms.c_cflag &= ~PARODD;

			// Data Bits
			tms.c_cflag &= ~CSIZE;
			tms.c_cflag |= CS8;

			// Stop Bits
			tms.c_cflag &= ~CSTOPB;

			tcsetattr(fd, TCSANOW, &tms);
			tcflush(fd, TCIOFLUSH);

			gettimeofday(&start, NULL);

			connect_flag = true;
			return true;
		}

		int isConnected(void)
		{
			return 1;
		}

		void close(void)
		{
			tcsetattr(fd, TCSANOW, &tms);
			local_close(fd);
			connect_flag = false;
		}

		int read(void)
		{
			unsigned char data;
			if(local_read(fd, &data, 1) != -1) return data;
			return -1;
		}

		int write(unsigned char *data, int size)
		{
			return local_write(fd, data, size);
		}

		unsigned long long int time(void)
		{
			struct timeval now;
			long seconds, useconds, mtime;			

			gettimeofday(&now, NULL);
			
			seconds = now.tv_sec - start.tv_sec;
			useconds = now.tv_usec - start.tv_usec;

			mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
			return mtime;
		}
		
	};
}
