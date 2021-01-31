/*
* @file    gs2d_b3m.h
* @author
* @date    2021/01/26
* @brief
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "gs2d_driver.h"
#include "gs2d_command.h"
#include "crc16.h"

/* Variables -----------------------------------------------------------------*/
namespace gs2d
{
	template<class SerialClass, unsigned int bufferSize = 20, unsigned int commandSize = 50>
	class B3M : public CommandHandler<SerialClass, bufferSize, commandSize>, public Driver
	{
	private:
		// ��R�[���o�b�N���̃f�[�^�󂯓n���p
		EventDataType responseData;
		Gs2dType<bool> isReceived;

		// �C���X�g���N�V����
		struct Instructions
		{
			static const uint8_t Load = 0x01;
			static const uint8_t Save = 0x02;
			static const uint8_t Read = 0x03;
			static const uint8_t Write = 0x04;
			static const uint8_t Reset = 0x05;
			static const uint8_t Position = 0x06;
		};

		struct Address
		{
			static const uint8_t Id = 0x00;
			static const uint8_t Baudrate = 0x01;
			static const uint8_t Offset = 0x09;
			static const uint8_t Deadband = 0x1C;
			static const uint8_t Mode = 0x28;
			static const uint8_t TargetPosition = 0x2A;
			static const uint8_t CurrentPosition = 0x2C;
			static const uint8_t Speed = 0x30;
			static const uint8_t CurrentSpeed = 0x32;
			static const uint8_t TargetTime = 0x36;
			static const uint8_t Temperature = 0x46;
			static const uint8_t Current = 0x48;
			static const uint8_t Voltage = 0x4A;
			static const uint8_t PGain = 0x5E;
			static const uint8_t IGain = 0x62;
			static const uint8_t DGain = 0x66;

			static const uint8_t TemperatureLimit = 0x0E;
			static const uint8_t CCWLimit = 0x05;
			static const uint8_t CWLimit = 0x07;
			static const uint8_t CurrentLimit = 0x11;
		};

		// ��M�����m�F�֐�
		bool isComplete(uint8_t* data, uint8_t length)
		{
			if (length == 0) return false;
			return (length == data[0]);
		}

		// ID�`�F�b�N�֐�
		bool checkId(uint8_t id)
		{
			if (id < 0 || id > 254) return false;
			return true;
		}

		// CheckSum
		uint8_t calculateCheckSum(uint8_t* data, uint8_t length)
		{
			uint32_t sum = 0;
			for (uint8_t i = 0; i < length - 1; i++) sum += data[i];
			return (sum & 0xFF);
		}

		// ��M���̃R�[���o�b�N
		void dataReceivedEvent(uint8_t* data, uint8_t length, uint8_t status)
		{
			uint32_t tmp = 0;

			// �G���[�̕ۑ�
			this->errorBits |= status;
			
			do {
				// �G���[������ΏI��
				if (this->errorBits != 0) break;

				// �������m�F
				if (length != data[0]) { this->errorBits |= ResponseError; break; }

				// CheckSum����
				if (calculateCheckSum(data, length) != data[length - 1]) { this->errorBits |= ResponseError; break; }
			} while (false);

			// �G���[���N�����Ă���΋����I��
			if (this->errorBits != 0) {
				// �R�[���o�b�N�݂�
				if (this->currentCommand.callback) {
					CallbackEventArgs e(this->errorBits);
					this->currentCommand.callback(e);
					if (operatingMode) return;
				}
				responseData.set((int32_t)0);
				isReceived.set(true);
				return;
			}

			// Param������ΐ؂肾��
			if (length > 5) {
				for (int i = 0; i < length - 5; i++) {
					tmp += (data[4 + i] << (i * 8));
				}
			}

			// �f�[�^���������ďI��
			if (this->currentCommand.responseProcess) {
				if (this->currentCommand.callback) {
					CallbackEventArgs e(data[3], this->errorBits, this->currentCommand.responseProcess(tmp));
					this->currentCommand.callback(e);
					if (!operatingMode) isReceived.set(true);
				}
				else {
					responseData = (this->currentCommand.responseProcess(tmp));
					isReceived.set(true);
				}
			}
			else {
				if (this->currentCommand.callback) {
					CallbackEventArgs e(data[3], this->errorBits, (int32_t)tmp);
					this->currentCommand.callback(e);
					if (!operatingMode) isReceived.set(true);
				}
				else {
					responseData.set((int32_t)tmp);
					isReceived.set(true);
				}
			}
		}

		EventDataType getFunction(uint8_t id, uint8_t instruction, uint8_t* param = 0, uint8_t length = 0, ResponseProcess responseProcess = 0, CallbackType callback = 0, uint8_t count = 1)
		{
			// �o�C�g��̒������v�Z
			uint8_t bufferLength = 5 + length;

			// �o�C�g��𐶐�
			uint8_t* command = new uint8_t[bufferLength];

			// �w�b�_��ID��ݒ�
			command[0] = bufferLength; command[1] = instruction;
			command[2] = 0; command[3] = id;

			// �p�����[�^��}��
			if (length) memcpy(command + 4, param, length);

			// CheckSum
			command[bufferLength - 1] = calculateCheckSum(command, bufferLength);

			// ��}���`�X���b�h���[�h���̓R�[���o�b�N���g��Ȃ��ꍇ�󂫑҂�
			if (!operatingMode || callback == 0) {
				while (!this->isTrafficFree.get());
				this->isReceived.set(false);
			}

			// Clear Error
			this->errorBits = 0;

			// �R�}���h���M
			this->addCommand(command, bufferLength, responseProcess, callback, count);
			delete [] command;

			// �}���`�X���b�h���[�h�ŃR�[���o�b�N������ΔC���ďI��
			if (operatingMode && callback != 0 || count == 0) return EventDataType((int32_t)0);

			// ��M�҂�
			while (!isReceived.get())
			{
				if (!this->operatingMode) this->listener();
			}

			return responseData;
		}
		void spin() { this->listener(); }

		// ------------------------------------------------------------------------------------------
		static void defaultWriteCallback(CallbackEventArgs e) {  }

		static EventDataType pingProcess(int32_t data) { return EventDataType((int32_t)((data >> 8) & 0xFF)); }
		static EventDataType torqueProcess(int32_t data) { return EventDataType((int32_t)((data & 0b10) ? 0 : 1)); }
		static EventDataType temperatureProcess(int32_t data) { return EventDataType((int32_t)(data / 100)); }
		static EventDataType currentProcess(int32_t data) { return EventDataType((int32_t)(data)); }
		static EventDataType voltageProcess(int32_t data) { return EventDataType((gFloat)(data / 1000.0)); }
		static EventDataType positionProcess(int32_t data) { return EventDataType((gFloat)(-(int16_t)data / 100.0)); }
		static EventDataType deadbandProcess(int32_t data) { return EventDataType((gFloat)(data / 100.0)); }
		static EventDataType targetTimeProcess(int32_t data) { return EventDataType((gFloat)(data / 1000.0)); }
		static EventDataType speedProcess(int32_t data) { return EventDataType((gFloat)((int16_t)data / 100.0)); }
		static EventDataType positionLimitProcess(int32_t data) { return EventDataType((gFloat)(-(int16_t)data / 100.0)); }

		// ------------------------------------------------------------------------------------------
	public:
		B3M() : responseData(EventDataType((int32_t)0)), isReceived(false) {}
		~B3M() {}

		// ------------------------------------------------------------------------------------------
		// General
		uint32_t readMemory(uint8_t id, uint16_t address, uint8_t length, CallbackType callback)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ address, length };

			// ���M
			return (int32_t)getFunction(id, Instructions::Read, param, 2, 0, callback);
		}
		void writeMemory(uint8_t id, uint16_t address, uint32_t data, uint8_t length)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			// �p�����[�^����
			uint8_t param[6];
			for (uint8_t i = 0; i < length; i++) param[i] = (data >> (i * 8)) & 0xFF;
			param[length] = address;
			param[length + 1] = 1;

			// ���M
			getFunction(id, Instructions::Write, param, length + 2, 0, defaultWriteCallback);
		}

		// Ping
		uint16_t ping(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^
			uint8_t param[2]{ 0xA3, 2 };

			return (int32_t)getFunction(id, Instructions::Read, param, 2, pingProcess, callback);
		}

		// Torque
		uint8_t readTorqueEnable(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::Mode, 1 };

			return (int32_t)getFunction(id, Instructions::Read, param, 2, torqueProcess, callback);
		}
		void writeTorqueEnable(uint8_t id, uint8_t torque)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			// �p�����[�^����
			uint8_t param[3]{ (torque) ? 0b00 : 0b10, Address::Mode, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 3, 0, defaultWriteCallback);
		}

		// Temperature
		uint16_t readTemperature(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::Temperature, 2 };

			return (int32_t)getFunction(id, Instructions::Read, param, 2, temperatureProcess, callback);
		}

		// Current
		int32_t readCurrent(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::Current, 2 };

			return (int32_t)getFunction(id, Instructions::Read, param, 2, 0, callback);
		}

		// Voltage
		gFloat readVoltage(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::Voltage, 2 };

			return (gFloat)getFunction(id, Instructions::Read, param, 2, voltageProcess, callback);
		}

		// Target Position
		gFloat readTargetPosition(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::TargetPosition, 2 };

			return (gFloat)getFunction(id, Instructions::Read, param, 2, positionProcess, callback);
		}
		void writeTargetPosition(uint8_t id, gFloat position)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			uint16_t pos = (position * -100);

			// �p�����[�^����
			uint8_t param[4]{ pos & 0xFF, (pos >> 8) & 0xFF, Address::TargetPosition, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 4, 0, defaultWriteCallback);
		}

		// Current Position
		gFloat readCurrentPosition(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::CurrentPosition, 2 };

			return (gFloat)getFunction(id, Instructions::Read, param, 2, positionProcess, callback);
		}

		// Offset
		gFloat readOffset(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::Offset, 2 };

			return (gFloat)getFunction(id, Instructions::Read, param, 2, positionProcess, callback);
		}
		void writeOffset(uint8_t id, gFloat offset)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			int16_t pos = (offset * -100);

			// �p�����[�^����
			uint8_t param[4]{ pos & 0xFF, (pos >> 8) & 0xFF, Address::Offset, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 4, 0, defaultWriteCallback);
		}

		// Deadband
		gFloat readDeadband(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::Deadband, 2 };

			return (gFloat)getFunction(id, Instructions::Read, param, 2, deadbandProcess, callback);
		}
		void writeDeadband(uint8_t id, gFloat deadband)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			uint16_t pos = (deadband * 100);

			// �p�����[�^����
			uint8_t param[4]{ pos & 0xFF, (pos >> 8) & 0xFF, Address::Deadband, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 4, 0, defaultWriteCallback);
		}

		// Target Time
		gFloat readTargetTime(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::TargetTime, 2 };

			return (gFloat)getFunction(id, Instructions::Read, param, 2, targetTimeProcess, callback);
		}
		void writeTargetTime(uint8_t id, gFloat targetTime)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			uint16_t time = (targetTime * 1000);

			// �p�����[�^����
			uint8_t param[4]{ time & 0xFF, (time >> 8) & 0xFF, Address::TargetTime, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 4, 0, defaultWriteCallback);
		}

		// Accel Time
		gFloat readAccelTime(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeAccelTime(uint8_t id, gFloat accelTime) { notSupport(); }

		// P Gain
		uint32_t readPGain(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::PGain, 4 };

			return (int32_t)getFunction(id, Instructions::Read, param, 2, 0, callback);
		}
		void writePGain(uint8_t id, uint32_t gain)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			// �p�����[�^����
			uint8_t param[6]{ gain & 0xFF, (gain >> 8) & 0xFF, (gain >> 16) & 0xFF, (gain >> 24) & 0xFF, Address::PGain, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 6, 0, defaultWriteCallback);
		}

		// I Gain
		uint32_t readIGain(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::IGain, 4 };

			return (int32_t)getFunction(id, Instructions::Read, param, 2, 0, callback);
		}
		void writeIGain(uint8_t id, uint32_t gain)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			// �p�����[�^����
			uint8_t param[6]{ gain & 0xFF, (gain >> 8) & 0xFF, (gain >> 16) & 0xFF, (gain >> 24) & 0xFF, Address::IGain, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 6, 0, defaultWriteCallback);
		}

		// D Gain
		uint32_t readDGain(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::DGain, 4 };

			return (int32_t)getFunction(id, Instructions::Read, param, 2, 0, callback);
		}
		void writeDGain(uint8_t id, uint32_t gain)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			// �p�����[�^����
			uint8_t param[6]{ gain & 0xFF, (gain >> 8) & 0xFF, (gain >> 16) & 0xFF, (gain >> 24) & 0xFF, Address::DGain, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 6, 0, defaultWriteCallback);
		}

		// Max Torque
		uint32_t readMaxTorque(uint8_t id, CallbackType callback = 0) { notSupport(); return 0; }
		void writeMaxTorque(uint8_t id, uint32_t maxTorque) { notSupport(); }

		// Speed
		gFloat readSpeed(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::CurrentSpeed, 4 };

			return (gFloat)getFunction(id, Instructions::Read, param, 2, speedProcess, callback);
		}
		void writeSpeed(uint8_t id, gFloat speed)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			int speedInt = speed * 100;

			// �p�����[�^����
			uint8_t param[6]{ speedInt & 0xFF, (speedInt >> 8) & 0xFF, Address::Speed, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 4, 0, defaultWriteCallback);
		}

		// ID
		uint32_t readID(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::Id, 1 };

			return (int32_t)getFunction(id, Instructions::Read, param, 2, 0, callback);
		}
		void writeID(uint8_t id, uint32_t newid)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }
			if (!checkId(newid)) { badInput(); return ; }

			// �p�����[�^����
			uint8_t param[6]{ newid, Address::Id, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 3, 0, defaultWriteCallback);
		}

		// ROM
		void saveRom(uint8_t id)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			getFunction(id, Instructions::Save, 0, 0, 0, defaultWriteCallback);
		}
		void loadRom(uint8_t id)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			getFunction(id, Instructions::Load, 0, 0, 0, defaultWriteCallback);
		}

		void resetMemory(uint8_t id) { notSupport(); return ; }

		// Baudrate
		uint32_t readBaudrate(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::Baudrate, 4 };

			return (int32_t)getFunction(id, Instructions::Read, param, 2, 0, callback);
		}
		void writeBaudrate(uint8_t id, uint32_t baudrate)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			// �p�����[�^����
			uint8_t param[6]{ baudrate & 0xFF, (baudrate >> 8) & 0xFF, (baudrate >> 16) & 0xFF, (baudrate >> 24) & 0xFF, Address::Baudrate, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 6, 0, defaultWriteCallback);
		}

		// CW Limit Position
		gFloat readLimitCWPosition(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::CWLimit, 2 };

			return (gFloat)getFunction(id, Instructions::Read, param, 2, positionLimitProcess, callback);
		}
		void writeLimitCWPosition(uint8_t id, gFloat limitPosition)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			uint16_t limitInt = (-limitPosition * 100.0);

			// �p�����[�^����
			uint8_t param[4]{ limitInt & 0xFF, (limitInt >> 8) & 0xFF, Address::CWLimit, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 4, 0, defaultWriteCallback);
		}

		// CCW Limit Position
		gFloat readLimitCCWPosition(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::CCWLimit, 2 };

			return (gFloat)getFunction(id, Instructions::Read, param, 2, positionLimitProcess, callback);
		}
		void writeLimitCCWPosition(uint8_t id, gFloat limitPosition)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			uint16_t limitInt = -limitPosition * 100.0;

			// �p�����[�^����
			uint8_t param[4]{ limitInt & 0xFF, (limitInt >> 8) & 0xFF, Address::CCWLimit, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 4, 0, defaultWriteCallback);
		}

		// Temperature Limit
		uint32_t readLimitTemperature(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::TemperatureLimit, 2 };

			return (gFloat)getFunction(id, Instructions::Read, param, 2, temperatureProcess, callback);
		}
		void writeLimitTemperature(uint8_t id, uint32_t temperature)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			int limitInt = temperature * 100.0;

			// �p�����[�^����
			uint8_t param[4]{ limitInt & 0xFF, (limitInt >> 8) & 0xFF, Address::TemperatureLimit, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 4, 0, defaultWriteCallback);
		}

		// Curent Limit
		uint32_t readLimitCurrent(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::CurrentLimit, 2 };

			return (int32_t)getFunction(id, Instructions::Read, param, 2, currentProcess, callback);
		}
		void writeLimitCurrent(uint8_t id, uint32_t current)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return ; }

			uint16_t limitInt = current;

			// �p�����[�^����
			uint8_t param[4]{ limitInt & 0xFF, (limitInt >> 8) & 0xFF, Address::CurrentLimit, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 4, 0, defaultWriteCallback);
		}

		// Drive Mode
		uint32_t readDriveMode(uint8_t id, CallbackType callback = 0)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return 0; }

			// �p�����[�^����
			uint8_t param[2]{ Address::Mode, 1 };

			return (gFloat)getFunction(id, Instructions::Read, param, 2, 0, callback);
		}
		void writeDriveMode(uint8_t id, uint32_t mode)
		{
			// ID�`�F�b�N
			if (!checkId(id)) { badInput(); return; }

			// �p�����[�^����
			uint8_t param[3]{ mode, Address::Mode, 1 };

			// ���M
			getFunction(id, Instructions::Write, param, 3, 0, defaultWriteCallback);
		}

		// Burst Function
		void burstReadMemory(uint8_t* idList, uint8_t count, uint16_t address, uint8_t length, CallbackType callback) { notSupport(); }
		void burstWriteMemory(uint8_t* idList, uint32_t* dataList, uint8_t count, uint16_t address, uint8_t length)
		{
			uint8_t* param = new uint8_t[(length + 1) * (count) + 1];
			uint8_t pos = 0;

			for (int i = 0; i < count; i++) {
				if (i != 0) param[pos++] = idList[i];
				
				for (int k = 0; k < length; k++) {
					param[pos++] = (dataList[i] >> (k * 8)) & 0xFF;
				}
			}

			param[pos++] = address;
			param[pos] = count;

			getFunction(idList[0], Instructions::Write, param, (length + 1) * (count) + 1, 0, 0, (count == 1) ? 1 : 0);
			delete[] param;
		}

		// Burst Function(Position)
		void burstReadPositions(uint8_t* idList, uint8_t count, CallbackType callback) { notSupport(); }
		void burstWriteTargetPositions(uint8_t* idList, gFloat* positionList, uint8_t count)
		{
			uint8_t* param = new uint8_t[3 * (count) + 1];
			uint8_t pos = 0;

			for (int i = 0; i < count; i++) {
				if (i != 0) param[pos++] = idList[i];

				int16_t position = positionList[i] * -100.0;
				param[pos++] = (position & 0xFF);
				param[pos++] = ((position >> 8) & 0xFF);
			}

			param[pos++] = Address::TargetPosition;
			param[pos] = count;

			getFunction(idList[0], Instructions::Write, param, 3 * (count) + 1, 0, 0, (count == 1) ? 1 : 0);
			delete[] param;
		}
    };
}
