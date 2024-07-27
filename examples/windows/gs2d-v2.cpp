// gs2d-v2.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//

#include <iostream>

#include "gs2d_robotis.h"
#include "gs2d_krs.h"
#include "gs2d_b3m.h"
#include "gs2d_futaba.h"
#include "WindowsSerial.h"

using namespace gs2d;

void burstReadCallback(CallbackEventArgs arg)
{
    std::cout << "Burst Read Result -> id:" << (int)arg.id << ", position:" << (gFloat)arg.data << std::endl;
}

int main()
{
    uint8_t id = 2;

    Driver* servo = new RobotisP20<gs2d::WindowsSerial>();
//    Driver* servo = new Futaba<gs2d::WindowsSerial>();
//    Driver* servo = new KRS<gs2d::WindowsSerial>();
//    Driver* servo = new B3M<gs2d::WindowsSerial>();

    // Ping
    std::cout << "Ping : " << (int)servo->ping(id) << std::endl;

    // Offset
    std::cout << "Offset Write : " << std::endl; servo->writeOffset(id, 3.0);
    std::cout << "Offset Read : " << servo->readOffset(id) << std::endl;

    // Deadband
    std::cout << "Deadband Write : " << std::endl; servo->writeDeadband(id, 3.0);
    std::cout << "Deadband Read : " << servo->readDeadband(id) << std::endl;

    // P Gain
    std::cout << "PGain Write : " << std::endl; servo->writePGain(id, 800);
    std::cout << "PGain Read : " << servo->readPGain(id) << std::endl;

    // I Gain 
    std::cout << "IGain Write : " << std::endl; servo->writeIGain(id, 400);
    std::cout << "IGain Read : " << servo->readIGain(id) << std::endl;

    // D Gain 
    std::cout << "DGain Write : " << std::endl; servo->writeDGain(id, 300);
    std::cout << "DGain Read : " << servo->readDGain(id) << std::endl;

    // Max Torque
    std::cout << "MaxTorque Write : " << std::endl; servo->writeMaxTorque(id, 80);
    std::cout << "MaxTorque Read : " << servo->readMaxTorque(id) << std::endl;

    // ID
//    std::cout << "ID Write : " << std::endl; servo->writeID(id, id);
    std::cout << "ID Read : " << servo->readID(id) << std::endl;

    // ROM
    std::cout << "Save ROM : " << std::endl; servo->saveRom(id);
    std::cout << "Load ROM : " << std::endl; servo->loadRom(id);
    std::cout << "ResetMemory : " << std::endl; servo->resetMemory(id);

    Sleep(1000);

    // Baudrate
    std::cout << "Baudrate Write : " << std::endl; servo->writeBaudrate(id, 115200);
    std::cout << "Baudrate Read : " << servo->readBaudrate(id) << std::endl;

    // CW Limit
    std::cout << "CW Limit Write : " << std::endl; servo->writeLimitCWPosition(id, -135);
    std::cout << "CW Limit Read : " << servo->readLimitCWPosition(id) << std::endl;

    // CCW Limit
    std::cout << "CCW Limit Write : " << std::endl; servo->writeLimitCCWPosition(id, 135);
    std::cout << "CCW Limit Read : " << servo->readLimitCCWPosition(id) << std::endl;

    // Temperature Limit
    std::cout << "Temperature Limit Write : " << std::endl; servo->writeLimitTemperature(id, 70);
    std::cout << "Temperature Limit Read : " << servo->readLimitTemperature(id) << std::endl;

    // Current Limit
    std::cout << "Current Limit Write : " << std::endl; servo->writeLimitCurrent(id, 5000);
    std::cout << "Current Limit Read : " << servo->readLimitCurrent(id) << std::endl;

    // Drive Mode
    std::cout << "Drive Mode Write : " << std::endl; servo->writeDriveMode(id, 4);
    std::cout << "Drive Mode Read : " << servo->readDriveMode(id) << std::endl;
    
    // Torque Enable
    std::cout << "Torque Enable Write : " << std::endl; servo->writeTorqueEnable(id, 1);
    std::cout << "Torque Enable Read : " << (int)servo->readTorqueEnable(id) << std::endl;

    // Temperature
    std::cout << "Temperature Read : " << servo->readTemperature(id) << std::endl;

    // Current
    std::cout << "Current Read : " << servo->readCurrent(id) << std::endl;

    // Voltage
    std::cout << "Voltage Read : " << servo->readVoltage(id) << std::endl;

    // Speed
    std::cout << "Speed Write : " << std::endl; servo->writeSpeed(id, 100);
    std::cout << "Speed Read : " << servo->readSpeed(id) << std::endl;

    // Accel Time
    std::cout << "Accel Time Write : " << std::endl; servo->writeAccelTime(id, 0.2);
    std::cout << "Accel Time Read : " << servo->readAccelTime(id) << std::endl;

    // Target Time
    std::cout << "Target Time Write : " << std::endl; servo->writeTargetTime(id, 0.1);
    std::cout << "Target Time Read : " << servo->readTargetTime(id) << std::endl;

    // Target Position
    std::cout << "Target Position Write : " << std::endl; servo->writeTargetPosition(id, 90);
    Sleep(1000);
    std::cout << "Target Position Write : " << std::endl; servo->writeTargetPosition(id, -33.3);
    Sleep(1000);
    std::cout << "Target Position Read : " << servo->readTargetPosition(id) << std::endl;

    // Current Position
    std::cout << "Current Position Read : " << servo->readCurrentPosition(id) << std::endl;

    // Burst Position
    uint8_t idList[1] = { id };
    gFloat positionList[1] = { 45.0 };
    std::cout << "Burst Position Write : " << std::endl; servo->burstWriteTargetPositions(idList, positionList, 1);

    servo->changeOperatingMode(true);

    std::cout << "Burst Position Read : " << std::endl; servo->burstReadPositions(idList, 1, burstReadCallback);
//    std::cout << "Burst Position Read : " << std::endl; servo->readCurrentPosition(id, burstReadCallback);
    while (true)servo->spin();
}


// プログラムの実行: Ctrl + F5 または [デバッグ] > [デバッグなしで開始] メニュー
// プログラムのデバッグ: F5 または [デバッグ] > [デバッグの開始] メニュー

// 作業を開始するためのヒント: 
//    1. ソリューション エクスプローラー ウィンドウを使用してファイルを追加/管理します 
//   2. チーム エクスプローラー ウィンドウを使用してソース管理に接続します
//   3. 出力ウィンドウを使用して、ビルド出力とその他のメッセージを表示します
//   4. エラー一覧ウィンドウを使用してエラーを表示します
//   5. [プロジェクト] > [新しい項目の追加] と移動して新しいコード ファイルを作成するか、[プロジェクト] > [既存の項目の追加] と移動して既存のコード ファイルをプロジェクトに追加します
//   6. 後ほどこのプロジェクトを再び開く場合、[ファイル] > [開く] > [プロジェクト] と移動して .sln ファイルを選択します
