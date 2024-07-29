// gs2d-v2.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//

#include <iostream>

#include "LinuxSerial.h"
#include "gs2d.h"

using namespace gs2d;

int main()
{
    uint8_t id = 2;

    Driver* servo = new RobotisP20<gs2d::LinuxSerial>();

    // Voltage
    std::cout << "Voltage Read : " << servo->readVoltage(id) << std::endl;

    // Torque ON
    std::cout << "Torque Enable Write : " << std::endl; servo->writeTorqueEnable(id, 1);

    // Target Position
    std::cout << "Target Position Write : " << std::endl; servo->writeTargetPosition(id, 45);
}
