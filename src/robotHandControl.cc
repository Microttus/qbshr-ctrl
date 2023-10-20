//
// Created by Martin Økter on 15/10/2023.
//

#include <iostream>
#include <unistd.h>

#include "qbSoftHandHandler.hh"
#include "qbSoftHandControl.hh"

#include "qbrobotics_research_api/qbsofthand_research_api.h"


qbSoftHandHandler my_hands_;

int main() {

    std::vector<qbSoftHandControl> qbSoftHand_devices = my_hands_.ReturnDeviceMap();
    std::cout << "There are " << qbSoftHand_devices.size() << " available qbSoftHand Research available for control" << std::endl;

    qbSoftHand_devices[0].SetMotorStates(true);
    sleep(2);
    qbSoftHand_devices[0].SetGripValue(10000,0);
    sleep(2);
    qbSoftHand_devices[0].SetGripValue(0,0);
    sleep(2);
    qbSoftHand_devices[0].GetCurrents();
    sleep(2);
    qbSoftHand_devices[0].GetPositions();
    sleep(2);

    std::cout << "Main function completed" << std::endl;
};