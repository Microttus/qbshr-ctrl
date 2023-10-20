//
// Created by Martin Økter on 07/10/2023.
//

#ifndef UR5_QBHAND_CONTROL_QBSOFTHANDHANDLER_HH
#define UR5_QBHAND_CONTROL_QBSOFTHANDHANDLER_HH

#include <string>
#include <map>
#include <chrono>
#include <vector>
#include <memory>
#include <set>

#include "qbSoftHandControl.hh"

#include "serial/serial.h"
#include "qbrobotics_research_api/qbsofthand_research_api.h"


class qbSoftHandHandler{
public:
    qbSoftHandHandler();
    ~qbSoftHandHandler();

    int ScanForDevices(const int &max_repeats);
    int OpenSerialPort(const std::string &serial_port);

    void testLib();

    std::vector<qbSoftHandControl> ReturnDeviceMap();

private:
    std::shared_ptr<qbrobotics_research_api::Communication> communication_handler_;                 // Handler to manage the communication with qbdevices
    std::vector<serial::PortInfo> serial_ports_;
    std::vector<qbrobotics_research_api::Communication::ConnectedDeviceInfo> device_ids_;           // IDs of connected devices
    std::map<int, std::shared_ptr<qbrobotics_research_api::qbSoftHandLegacyResearch> > soft_hands_detected_;
    std::vector<qbSoftHandControl> device_list_;

    std::vector<int> test_list;
};

#endif //UR5_QBHAND_CONTROL_QBSOFTHANDHANDLER_HH
