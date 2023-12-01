//
// Created by Martin Økter on 07/10/2023.
//

#include "qbSoftHandControl.hh"

#include "serial/serial.h"
#include <qbrobotics_research_api/qbsofthand_research_api.h>

//#include "../../libs/qbdevice-api-7.x.x/serial/include/serial/serial.h"
//#include "../../libs/qbdevice-api-7.x.x/qbrobotics-driver/libs/research/include/qbrobotics_research_api/qbsofthand_research_api.h"


#include <iostream>
#include <string>
#include <vector>

qbSoftHandControl::qbSoftHandControl(int hand_nr_, std::map<int, std::shared_ptr<qbrobotics_research_api::qbSoftHandLegacyResearch> > *device_info)
: this_soft_hand_(*device_info)
, dev_id_(hand_nr_)
{
    // Do nothing
}

qbSoftHandControl::~qbSoftHandControl() {

}

std::vector <int16_t> qbSoftHandControl::GetControlReference() {
    std::vector<int16_t> control_references;
    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getControlReferences(control_references);
    for (auto &reference:control_references){
        std::cout << reference << " " << std::endl;
    }
    return control_references;
}

std::string qbSoftHandControl::GetDeviceInfo() {
    std::string info_string;
    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getInfo(INFO_ALL, info_string);
    std::cout << info_string << std::endl << "----" << std::endl;
    return std::string();
}

std::vector<int16_t> qbSoftHandControl::GetCurrents() {

    std::vector<int16_t> currents;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getCurrents(currents);
    //for (auto &current:currents){
    //    std::cout << current << " " << std::endl;
    //}

    return currents;
}

std::vector<int16_t> qbSoftHandControl::GetPositions() {

    std::vector<int16_t> positions_;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getPositions(positions_);
    //for (auto &position:positions_){
    //    std::cout << position << " " << std::endl;
    //}
    return positions_;
}

std::vector<int16_t> qbSoftHandControl::GetVelocities() {

    std::vector<int16_t> velocities_;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getVelocities(velocities_);
    for (auto &velocity:velocities_){
        std::cout << velocity << " ";
    }
    return velocities_;
}

std::vector<int16_t> qbSoftHandControl::GetAccelerations() {

    std::vector<int16_t> accelerations_;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getAccelerations(accelerations_);
    for (auto &acceleration:accelerations_){
        std::cout << acceleration << " ";
    }
    return accelerations_;
}

bool qbSoftHandControl::SetMotorStates(bool active) {

    if (qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->setMotorStates(active) == 0){
        std::cout << "Success setting motor state" << std::endl;
    } else {
        std::cout << "Something went wrong while setting motors state" << std::endl;
    }
    return false;
}

bool qbSoftHandControl::GetMotorStates() {
    bool activate = false;
    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getMotorStates(activate);
    if(activate){
        std::cout << "Motors are active" << std::endl;
    } else {
        std::cout << "Motors are not active" << std::endl;
    }
    return activate;
}

void qbSoftHandControl::SetGripValue(int grip, int spread) {  //TODO: Ask Muri about this formulation
    std::vector<int16_t> control_references;
    control_references.push_back(grip);
    control_references.push_back(spread);
    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->setControlReferences(control_references);
    /*
    for (auto &control_reference:control_references){
        std::cout << control_reference << " ";
    }
    */
}

std::vector<float> qbSoftHandControl::GetPositionPID() {

    std::vector<float> PID_;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getParamPositionPID(PID_);
    for (auto &param:PID_){
        std::cout << param << " ";
    }

    return PID_;
}

std::vector<float> qbSoftHandControl::GetCurrentPID() {

    std::vector<float> PID_;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getParamCurrentPID(PID_);
    for (auto &param:PID_){
        std::cout << param << " ";
    }
    return PID_;
}

int qbSoftHandControl::GetId() {

    uint8_t device_id_;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getParamId(device_id_);
    std::cout << (int)device_id_ << " ";

    return device_id_;
}

int qbSoftHandControl::GetStartupActivation() {

    uint8_t activation;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getParamStartupActivation(activation);
    std::cout << (int)activation;

    return activation;
}

int qbSoftHandControl::GetInputMode() {

    uint8_t input_mode;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getParamInputMode(input_mode);
    std::cout << (int)input_mode;

    return input_mode;
}

int qbSoftHandControl::GetControlMode() {

    uint8_t control_mode;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getParamControlMode(control_mode);
    std::cout << (int)control_mode;

    return control_mode;
}

std::vector<uint8_t> qbSoftHandControl::GetEncoderResolution() {

    std::vector<uint8_t> encoder_resolutions_;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getParamEncoderResolutions(encoder_resolutions_);
    for (auto &param:encoder_resolutions_){
        std::cout << (int)param << " ";
    }
    return encoder_resolutions_;
}

std::vector<int16_t> qbSoftHandControl::GetEncoderOffsets() {

    std::vector<int16_t> encoder_offsets_;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getParamEncoderOffsets(encoder_offsets_);
    for (auto &offset:encoder_offsets_){
        std::cout << offset << " ";
    }
    return encoder_offsets_;
}

std::vector<float> qbSoftHandControl::GetEncoderMultipliers() {

    std::vector<float> encoder_multipliers_;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getParamEncoderMultipliers(encoder_multipliers_);
    for (auto &param:encoder_multipliers_){
        std::cout << param << " ";
    }
    return encoder_multipliers_;
}

int qbSoftHandControl::GetUsePositionLimits() {

    uint8_t use_position_limits_;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getParamUsePositionLimits(use_position_limits_);
    std::cout << (int)use_position_limits_;

    return use_position_limits_;
}

std::vector<int32_t> qbSoftHandControl::GetPositionLimits() {

    std::vector<int32_t> position_limits_;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getParamPositionLimits(position_limits_);
    for (auto &param:position_limits_){
        std::cout << (int)param << " ";
    }
    return position_limits_;
}

std::vector<int32_t> qbSoftHandControl::GetPositionMaxSteps() {

    std::vector<int32_t> position_max_steps_;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getParamPositionMaxSteps(position_max_steps_);
    for (auto &param:position_max_steps_){
        std::cout << (int)param << " ";
    }
    return position_max_steps_;
}

int qbSoftHandControl::GetCurrentLimit() {

    int16_t current_limit_;

    qbSoftHandControl::this_soft_hand_.at(qbSoftHandControl::dev_id_)->getParamCurrentLimit(current_limit_);
    std::cout << (int)current_limit_;
    return current_limit_;
}
