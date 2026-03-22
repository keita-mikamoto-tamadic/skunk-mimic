#include "dummy_driver.hpp"
#include <cmath>
#include <iostream>

bool DummyDriver::Open(const std::string& device) {
    std::cout << "dummy mode (" << device << ")" << std::endl;
    return true;
}

void DummyDriver::Close() {}

void DummyDriver::SendCommands(const std::vector<AxisRef>&,
                               const std::vector<AxisConfig>&) {}

void DummyDriver::SendQueries(const std::vector<AxisConfig>&) {}

std::vector<AxisAct> DummyDriver::ReceiveStatus(
        const std::vector<AxisConfig>& axes, int) {
    std::vector<AxisAct> acts(axes.size());
    dummy_pos_ = std::fmod(dummy_pos_ + 0.001, 10.0);
    dummy_vel_ = std::fmod(dummy_vel_ + 0.01, 10.0);
    for (auto& act : acts) {
        act.position = dummy_pos_;
        act.velocity = dummy_vel_;
        act.torque = 0.0;
        act.fault = 0;
    }
    return acts;
}

void DummyDriver::SendAllOff(const std::vector<AxisConfig>&) {}
