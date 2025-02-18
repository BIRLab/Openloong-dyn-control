#pragma once
#include "robot_interface.h"

class HWPlugin : public RobotInterface {
public:
    virtual void waitReady() {};
    void updateSensorValues() override {};
    void setMotorsTorque(std::vector<double> &tauIn) override {};
    void dataBusWrite(DataBus &busIn) override {};
};
