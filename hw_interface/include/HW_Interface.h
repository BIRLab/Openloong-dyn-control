#pragma once

#include "robot_interface.h"

class HW_Interface : RobotInterface {
public:
    HW_Interface();
    void updateSensorValues() override;
    void setMotorsTorque(std::vector<double> &tauIn) override;
    void dataBusWrite(DataBus &busIn) override;
};


