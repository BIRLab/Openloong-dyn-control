#pragma once

#include "data_bus.h"
#include <vector>

class RobotInterface {
    virtual void updateSensorValues() = 0;
    virtual void setMotorsTorque(std::vector<double> &tauIn) = 0;
    virtual void dataBusWrite(DataBus &busIn) = 0;
};
