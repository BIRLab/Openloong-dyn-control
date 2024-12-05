#pragma once

#include "robot_interface.h"
#include <memory>

class MotorManager;

class HW_Interface : RobotInterface {
public:
    HW_Interface();
    ~HW_Interface();
    void updateSensorValues() override;
    void setMotorsTorque(std::vector<double> &tauIn) override;
    void dataBusWrite(DataBus &busIn) override;

private:
    std::shared_ptr<MotorManager> m;
};
