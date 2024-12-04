#pragma once

#include "robot_interface.h"
#include <memory>

class MotorManager;

class HW_Interface : RobotInterface {
public:
    HW_Interface();
    void updateSensorValues() override;
    void setMotorsTorque(std::vector<double> &tauIn) override;
    void dataBusWrite(DataBus &busIn) override;
    void run();
    void wait_ready();
private:
    std::shared_ptr<MotorManager> m;
};
