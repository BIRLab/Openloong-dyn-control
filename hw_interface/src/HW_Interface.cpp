#include "HW_Interface.h"
#include "motor_manager.h"
#include "ahrs.h"

class HW_Interface::Implementation {
public:
    void wait_ready() {
        m.wait_ready();
    }

    void updateSensorValues() {
        m.updateSensorValues();
    }

    void setMotorsTorque(std::vector<double> &tauIn) {
        m.setMotorsTorque(tauIn);
    }

    void dataBusWrite(DataBus &busIn) {
        m.dataBusWrite(busIn);
        a.dataBusWrite(busIn);
    }

private:
    MotorManager m;
    AHRS a;
};

HW_Interface::HW_Interface() : i{std::make_unique<Implementation>()} {
    i->wait_ready();
}

HW_Interface::~HW_Interface() = default;

void HW_Interface::updateSensorValues() {
    i->updateSensorValues();
}

void HW_Interface::setMotorsTorque(std::vector<double> &tauIn) {
    i->setMotorsTorque(tauIn);
}

void HW_Interface::dataBusWrite(DataBus &busIn) {
    i->dataBusWrite(busIn);
}
