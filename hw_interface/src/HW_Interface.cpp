#include "HW_Interface.h"
#include "motor_manager.h"
#include "t265.h"

class HW_Interface::Implementation {
public:
    void wait_ready() {
        m.wait_ready();
        t.wait_ready();
    }

    void updateSensorValues() {
        m.updateSensorValues();
        t.updateSensorValues();
    }

    void setMotorsPVT(const DataBus &busIn) {
        m.setMotorsPVT(busIn);
    }

    void dataBusWrite(DataBus &busIn) {
        m.dataBusWrite(busIn);
        t.dataBusWrite(busIn);
        busIn.updateQ();
    }

private:
    MotorManager m;
    T265 t;
};

HW_Interface::HW_Interface() : i{std::make_unique<Implementation>()} {
    i->wait_ready();
}

HW_Interface::~HW_Interface() = default;

void HW_Interface::updateSensorValues() {
    i->updateSensorValues();
}

[[deprecated]] void HW_Interface::setMotorsTorque(std::vector<double> &tauIn) {
    std::cerr << "setMotorsTorque() is unsupported, please use setMotorsPVT() instead" << std::endl;
}

void HW_Interface::setMotorsPVT(const DataBus &busIn) {
    i->setMotorsPVT(busIn);
}

void HW_Interface::dataBusWrite(DataBus &busIn) {
    i->dataBusWrite(busIn);
}
