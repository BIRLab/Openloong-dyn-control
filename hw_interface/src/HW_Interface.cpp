#include "HW_Interface.h"
#include "motor_manager.h"
#include "t265.h"

class HW_Interface::Implementation {
public:
    Implementation() {
        add_plugin<MotorManager>();
        add_plugin<T265>();
    }

    void waitReady() {
        for (const std::unique_ptr<HWPlugin>& p : plugins) {
            p->waitReady();
        }
    }

    void updateSensorValues() {
        for (const std::unique_ptr<HWPlugin>& p : plugins) {
            p->updateSensorValues();
        }
    }

    void setMotorsTorque(std::vector<double> &tauIn) {
        for (const std::unique_ptr<HWPlugin>& p : plugins) {
            p->setMotorsTorque(tauIn);
        }
    }

    void dataBusWrite(DataBus &busIn) {
        for (const std::unique_ptr<HWPlugin>& p : plugins) {
            p->dataBusWrite(busIn);
        }
        busIn.updateQ();
    }

private:
    std::vector<std::unique_ptr<HWPlugin>> plugins;

    template<typename T>
    void add_plugin() {
        plugins.push_back(std::make_unique<T>());
    }
};

HW_Interface::HW_Interface() : i{std::make_unique<Implementation>()} {
    i->waitReady();
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
