#include "HW_Interface.h"
#include <thread>

using namespace std::chrono_literals;

int main(int argc, const char** argv) {
    HW_Interface hw_interface;
    DataBus RobotState(17);
    std::vector<double> tq(11, 1.5);
    for (int i = 0; i < 1000; ++i) {
        hw_interface.updateSensorValues();
        hw_interface.setMotorsTorque(tq);
        hw_interface.dataBusWrite(RobotState);
        std::this_thread::sleep_for(1ms);
    }
}
