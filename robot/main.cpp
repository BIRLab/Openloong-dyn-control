#include "HW_Interface.h"
#include <thread>

using namespace std::chrono_literals;

void robot_main(HW_Interface& hw_interface) {
    DataBus RobotState(17);
    std::vector<double> tq(11, 1.5);
    while (true) {
        hw_interface.updateSensorValues();
        hw_interface.setMotorsTorque(tq);
        hw_interface.dataBusWrite(RobotState);
        std::this_thread::sleep_for(1ms);
    }
}

int main(int argc, const char** argv) {
    HW_Interface hw_interface;
    std::thread main_thread([&]() {
        hw_interface.wait_ready();
        robot_main(hw_interface);
    });
    hw_interface.run();
    main_thread.join();
    return 0;
}
