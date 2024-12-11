#pragma once
#include "robot_interface.h"
#include <lely/ev/loop.hpp>
#include <lely/ev/fiber_exec.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/type_traits.hpp>
#include <thread>
#include <unordered_map>
#include <memory>

class MotorDriver;

class MotorManager {
public:
    explicit MotorManager();
    ~MotorManager();
    void wait_ready();
    void updateSensorValues();
    void setMotorsTorque(std::vector<double> &tauIn);
    void dataBusWrite(DataBus &busIn);

private:
    lely::io::IoGuard io_guard;
    lely::io::Context ctx;
    lely::io::Poll poll;
    lely::ev::Loop loop;
    lely::ev::Executor exec;
    lely::io::Timer timer;
    std::thread loop_thread;

    struct MotorBus {
        MotorBus(MotorManager *m, const std::string& bus_name);

        lely::io::CanController controller;
        lely::io::CanChannel channel;
        lely::canopen::AsyncMaster master;
    };

    std::unordered_map<std::string, std::shared_ptr<MotorBus>> buses;
    std::unordered_map<int, std::shared_ptr<MotorDriver>> drivers;

    void wait_ready_state(bool ready);
};
