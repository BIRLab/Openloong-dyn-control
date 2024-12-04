#include "HW_Interface.h"
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/type_traits.hpp>
#include <iostream>
#include <unordered_map>
#include <cmath>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;

class MotorDriver : public lely::canopen::FiberDriver {
public:
    explicit MotorDriver(ev_exec_t* exec, lely::canopen::AsyncMaster& master, uint8_t id, int32_t encoder, int32_t offset) : lely::canopen::FiberDriver(exec, master, id), encoder{encoder}, offset{offset} { }

    // constance
    const int32_t encoder;
    const int32_t offset;

    // motor feedback
    double position{};        // rad
    double velocity{};        // rad / s
    double current{};         // A
    double torque{};          // Nm

    bool ready{false};

    void UpdateFeedback() {
        position = 2 * M_PI * (position_raw - offset) / encoder;
        velocity = 2 * M_PI * velocity_raw / encoder;
        current = (double)current_raw * (double)max_current / 1000.0;
        torque = (double)torque_raw / 1000.0;
    }

    void sendCommand(double target_current) {
        tpdo_mapped[0x6071][0] = (int16_t)(1000000.0 * target_current / max_current);
        tpdo_mapped[0x6071][0].WriteEvent();
    }

private:
    uint32_t max_current{};
    uint16_t status{};
    uint16_t error_code{};
    int32_t position_raw{};
    int32_t velocity_raw{};
    int16_t current_raw{};
    int32_t torque_raw{};

    void OnBoot(lely::canopen::NmtState st, char es, const std::string& what) noexcept override {
        if (!es || es == 'L') {
            // boot
            tpdo_mapped[0x6040][0] = (uint16_t)128;
            tpdo_mapped[0x6040][0].WriteEvent();
            tpdo_mapped[0x6040][0] = (uint16_t)6;
            tpdo_mapped[0x6040][0].WriteEvent();
            tpdo_mapped[0x6040][0] = (uint16_t)7;
            tpdo_mapped[0x6040][0].WriteEvent();
            tpdo_mapped[0x6040][0] = (uint16_t)15;
            tpdo_mapped[0x6040][0].WriteEvent();

            AsyncWait(500ms).submit(master.get_executor(), [this](){
                std::cout << "motor " << static_cast<int>(id()) << " booted successfully" << std::endl;
                ready = true;
            });
        } else {
            std::cout << "motor " << static_cast<int>(id()) << " failed to boot: " << what << std::endl;
        }
    }

    void OnConfig(std::function<void(std::error_code ec)> res) noexcept override {
        try {
            // read motor rated current
            max_current = Wait(AsyncRead<uint32_t>(0x6075, 0));

            res({});
        } catch (lely::canopen::SdoError& e) {
            res(e.code());
        }
    }

    void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override {
        switch (idx) {
            case 0x6041:
                // Statusword
                status = rpdo_mapped[0x6041][0];
                break;
            case 0x603F:
                // Error Code
                error_code = rpdo_mapped[0x603F][0];
                break;
            case 0x6064:
                // Position actual value
                position_raw = rpdo_mapped[0x6064][0];
                break;
            case 0x606C:
                // Velocity actual value
                velocity_raw = rpdo_mapped[0x606C][0];
                break;
            case 0x6077:
                // Torque actual value
                current_raw = rpdo_mapped[0x6077][0];
                break;
            case 0x3B69:
                // Torque sensor
                torque_raw = rpdo_mapped[0x3B69][0];
                break;
            default:
                break;
        }
    }
};

struct MotorDescription {
    std::string bus_name;
    uint8_t node_id;
    uint8_t global_id;
    int32_t encoder;
    int32_t offset;
};

class MotorManager {
public:
    explicit MotorManager(const std::vector<MotorDescription>& desc) : io_guard(), ctx(), poll(ctx), loop(poll.get_poll()), exec(loop.get_executor()), timer(poll, exec, CLOCK_MONOTONIC) {
        for (const auto& d : desc) {
            auto it = buses.find(d.bus_name);
            if (it == buses.end()) {
                auto r = buses.emplace(d.bus_name, std::make_shared<MotorBus>(this, d));
                it = r.first;
            }
            drivers[d.global_id] = std::make_shared<MotorDriver>(exec, it->second->master, d.node_id, d.encoder, d.offset);
        }
        for (const auto& b : buses) {
            b.second->master.Reset();
        }
    }

    void run() {
        loop.run();
    }

    void wait_ready() {
        bool all_ready = false;
        while (!all_ready) {
            all_ready = true;
            for (const auto& m : drivers) {
                if (!m.second->ready)
                    all_ready = false;
            }
        }
        usleep(500000);
    }

    void updateSensorValues() {
        for (const auto& d : drivers) {
            d.second->UpdateFeedback();
        }
    }

    void setMotorsTorque(std::vector<double> &tauIn) {
        for (uint8_t i = 0; i < (uint8_t)tauIn.size(); ++i) {
            auto m = drivers.find(i);
            if (m != drivers.end()) {
                m->second->sendCommand(tauIn[i]);
            } else {
                std::cerr << "Motor " << (int)i << " is offline." << std::endl;
            }
        }
    }

    void dataBusWrite(DataBus &busIn) {

        uint8_t max_motor_id = std::max_element(drivers.begin(), drivers.end(), [](auto a, auto b) {return a.first < b.first;})->first;

        busIn.motors_pos_cur.resize(max_motor_id);
        busIn.motors_vel_cur.resize(max_motor_id);
        busIn.motors_tor_cur.resize(max_motor_id);

        for (const auto& m : drivers) {
            busIn.motors_pos_cur[m.first] = m.second->position;
            busIn.motors_vel_cur[m.first] = m.second->velocity;
            busIn.motors_tor_cur[m.first] = m.second->current;
        }

        busIn.rpy[0] = 0;
        busIn.rpy[1] = 0;
        busIn.rpy[2] = 0;

        busIn.baseLinVel[0] = 0;
        busIn.baseLinVel[1] = 0;
        busIn.baseLinVel[2] = 0;

        busIn.baseAcc[0] = 0;
        busIn.baseAcc[1] = 0;
        busIn.baseAcc[2] = 0;

        busIn.baseAngVel[0] = 0;
        busIn.baseAngVel[1] = 0;
        busIn.baseAngVel[2] = 0;

        busIn.updateQ();
    }

private:
    lely::io::IoGuard io_guard;
    lely::io::Context ctx;
    lely::io::Poll poll;
    lely::ev::Loop loop;
    lely::ev::Executor exec;
    lely::io::Timer timer;

    struct MotorBus {
        MotorBus(MotorManager *m, const MotorDescription &d) : controller(d.bus_name.c_str()),
                                                               channel(m->poll, m->exec),
                                                               master(m->timer, channel, "config/master.dcf", "") {
            channel.open(controller);
        }

        lely::io::CanController controller;
        lely::io::CanChannel channel;
        lely::canopen::AsyncMaster master;
    };

    std::unordered_map<std::string, std::shared_ptr<MotorBus>> buses;
    std::unordered_map<int, std::shared_ptr<MotorDriver>> drivers;
};

HW_Interface::HW_Interface() {
    std::vector<MotorDescription> desc {
        {"robot_can0", 1, 0, 524288, 262143},
        {"robot_can0", 2, 1, 524288, 262143},
        {"robot_can1", 3, 2, 524288, 262143},
        {"robot_can1", 4, 3, 524288, 262143},
        {"robot_can1", 5, 4, 524288, 262143},
        {"robot_can2", 6, 5, 524288, 262143},
        {"robot_can2", 7, 6, 524288, 262143},
        {"robot_can3", 8, 7, 524288, 262143},
        {"robot_can3", 9, 8, 524288, 262143},
        {"robot_can3", 10, 9, 524288, 262143},
        {"robot_can2", 11, 10, 524288, 262143}
    };
    m = std::make_shared<MotorManager>(desc);
}

void HW_Interface::updateSensorValues() {
    m->updateSensorValues();
}

void HW_Interface::setMotorsTorque(std::vector<double> &tauIn) {
    m->setMotorsTorque(tauIn);
}

void HW_Interface::dataBusWrite(DataBus &busIn) {
    m->dataBusWrite(busIn);
}

void HW_Interface::run() {
    m->run();
}

void HW_Interface::wait_ready() {
    m->wait_ready();
}
