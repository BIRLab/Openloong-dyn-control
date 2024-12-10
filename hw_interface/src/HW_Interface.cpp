#include "HW_Interface.h"
#include <lely/ev/loop.hpp>
#include <lely/ev/fiber_exec.hpp>
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
#include <mutex>

using namespace std::chrono_literals;

class MotorDriver : public lely::canopen::FiberDriver {
public:
    explicit MotorDriver(ev_exec_t* exec, lely::canopen::AsyncMaster& master, uint8_t id, int32_t motor_encoder, int32_t motor_offset, bool motor_reverse) : lely::canopen::FiberDriver(exec, master, id), encoder{motor_encoder}, offset{motor_offset}, reverse{motor_reverse ? -1.0 : 1.0} { }

    // constance
    const int32_t encoder;
    const int32_t offset;
    const double reverse;

    // motor feedback
    std::mutex feedback_mutex;
    double position{};        // rad
    double velocity{};        // rad / s
    double current{};         // A
    double torque{};          // Nm

    bool ready{false};

    void updateFeedback() {
        std::scoped_lock lock(feedback_mutex);
        position = reverse * 2 * M_PI * (position_raw - offset) / encoder;
        velocity = reverse * 2 * M_PI * velocity_raw / encoder;
        current = reverse * (double)current_raw * (double)max_current / 1000.0;
        torque = reverse * (double)torque_raw / 1000.0;
    }

    void readFeedback(double& position_out, double& velocity_out, double& current_out, double& torque_out) {
        std::scoped_lock lock(feedback_mutex);
        position_out = position;
        velocity_out = velocity;
        current_out = current;
        torque_out = torque;
    }

    void sendCommand(double target_current) {
        tpdo_mapped[0x6071][0] = (int16_t)(reverse * 1000000.0 * target_current / max_current);
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

    void OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override {
        try {
            Wait(AsyncWrite<uint32_t>(0x6081, 0, 0));
            Wait(AsyncWrite<uint16_t>(0x6040, 0, 6));
            ready = false;
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
    bool reverse;
};

class MotorManager {
public:
    explicit MotorManager(const std::vector<MotorDescription>& desc) : io_guard(), ctx(), poll(ctx), loop(poll.get_poll()), exec(loop.get_executor()), timer(poll, exec, CLOCK_MONOTONIC) {
        for (const auto& d : desc) {
            auto it = buses.find(d.bus_name);
            if (it == buses.end()) {
                try {
                    auto r = buses.emplace(d.bus_name, std::make_shared<MotorBus>(this, d));
                    it = r.first;
                } catch (std::system_error const& ex) {
                    throw std::runtime_error(ex.what() + std::string(" - (" + d.bus_name + ")"));
                }
            }
            drivers[d.global_id] = std::make_shared<MotorDriver>(exec, it->second->master, d.node_id, d.encoder, d.offset, d.reverse);
        }
        for (const auto& b : buses) {
            b.second->master.Reset();
        }
        loop_thread = std::thread([this](){
            lely::ev::FiberThread ft;
            loop.run();
        });
    }

    void stop() {
        for (const auto& b : buses) {
            b.second->master.AsyncDeconfig();
        }
        wait_ready(false);
        ctx.shutdown();
        loop_thread.join();
    }

    void wait_ready(bool target = true) {
        bool all_ready = false;
        while (!all_ready) {
            all_ready = true;
            for (const auto& m : drivers) {
                if (m.second->ready != target)
                    all_ready = false;
            }
        }
        std::this_thread::sleep_for(500ms);
    }

    void updateSensorValues() {
        for (const auto& d : drivers) {
            d.second->updateFeedback();
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
            double torque_sensor;
            m.second->readFeedback(busIn.motors_pos_cur[m.first], busIn.motors_vel_cur[m.first], busIn.motors_tor_cur[m.first], torque_sensor);
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
    std::thread loop_thread;

    struct MotorBus {
        MotorBus(MotorManager *m, const MotorDescription &d) : controller(d.bus_name.c_str()),
                                                               channel(m->poll, m->exec),
                                                               master(m->timer, channel, "../config/master.dcf", "") {
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
        #include "motor_descriptions.h"
    };

    m = std::make_shared<MotorManager>(desc);
    m->wait_ready();
}

HW_Interface::~HW_Interface() {
    m->stop();
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
