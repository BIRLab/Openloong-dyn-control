#include "motor_manager.h"
#include "cl_color.h"

using namespace std::chrono_literals;

struct MotorDescription {
    std::string bus_name;
    uint8_t node_id;
    uint8_t global_id;
    int32_t encoder;
    int32_t offset;
    bool reverse;
};

class MotorDriver : public lely::canopen::FiberDriver {
public:
    explicit MotorDriver(ev_exec_t* exec, lely::canopen::AsyncMaster& master, const MotorDescription& desc) : lely::canopen::FiberDriver(exec, master, desc.node_id), encoder{desc.encoder}, offset{desc.offset}, reverse{desc.reverse ? -1.0 : 1.0} { }

    // constance
    const int32_t encoder;
    const int32_t offset;
    const double reverse;

    // motor feedback
    std::mutex feedback_mutex;
    double position{};              // rad
    double velocity{};              // rad / s
    double torque{};                // Nm

    bool ready{false};

    void updateFeedback() {
        std::scoped_lock lock(feedback_mutex);
        position = reverse * 2 * M_PI * (position_raw - offset) / encoder;
        velocity = reverse * 2 * M_PI * velocity_raw / encoder;
        torque = reverse * (double)torque_raw * (double)rated_torque / 1000000.0;
    }

    void readFeedback(double& position_out, double& velocity_out, double& torque_out) {
        std::scoped_lock lock(feedback_mutex);
        position_out = position;
        velocity_out = velocity;
        torque_out = torque;
    }

    void sendCommand(double target_torque) {
        tpdo_mapped[0x6071][0] = (int16_t)(reverse * 1000000.0 * target_torque / rated_torque);
        tpdo_mapped[0x6071][0].WriteEvent();
    }

private:
    uint32_t rated_torque{};
    int32_t position_raw{};
    int32_t velocity_raw{};
    int16_t torque_raw{};

    void OnBoot(lely::canopen::NmtState st, char es, const std::string& what) noexcept override {
        if (!es || es == 'L') {
            // boot
            Wait(AsyncWrite<uint16_t>(0x6040, 0, 128));
            Wait(AsyncWrite<uint16_t>(0x6040, 0, 6));
            Wait(AsyncWrite<uint16_t>(0x6040, 0, 7));
            Wait(AsyncWrite<uint16_t>(0x6040, 0, 15));

            AsyncWait(500ms).submit(master.get_executor(), [this](){
                std::cout << CL_BOLDGREEN << "motor " << static_cast<int>(id()) << " booted successfully" << CL_RESET << std::endl;
                ready = true;
            });
        } else {
            std::cerr << "motor " << static_cast<int>(id()) << " failed to boot: " << what << std::endl;
        }
    }

    void OnConfig(std::function<void(std::error_code ec)> res) noexcept override {
        try {
            // read motor rated torque
            rated_torque = Wait(AsyncRead<uint32_t>(0x6076, 0));

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
                torque_raw = rpdo_mapped[0x6077][0];
                break;
            default:
                break;
        }
    }
};

MotorManager::MotorManager() : io_guard(), ctx(), poll(ctx), loop(poll.get_poll()), exec(loop.get_executor()), timer(poll, exec, CLOCK_MONOTONIC) {
    const std::vector<MotorDescription> desc {
        #include "motor_descriptions.h"
    };
    for (const auto& d : desc) {
        auto it = buses.find(d.bus_name);
        if (it == buses.end()) {
            try {
                auto r = buses.emplace(d.bus_name, std::make_shared<MotorBus>(this, d.bus_name));
                it = r.first;
            } catch (std::system_error const& ex) {
                throw std::runtime_error(ex.what() + std::string(" - (" + d.bus_name + ")"));
            }
        }
        drivers[d.global_id] = std::make_shared<MotorDriver>(exec, it->second->master, d);
    }
    for (const auto& b : buses) {
        b.second->master.Reset();
    }
    loop_thread = std::thread([this](){
        lely::ev::FiberThread ft;
        loop.run();
    });
}

MotorManager::~MotorManager() {
    for (const auto& b : buses) {
        b.second->master.AsyncDeconfig();
    }
    wait_ready_state(false);
    ctx.shutdown();
    loop_thread.join();
}

void MotorManager::wait_ready() {
    wait_ready_state(true);
}

void MotorManager::updateSensorValues() {
    for (const auto& d : drivers) {
        d.second->updateFeedback();
    }
}

void MotorManager::setMotorsTorque(std::vector<double> &tauIn) {
    for (uint8_t i = 0; i < (uint8_t)tauIn.size(); ++i) {
        auto m = drivers.find(i);
        if (m != drivers.end()) {
            m->second->sendCommand(tauIn[i]);
        } else {
            std::cerr << "Motor " << (int)i << " is offline." << std::endl;
        }
    }
}

void MotorManager::dataBusWrite(DataBus &busIn) {
    uint8_t max_motor_id = std::max_element(drivers.begin(), drivers.end(), [](auto a, auto b) {return a.first < b.first;})->first;

    busIn.motors_pos_cur.resize(max_motor_id);
    busIn.motors_vel_cur.resize(max_motor_id);
    busIn.motors_tor_cur.resize(max_motor_id);

    for (const auto& m : drivers) {
        m.second->readFeedback(busIn.motors_pos_cur[m.first], busIn.motors_vel_cur[m.first], busIn.motors_tor_cur[m.first]);
    }
}

MotorManager::MotorBus::MotorBus(MotorManager *m, const std::string& bus_name) : controller(bus_name.c_str()),
                                                                                 channel(m->poll, m->exec),
                                                                                 master(m->timer, channel, "../config/master.dcf", "") {
    channel.open(controller);
}

void MotorManager::wait_ready_state(bool ready) {
    bool all_ready = false;
    while (!all_ready) {
        all_ready = true;
        for (const auto& m : drivers) {
            if (m.second->ready != ready) {
                all_ready = false;
                std::this_thread::sleep_for(10ms);
            }
        }
    }
    std::this_thread::sleep_for(500ms);
}
