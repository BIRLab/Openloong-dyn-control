#include "ahrs.h"
#include "crc_table.h"
#include "cl_color.h"

using namespace std::chrono_literals;

enum class ProcessStage {
    STAGE_SOF,
    STAGE_TYPE,
    STAGE_LENGTH,
    STAGE_SN,
    STAGE_CRC8,
    STAGE_CRC16,
    STAGE_DATA,
    STAGE_EOF
};

AHRS::AHRS() : sp("/dev/ahrs", 921600), running{true}, ready{false} {
    assert(sp.init());
    loop_thread = std::thread([this] { process_data(); });
}

AHRS::~AHRS() {
    running = false;
    loop_thread.join();
}

void AHRS::waitReady() {
    while (!ready) {
        std::this_thread::sleep_for(10ms);
    }
    std::cout << CL_BOLDGREEN << "ahrs booted successfully" << CL_RESET << std::endl;
}

void AHRS::updateSensorValues() {
    std::scoped_lock<std::mutex> lock { data_mutex };
}

void AHRS::dataBusWrite(DataBus &busIn) {
    std::scoped_lock<std::mutex> lock { data_mutex };

    busIn.rpy[0] = roll;
    busIn.rpy[1] = pitch;
    busIn.rpy[2] = yaw;

    busIn.baseLinVel[0] = velocity_x;
    busIn.baseLinVel[1] = velocity_y;
    busIn.baseLinVel[2] = velocity_z;

    busIn.baseAcc[0] = accelerometer_x;
    busIn.baseAcc[1] = accelerometer_y;
    busIn.baseAcc[2] = accelerometer_z;

    busIn.baseAngVel[0] = gyroscope_x;
    busIn.baseAngVel[1] = gyroscope_y;
    busIn.baseAngVel[2] = gyroscope_z;

    busIn.updateQ();
}

void AHRS::process_data() {
    uint8_t frame[263];
    size_t ptr;
    uint8_t buffer[256];
    long long read_size;
    ProcessStage stage { ProcessStage::STAGE_SOF };

    while (running) {
        read_size = sp.read(buffer, 256);
        if (read_size < 0)
            continue;

        for (long long i = 0; i < read_size; ++i) {
            uint8_t ch = buffer[i];

            switch (stage) {
                case ProcessStage::STAGE_SOF:
                    if (ch == 0xfc) {
                        frame[0] = ch;
                        stage = ProcessStage::STAGE_TYPE;
                    }
                    break;
                case ProcessStage::STAGE_TYPE:
                    if (ch == 0x40 || ch == 0x41 || ch == 0x42) {
                        frame[1] = ch;
                        stage = ProcessStage::STAGE_LENGTH;
                    } else {
                        stage = ProcessStage::STAGE_SOF;
                    }
                    break;
                case ProcessStage::STAGE_LENGTH:
                    if ((frame[1] == 0x40 && ch == 0x38) || (frame[1] == 0x41 && ch == 0x30) || (frame[1] == 0x42 && ch == 0x48)) {
                        frame[2] = ch;
                        stage = ProcessStage::STAGE_SN;
                    } else {
                        stage = ProcessStage::STAGE_SOF;
                    }
                    break;
                case ProcessStage::STAGE_SN:
                    frame[3] = ch;
                    stage = ProcessStage::STAGE_CRC8;
                    break;
                case ProcessStage::STAGE_CRC8:
                    if (CRC8_Table(frame, 4) == ch) {
                        frame[4] = ch;
                        ptr = 5;
                        stage = ProcessStage::STAGE_CRC16;
                    } else {
                        stage = ProcessStage::STAGE_SOF;
                    }
                    break;
                case ProcessStage::STAGE_CRC16:
                    frame[ptr++] = ch;
                    if (ptr > 6) {
                        stage = ProcessStage::STAGE_DATA;
                    }
                    break;
                case ProcessStage::STAGE_DATA:
                    frame[ptr++] = ch;
                    if (ptr > 6 + frame[2]) {
                        stage = ProcessStage::STAGE_EOF;
                    }
                    break;
                case ProcessStage::STAGE_EOF:
                    if (ch == 0xfd && CRC16_Table(frame + 7, frame[2]) == ((frame[5] << 8) | frame[6])) {
                        // parse frame
                        std::scoped_lock<std::mutex> lock { data_mutex };
                        ready = true;
                        switch (frame[1]) {
                            case 0x40:
                                gyroscope_x = *reinterpret_cast<float*>(&frame[7]);
                                gyroscope_y = *reinterpret_cast<float*>(&frame[11]);
                                gyroscope_z = *reinterpret_cast<float*>(&frame[15]);
                                accelerometer_x = *reinterpret_cast<float*>(&frame[19]);
                                accelerometer_y = *reinterpret_cast<float*>(&frame[23]);
                                accelerometer_z = *reinterpret_cast<float*>(&frame[27]);
                                break;
                            case 0x41:
                                roll_speed = *reinterpret_cast<float*>(&frame[7]);
                                pitch_speed = *reinterpret_cast<float*>(&frame[11]);
                                yaw_speed = *reinterpret_cast<float*>(&frame[15]);
                                roll = *reinterpret_cast<float*>(&frame[19]);
                                pitch = *reinterpret_cast<float*>(&frame[23]);
                                yaw = *reinterpret_cast<float*>(&frame[27]);
                                break;
                            case 0x42:
                                velocity_x = *reinterpret_cast<float*>(&frame[7]);
                                velocity_y = *reinterpret_cast<float*>(&frame[11]);
                                velocity_z = *reinterpret_cast<float*>(&frame[15]);
                                break;
                            default:
                                break;
                        }
                    }
                    stage = ProcessStage::STAGE_SOF;
                    break;
            }
        }
    }
}
