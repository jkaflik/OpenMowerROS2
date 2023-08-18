#include "openmower_hw_comms/mainboard.hpp"
#include "openmower_hw_comms/crc16.h"
#include "serial_driver/serial_driver.hpp"

#include <thread>
#include <vector>

namespace openmower_hw_comms
{
    class Mainboard::SerialImpl {
    public:
        SerialImpl()
        : owned_ctx{new IoContext(2)},
        serial_driver{new drivers::serial_driver::SerialDriver(*owned_ctx)}
        {}

        ~SerialImpl()
        {
            if (owned_ctx) {
                owned_ctx->waitForExit();
            }
        }

        void connect(const std::string &port, uint32_t baudRate = 115200) {
            auto fc = drivers::serial_driver::FlowControl::HARDWARE;
            auto pt = drivers::serial_driver::Parity::NONE;
            auto sb = drivers::serial_driver::StopBits::ONE;

            device_config = std::make_unique<drivers::serial_driver::SerialPortConfig>(baudRate, fc, pt, sb);
            serial_driver->init_port(port, *device_config);
            if (!serial_driver->port()->is_open()) {
                serial_driver->port()->open();
            }
        }

        void onDataReceived() {
            std::vector<uint8_t> buffer;

            while (on_data_received_run) {
                const auto bytes_read = serial_driver->port()->receive(buffer);

                if (!buffer.empty()) {
                    if (!validateCRC16(buffer)) {
                        continue;
                    }

                    std::vector<uint8_t> data(buffer.begin(), buffer.end() - 2);

                    
                }
            }
        }

        bool validateCRC16(const std::vector<uint8_t> &buffer) {
            if (buffer.size() < 2) {
                return false;
            }

            std::vector<uint8_t> data(buffer.begin(), buffer.end() - 2);

            // calculateCRC16 for the buffer data without two last bytes
            uint32_t crc = calculateCRC16(data);
            uint32_t crc_packet = (buffer[buffer.size() - 4] << 24) | (buffer[buffer.size() - 3] << 16) | (buffer[buffer.size() - 2] << 8) | buffer[buffer.size() - 1];
            return crc == crc_packet;
        }

        bool on_data_received_run;

        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config;

        StatusHandlerFunction status_handler;
        IMUHandlerFunction imu_handler;
        HeatbeatHandler heartbeat_handler;
        UIEventHandler ui_event_handler;

    private:        
        std::unique_ptr<IoContext> owned_ctx{};
        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver;        

        std::vector<uint8_t> buffer;
    };

    void Mainboard::connect(const std::string &port)
    {
        if (port.empty()) {
            throw std::invalid_argument("Mainboard serial port cannot be empty");
        }

        connect(port);
    }
}