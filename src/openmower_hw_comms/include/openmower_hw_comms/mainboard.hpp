#pragma once

#include "openmower_hw_comms/ll_datatypes.h"

#include <exception>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace openmower_hw_comms
{
    class Mainboard
    {
    public:
        typedef std::function<void(const StatusPacket &)> StatusHandlerFunction;
        typedef std::function<void(const IMUPacket &)> IMUHandlerFunction;
        typedef std::function<void(const HeatbeatPacket &)> HeatbeatHandler;
        typedef std::function<void(const UIEventPacket &)> UIEventHandler;

        Mainboard(const std::string &port = std::string());
        ~Mainboard();

        void setStatusHandler(const StatusHandlerFunction &handler) {
            serial_impl.status_handler = handler;
        }

        void setIMUHandler(const IMUHandlerFunction &handler) {
            serial_impl.imu_handler = handler;
        }

        void setHeatbeatHandler(const HeatbeatHandler &handler) {
            serial_impl.heartbeat_handler = handler;
        }

        void setUIEventHandler(const UIEventHandler &handler) {
            serial_impl.ui_event_handler = handler;
        }

    private:
        class SerialImpl;
        std::unique_ptr<SerialImpl> serial_impl;
        void connect(const std::string &port);
    };
}