/**
 * @file motor_dji_hwinterface.hpp
 * @brief 大疆RoboMaster电机驱动器的硬件接口
 */
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "motor_dji_hardware/motor_dji_hwinterface.hpp"

#include <vector>

using namespace sockcanpp;
using namespace sockcanpp::exceptions;

namespace motor_dji_hardware {
    void MotorDjiHWInterface::canLoop() {
        while (!stop_thread_) {
            try {
                CanMessage canMessage = readMessage();
                std::stringstream ss;
                for (auto byte: canMessage.getFrameData()) {
                    ss << std::hex << std::setw(2) << std::setfill('0')
                            << (int) byte << " ";
                }
                RCLCPP_DEBUG_STREAM(
                    this->get_logger(),
                    "Received CAN message: [" << std::hex << canMessage.getRawFrame().can_id
                    << std::hex << "] Data=" << ss.str());

                can_frame frame = canMessage.getRawFrame();
                if ((int) frame.can_id == 0x205) {
                    int16_t angle = (frame.data[0] << 8) | frame.data[1];
                    int16_t velocity = (frame.data[2] << 8) | frame.data[3];
                    int16_t torque = (frame.data[4] << 8) | frame.data[5];

                    state_[0] = static_cast<double>(angle); // Mechanical angle
                    state_[1] = static_cast<double>(velocity); // Speed
                    state_[2] = static_cast<double>(torque); // Torque
                }
            } catch (std::exception &e) {
                RCLCPP_ERROR(
                    this->get_logger(), "Error receiving CAN message: %s",
                    e.what());
            }
            can_frame frame{};
            frame.can_id = 0x1ff;
            frame.can_dlc = 8;
            frame.data[0] = static_cast<int>(command_[0]) >> 8;
            frame.data[1] = static_cast<int>(command_[0]) & 0xff;
            can_device_->sendMessage(CanMessage(frame));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    hardware_interface::CallbackReturn
    MotorDjiHWInterface::on_init(const hardware_interface::HardwareInfo &info) {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        _defaultSenderId = 0;
        _canFilterMask = 0;
        _canProtocol = CAN_RAW;
        _canInterface = "can0";
        try {
            RCLCPP_INFO(
                this->get_logger(), "Activating cans interface %s",
                "can0");
            initialiseSocketCan();
            can_device_ = std::make_shared<CanDriver>("can0", CAN_RAW);
        }
        // catch any exception
        catch (std::exception &e) {
            RCLCPP_ERROR(
                this->get_logger(), "Error activating CAN interface: %s",
                e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        stop_thread_ = false;

        // Initialize state and command vectors
        state_.resize(3, 0.0); // [angle, velocity, torque]
        command_.resize(1, 0.0); // [desired_voltage]

        RCLCPP_INFO(
            this->get_logger(), "Activated hardware interface can0");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MotorDjiHWInterface::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("MotorDjiHardware"), "Configuring hardware");
        RCLCPP_INFO(rclcpp::get_logger("MotorDjiHardware"), "Configured hardware");
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> MotorDjiHWInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (const auto &joint: info_.joints) {
            state_interfaces.emplace_back(
                joint.name, hardware_interface::HW_IF_POSITION, &state_[0]);
            state_interfaces.emplace_back(
                joint.name, hardware_interface::HW_IF_VELOCITY, &state_[1]);
            state_interfaces.emplace_back(
                joint.name, hardware_interface::HW_IF_TORQUE, &state_[2]);

            RCLCPP_INFO(
               get_logger(),
               "Joint '%s' state interface exported", joint.name.c_str());

        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> MotorDjiHWInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (const auto &joint: info_.joints) {
            command_interfaces.emplace_back(
                joint.name, "voltage", &command_[0]);
            RCLCPP_INFO(
               get_logger(),
               "Joint '%s' command interface exported", joint.name.c_str());
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn MotorDjiHWInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        can_loop_thread_ = std::thread(&MotorDjiHWInterface::canLoop, this);
        RCLCPP_INFO(rclcpp::get_logger("MotorDjiHardware"), "Activating hardware");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MotorDjiHWInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("MotorDjiHardware"), "Deactivating hardware");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type
    MotorDjiHWInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
    MotorDjiHWInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Motor state: [angle, velocity, torque] = [%f, %f, %f]",
            state_[0], state_[1], state_[2]);
        return hardware_interface::return_type::OK;
    }

    MotorDjiHWInterface::~MotorDjiHWInterface() {
        stop_thread_ = true;
        if (can_loop_thread_.joinable()) {
            can_loop_thread_.join();
        }
        if (can_device_) {
            CanDriver::uninitialiseSocketCan();
            can_device_.reset();
        }
    }
};

// Export to pluginlib
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(motor_dji_hardware::MotorDjiHWInterface, hardware_interface::SystemInterface)
