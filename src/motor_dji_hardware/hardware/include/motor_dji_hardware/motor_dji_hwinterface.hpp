/**
 * @file motor_DJI_hwinterface.hpp
 * @brief DJI电机驱动器的硬件接口
 */

#ifndef MOTOR_DJI_HARDWARE__MOTOR_DJI_HWINTERFACE_HPP_
#define MOTOR_DJI_HARDWARE__MOTOR_DJI_HWINTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "CanDriver.hpp"
#include <thread>

using namespace sockcanpp;

namespace motor_dji_hardware {
    class MotorDjiHWInterface : public hardware_interface::SystemInterface, public CanDriver {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MotorDjiHWInterface)

    private:
        std::shared_ptr<CanDriver> can_device_;
        std::thread can_loop_thread_;
        bool stop_thread_;

        std::vector<double> state_;
        std::vector<double> command_;

        void canLoop();

    public:
        MotorDjiHWInterface() = default;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ~MotorDjiHWInterface() override;
    };
}; // namespace motor_dji_hardware

#endif  // MOTOR_DJI_HARDWARE__MOTOR_DJI_HWINTERFACE_HPP_
