#include "diffdrive_arduino/diffdrive_arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

DiffDriveArduino::DiffDriveArduino() : logger_(rclcpp::get_logger("DiffDriveArduino")) {}

CallbackReturn DiffDriveArduino::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Configuring...");

    time_ = std::chrono::system_clock::now();

    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    // Set up the wheels
    l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
    r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

    // Set up the Arduino
    arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

    this->laser_range_ = 0;
    this->servo_cmd_ = 0;

    RCLCPP_INFO(logger_, "Finished Configuration");

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces() {
    // We need to set up a position and a velocity interface for each wheel

    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

    state_interfaces.emplace_back(
        hardware_interface::StateInterface("laser", "range", &laser_range_));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces() {
    // We need to set up a velocity command interface for each wheel

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "gripper_right_joint", hardware_interface::HW_IF_EFFORT, &servo_cmd_));

    return command_interfaces;
}

CallbackReturn DiffDriveArduino::on_activate(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    RCLCPP_INFO(logger_, "Starting Controller...");

    arduino_.sendEmptyMsg();
    // arduino.setPidValues(9,7,0,100);
    // arduino.setPidValues(14,7,0,100);
    // arduino_.setPidValues(30, 20, 0, 100);

    return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveArduino::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    RCLCPP_INFO(logger_, "Stopping Controller...");

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduino::read(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
    (void)time;
    (void)period;

    // TODO fix chrono duration
    // Calculate time delta
    auto new_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_time - time_;
    double deltaSeconds = diff.count();
    time_ = new_time;

    if (!arduino_.connected()) {
        return return_type::ERROR;
    }

    arduino_.readEncoderValues(l_wheel_.enc, r_wheel_.enc);

    double pos_prev = l_wheel_.pos;
    l_wheel_.pos = l_wheel_.calcEncAngle();
    l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

    pos_prev = r_wheel_.pos;
    r_wheel_.pos = r_wheel_.calcEncAngle();
    r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;

    int laser_range_mm;
    arduino_.readLaserRange(laser_range_mm);
    laser_range_ = (double)laser_range_mm / 1000.0;

    return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::write(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
    (void)time;
    (void)period;

    if (!arduino_.connected()) {
        return return_type::ERROR;
    }

    // Map servo value from -0.1 to 0.1 to 35 to 75
    double servo_cmd_clamped = std::min(std::max(servo_cmd_, -0.1), 0.1);
    int servo_val = (int)(35.5 + (servo_cmd_clamped + 0.1) * 5 * (75 - 35));
    arduino_.setServoValue(servo_val);

    arduino_.setMotorValues(
        l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate,
        r_wheel_.cmd / r_wheel_.rads_per_count / cfg_.loop_rate);

    return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(DiffDriveArduino, hardware_interface::SystemInterface)