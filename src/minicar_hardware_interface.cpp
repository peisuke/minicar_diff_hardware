#include "robot_hardware/minicar_hardware_interface.hpp"
#include "robot_hardware/pca9685_controller.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hardware
{

hardware_interface::CallbackReturn MinicarHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // パラメータ読み込み
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  encoder_ticks_per_revolution_ = std::stod(info_.hardware_parameters["encoder_ticks_per_revolution"]);
  
  // I2C/PCA9685設定読み込み
  i2c_device_ = info_.hardware_parameters.count("i2c_device") ? 
                info_.hardware_parameters["i2c_device"] : "/dev/i2c-1";
  pca9685_address_ = info_.hardware_parameters.count("pca9685_address") ? 
                     std::stoi(info_.hardware_parameters["pca9685_address"]) : 0x40;

  // ジョイント情報設定
  joint_names_.clear();
  hw_commands_.resize(info_.joints.size());
  hw_positions_.resize(info_.joints.size());
  hw_velocities_.resize(info_.joints.size());
  encoder_counts_.resize(info_.joints.size());
  last_encoder_positions_.resize(info_.joints.size());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
    
    // 各ジョイントが位置・速度インターフェースを持つことを確認
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MinicarHardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MinicarHardwareInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MinicarHardwareInterface"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // PCA9685コントローラー初期化
  pca9685_ = std::make_unique<PCA9685Controller>();
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MinicarHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MinicarHardwareInterface"), "Configuring ...please wait...");

  // PCA9685初期化
  if (!initialize_hardware())
  {
    RCLCPP_FATAL(rclcpp::get_logger("MinicarHardwareInterface"), "Failed to initialize hardware");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 初期値設定
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    hw_commands_[i] = 0;
    hw_positions_[i] = 0;
    hw_velocities_[i] = 0;
    encoder_counts_[i] = 0;
    last_encoder_positions_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("MinicarHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MinicarHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MinicarHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MinicarHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MinicarHardwareInterface"), "Activating ...please wait...");
  
  // モーター有効化（停止状態で開始）
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    hw_commands_[i] = 0;
  }
  
  if (pca9685_ && pca9685_->is_initialized())
  {
    pca9685_->stop_all_motors();
  }

  RCLCPP_INFO(rclcpp::get_logger("MinicarHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MinicarHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MinicarHardwareInterface"), "Deactivating ...please wait...");
  
  // モーター停止
  if (pca9685_ && pca9685_->is_initialized())
  {
    pca9685_->stop_all_motors();
  }

  RCLCPP_INFO(rclcpp::get_logger("MinicarHardwareInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MinicarHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // エンコーダー読み取り
  read_encoders();
  
  // 位置・速度計算
  for (size_t i = 0; i < encoder_counts_.size(); i++)
  {
    double position = encoder_counts_to_radians(encoder_counts_[i]);
    double velocity = (position - last_encoder_positions_[i]) / period.seconds();
    
    hw_positions_[i] = position;
    hw_velocities_[i] = velocity;
    last_encoder_positions_[i] = position;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MinicarHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // モーター制御コマンド適用
  if (pca9685_ && pca9685_->is_initialized())
  {
    // 左右のモーターに速度を設定
    // TODO: 実際のモーター配置に応じて調整が必要
    for (size_t i = 0; i < hw_commands_.size(); i++)
    {
      int motor_index = i / 2; // 前後輪を同じモーターとして扱う
      pca9685_->set_motor_speed(motor_index, hw_commands_[i]);
    }
  }

  return hardware_interface::return_type::OK;
}

bool MinicarHardwareInterface::initialize_hardware()
{
  if (!pca9685_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MinicarHardwareInterface"), "PCA9685 controller not created");
    return false;
  }
  
  if (!pca9685_->initialize(i2c_device_, pca9685_address_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("MinicarHardwareInterface"), "Failed to initialize PCA9685");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("MinicarHardwareInterface"), "Hardware initialized successfully");
  return true;
}

void MinicarHardwareInterface::cleanup_hardware()
{
  if (pca9685_)
  {
    pca9685_->cleanup();
  }
}

void MinicarHardwareInterface::read_encoders()
{
  // TODO: 実際のエンコーダー読み取り実装
  // 現在は簡易的な実装（シミュレーション用）
  static long left_count = 0;
  static long right_count = 0;
  
  // エンコーダー読み取りロジックを実装
  // Raspberry PiのGPIOを使ったロータリーエンコーダー読み取り
  
  // 現在は仮の値を設定
  if (encoder_counts_.size() >= 4)
  {
    encoder_counts_[0] = left_count;   // left_front_wheel
    encoder_counts_[1] = right_count;  // right_front_wheel
    encoder_counts_[2] = left_count;   // left_rear_wheel
    encoder_counts_[3] = right_count;  // right_rear_wheel
  }
}

double MinicarHardwareInterface::encoder_counts_to_radians(long counts)
{
  return (counts * 2.0 * M_PI) / encoder_ticks_per_revolution_;
}

}  // namespace robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_hardware::MinicarHardwareInterface, hardware_interface::SystemInterface)