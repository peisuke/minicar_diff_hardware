#ifndef ROBOT_HARDWARE__MINICAR_HARDWARE_INTERFACE_HPP_
#define ROBOT_HARDWARE__MINICAR_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace robot_hardware
{
class MinicarHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MinicarHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  // ホイール状態
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // エンコーダー関連
  std::vector<long> encoder_counts_;
  std::vector<double> last_encoder_positions_;
  
  // PCA9685制御用
  std::unique_ptr<class PCA9685Controller> pca9685_controller_;
  
  // ハードウェア初期化・制御関数
  bool initialize_hardware();
  void cleanup_hardware();
  void read_encoders();
  double encoder_counts_to_radians(long counts);
  
  // パラメータ
  double wheel_radius_;
  double encoder_ticks_per_revolution_;
  std::vector<std::string> joint_names_;
  std::string i2c_device_;
  uint8_t pca9685_address_;
};

}  // namespace robot_hardware

#endif  // ROBOT_HARDWARE__MINICAR_HARDWARE_INTERFACE_HPP_