#ifndef ROBOT_HARDWARE__PCA9685_CONTROLLER_HPP_
#define ROBOT_HARDWARE__PCA9685_CONTROLLER_HPP_

#include <cstdint>
#include <string>

namespace robot_hardware
{

class PCA9685Controller
{
public:
  PCA9685Controller();
  ~PCA9685Controller();

  bool initialize(const std::string& i2c_device = "/dev/i2c-1", uint8_t address = 0x40);
  void cleanup();
  bool is_initialized() const { return i2c_fd_ >= 0; }

  void set_pwm_frequency(float freq_hz);
  void set_channel_pwm(int channel, uint16_t duty);
  void set_motor_speed(int motor_index, double velocity_rad_per_sec);
  void stop_all_motors();

private:
  // I2C通信
  int i2c_fd_;
  uint8_t pca9685_addr_;
  
  // PCA9685レジスタ
  static const uint8_t MODE1 = 0x00;
  static const uint8_t MODE2 = 0x01;
  static const uint8_t PRESCALE = 0xFE;
  static const uint8_t LED0_ON_L = 0x06;
  
  // モーターチャンネル設定
  static const int PWM_R = 0;  // Right motor speed
  static const int R_IN1 = 1;
  static const int R_IN2 = 2;
  static const int PWM_L = 5;  // Left motor speed  
  static const int L_IN1 = 3;
  static const int L_IN2 = 4;
  
  // 制御パラメータ
  double max_velocity_rad_per_sec_;
  
  // I2C通信ヘルパー
  void i2c_write8(uint8_t reg, uint8_t value);
  uint8_t i2c_read8(uint8_t reg);
  void pca9685_set_pwm(int channel, uint16_t on, uint16_t off);
  
  // モーター制御ヘルパー
  uint16_t velocity_to_pwm_duty(double velocity_rad_per_sec);
  void set_motor_direction_and_speed(int motor_index, double velocity);
};

} // namespace robot_hardware

#endif // ROBOT_HARDWARE__PCA9685_CONTROLLER_HPP_