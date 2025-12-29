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
  
  // Velocity scaling parameter setter
  bool set_max_velocity_rad_per_sec(double max_velocity);

private:
  // I2C通信
  int i2c_fd_;
  uint8_t pca9685_addr_;
  
  // PCA9685レジスタ
  static const uint8_t MODE1 = 0x00;
  static const uint8_t MODE2 = 0x01;
  static const uint8_t PRESCALE = 0xFE;
  static const uint8_t LED0_ON_L = 0x06;
  
  // モーターチャンネル設定 (OSOYOO PWM HAT V2.0)
  static const int RIGHT_MOTOR_PWM_CH = 0;  // Channel 0 for right motor PWM
  static const int LEFT_MOTOR_PWM_CH = 1;   // Channel 1 for left motor PWM
  
  // 方向制御は引き続きGPIO使用
  static const int RIGHT_MOTOR_DIR_PIN1 = 23;
  static const int RIGHT_MOTOR_DIR_PIN2 = 24;
  static const int LEFT_MOTOR_DIR_PIN1 = 27;
  static const int LEFT_MOTOR_DIR_PIN2 = 22;
  
  // 制御パラメータ
  double max_velocity_rad_per_sec_;
  
  // GPIO制御用 (方向制御のため)
  volatile uint32_t* gpio_map_;
  int mem_fd_;
  static constexpr size_t GPIO_BASE = 0x00000000;
  static constexpr size_t GPIO_LENGTH = 0xB4;
  static constexpr int GPFSEL0 = 0;
  static constexpr int GPFSEL1 = 1;
  static constexpr int GPFSEL2 = 2;
  static constexpr int GPSET0 = 7;
  static constexpr int GPCLR0 = 10;
  
  // I2C通信ヘルパー
  void i2c_write8(uint8_t reg, uint8_t value);
  uint8_t i2c_read8(uint8_t reg);
  void pca9685_set_pwm(int channel, uint16_t on, uint16_t off);
  
  // モーター制御ヘルパー
  uint16_t velocity_to_pwm_duty(double velocity_rad_per_sec);
  void set_motor_direction_and_speed(int motor_index, double velocity);
  
  // GPIO制御ヘルパー
  bool setup_gpio_memory();
  void cleanup_gpio_memory();
  void set_gpio_function(int pin, int function);
  void gpio_write(int pin, int value);
};

} // namespace robot_hardware

#endif // ROBOT_HARDWARE__PCA9685_CONTROLLER_HPP_