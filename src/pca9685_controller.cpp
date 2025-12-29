#include "robot_hardware/pca9685_controller.hpp"

#include <iostream>
#include <limits>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <algorithm>

namespace robot_hardware
{

PCA9685Controller::PCA9685Controller()
: i2c_fd_(-1), pca9685_addr_(0x40), max_velocity_rad_per_sec_(1.0), gpio_map_(nullptr), mem_fd_(-1)
{
}

PCA9685Controller::~PCA9685Controller()
{
  cleanup();
}

bool PCA9685Controller::initialize(const std::string& i2c_device, uint8_t address)
{
  pca9685_addr_ = address;
  
  // I2C デバイスオープン
  i2c_fd_ = open(i2c_device.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    std::cerr << "Failed to open I2C device: " << i2c_device << std::endl;
    return false;
  }

  // PCA9685 をスレーブとして選択
  if (ioctl(i2c_fd_, I2C_SLAVE, pca9685_addr_) < 0) {
    std::cerr << "Failed to set I2C_SLAVE address: 0x" << std::hex << (int)pca9685_addr_ << std::endl;
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  // GPIO setup for direction control
  if (!setup_gpio_memory()) {
    std::cerr << "Failed to setup GPIO memory mapping" << std::endl;
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  // Configure direction pins as outputs
  set_gpio_function(RIGHT_MOTOR_DIR_PIN1, 1);
  set_gpio_function(RIGHT_MOTOR_DIR_PIN2, 1);
  set_gpio_function(LEFT_MOTOR_DIR_PIN1, 1);
  set_gpio_function(LEFT_MOTOR_DIR_PIN2, 1);

  // PCA9685 初期化
  i2c_write8(MODE1, 0x00);  // 通常モード
  i2c_write8(MODE2, 0x04);  // OUTDRV
  set_pwm_frequency(1000.0f); // 1kHz
  
  // 全モーター停止
  stop_all_motors();
  
  std::cout << "PCA9685 initialized successfully" << std::endl;
  return true;
}

void PCA9685Controller::cleanup()
{
  if (i2c_fd_ >= 0) {
    stop_all_motors();
    close(i2c_fd_);
    i2c_fd_ = -1;
  }
  cleanup_gpio_memory();
}

void PCA9685Controller::set_pwm_frequency(float freq_hz)
{
  if (i2c_fd_ < 0) return;
  
  // データシートの計算式: prescale = round(25MHz / (4096 * freq)) - 1
  float prescaleval = 25000000.0f / (4096.0f * freq_hz) - 1.0f;
  uint8_t prescale = static_cast<uint8_t>(std::floor(prescaleval + 0.5f));

  uint8_t oldmode = i2c_read8(MODE1);
  uint8_t sleep = (oldmode & 0x7F) | 0x10; // sleep ビットセット
  i2c_write8(MODE1, sleep);
  i2c_write8(PRESCALE, prescale);
  i2c_write8(MODE1, oldmode);
  usleep(5000); // 5ms
  i2c_write8(MODE1, oldmode | 0xa1); // auto-increment 有効 & restart
}

void PCA9685Controller::set_channel_pwm(int channel, uint16_t duty)
{
  if (i2c_fd_ < 0) return;
  if (duty > 4095) duty = 4095;
  
  // ON=0, OFF=duty という形で出力
  pca9685_set_pwm(channel, 0, duty);
}

void PCA9685Controller::set_motor_speed(int motor_index, double velocity_rad_per_sec)
{
  if (i2c_fd_ < 0) return;
  
  set_motor_direction_and_speed(motor_index, velocity_rad_per_sec);
}

void PCA9685Controller::stop_all_motors()
{
  if (i2c_fd_ < 0) return;
  
  // PWM停止
  set_channel_pwm(RIGHT_MOTOR_PWM_CH, 0);
  set_channel_pwm(LEFT_MOTOR_PWM_CH, 0);
  
  // 方向ピンを停止状態に
  if (gpio_map_) {
    gpio_write(RIGHT_MOTOR_DIR_PIN1, 0);
    gpio_write(RIGHT_MOTOR_DIR_PIN2, 0);
    gpio_write(LEFT_MOTOR_DIR_PIN1, 0);
    gpio_write(LEFT_MOTOR_DIR_PIN2, 0);
  }
}

void PCA9685Controller::i2c_write8(uint8_t reg, uint8_t value)
{
  if (i2c_fd_ < 0) return;
  
  uint8_t buf[2] = {reg, value};
  if (write(i2c_fd_, buf, 2) != 2) {
    std::cerr << "I2C write error" << std::endl;
  }
}

uint8_t PCA9685Controller::i2c_read8(uint8_t reg)
{
  if (i2c_fd_ < 0) return 0;
  
  if (write(i2c_fd_, &reg, 1) != 1) {
    std::cerr << "I2C write (for read) error" << std::endl;
    return 0;
  }
  
  uint8_t value;
  if (read(i2c_fd_, &value, 1) != 1) {
    std::cerr << "I2C read error" << std::endl;
    return 0;
  }
  
  return value;
}

void PCA9685Controller::pca9685_set_pwm(int channel, uint16_t on, uint16_t off)
{
  if (i2c_fd_ < 0) return;
  
  uint8_t reg = LED0_ON_L + 4 * channel;
  uint8_t buf[5];
  buf[0] = reg;
  buf[1] = on & 0xFF;
  buf[2] = (on >> 8) & 0x0F;
  buf[3] = off & 0xFF;
  buf[4] = (off >> 8) & 0x0F;

  if (write(i2c_fd_, buf, 5) != 5) {
    std::cerr << "I2C write set_pwm error" << std::endl;
  }
}

uint16_t PCA9685Controller::velocity_to_pwm_duty(double velocity_rad_per_sec)
{
  // 速度の絶対値を0-4095のPWMデューティに変換
  // Guard against invalid scaling to avoid division-by-zero / NaN.
  if (!(max_velocity_rad_per_sec_ > 0.0) || !std::isfinite(max_velocity_rad_per_sec_))
  {
    return 0;
  }

  double abs_velocity = std::abs(velocity_rad_per_sec);
  
  // Deadzone: 微小速度でのモーター音を防ぐため、最小閾値以下は停止
  const double MIN_VELOCITY_THRESHOLD = 0.1; // 0.1 rad/s以下は停止
  if (abs_velocity < MIN_VELOCITY_THRESHOLD)
  {
    return 0;
  }
  
  double normalized = std::min(abs_velocity / max_velocity_rad_per_sec_, 1.0);
  return static_cast<uint16_t>(4095 * normalized);
}

void PCA9685Controller::set_motor_direction_and_speed(int motor_index, double velocity)
{
  if (i2c_fd_ < 0 || !gpio_map_) return;
  
  uint16_t pwm_duty = velocity_to_pwm_duty(velocity);
  bool forward = velocity >= 0;
  
  if (motor_index == 0) { // 右モーター
    // GPIO方向制御
    gpio_write(RIGHT_MOTOR_DIR_PIN1, forward ? 1 : 0);
    gpio_write(RIGHT_MOTOR_DIR_PIN2, forward ? 0 : 1);
    // PCA9685 PWM制御
    set_channel_pwm(RIGHT_MOTOR_PWM_CH, pwm_duty);
  }
  else if (motor_index == 1) { // 左モーター  
    // GPIO方向制御
    gpio_write(LEFT_MOTOR_DIR_PIN1, forward ? 1 : 0);
    gpio_write(LEFT_MOTOR_DIR_PIN2, forward ? 0 : 1);
    // PCA9685 PWM制御
    set_channel_pwm(LEFT_MOTOR_PWM_CH, pwm_duty);
  }
}

bool PCA9685Controller::setup_gpio_memory()
{
  // Open /dev/gpiomem
  mem_fd_ = open("/dev/gpiomem", O_RDWR | O_SYNC);
  if (mem_fd_ < 0) {
    std::cerr << "Failed to open /dev/gpiomem" << std::endl;
    return false;
  }
  
  // Memory map GPIO registers
  gpio_map_ = (volatile uint32_t*)mmap(
    nullptr,           // Any address
    GPIO_LENGTH,       // Size of mapping
    PROT_READ | PROT_WRITE, // Read/Write access
    MAP_SHARED,        // Shared mapping
    mem_fd_,          // File descriptor
    GPIO_BASE          // Offset
  );
  
  if (gpio_map_ == MAP_FAILED) {
    std::cerr << "Failed to mmap GPIO memory" << std::endl;
    close(mem_fd_);
    mem_fd_ = -1;
    return false;
  }
  
  return true;
}

void PCA9685Controller::cleanup_gpio_memory()
{
  if (gpio_map_) {
    munmap((void*)gpio_map_, GPIO_LENGTH);
    gpio_map_ = nullptr;
  }
  
  if (mem_fd_ >= 0) {
    close(mem_fd_);
    mem_fd_ = -1;
  }
}

void PCA9685Controller::set_gpio_function(int pin, int function)
{
  if (!gpio_map_) return;
  
  int reg_index = pin / 10;
  int bit_offset = (pin % 10) * 3;
  
  // Clear the 3 bits for this pin
  gpio_map_[reg_index] &= ~(7 << bit_offset);
  
  // Set the function (1 = output, 0 = input)
  gpio_map_[reg_index] |= (function & 7) << bit_offset;
}

void PCA9685Controller::gpio_write(int pin, int value)
{
  if (!gpio_map_) return;
  
  if (value) {
    // Set pin HIGH using GPSET0 register
    gpio_map_[GPSET0] = 1 << pin;
  } else {
    // Set pin LOW using GPCLR0 register  
    gpio_map_[GPCLR0] = 1 << pin;
  }
}

bool PCA9685Controller::set_max_velocity_rad_per_sec(double max_velocity)
{
  if (max_velocity <= 0.0) {
    std::cerr << "Invalid max_velocity_rad_per_sec: " << max_velocity 
              << " (must be > 0)" << std::endl;
    return false;
  }
  
  max_velocity_rad_per_sec_ = max_velocity;
  std::cout << "Updated max_velocity_rad_per_sec to: " << max_velocity_rad_per_sec_ << std::endl;
  return true;
}

} // namespace robot_hardware