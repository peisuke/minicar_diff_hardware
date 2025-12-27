#ifndef ROBOT_HARDWARE__GPIO_MOTOR_CONTROLLER_HPP_
#define ROBOT_HARDWARE__GPIO_MOTOR_CONTROLLER_HPP_

#include <cstdint>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

namespace robot_hardware
{

class GPIOMotorController
{
public:
  GPIOMotorController();
  ~GPIOMotorController();

  bool initialize();
  void cleanup();
  bool is_initialized() const { return initialized_; }

  void set_motor_speed(int motor_index, double velocity_rad_per_sec);
  void stop_all_motors();

private:
  bool initialized_;
  double max_velocity_rad_per_sec_;
  
  // GPIO pin definitions (same as Python code)
  static constexpr int SPEED_PIN_R = 18;    // RIGHT PWM pin
  static constexpr int RIGHT_MOTOR_DIR_PIN1 = 23;  // Right Motor direction pin 1
  static constexpr int RIGHT_MOTOR_DIR_PIN2 = 24;  // Right Motor direction pin 2
  
  static constexpr int SPEED_PIN_L = 19;    // LEFT PWM pin
  static constexpr int LEFT_MOTOR_DIR_PIN1 = 27;   // Left Motor direction pin 1
  static constexpr int LEFT_MOTOR_DIR_PIN2 = 22;   // Left Motor direction pin 2
  
  static constexpr int PWM_FREQ = 1000;     // PWM frequency in Hz
  static constexpr int PWM_RANGE = 1024;    // PWM range (0-1023)
  
  // GPIO memory mapping
  static constexpr size_t GPIO_BASE = 0x00000000;  // Base for /dev/gpiomem
  static constexpr size_t GPIO_LENGTH = 0xB4;      // Length of GPIO registers
  volatile uint32_t* gpio_map_;
  int mem_fd_;
  
  // Software PWM threading
  std::thread pwm_thread_r_;
  std::thread pwm_thread_l_;
  std::atomic<bool> pwm_running_;
  std::atomic<int> pwm_value_r_;
  std::atomic<int> pwm_value_l_;
  std::mutex pwm_mutex_;
  
  // Helper functions
  void set_motor_direction_and_speed(int motor_index, double velocity);
  int velocity_to_pwm_value(double velocity_rad_per_sec);
  void gpio_write(int pin, int value);
  void pwm_write(int pin, int value);
  bool setup_gpio_memory();
  void cleanup_gpio_memory();
  void set_gpio_function(int pin, int function);
  
  // Software PWM functions
  void start_pwm_threads();
  void stop_pwm_threads();
  void pwm_thread_worker(int pin, std::atomic<int>& pwm_value);
  
  // GPIO register offsets
  static constexpr int GPFSEL0 = 0;   // GPIO Function Select 0
  static constexpr int GPFSEL1 = 1;   // GPIO Function Select 1  
  static constexpr int GPFSEL2 = 2;   // GPIO Function Select 2
  static constexpr int GPSET0 = 7;    // GPIO Pin Output Set 0
  static constexpr int GPCLR0 = 10;   // GPIO Pin Output Clear 0
};

} // namespace robot_hardware

#endif // ROBOT_HARDWARE__GPIO_MOTOR_CONTROLLER_HPP_