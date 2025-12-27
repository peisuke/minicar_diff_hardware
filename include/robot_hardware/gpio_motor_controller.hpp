#ifndef ROBOT_HARDWARE__GPIO_MOTOR_CONTROLLER_HPP_
#define ROBOT_HARDWARE__GPIO_MOTOR_CONTROLLER_HPP_

#include <cstdint>
#include <string>

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
  static const int SPEED_PIN_R = 18;    // RIGHT PWM pin
  static const int RIGHT_MOTOR_DIR_PIN1 = 23;  // Right Motor direction pin 1
  static const int RIGHT_MOTOR_DIR_PIN2 = 24;  // Right Motor direction pin 2
  
  static const int SPEED_PIN_L = 19;    // LEFT PWM pin
  static const int LEFT_MOTOR_DIR_PIN1 = 27;   // Left Motor direction pin 1
  static const int LEFT_MOTOR_DIR_PIN2 = 22;   // Left Motor direction pin 2
  
  static const int PWM_FREQ = 1000;     // PWM frequency in Hz
  static const int PWM_RANGE = 1024;    // PWM range (0-1023)
  
  // GPIO memory mapping
  static const size_t GPIO_BASE = 0x00000000;  // Base for /dev/gpiomem
  static const size_t GPIO_LENGTH = 0xB4;      // Length of GPIO registers
  static const size_t PWM_BASE = 0x0020C000;   // PWM base address
  static const size_t PWM_LENGTH = 0x28;       // PWM registers length
  volatile uint32_t* gpio_map_;
  volatile uint32_t* pwm_map_;
  int mem_fd_;
  int pwm_fd_;
  
  // Helper functions
  void set_motor_direction_and_speed(int motor_index, double velocity);
  int velocity_to_pwm_value(double velocity_rad_per_sec);
  void gpio_write(int pin, int value);
  void pwm_write(int pin, int value);
  bool setup_gpio_memory();
  void cleanup_gpio_memory();
  void set_gpio_function(int pin, int function);
  
  // GPIO register offsets
  static const int GPFSEL0 = 0;   // GPIO Function Select 0
  static const int GPFSEL1 = 1;   // GPIO Function Select 1  
  static const int GPFSEL2 = 2;   // GPIO Function Select 2
  static const int GPSET0 = 7;    // GPIO Pin Output Set 0
  static const int GPCLR0 = 10;   // GPIO Pin Output Clear 0
  
  // PWM register offsets  
  static const int PWM_CTL = 0;   // PWM Control
  static const int PWM_STA = 1;   // PWM Status
  static const int PWM_DMAC = 2;  // PWM DMA Configuration
  static const int PWM_RNG1 = 4;  // PWM Channel 1 Range
  static const int PWM_DAT1 = 5;  // PWM Channel 1 Data
  static const int PWM_FIF1 = 6;  // PWM FIFO Input
  static const int PWM_RNG2 = 8;  // PWM Channel 2 Range
  static const int PWM_DAT2 = 9;  // PWM Channel 2 Data
};

} // namespace robot_hardware

#endif // ROBOT_HARDWARE__GPIO_MOTOR_CONTROLLER_HPP_