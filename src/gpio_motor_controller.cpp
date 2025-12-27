#include "robot_hardware/gpio_motor_controller.hpp"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

namespace robot_hardware
{

GPIOMotorController::GPIOMotorController()
: initialized_(false), max_velocity_rad_per_sec_(1.0), gpio_map_(nullptr), mem_fd_(-1),
  pwm_running_(false), pwm_value_r_(0), pwm_value_l_(0)
{
}

GPIOMotorController::~GPIOMotorController()
{
  cleanup();
}

bool GPIOMotorController::initialize()
{
  if (!setup_gpio_memory()) {
    std::cerr << "Failed to setup GPIO memory mapping" << std::endl;
    return false;
  }
  
  // Configure GPIO pins as outputs
  set_gpio_function(RIGHT_MOTOR_DIR_PIN1, 1); // Output
  set_gpio_function(RIGHT_MOTOR_DIR_PIN2, 1); // Output  
  set_gpio_function(LEFT_MOTOR_DIR_PIN1, 1);  // Output
  set_gpio_function(LEFT_MOTOR_DIR_PIN2, 1);  // Output
  set_gpio_function(SPEED_PIN_R, 1);          // Output
  set_gpio_function(SPEED_PIN_L, 1);          // Output
  
  // Start PWM threads
  start_pwm_threads();
  
  // Initialize motors to stopped state
  stop_all_motors();
  
  initialized_ = true;
  std::cout << "GPIO Motor Controller initialized successfully" << std::endl;
  return true;
}

void GPIOMotorController::cleanup()
{
  if (initialized_) {
    stop_all_motors();
    stop_pwm_threads();
    cleanup_gpio_memory();
    initialized_ = false;
  }
}


void GPIOMotorController::set_motor_speed(int motor_index, double velocity_rad_per_sec)
{
  if (!initialized_) return;
  
  set_motor_direction_and_speed(motor_index, velocity_rad_per_sec);
}

void GPIOMotorController::stop_all_motors()
{
  if (!initialized_) return;
  
  // Stop right motor
  gpio_write(RIGHT_MOTOR_DIR_PIN1, 0);
  gpio_write(RIGHT_MOTOR_DIR_PIN2, 0);
  pwm_write(SPEED_PIN_R, 0);
  
  // Stop left motor
  gpio_write(LEFT_MOTOR_DIR_PIN1, 0);
  gpio_write(LEFT_MOTOR_DIR_PIN2, 0);
  pwm_write(SPEED_PIN_L, 0);
}

void GPIOMotorController::set_motor_direction_and_speed(int motor_index, double velocity)
{
  if (!initialized_) return;
  
  int pwm_value = velocity_to_pwm_value(velocity);
  bool forward = velocity >= 0;
  
  if (motor_index == 0) { // Right motor
    gpio_write(RIGHT_MOTOR_DIR_PIN1, forward ? 1 : 0);
    gpio_write(RIGHT_MOTOR_DIR_PIN2, forward ? 0 : 1);
    pwm_write(SPEED_PIN_R, pwm_value);
  }
  else if (motor_index == 1) { // Left motor
    gpio_write(LEFT_MOTOR_DIR_PIN1, forward ? 1 : 0);
    gpio_write(LEFT_MOTOR_DIR_PIN2, forward ? 0 : 1);
    pwm_write(SPEED_PIN_L, pwm_value);
  }
}

int GPIOMotorController::velocity_to_pwm_value(double velocity_rad_per_sec)
{
  // Convert velocity to PWM value (0-1023)
  double abs_velocity = std::abs(velocity_rad_per_sec);
  double normalized = std::min(abs_velocity / max_velocity_rad_per_sec_, 1.0);
  int pwm_value = static_cast<int>(PWM_RANGE * normalized);
  
  // Set minimum PWM value for motor to actually turn
  if (pwm_value > 0 && pwm_value < 200) {
    pwm_value = 200;  // Minimum PWM to ensure motor rotation
  }
  
  return pwm_value;
}

void GPIOMotorController::gpio_write(int pin, int value)
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

void GPIOMotorController::pwm_write(int pin, int value)
{
  // Update PWM values for threaded software PWM
  if (pin == SPEED_PIN_R) {
    pwm_value_r_ = value;
  } else if (pin == SPEED_PIN_L) {
    pwm_value_l_ = value;
  }
}

bool GPIOMotorController::setup_gpio_memory()
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

void GPIOMotorController::cleanup_gpio_memory()
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

void GPIOMotorController::set_gpio_function(int pin, int function)
{
  if (!gpio_map_) return;
  
  int reg_index = pin / 10;
  int bit_offset = (pin % 10) * 3;
  
  // Clear the 3 bits for this pin
  gpio_map_[reg_index] &= ~(7 << bit_offset);
  
  // Set the function (1 = output, 0 = input)
  gpio_map_[reg_index] |= (function & 7) << bit_offset;
}

void GPIOMotorController::start_pwm_threads()
{
  pwm_running_ = true;
  pwm_thread_r_ = std::thread(&GPIOMotorController::pwm_thread_worker, this, SPEED_PIN_R, std::ref(pwm_value_r_));
  pwm_thread_l_ = std::thread(&GPIOMotorController::pwm_thread_worker, this, SPEED_PIN_L, std::ref(pwm_value_l_));
}

void GPIOMotorController::stop_pwm_threads()
{
  pwm_running_ = false;
  
  if (pwm_thread_r_.joinable()) {
    pwm_thread_r_.join();
  }
  
  if (pwm_thread_l_.joinable()) {
    pwm_thread_l_.join();
  }
}

void GPIOMotorController::pwm_thread_worker(int pin, std::atomic<int>& pwm_value)
{
  const int pwm_period_us = 1000000 / PWM_FREQ;  // PWM period in microseconds
  
  while (pwm_running_) {
    int current_value = pwm_value.load();
    
    if (current_value == 0) {
      // Always OFF
      gpio_write(pin, 0);
      usleep(pwm_period_us);
    } else if (current_value >= PWM_RANGE) {
      // Always ON
      gpio_write(pin, 1);
      usleep(pwm_period_us);
    } else {
      // PWM duty cycle
      int on_time_us = (current_value * pwm_period_us) / PWM_RANGE;
      int off_time_us = pwm_period_us - on_time_us;
      
      gpio_write(pin, 1);
      usleep(on_time_us);
      gpio_write(pin, 0);
      usleep(off_time_us);
    }
  }
  
  // Ensure pin is OFF when thread stops
  gpio_write(pin, 0);
}

} // namespace robot_hardware