cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Flight_Control)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Driver/DEBUG/Src/debug.c
    HALPort/DWT/Src/dwt.c
    HALPort/LED/Src/led.c
    HALPort/I2C_SOFT/Src/i2c_soft.c
    HALPort/IWDG_PORT/Src/iwdg_port.c
    Middleware/KALMAN_FILTER/Src/kalman_filter.c
    Driver/MPU6050/Src/mpu6050.c
    Driver/SPL06/Src/spl06.c
    HALPort/SPI_PORT/Src/spi_port.c
    HALPort/NRF24L01_GPIO/Src/nrf24l01_gpio.c
    Driver/NRF24L01/Src/nrf24l01.c
    HALPort/PWM_PORT/Src/pwm_port.c
    Driver/MOTOR/Src/motor.c
    HALPort/ADC_PORT/Src/adc_port.c
    Driver/BATTERY/Src/battery.c
    Middleware/MY_MATH/Src/my_math.c
    Middleware/IMU/Src/imu.c
    Middleware/ANO_DT/Src/ano_dt.c
    Middleware/REMOTE/Src/remote.c
    HALPort/UART1_PORT/Src/uart1_port.c
    HALPort/EEPROM_EMU/Src/eeprom_emu.c
    Middleware/FILTER/Src/filter.c
    Middleware/PID/Src/pid.c
    Middleware/FLOW/Src/flow.c
    Middleware/CONTROL/Src/control.c
    Middleware/CONTROL/Src/control_attitude.c
    Middleware/CONTROL/Src/control_mixer.c
    Middleware/CONTROL/Src/control_height.c
    Middleware/CONTROL/Src/control_descent.c
    Middleware/CONTROL/Src/control_mode.c
    Middleware/CONTROL/Src/control_position.c
    Middleware/ANO_PARAM/Src/ano_param.c
    Middleware/LED_CTRL/Src/led_ctrl.c
    Middleware/SCHEDULER/Src/scheduler.c
    Middleware/APP/Src/app.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Driver/DEBUG/Inc
    HALPort/DWT/Inc
    HALPort/LED/Inc
    HALPort/I2C_SOFT/Inc
    HALPort/IWDG_PORT/Inc
    Middleware/KALMAN_FILTER/Inc
    Driver/MPU6050/Inc
    Driver/SPL06/Inc
    HALPort/SPI_PORT/Inc
    HALPort/NRF24L01_GPIO/Inc
    Driver/NRF24L01/Inc
    HALPort/PWM_PORT/Inc
    Driver/MOTOR/Inc
    HALPort/ADC_PORT/Inc
    Driver/BATTERY/Inc
    Middleware/MY_MATH/Inc
    Middleware/IMU/Inc
    Middleware/ANO_DT/Inc
    Middleware/REMOTE/Inc
    HALPort/UART1_PORT/Inc
    HALPort/EEPROM_EMU/Inc
    Middleware/FILTER/Inc
    Middleware/PID/Inc
    Middleware/FLOW/Inc
    Middleware/CONTROL/Inc
    Middleware/ANO_PARAM/Inc
    Middleware/LED_CTRL/Inc
    Middleware/SCHEDULER/Inc
    Middleware/APP/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
 
# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# 启用 newlib-nano 的 printf 浮点支持（%f/%e/%g），否则浮点打印为空
# 必须用 target_link_options，不能写在 target_link_libraries 里（会被拆成两项）
target_link_options( ${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)

# 保持 -g3 可调试，但用 -Os 压体积（后出现的优化等级覆盖前面的 -O0）
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Os -g3)
