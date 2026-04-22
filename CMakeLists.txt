cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(remote_two)

target_sources(app PRIVATE
    src/main.c
    src/haptic.c
    src/remote_control.c
    src/BLE_commands.c
    src/ui_display.c
    src/drivers/as5048a.c
    src/drivers/bldc_driver_6pwm.c
    src/drivers/bldc_motor.c
)

target_include_directories(app PRIVATE src)
