cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(remote_two)

target_sources(app PRIVATE
    # src/main.c
    # src/pwr_ctrl_early.c
    # src/ui_display.c
    # src/haptic.c
    # src/remote_control.c
    # src/BLE_commands.c
    # src/drivers/tmag5170_encoder.c
    # src/drivers/bldc_driver_3pwm.c
    # src/drivers/bldc_motor.c
    src/TMAG_test.c
)

target_include_directories(app PRIVATE src)
