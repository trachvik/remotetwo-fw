cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(remote_two)

if(CONFIG_APP_DISP_TEST)
    target_sources(app PRIVATE
        src/disp_test.c
        src/haptic.c
        src/drivers/bldc_driver_3pwm.c
        src/drivers/bldc_motor.c
        src/drivers/tmag5170_sensor.c
    )
elseif(CONFIG_APP_DRV_TEST)
    target_sources(app PRIVATE
        src/DRV_test.c
        src/drivers/bldc_driver_3pwm.c
    )
else()
    target_sources(app PRIVATE
        src/main.c
        src/ui_display.c
        src/haptic.c
        src/remote_control.c
        src/BLE_commands.c
        src/drivers/tmag5170_sensor.c
        src/drivers/bldc_driver_3pwm.c
        src/drivers/bldc_motor.c
    )
endif()

target_include_directories(app PRIVATE src)
