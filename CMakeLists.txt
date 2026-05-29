cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(remote_two)

if (CONFIG_APP_DRV_TEST)
    # DRV8311H 3PWM open-loop driver verification
    target_sources(app PRIVATE
        src/DRV_test.c
        src/drivers/bldc_driver_3pwm.c
    )
elseif (CONFIG_APP_TMAG_TEST)
    # TMAG5170 sensor stand-alone test
    target_sources(app PRIVATE
        src/TMAG_test.c
    )
else()
    # Main application
    target_sources(app PRIVATE
        src/main.c
        src/ui_display.c
        src/haptic.c
        src/remote_control.c
        src/BLE_commands.c
        src/drivers/as5048a.c
        src/drivers/bldc_driver_3pwm.c
        src/drivers/bldc_motor.c
    )
endif()

target_include_directories(app PRIVATE src)
