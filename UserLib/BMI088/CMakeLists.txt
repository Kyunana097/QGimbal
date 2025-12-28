cmake_minimum_required(VERSION 3.30)

project(bmi088)

add_library(bmi088 INTERFACE)

target_sources(bmi088 INTERFACE
        BMI088Driver/BMI088driver.c
        BMI088Driver/BMI088Middleware.c
        bsp_imu_pwm.c
        bsp_spi.c
        IMU_Task.cpp
        PID.c
)

target_include_directories(bmi088 INTERFACE
        ./
)