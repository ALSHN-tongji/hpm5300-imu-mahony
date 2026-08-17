# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

set(BOARD user_board)
set(BOARD_SEARCH_PATH ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(CUSTOM_GCC_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/linkers/gcc/user_linker.ld)
set(CUSTOM_SES_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/linkers/segger/user_linker.icf)
set(SES_LINKER_VARIANT "SEGGER")
add_compile_options(-gdwarf-4)
set(CONFIG_HPM_I2C 1)

# Set to 1 to accelerate mahony_dsp.c with Andes RISC-V DSP intrinsics
set(CONFIG_HPM_MATH_DSP 0)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})
project(imu_app)

sdk_app_inc(src)
sdk_app_src(src/main.c)
sdk_app_src(src/mpu6050.c)
sdk_app_src(src/mahony.c)
sdk_app_src(src/mahony_dsp.c)
sdk_app_src(src/ekf7.c)
sdk_app_src(src/madgwick.c)

generate_ide_projects()
