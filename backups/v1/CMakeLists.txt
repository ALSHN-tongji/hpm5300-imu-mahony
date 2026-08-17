# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

# ---- Custom board ----
# BOARD=user_board → SDK searches BOARD_SEARCH_PATH for user_board.yaml
set(BOARD user_board)
set(BOARD_SEARCH_PATH ${CMAKE_CURRENT_SOURCE_DIR}/..)

# ---- Custom linker scripts ----
set(CUSTOM_GCC_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/linkers/gcc/user_linker.ld)
set(CUSTOM_SES_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/linkers/segger/user_linker.icf)

# ---- Segger linker variant (standard) ----
# user_linker.icf provides __FLASH_segment_used_size__ (compatible with HPM_SES_USE_SEGGER_LD)
set(SES_LINKER_VARIANT "SEGGER")

# ---- Debug format: DWARF-4 (toolchain linker doesn't handle DWARF-5 relocs) ----
add_compile_options(-gdwarf-4)

# ---- Enable I2C ----
set(CONFIG_HPM_I2C 1)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(imu_app)

# ---- App sources ----
sdk_app_inc(src)
sdk_app_src(src/main.c)
sdk_app_src(src/mpu6050.c)
sdk_app_src(src/mahony.c)
sdk_app_src(src/ekf7.c)
sdk_app_src(src/madgwick.c)

generate_ide_projects()
