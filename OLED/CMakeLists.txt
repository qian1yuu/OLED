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


# ===================== STM32F103C8T6 浮点配置 =====================
add_compile_options(
        -mcpu=cortex-m3
        -mfloat-abi=soft
        -mthumb
        -O0 -g  # 新增这行：调试用，避免浮点出错
)

# 链接器选项（补全浮点printf支持）
add_link_options(
        -mcpu=cortex-m3
        -mfloat-abi=soft
        -mthumb
        -u _printf_float  # 浮点数printf核心
)


# ===================== 浮点配置结束 =====================



# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME OLED)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Src/main.c  # 新增：程序入口，必须加
        User/Inc/oled.h
        User/Src/oled.c
        User/Inc/oledfont.h
        Mycode/Inc/dmpKey.h
        Mycode/Inc/dmpmap.h
        Mycode/Inc/IIC.h
        Mycode/Inc/inv_mpu.h
        Mycode/Inc/inv_mpu_dmp_motion_driver.h
        Mycode/Inc/mpu6050.h
        Mycode/Src/dmpKey.c
        Mycode/Src/dmpmap.c
        Mycode/Src/IIC.c
        Mycode/Src/inv_mpu.c
        Mycode/Src/inv_mpu_dmp_motion_driver.c
        Mycode/Src/mpu6050.c
        User/Src/motor.c
        User/Inc/motor.h
        User/Src/encoder.c
        User/Inc/encoder.h
        User/Src/PID.c
        User/Inc/PID.h
        User/Src/sr04.c
        User/Inc/sr04.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
        User/Src/oled.c
        User/Src/motor.c
        User/Src/encoder.c
        User/Src/PID.c
        User/Src/sr04.c
        User/Src/ai.c

        Mycode/Src/dmpKey.c
        Mycode/Src/dmpmap.c
        Mycode/Src/IIC.c
        Mycode/Src/inv_mpu.c
        Mycode/Src/inv_mpu_dmp_motion_driver.c
        Mycode/Src/mpu6050.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        User/Inc
        Mycode/Inc
        Core/Inc  # 新增：main.c的头文件路径
        cmake/stm32cubemx/Inc  # 新增：HAL库头文件路径
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
