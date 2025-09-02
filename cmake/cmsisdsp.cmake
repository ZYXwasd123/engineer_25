# 设置CMSIS-DSP的根目录
set(CMSIS_DSP_ROOT "C:/abc/keil_packs/arm/CMSIS-DSP/1.16.2")

# 设置必要的CMSIS-DSP选项
set(NEON OFF CACHE BOOL "Neon acceleration")
set(LOOPUNROLL ON CACHE BOOL "Loop unrolling")
set(ROUNDING ON CACHE BOOL "Rounding")
set(MATRIXCHECK ON CACHE BOOL "Matrix Checks")
set(LAXVECTORCONVERSIONS ON CACHE BOOL "Lax vector conversions")

# 添加CMSIS-DSP源代码目录
add_subdirectory(
        "${CMSIS_DSP_ROOT}/Source"
        "${CMAKE_SOURCE_DIR}/libs"
)

# 为库添加本项目的头文件搜索路径 ${INCLUDE_DIR} 为主目录下的头文件路径
target_include_directories(CMSISDSP PUBLIC ${INCLUDE_DIR})

# 链接库
target_link_libraries(${PROJECT_NAME} PRIVATE CMSISDSP)
