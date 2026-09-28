# 源文件：只收 .c（HTML 等资源不能被当源码编译）
file(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.c)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "."
    REQUIRES esp_driver_twai
)
