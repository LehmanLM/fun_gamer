cmake_minimum_required(VERSION 3.12.4)

# 匹配 my_component 根目录下的所有 .c 文件
file(GLOB MY_COMPONENT_SRCS "*.c")

# 注册组件
idf_component_register(
    SRCS ${MY_COMPONENT_SRCS}
    INCLUDE_DIRS "."
    REQUIRES esp_rom soc nofrendo
)