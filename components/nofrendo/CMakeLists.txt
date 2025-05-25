cmake_minimum_required(VERSION 3.12.4)

# 匹配 my_component 根目录下的所有 .c 文件
file(GLOB MY_COMPONENT_SRCS "*.c")

# 匹配 A/、B/、C/ 子目录下的所有 .c 文件
file(GLOB MY_COMPONENT_SRCS_CPU "cpu/*.c")
file(GLOB MY_COMPONENT_SRCS_LIBSNSS "libsnss/*.c")
file(GLOB MY_COMPONENT_SRCS_MAPPERS "mappers/*.c")
file(GLOB MY_COMPONENT_SRCS_NES "nes/*.c")
file(GLOB MY_COMPONENT_SRCS_SNDHRDW "sndhrdw/*.c")

# 合并所有 .c 文件
list(APPEND MY_COMPONENT_SRCS
    ${MY_COMPONENT_SRCS_CPU}
    ${MY_COMPONENT_SRCS_LIBSNSS}
    ${MY_COMPONENT_SRCS_MAPPERS}
    ${MY_COMPONENT_SRCS_NES}
    ${MY_COMPONENT_SRCS_SNDHRDW}
)

# 注册组件
idf_component_register(
    SRCS ${MY_COMPONENT_SRCS}
    INCLUDE_DIRS "." "cpu" "libsnss" "mappers" "nes" "sndhrdw"
    REQUIRES nofrendo-esp32
)

# 添加编译选项
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=char-subscripts
    -Wno-error=attributes
    -Wno-error=misleading-indentation
)