cmake_minimum_required(VERSION 3.12.4)

# ƥ�� my_component ��Ŀ¼�µ����� .c �ļ�
file(GLOB MY_COMPONENT_SRCS "psxcontroller.c" "*.c")

# ע�����?
idf_component_register(
    SRCS ${MY_COMPONENT_SRCS}
    INCLUDE_DIRS "."
    REQUIRES esp_rom soc nofrendo esp_driver_gpio esp_driver_i2s
)