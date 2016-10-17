F427_TARGETS    += $(TARGET)
FEATURES        += SDCARD VCP

HSE_VALUE = 24000000

TARGET_SRC = \
            drivers/accgyro_spi_mpu9250.c \
            drivers/accgyro_spi_icm20608.c \
            drivers/compass_ak8963.c
