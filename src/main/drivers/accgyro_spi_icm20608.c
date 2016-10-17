/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "io.h"

#include "system.h"
#include "exti.h"
#include "bus_spi.h"
#include "gyro_sync.h"
#include "light_led.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_spi_icm20608.h"

#define DISABLE_ICM20608       IOHi(icmSpi20608CsPin)
#define ENABLE_ICM20608        IOLo(icmSpi20608CsPin)

static IO_t icmSpi20608CsPin = IO_NONE;
 
bool icm20608WriteRegister(uint8_t reg, uint8_t data)
{
    ENABLE_ICM20608;
    spiTransferByte(ICM20608_SPI_INSTANCE, reg);
    spiTransferByte(ICM20608_SPI_INSTANCE, data);
    DISABLE_ICM20608;

    return true;
}

bool icm20608ReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_ICM20608;
    spiTransferByte(ICM20608_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(ICM20608_SPI_INSTANCE, data, NULL, length);
    DISABLE_ICM20608;

    return true;
}

static void icm20608SpiInit(void)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }

    icmSpi20608CsPin = IOGetByTag(IO_TAG(ICM20608_CS_PIN));
    IOInit(icmSpi20608CsPin, OWNER_MPU, RESOURCE_SPI_CS, 0);
    IOConfigGPIO(icmSpi20608CsPin, SPI_IO_CS_CFG);

    spiSetDivisor(ICM20608_SPI_INSTANCE, SPI_CLOCK_FAST);

    hardwareInitialised = true;
}


bool icm20608SpiDetect(void)
{
    uint8_t tmp;
    uint8_t attemptsRemaining = 20;
    
    icm20608SpiInit();
    
    spiSetDivisor(ICM20608_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON); //low speed

    icm20608WriteRegister(MPU_RA_PWR_MGMT_1, ICM20608_BIT_RESET);

    do {
        delay(150);

        icm20608ReadRegister(MPU_RA_WHO_AM_I, 1, &tmp);
        if (tmp == ICM20608_WHO_AM_I_CONST) {
            break;
        }
        if (!attemptsRemaining) {
            return false;
        }
    } while (attemptsRemaining--);

    spiSetDivisor(ICM20608_SPI_INSTANCE, SPI_CLOCK_FAST);

    return true;
}

bool icm20608SpiAccDetect(acc_t *acc)
{
    if (mpuDetectionResult.sensor != ICM_20608_SPI) {
        return false;
    }

    acc->init = icm20608AccInit;
    acc->read = mpuAccRead;

    return true;
}

bool icm20608SpiGyroDetect(gyro_t *gyro)
{
    if (mpuDetectionResult.sensor != ICM_20608_SPI) {
        return false;
    }

    gyro->init = icm20608GyroInit;
    gyro->read = mpuGyroRead;
    gyro->intStatus = checkMPUDataReady;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}

void icm20608AccInit(acc_t *acc)
{
    mpuIntExtiInit();

    acc->acc_1G = 512 * 8;
}

void icm20608GyroInit(uint8_t lpf)
{
    mpuIntExtiInit();
    
    spiSetDivisor(ICM20608_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON);

    mpuConfiguration.write(MPU_RA_PWR_MGMT_1, ICM20608_BIT_RESET);
    delay(100);
    mpuConfiguration.write(MPU_RA_SIGNAL_PATH_RESET, 0x03);
    delay(100);
    mpuConfiguration.write(MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    delay(15);
    mpuConfiguration.write(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    delay(15);
    mpuConfiguration.write(MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);
    delay(15);
    mpuConfiguration.write(MPU_RA_CONFIG, lpf);
    delay(15);
    mpuConfiguration.write(MPU_RA_SMPLRT_DIV, gyroMPU6xxxCalculateDivider());
    delay(100);

    // Data ready interrupt configuration
    mpuConfiguration.write(MPU_RA_INT_PIN_CFG, 0x10);  // INT_RD_CLEAR

    delay(15);
    
#ifdef USE_MPU_DATA_READY_SIGNAL
    mpuConfiguration.write(MPU_RA_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable
#endif

    spiSetDivisor(ICM20608_SPI_INSTANCE, SPI_CLOCK_FAST);
}