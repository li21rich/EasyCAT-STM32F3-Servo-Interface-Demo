#include "EtherCAT_Driver.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi1;

// Helper to handle CS (Adjust GPIO_PIN_x and PORT to your schematic)
#define ECAT_CS_LOW()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define ECAT_CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
uint32_t ECAT_ReadReg(uint16_t address, uint8_t len) {
    uint8_t tx[4] = {0x03, (address >> 8) & 0xFF, address & 0xFF, 0x00};
    uint8_t rx[8] = {0};
    uint32_t result = 0;

    ECAT_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 3, 100);
    HAL_SPI_Receive(&hspi1, rx, len, 100);
    ECAT_CS_HIGH();

    for(int i = 0; i < len; i++) result |= (rx[i] << (8 * i));
    return result;
}

void ECAT_WriteReg(uint16_t address, uint32_t data, uint8_t len) {
    uint8_t tx[8];
    tx[0] = 0x02; // 0x02 is Write Command
    tx[1] = (address >> 8) & 0xFF;
    tx[2] = address & 0xFF;
    for(int i = 0; i < len; i++) tx[3 + i] = (data >> (8 * i)) & 0xFF;

    ECAT_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 3 + len, 100);
    ECAT_CS_HIGH();
}