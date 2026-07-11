#include "EtherCAT_Driver.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi1;

// Helper to handle CS (Adjust GPIO_PIN_x and PORT to your schematic)
#define ECAT_CS_LOW()  HAL_GPIO_WritePin(EASYCAT_CS_GPIO_Port, EASYCAT_CS_Pin, GPIO_PIN_RESET)
#define ECAT_CS_HIGH() HAL_GPIO_WritePin(EASYCAT_CS_GPIO_Port, EASYCAT_CS_Pin, GPIO_PIN_SET)

uint32_t ECAT_ReadReg(uint16_t address, uint8_t len) {
    uint8_t tx[12] = {0};
    uint8_t rx[12] = {0};
    uint32_t result = 0;

    if (len > 8) len = 8; // Prevent buffer overflow

    tx[0] = 0x03; // Read Command
    tx[1] = (address >> 8) & 0xFF;
    tx[2] = address & 0xFF;

    ECAT_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3 + len, 100);
    ECAT_CS_HIGH();

    for(int i = 0; i < len; i++) {
        result |= ((uint32_t)rx[3 + i] << (8 * i));
    }
    return result;
}

void ECAT_WriteReg(uint16_t address, uint32_t data, uint8_t len) {
    uint8_t tx[12] = {0};
    uint8_t rx[12] = {0};

    if (len > 8) len = 8; // Prevent buffer overflow

    tx[0] = 0x02; // Write Command
    tx[1] = (address >> 8) & 0xFF;
    tx[2] = address & 0xFF;
    for(int i = 0; i < len; i++) {
        tx[3 + i] = (data >> (8 * i)) & 0xFF;
    }

    ECAT_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3 + len, 100);
    ECAT_CS_HIGH();
}