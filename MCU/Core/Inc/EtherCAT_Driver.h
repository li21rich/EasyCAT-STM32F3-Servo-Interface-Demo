#ifndef ETHERCAT_DRIVER_H
#define ETHERCAT_DRIVER_H

#include "main.h"

// LAN9252 Register Addresses
#define LAN9252_BYTE_TEST   0x0064
#define LAN9252_ID_REV      0x0050
#define LAN9252_RESET_CTL   0x01F8

// Basic SPI Helper
void ECAT_WriteReg(uint16_t address, uint32_t data, uint8_t len);
uint32_t ECAT_ReadReg(uint16_t address, uint8_t len);

#endif