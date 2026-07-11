# soes-stm32f3

## 🧰 Overview
This repository contains my port of **SOES (Simple Open EtherCAT Slave)** to the **STM32F303VBTX**.  
It is based on the official SOES stack, with modifications specific to STM32F3 hardware and my application.  
The EtherCAT Slave Controller (ESC) is accessed using a **polling-based driver** rather than interrupts or DMA.

---

## 🔑 My Modifications

### 1. STM32F3 Port (`lib/osal/stm32f3_port/`)
- Implemented **polling-based ESC access** for STM32F3.  
- Removed dependency on DMA/interrupt-driven operation.  
- Added minimal HAL functions for:
  - SPI read/write in polling mode
  - Timer tick handling for SOES main loop
  - PHY reset and basic configuration

### 2. ESC Driver Integration (`lib/drivers/`)
- Adapted the SOES ESC driver to STM32 HAL SPI (polling).  
- Simplified buffer management to work without ISR callbacks.  
- Ensures deterministic cycle timing by checking ESC registers in the main loop.

### 3. Object Dictionary (`lib/soes/objdict.c`)
- Extended with **custom PDOs/SDOs** for application-specific process data.  
- Updated `ecat_appl.c` to handle read/write of these objects.  
- PDO mapping synchronized with the TwinCAT XML configuration.

### 4. TwinCAT Configuration (`twincat/`)
- Created a custom XML slave description for TwinCAT.  
- Mapped PDO entries from the modified `objdict.c`.  
- Verified with Beckhoff TwinCAT master (slave detected and process data exchanged).

---

## ⚙️ Build & Run
1. Open `soes-stm32f3.ioc` in **STM32CubeIDE**.  
2. Build the project and flash it onto the STM32F3 via **ST-Link**.  
3. Connect the device to an EtherCAT master (e.g., TwinCAT).  
4. Import the XML from `twincat/` into your master project.  
5. Scan the bus → the STM32F3 slave will appear with your custom PDO mapping.

---

## 🧪 Testing
- Verified EtherCAT communication in **polling mode** (no DMA/interrupts).  
- Tested with TwinCAT System Manager → PDOs exchanged successfully.  
- Debug messages available via UART/GPIO.  

---

## 📄 License
This repository follows the license of the upstream SOES project (GPLv2).  
All STM32F3 polling-based porting and application-specific modifications are authored by me.
