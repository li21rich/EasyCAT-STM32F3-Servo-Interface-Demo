#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // Required for usleep
#include <ecrt.h>

#define VENDOR_ID       0x0000079A
#define PRODUCT_CODE    0x00DEFEDE

// Master pointers
static ec_master_t *master = NULL;
static ec_domain_t *domain1 = NULL;
static uint8_t *domain1_pd = NULL;

// Offsets for the PDO entries
static unsigned int off_rx_byte0;
static unsigned int off_tx_byte0;

int main() {
    master = ecrt_request_master(0);
    if (!master) return -1;

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) return -1;

    ec_slave_config_t *sc = ecrt_master_slave_config(master, 0, 0, VENDOR_ID, PRODUCT_CODE);
    if (!sc) return -1;

    // Register entries using the ESC's hardware PDO indices (0x0005 = output, 0x0006 = input)
    // These are the raw byte registers of the EasyCAT LAN9252 ESC, NOT CANopen object dictionary indices.
    ec_pdo_entry_reg_t domain_regs[] = {
        {0, 0, VENDOR_ID, PRODUCT_CODE, 0x0005, 0x01, &off_rx_byte0}, // RxPDO: outputs to slave (byte 0)
        {0, 0, VENDOR_ID, PRODUCT_CODE, 0x0006, 0x01, &off_tx_byte0}, // TxPDO: inputs from slave (byte 0)
        {}
    };

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain_regs)) {
        printf("ERROR: Failed to register PDO entries. Check vendor/product code and PDO mapping.\n");
        return -1;
    }

    ecrt_master_activate(master);
    domain1_pd = ecrt_domain_data(domain1);

    printf("EtherCAT Master Active. Starting control loop...\n");

    uint32_t counter = 0;
    uint8_t servo_angle = 90; // start at center (90 degrees)

    while (1) {
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        // --- DATA EXCHANGE AREA ---
        // Sweep servo angle 0..180 then back
        if (counter % 2000 == 0) {
            static int8_t dir = 1;
            if (servo_angle >= 180) dir = -1;
            if (servo_angle <= 0)   dir = 1;
            servo_angle += dir * 10;
            printf("[cycle %u] Servo angle: %d deg\n", counter, servo_angle);
        }

        // Write servo angle as first output byte → mapped to led[0].state in firmware
        domain1_pd[off_rx_byte0] = servo_angle;

        // Read: Print every 1000 cycles
        if (counter % 1000 == 0) {
            printf("[cycle %u] Sent: 0x%02X | Received first 8 bytes: ", 
                    counter, domain1_pd[off_rx_byte0]);

            for (int i = 0; i < 8; i++) {
                printf("%02X ", domain1_pd[i]);
            }
            printf("\n");
        }

        counter++;
        // --------------------------

        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        
        usleep(1000); // 1ms cycle time
    }

    return 0;
}