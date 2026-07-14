#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <ecrt.h>

#define VENDOR_ID       0x0000079A
#define PRODUCT_CODE    0x00DEFEDE
#define CYCLE_US        1000        // 1 ms cycle

// Master pointers
static ec_master_t *master = NULL;
static ec_domain_t *domain1 = NULL;
static uint8_t *domain1_pd = NULL;

// Offsets for the PDO entries
static unsigned int off_rx_byte0;
static unsigned int off_tx_byte0;

// For clean shutdown on Ctrl+C
static volatile int running = 1;
void handle_signal(int sig) {
    (void)sig;
    running = 0;
}

// Elapsed time in ms since first call
static uint64_t elapsed_ms(void) {
    static struct timespec t0 = {0};
    struct timespec now;
    if (t0.tv_sec == 0) clock_gettime(CLOCK_MONOTONIC, &t0);
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t)(now.tv_sec - t0.tv_sec) * 1000 +
           (now.tv_nsec - t0.tv_nsec) / 1000000;
}

int main() {
    signal(SIGINT, handle_signal);

    master = ecrt_request_master(0);
    if (!master) { fprintf(stderr, "Failed to request master 0\n"); return -1; }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) { fprintf(stderr, "Failed to create domain\n"); return -1; }

    ec_slave_config_t *sc = ecrt_master_slave_config(master, 0, 0, VENDOR_ID, PRODUCT_CODE);
    if (!sc) { fprintf(stderr, "Failed to get slave config\n"); return -1; }

    // Register entries using the ESC's hardware PDO indices (0x0005 = output, 0x0006 = input)
    // These are the raw byte registers of the EasyCAT LAN9252 ESC, NOT CANopen object dictionary indices.
    ec_pdo_entry_reg_t domain_regs[] = {
        {0, 0, VENDOR_ID, PRODUCT_CODE, 0x0005, 0x01, &off_rx_byte0}, // RxPDO: outputs to slave (byte 0)
        {0, 0, VENDOR_ID, PRODUCT_CODE, 0x0006, 0x01, &off_tx_byte0}, // TxPDO: inputs from slave (byte 0)
        {}
    };

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain_regs)) {
        fprintf(stderr, "ERROR: Failed to register PDO entries. Check vendor/product code and PDO mapping.\n");
        return -1;
    }

    ecrt_master_activate(master);
    domain1_pd = ecrt_domain_data(domain1);

    printf("\n");
    printf("╔══════════════════════════════════════════════╗\n");
    printf("║   EtherCAT Master — EasyCAT Servo Demo      ║\n");
    printf("╠══════════════════════════════════════════════╣\n");
    printf("║  Vendor:  0x%08X (AB&T)              ║\n", VENDOR_ID);
    printf("║  Product: 0x%08X (EasyCAT 32+32)     ║\n", PRODUCT_CODE);
    printf("║  Cycle:   %d us                              ║\n", CYCLE_US);
    printf("║  RxPDO:   ESC reg 0x0005 (output to slave)   ║\n");
    printf("║  TxPDO:   ESC reg 0x0006 (input from slave)  ║\n");
    printf("╚══════════════════════════════════════════════╝\n");
    printf("\n");

    uint32_t counter = 0;
    uint8_t servo_angle = 90; // start at center (90 degrees)
    int8_t dir = 1;

    while (running) {
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        // --- DATA EXCHANGE AREA ---
        // Sweep servo angle 0..180 then back
        if (counter % 2000 == 0) {
            if (servo_angle >= 180) dir = -1;
            if (servo_angle <= 0)   dir = 1;
            servo_angle += dir * 10;
        }

        // Write servo angle as first output byte → mapped to led[0].state in firmware
        domain1_pd[off_rx_byte0] = servo_angle;

        // Status line: print every 500 cycles (~500 ms)
        if (counter % 500 == 0) {
            uint64_t ms = elapsed_ms();
            uint8_t rx0 = domain1_pd[off_tx_byte0]; // first byte received back

            printf("[%5llu ms] [cycle %6u] TX: angle=%3d deg (0x%02X) | "
                   "RX: byte0=0x%02X (%3d) | raw: ",
                   (unsigned long long)ms, counter,
                   servo_angle, servo_angle, rx0, rx0);

            for (int i = 0; i < 8; i++) {
                printf("%02X ", domain1_pd[i]);
            }
            printf("\n");
        }

        counter++;
        // --------------------------

        ecrt_domain_queue(domain1);
        ecrt_master_send(master);

        usleep(CYCLE_US);
    }

    printf("\nShutting down master...\n");
    ecrt_master_deactivate(master);
    printf("Done.\n");
    return 0;
}
