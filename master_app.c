#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <ecrt.h>

#define VENDOR_ID       0x0000079A
#define PRODUCT_CODE    0x00DEFEDE

static ec_master_t *master = NULL;
static ec_domain_t *domain1 = NULL;
static uint8_t *domain1_pd = NULL;

static unsigned int off_rx_byte0;
static unsigned int off_tx_byte0;

static volatile sig_atomic_t running = 1;

void sigint_handler(int sig) {
    (void)sig;
    running = 0;
}

int main() {
    signal(SIGINT, sigint_handler);

    master = ecrt_request_master(0);
    if (!master) { fprintf(stderr, "Failed to request master\n"); return -1; }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) { fprintf(stderr, "Failed to create domain\n"); return -1; }

    ec_slave_config_t *sc = ecrt_master_slave_config(master, 0, 0, VENDOR_ID, PRODUCT_CODE);
    if (!sc) { fprintf(stderr, "Failed to get slave config\n"); return -1; }

    ec_pdo_entry_reg_t domain_regs[] = {
        {0, 0, VENDOR_ID, PRODUCT_CODE, 0x0005, 0x01, &off_rx_byte0},
        {0, 0, VENDOR_ID, PRODUCT_CODE, 0x0006, 0x01, &off_tx_byte0},
        {}
    };

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain_regs)) {
        fprintf(stderr, "ERROR: Failed to register PDO entries.\n");
        return -1;
    }

    ecrt_master_activate(master);
    domain1_pd = ecrt_domain_data(domain1);

    printf("EtherCAT Master Active. Starting control loop...\n");
    printf("Servo sweep: 50° → 130° → 50° (5° steps, 2s interval)\n");
    printf("Ctrl+C to exit cleanly.\n");

    uint32_t counter = 0;
    uint8_t servo_angle = 90; // start center

    while (running) {
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        // Check domain state every 1000 cycles
        if (counter % 1000 == 0) {
            ec_domain_state_t ds;
            ecrt_domain_state(domain1, &ds);
            if (ds.working_counter == 0) {
                printf("[%u] WARN: domain working_counter=0 (slave not responding?)\n", counter);
            }
        }

        // Sweep 50–130 at 5° steps, every 2 seconds (2000 cycles)
        if (counter % 2000 == 0) {
            static int8_t dir = 1;
            if (servo_angle >= 130) dir = -1;
            if (servo_angle <= 50)  dir = 1;
            servo_angle += dir * 5;
        }

        // Write servo angle to output byte
        domain1_pd[off_rx_byte0] = servo_angle;

        // Per-cycle print: angle + Tx byte + Rx first 8 bytes
        printf("[%u] Angle: %3d°  Tx:0x%02X  Rx:", counter, servo_angle,
               domain1_pd[off_rx_byte0]);
        for (int i = 0; i < 8; i++) {
            printf("%02X ", domain1_pd[i]);
        }
        printf("\n");

        counter++;

        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        usleep(1000);
    }

    /* Clean shutdown on Ctrl+C */
    printf("\nShutting down... releasing master.\n");
    ecrt_master_deactivate(master);
    ecrt_release_master(master);
    printf("Master released. Goodbye.\n");
    return 0;
}