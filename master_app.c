#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>
#include <ecrt.h>

#define VENDOR_ID       0x0000079A
#define PRODUCT_CODE    0x00DEFEDE

static ec_master_t *master = NULL;
static ec_domain_t *domain = NULL;
static ec_slave_config_t *sc = NULL;

static uint8_t *domain_pd = NULL;

static int off_rx;
static int off_tx;

static volatile sig_atomic_t running = 1;

static void sigint_handler(int sig)
{
    (void)sig;
    running = 0;
}

int main()
{
    signal(SIGINT, sigint_handler);

    master = ecrt_request_master(0);
    if (!master) {
        fprintf(stderr, "Failed master\n");
        return -1;
    }

    domain = ecrt_master_create_domain(master);
    if (!domain) {
        fprintf(stderr, "Failed domain\n");
        return -1;
    }

    sc = ecrt_master_slave_config(master, 0, 0, VENDOR_ID, PRODUCT_CODE);
    if (!sc) {
        fprintf(stderr, "Failed slave config\n");
        return -1;
    }

    /* Reverting to the 1-byte mapping that gave WC=3 */
    ec_pdo_entry_info_t pdo_entries[] = {
        {0x0005, 0x01, 8}, // Output Byte 0 (servo angle)
        {0x0006, 0x01, 8}  // Input Byte 0 (echo)
    };

    ec_pdo_info_t pdo_info[] = {
        {0x1600, 1, &pdo_entries[0]},
        {0x1A00, 1, &pdo_entries[1]}
    };

    ec_sync_info_t syncs[] = {
        {2, EC_DIR_OUTPUT, 1, &pdo_info[0], EC_WD_ENABLE},
        {3, EC_DIR_INPUT,  1, &pdo_info[1], EC_WD_DISABLE},
        {0xFF}
    };

    ecrt_slave_config_pdos(sc, EC_END, syncs);

    /* 2. Register PDO entries to get memory offsets */
    // this should not be necessary. remove this:
    off_tx = ecrt_slave_config_reg_pdo_entry(sc, 0x0005, 0x01, domain, NULL);
    off_rx = ecrt_slave_config_reg_pdo_entry(sc, 0x0006, 0x01, domain, NULL);
    
    if (off_tx < 0 || off_rx < 0) {
        fprintf(stderr, "PDO registration failed! off_tx=%d, off_rx=%d\n", off_tx, off_rx);
        return -1;
    }

    if (ecrt_master_activate(master)) {
        fprintf(stderr, "Activation failed\n");
        return -1;
    }

    domain_pd = ecrt_domain_data(domain);
    if (!domain_pd) {
        fprintf(stderr, "No domain data\n");
        return -1;
    }

    printf("EtherCAT Master Active | off_rx=%d, off_tx=%d\n", off_rx, off_tx);

    uint32_t counter = 0;
    uint8_t angle = 180;

    while (running && counter < 3001) { // i added auto stop
        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        
        /* Write command */
        domain_pd[off_tx] = angle;
        
        /* Read echo */
        uint8_t tx_echo = domain_pd[off_rx];

        if (counter % 500 == 0) {
            ec_domain_state_t ds;
            ecrt_domain_state(domain, &ds);
            
            printf("[%u] TX sent=%u | RX echo=%u | WC=%d\n",
                   counter, angle, tx_echo, ds.working_counter);
            
            // Debug: print a few bytes around the offsets
            printf("Domain raw: ");
            for(int i=0; i<16; i++) printf("%02X ", domain_pd[i]);
            printf("\n");
        }

        angle = (angle >= 180) ? 0 : angle + 1;

        ecrt_domain_queue(domain);
        ecrt_master_send(master);

        usleep(3000);
        counter++;
    }

    printf("Shutdown\n");
    ecrt_master_deactivate(master);
    ecrt_release_master(master);

    return 0;
}
