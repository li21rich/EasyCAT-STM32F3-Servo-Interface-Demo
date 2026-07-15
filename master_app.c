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
static ec_slave_config_t *sc = NULL;          
static ec_master_state_t master_state = {};
static ec_domain_state_t domain1_state = {};
static ec_slave_config_state_t sc_state = {};

static void check_state(void)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
    if (ms.link_up != master_state.link_up || ms.al_states != master_state.al_states) {
        printf("Master: link %s, al_states=0x%X, slaves_responding=%u\n",
               ms.link_up ? "up" : "DOWN", ms.al_states, ms.slaves_responding);
    }
    master_state = ms;

    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);
    if (ds.working_counter != domain1_state.working_counter ||
        ds.wc_state != domain1_state.wc_state) {
        static const char *wc_name[] = {"ZERO", "INCOMPLETE", "COMPLETE"};
        printf("Domain: wkc=%u (%s)\n", ds.working_counter, wc_name[ds.wc_state]);
    }
    domain1_state = ds;

    ec_slave_config_state_t s;
    ecrt_slave_config_state(sc, &s);
    if (s.al_state != sc_state.al_state || s.online != sc_state.online ||
        s.operational != sc_state.operational) {
        printf("Slave: al_state=%u (1=INIT,2=PREOP,4=SAFEOP,8=OP) online=%d operational=%d\n",
               s.al_state, s.online, s.operational);
    }
    sc_state = s;
}
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

    sc = ecrt_master_slave_config(master, 0, 0, VENDOR_ID, PRODUCT_CODE);
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
    ////// manual fix
    ecrt_slave_config_pdos(sc, EC_END, NULL);

    // 2. Define the entries manually to match your Object Dictionary
    static ec_pdo_entry_info_t rx_pdo_entries[] = {
        {0x7000, 0x01, 8}, // Maps to Obj.led[0].state
    };

    static ec_pdo_info_t rx_pdos[] = {
        {0x1600, 1, rx_pdo_entries}, // 0x1600 is the RxPDO index
    };

    static ec_pdo_entry_info_t tx_pdo_entries[] = {
        {0x6000, 0x01, 8}, // Adjust based on needed TX mapping
    };

    static ec_pdo_info_t tx_pdos[] = {
        {0x1A00, 1, tx_pdo_entries}, // 0x1A00 is the TxPDO index
    };

    // 3. Apply the configuration
    if (ecrt_slave_config_pdos(sc, EC_END, NULL)) {
        fprintf(stderr, "Failed to clear default PDOs\n"); return -1;
    }
    if (ecrt_slave_config_pdo_assign_add(sc, 2, 0x1600)) { // SM2 = Rx
        fprintf(stderr, "Failed to assign RxPDO\n"); return -1;
    }
    if (ecrt_slave_config_pdo_assign_add(sc, 3, 0x1A00)) { // SM3 = Tx
        fprintf(stderr, "Failed to assign TxPDO\n"); return -1;
    }  /////// manual fix

    if (ecrt_master_activate(master)) {
        fprintf(stderr, "Failed to activate master\n");
        return -1;
    }
    
    domain1_pd = ecrt_domain_data(domain1);
    if (!domain1_pd) {
        fprintf(stderr, "Failed to get domain data pointer\n");
        return -1;
    }
    printf("EtherCAT Master Active. Starting control loop...\n");
    printf("Ctrl+C to exit cleanly.\n");

    uint32_t counter = 0;
    uint8_t servo_angle = 90; // start center

    while (running) {
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
        check_state();
        
        if (counter % 800 == 0) {
            static int8_t dir = 1;
            if (servo_angle >= 150) dir = -1;
            if (servo_angle <= 30)  dir = 1;
            servo_angle += dir * 20;
        }

        // Write servo angle to output byte
        domain1_pd[off_rx_byte0] = servo_angle;
        
        if (counter % 800 == 0) {
            printf("[%u] Angle: %3d°  Tx:0x%02X  Rx:", counter, servo_angle,
                   domain1_pd[off_rx_byte0]);
            for (int i = 0; i < 8; i++) {
                printf("%02X ", domain1_pd[i]);
            }
            printf("\n");
        }

        counter++;

        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        usleep(3000);
    }

    /* Clean shutdown on Ctrl+C */
    printf("\nShutting down... releasing master.\n");
    ecrt_master_deactivate(master);
    ecrt_release_master(master);
    printf("Master released. Goodbye.\n");
    return 0;
}