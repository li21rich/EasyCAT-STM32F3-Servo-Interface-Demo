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

static unsigned int off_rx;
static unsigned int off_tx;

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
    if (!master)
    {
        fprintf(stderr,"Failed master\n");
        return -1;
    }


    domain = ecrt_master_create_domain(master);
    if (!domain)
    {
        fprintf(stderr,"Failed domain\n");
        return -1;
    }


    sc = ecrt_master_slave_config(
        master,
        0,
        0,
        VENDOR_ID,
        PRODUCT_CODE
    );

    if (!sc)
    {
        fprintf(stderr,"Failed slave config\n");
        return -1;
    }
    /* 1. Define the variables we are sending/receiving */
    ec_pdo_entry_info_t slave_pdo_entries[] = {
        {0x0005, 0x01, 8}, /* RxPDO variable (8-bit) */
        {0x0006, 0x01, 8}, /* TxPDO variable (8-bit) */
    };

    /* 2. Map the variables into PDOs */
    ec_pdo_info_t slave_pdos[] = {
        {0x1600, 1, slave_pdo_entries + 0}, /* Assign to RxPDO */
        {0x1A00, 1, slave_pdo_entries + 1}, /* Assign to TxPDO */
    };

    /* 3. Assign the PDOs to the exact Sync Managers SOES expects (SM2/SM3) */
    ec_sync_info_t slave_syncs[] = {
        {2, EC_DIR_OUTPUT, 1, slave_pdos + 0, EC_WD_ENABLE},  /* SM2 (Output) */
        {3, EC_DIR_INPUT,  1, slave_pdos + 1, EC_WD_DISABLE}, /* SM3 (Input) */
        {0xFF}
    };

    /* 4. Force the Master to use this configuration */
    if (ecrt_slave_config_pdos(sc, EC_END, slave_syncs)) {
        fprintf(stderr, "Failed to configure slave PDOs\n");
        return -1;
    }

    /* 5. Register the PDO entries to get memory offsets */
    ec_pdo_entry_reg_t domain_regs[] =
    {
        {
            0,            /* Alias */
            0,            /* Position */
            VENDOR_ID,
            PRODUCT_CODE,
            0x0005,       /* Index (Slave RX / Master TX) */
            0x01,         /* Subindex */
            &off_tx       /* Target offset variable */
        },
        {
            0,            /* Alias */
            0,            /* Position */
            VENDOR_ID,
            PRODUCT_CODE,
            0x0006,       /* Index (Slave TX / Master RX) */
            0x01,         /* Subindex */
            &off_rx       /* Target offset variable */
        },
        {}                /* Empty termination struct */
    };

    /* 6. Register the list to the domain */
    if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs))
    {
        fprintf(stderr,"PDO registration failed\n");
        return -1;
    }



    if (ecrt_master_activate(master))
    {
        fprintf(stderr,"Activation failed\n");
        return -1;
    }


    domain_pd = ecrt_domain_data(domain);

    if (!domain_pd)
    {
        fprintf(stderr,"No domain data\n");
        return -1;
    }


    printf("EtherCAT Master Active\n");

    printf("RX offset=%u\n",off_rx);
    printf("TX offset=%u\n",off_tx);



    uint32_t counter = 0;
    uint8_t angle = 180;


while(running)
{
    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    /* 
     * 1. Send servo command (Master TX -> Slave RX)
     * Write to off_tx (offset 0)
     */
    domain_pd[off_tx] = angle;

    if(counter % 500 == 0)
    {
        /* 
         * 2. Read the echo back (Master RX <- Slave TX)
         * Read from off_rx (offset 32)
         */
        uint8_t tx_echo = domain_pd[off_rx];

        // Tip: Labeled as 'TX angle' here to match Master perspective (sending out)
        printf(
            "[%u] TX angle sent=%u | RX echo received=%u\n",
            counter,
            angle,
            tx_echo
        );
    }

    angle++;
    if(angle > 180)
        angle = 0;

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