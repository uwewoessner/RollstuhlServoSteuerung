#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <string.h>
#include <sched.h> /* sched_setscheduler() */

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

// Application parameters
#define FREQUENCY 1000
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define MEASURE_TIMING

/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
        (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
#define JMCDrive 0x66668888,0x20199307

/****************************************************************************/
/* PDO entries offsets */
static struct {
    unsigned int ctrl_word;
    unsigned int target_torque;
    unsigned int op_mode;
    unsigned int direction;
    unsigned int status_word;
    unsigned int act_velocity;
    unsigned int act_position;
} offset;
#define AkdSlavePos    0,0  /* EtherCAT address on the bus */

const static ec_pdo_entry_reg_t domain1_regs[] = {
    { AkdSlavePos, JMCDrive, 0x6040, 0, &offset.ctrl_word },
    { AkdSlavePos, JMCDrive, 0x6060, 0, &offset.op_mode },
    { AkdSlavePos, JMCDrive, 0x6071, 0, &offset.target_torque },
    { AkdSlavePos, JMCDrive, 0x607e, 0, &offset.direction },
    { AkdSlavePos, JMCDrive, 0x6041, 0, &offset.status_word },
    { AkdSlavePos, JMCDrive, 0x606C, 0, &offset.act_velocity },
    { AkdSlavePos, JMCDrive, 0x6064, 0, &offset.act_position },
    {}
};

/* AKD */
ec_pdo_entry_info_t akd_pdo_entries[] = {
    /* RxPdo 0x1600 */
    { 0x6040, 0x00, 16 }, /* DS402 command word */
    { 0x6060, 0x00, 8 }, /* op mode */
    { 0x6071, 0x00, 16 }, /* target torque, in 1/1000 of nominal torque */
    { 0x607e, 0x00, 8 }, /* direction */
    /* TxPDO 0x1a00 */
    { 0x6041, 0x00, 16 }, /* DS402 status word */
    { 0x606C, 0x00, 32 }, /* actual velocity, in rpm */
    { 0x6064, 0x00, 32 }, /* actual position */
};

ec_pdo_info_t akd_pdos[] = {
    { 0x1600, 4, akd_pdo_entries + 0 },
    { 0x1a00, 3, akd_pdo_entries + 4 },
};

/*ec_sync_info_t jmc_syncs[] = {
    { 0, EC_DIR_OUTPUT, 0, akd_pdos + 0, EC_WD_ENABLE },
    { 1, EC_DIR_INPUT, 0, akd_pdos + 1, EC_WD_DISABLE },
    { 0xFF }
};*/
ec_sync_info_t jmc_syncs[] = {
    { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
    { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
    { 2, EC_DIR_OUTPUT, 1, akd_pdos + 0, EC_WD_ENABLE },
    { 3, EC_DIR_INPUT, 1, akd_pdos + 1, EC_WD_DISABLE },
    { 0xFF }
};


// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

#define BusCouplerPos    0, 0


static uint32_t actualPos = 0;
static unsigned int counter = 0;
static unsigned int counter2 = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};

/*****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}
int currentTorque = 100;
uint8_t dir = 0x0;
/****************************************************************************/

void cyclic_task()
{
    struct timespec wakeupTime, time;
#ifdef MEASURE_TIMING
    struct timespec startTime, endTime, lastStartTime = {};
    uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
             latency_min_ns = 0, latency_max_ns = 0,
             period_min_ns = 0, period_max_ns = 0,
             exec_min_ns = 0, exec_max_ns = 0;
#endif

    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);

    while(1) {
        wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

        // Write application time to master
        //
        // It is a good idea to use the target time (not the measured time) as
        // application time, because it is more stable.
        //
        ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;

        if (latency_ns > latency_max_ns) {
            latency_max_ns = latency_ns;
        }
        if (latency_ns < latency_min_ns) {
            latency_min_ns = latency_ns;
        }
        if (period_ns > period_max_ns) {
            period_max_ns = period_ns;
        }
        if (period_ns < period_min_ns) {
            period_min_ns = period_ns;
        }
        if (exec_ns > exec_max_ns) {
            exec_max_ns = exec_ns;
        }
        if (exec_ns < exec_min_ns) {
            exec_min_ns = exec_ns;
        }
#endif

        // receive process data
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        // check process data state (optional)
        check_domain1_state();

        if (counter2) {
            counter2--;
        } else { // do this at 1 Hz
            counter2 = FREQUENCY/10;
            if(master_state.al_states == 0x08)
            {
            // calculate new process data
            if(blink)
            currentTorque++;
            else
            currentTorque--;
            if(currentTorque > 100)
            {
                blink = !blink;
                currentTorque = 100;
            }
            if(currentTorque < -100)
            {
                blink = !blink;
                currentTorque = -100;
            }
            if(currentTorque > 0)
                dir = 0x0;
            else
                dir=0xff;
            } 
        }

        if (counter) {
            counter--;
        } else { // do this at 1 Hz
            counter = FREQUENCY;

     
            // check for master state (optional)
            check_master_state();
            if(master_state.al_states == 0x08)
            {
            printf("actualPos: %d\n",actualPos);
            printf("actualtorque: %d\n",currentTorque);
            printf("dir: %d\n",dir);
#ifdef MEASURE_TIMING
            // output timing stats
            printf("period     %10u ... %10u\n",
                    period_min_ns, period_max_ns);
            printf("exec       %10u ... %10u\n",
                    exec_min_ns, exec_max_ns);
            printf("latency    %10u ... %10u\n",
                    latency_min_ns, latency_max_ns);
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
#endif
             }

        }
        dir=0xff;

        // write process data
        EC_WRITE_U16(domain1_pd + offset.ctrl_word, 0x000f);
        EC_WRITE_S16(domain1_pd + offset.target_torque, currentTorque);
        //EC_WRITE_U8(domain1_pd + offset.op_mode, 10);
        EC_WRITE_U8(domain1_pd + offset.op_mode, 10);
        EC_WRITE_U8(domain1_pd + offset.direction, dir);
        // readPos
        actualPos = EC_READ_U32(domain1_pd + offset.act_position);


        if (sync_ref_counter) {
            sync_ref_counter--;
        } else {
            sync_ref_counter = 1; // sync every cycle

            clock_gettime(CLOCK_TO_USE, &time);
            ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));
        }
        ecrt_master_sync_slave_clocks(master);

        // send process data
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &endTime);
#endif
    }
}

    ec_slave_config_t *sc;
void setSDO8(int index, int subindex, uint8_t data)
{
 uint32_t abort_code;
    int ret=0;
    ret=ecrt_master_sdo_download( master, 0, index, subindex, (uint8_t *)&data, 1 ,&abort_code);
    if(ret < 0)
    {
        printf("error index%d subindex%d error%d\n",index,subindex,abort_code);
    }
    ecrt_slave_config_sdo8( sc, index, subindex, data);
}
void setSDO16(int index, int subindex, uint16_t data)
{
 uint32_t abort_code;
    int ret=0;
    ret=ecrt_master_sdo_download( master, 0, index, subindex, (uint8_t *)&data, 2 ,&abort_code);
    if(ret < 0)
    {
        printf("error index%d subindex%d error%d\n",index,subindex,abort_code);
    }
    ecrt_slave_config_sdo16( sc, index, subindex, data);
}
void setSDO32(int index, int subindex, uint32_t data)
{
 uint32_t abort_code;
    int ret=0;
    ret=ecrt_master_sdo_download( master, 0, index, subindex, (uint8_t *)&data, 4 ,&abort_code);
    if(ret < 0)
    {
        printf("error index%d subindex%d error%d\n",index,subindex,abort_code);
    }
    ecrt_slave_config_sdo32( sc, index, subindex, data);
}
/****************************************************************************/

int main(int argc, char **argv)
{

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        return -1;
    }

    master = ecrt_request_master(0);
    if (!master)
        return -1;

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
        return -1;

    // Create configuration for bus coupler
    sc = ecrt_master_slave_config(master, BusCouplerPos, JMCDrive);
    if (!sc)
        return -1;

    /* Configure AKD flexible PDO */
    printf("Configuring AKD with flexible PDO...\n");
    /* Clear RxPdo */
    ecrt_slave_config_sdo8( sc, 0x1C12, 0, 0 ); /* clear sm pdo 0x1c12 */
    ecrt_slave_config_sdo8( sc, 0x1600, 0, 0 ); /* clear RxPdo 0x1600 */
    ecrt_slave_config_sdo8( sc, 0x1601, 0, 0 ); /* clear RxPdo 0x1601 */
    ecrt_slave_config_sdo8( sc, 0x1602, 0, 0 ); /* clear RxPdo 0x1602 */
    ecrt_slave_config_sdo8( sc, 0x1603, 0, 0 ); /* clear RxPdo 0x1603 */

    /* Define RxPdo */
    ecrt_slave_config_sdo32( sc, 0x1600, 1, 0x60400010 ); /* 0x6040:0/16bits, control word */
    ecrt_slave_config_sdo32( sc, 0x1600, 2, 0x60600008 ); /* 0x6060:0/8bits op mode*/
    ecrt_slave_config_sdo32( sc, 0x1600, 3, 0x60710010 ); /* 0x6071:0/16bits target torque*/
    ecrt_slave_config_sdo32( sc, 0x1600, 4, 0x607e0008 ); /* 0x607e:0/8bits direction*/
    ecrt_slave_config_sdo8( sc, 0x1600, 0, 4 ); /* set number of PDO entries for 0x1600 */

    ecrt_slave_config_sdo16( sc, 0x1C12, 1, 0x1600 ); /* list all RxPdo in 0x1C12:1-4 */
    ecrt_slave_config_sdo8( sc, 0x1C12, 0, 1 ); /* set number of RxPDO */

    /* Clear TxPdo */
    ecrt_slave_config_sdo8( sc, 0x1C13, 0, 0 ); /* clear sm pdo 0x1c13 */
    ecrt_slave_config_sdo8( sc, 0x1A00, 0, 0 ); /* clear TxPdo 0x1A00 */
    ecrt_slave_config_sdo8( sc, 0x1A01, 0, 0 ); /* clear TxPdo 0x1A01 */
    ecrt_slave_config_sdo8( sc, 0x1A02, 0, 0 ); /* clear TxPdo 0x1A02 */
    ecrt_slave_config_sdo8( sc, 0x1A03, 0, 0 ); /* clear TxPdo 0x1A03 */

    /* Define TxPdo */
    ecrt_slave_config_sdo32( sc, 0x1A00, 1, 0x60410010 ); /* 0x6041:0/16bits, status word */
    ecrt_slave_config_sdo32( sc, 0x1A00, 2, 0x606C0020 );  /* 0x606c:0/32bits, act velocity */
    ecrt_slave_config_sdo32( sc, 0x1A00, 2, 0x60640020 );  /* 0x606c:0/32bits, act position */
    ecrt_slave_config_sdo8( sc, 0x1A00, 0, 3 ); /* set number of PDO entries for 0x1A00 */

    ecrt_slave_config_sdo16( sc, 0x1C13, 1, 0x1A00 ); /* list all TxPdo in 0x1C13:1-4 */
    ecrt_slave_config_sdo8( sc, 0x1C13, 0, 1 ); /* set number of TxPDO */

    // configure SYNC signals for this slave
    //ecrt_slave_config_dc(sc, 0x0700, PERIOD_NS, 4400000, 0, 0);
    ecrt_slave_config_dc(sc, 0x0300, PERIOD_NS, 500000, 0, 0);


    printf("Configuring PDOs...\n");
    if ( ecrt_slave_config_pdos( sc, EC_END, jmc_syncs ) ) {
        fprintf( stderr, "Failed to configure JMC PDOs.\n" );
        exit( EXIT_FAILURE );
    }
    

    printf("registering PDOs...\n");
    if ( ecrt_domain_reg_pdo_entry_list( domain1, domain1_regs ) ) {
        fprintf( stderr, "PDO entry registration failed!\n" );
        exit( EXIT_FAILURE );
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master))
    {
        printf("activation failed\n");
        return -1;
    }

    printf("registering PDOs...\n");
    if ( ecrt_domain_reg_pdo_entry_list( domain1, domain1_regs ) ) {
        fprintf( stderr, "PDO entry registration failed!\n" );
        exit( EXIT_FAILURE );
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }

    /* Set priority */

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    printf("Starting cyclic function.\n");
    cyclic_task();

    return 0;
}

/****************************************************************************/
