float P=0.06,I=0.000001,D=0.00000001;
    float targetSpeed = 1.5;
float currentTorque1 = 0;
float currentTorque2 = 0;
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
#include <limits.h>
#include <sched.h> /* sched_setscheduler() */
#include "myTime.h"
#include "Filters.h"
long myTimeCT=0;

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/
#define MAX_TORQUE 800
// Application parameters
#define FREQUENCY 1000
#define CLOCK_TO_USE CLOCK_MONOTONIC
//#define MEASURE_TIMING
const int counterResolution=30000;

/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

const double dt = 1.0/FREQUENCY;
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
    unsigned int status_word;
    unsigned int act_velocity;
    unsigned int act_position;
} offset;
#define JMCSlave1Pos    0,0  /* EtherCAT address on the bus */
#define JMCSlave2Pos    0,1  /* EtherCAT address on the bus */

const static ec_pdo_entry_reg_t domain1_regs[] = {
    { JMCSlave1Pos, JMCDrive, 0x6040, 0, &offset.ctrl_word },
    { JMCSlave1Pos, JMCDrive, 0x6060, 0, &offset.op_mode },
    { JMCSlave1Pos, JMCDrive, 0x6071, 0, &offset.target_torque },
    { JMCSlave1Pos, JMCDrive, 0x6041, 0, &offset.status_word },
    { JMCSlave1Pos, JMCDrive, 0x606C, 0, &offset.act_velocity },
    { JMCSlave1Pos, JMCDrive, 0x6064, 0, &offset.act_position },
    {}
};

const static ec_pdo_entry_reg_t domain2_regs[] = {
    { JMCSlave2Pos, JMCDrive, 0x6040, 0, &offset.ctrl_word },
    { JMCSlave2Pos, JMCDrive, 0x6060, 0, &offset.op_mode },
    { JMCSlave2Pos, JMCDrive, 0x6071, 0, &offset.target_torque },
    { JMCSlave2Pos, JMCDrive, 0x6041, 0, &offset.status_word },
    { JMCSlave2Pos, JMCDrive, 0x606C, 0, &offset.act_velocity },
    { JMCSlave2Pos, JMCDrive, 0x6064, 0, &offset.act_position },
    {}
};

/* AKD */
ec_pdo_entry_info_t akd_pdo_entries[] = {
    /* RxPdo 0x1600 */
    { 0x6040, 0x00, 16 }, /* DS402 command word */
    { 0x6060, 0x00, 8 }, /* op mode */
    { 0x6071, 0x00, 16 }, /* target torque, in 1/1000 of nominal torque */
    /* TxPDO 0x1a00 */
    { 0x6041, 0x00, 16 }, /* DS402 status word */
    { 0x606C, 0x00, 32 }, /* actual velocity, in rpm */
    { 0x6064, 0x00, 32 }, /* actual position */
};

ec_pdo_info_t akd_pdos[] = {
    { 0x1600, 3, akd_pdo_entries + 0 },
    { 0x1a00, 3, akd_pdo_entries + 3 },
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
static ec_domain_t *domain2 = NULL;
static ec_domain_state_t domain1_state = {};
static ec_domain_state_t domain2_state = {};

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;
static uint8_t *domain2_pd = NULL;


float oldE1=0.0;
float oldE2=0.0;
float E1=0.0;
float E2=0.0;

float ES1=0.0;
float ES2=0.0;

static uint32_t actualPos1 = 0;
static uint32_t actualPos2 = 0;
static unsigned int counter = 0;
static unsigned int counter2 = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};
const struct timespec startupcycletime = {0, PERIOD_NS*1};

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
void check_domain2_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain2, &ds);

    if (ds.working_counter != domain2_state.working_counter)
        printf("Domain2: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain2_state.wc_state)
        printf("Domain2: State %u.\n", ds.wc_state);

    domain2_state = ds;
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
float v1 = 0;
float v2 = 0;
float a = 0;
float m = 90;
float currentPos = 0;
uint32_t lastPos1 = 0;
uint32_t lastPos2 = 0;
uint8_t dir = 0x0;
/****************************************************************************/
FilterOnePole filter1;
FilterOnePole filter2;

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
    bool initPos = true;
    double f = 0;
    double ds1 = 0;
    double ds2 = 0;
    int32_t countDiff1 = 0;
    int32_t countDiff2 = 0;
    bool startup = true;

    while(1)
    {
        incTime();
        if(startup)
        wakeupTime = timespec_add(wakeupTime, startupcycletime);
        else
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
        ecrt_domain_process(domain2);

        // check process data state (optional)
        check_domain1_state();
        check_domain2_state();
       if(master_state.al_states == 0x08)
       {
        startup=false;
        // readPos
        actualPos1 = EC_READ_U32(domain1_pd + offset.act_position);
        actualPos2 = EC_READ_U32(domain2_pd + offset.act_position);
        if(initPos)
        {
            lastPos1 = actualPos1;
            lastPos2 = actualPos2;
            initPos=false;
        }
        countDiff1 = actualPos1 - lastPos1;
        if(llabs(countDiff1) > 1000000)
        {
           fprintf(stderr,"overrunn1 %d %d %d\n",(int)llabs(countDiff1), actualPos1, lastPos1);
           if(actualPos1 > lastPos1)
               countDiff1 = ((int64_t)actualPos1 - UINT_MAX) - lastPos1;
           else
               countDiff1 = actualPos1 - ((int64_t)lastPos1 - UINT_MAX) ;
        }
        countDiff2 = actualPos2 - lastPos2;
        if(abs(countDiff2) > 1000000)
        {
           fprintf(stderr,"overrunn2\n");
           if(actualPos2 > lastPos2)
               countDiff2 = ((int64_t)actualPos2 - UINT_MAX) - lastPos2;
           else
               countDiff2 = actualPos2 - ((int64_t)lastPos2 - UINT_MAX) ;
        }
        lastPos1 = actualPos1;
        lastPos2 = actualPos2;
        filter1.input(((double)((int)countDiff1) / (double)counterResolution) * -0.314159265358979323846264338);// (M_PI*D)
        filter2.input(((double)((int)countDiff2) / (double)counterResolution) * -0.314159265358979323846264338);// (M_PI*D)
        ds1 =  filter1.output();
        ds2 =  filter2.output();
        ds1 = (((double)((int)countDiff1) / (double)counterResolution) * -0.314159265358979323846264338);// (M_PI*D)
        ds2 = (((double)((int)countDiff2) / (double)counterResolution) * -0.314159265358979323846264338);// (M_PI*D)
        //reibung + Luftwiderstand
        f = -v1 * 0.001;
        a =  f/m;
        v1 = v1 - a*dt;
        double dsTarget = v1*dt;
        //currentTorque1 = (dsTarget - ds1) * -1000000000.0;
        //fprintf(stderr,"dsTarget %f\n",(float)(dsTarget - ds1)*1000000000.0);
        //currentTorque1 = 0;
        v1 = (ds1)/dt;
        v2 = (ds2)/dt;
        E1 = targetSpeed - v1; 
        E2 = targetSpeed - v2; 
        currentTorque1 -=  (E1 * P) + (ES1*I );//+ (E1/oldE1*D;
        currentTorque2 -=  E2 * P + ES2*I; // + E2/oldE2*D;
        ES1 += E1;
        ES2 += E2;
        oldE1 = E1;
        oldE2 = E2;
        //fprintf(stderr,"v1 %f cT %f\n",v1,currentTorque1);
        //fprintf(stderr,"v2 %f cT %f\n",v2,currentTorque2);
        

        if (counter2) {
            counter2--;
        } else { // do this at 100 Hz
            counter2 = FREQUENCY/100;
        }
        if(currentTorque1 > MAX_TORQUE)
        {
            currentTorque1 = MAX_TORQUE;
        }
        if(currentTorque1 < -MAX_TORQUE)
        {
            currentTorque1 = -MAX_TORQUE;
        }
        if(currentTorque2 > MAX_TORQUE)
        {
            currentTorque2 = MAX_TORQUE;
        }
        if(currentTorque2 < -MAX_TORQUE)
        {
            currentTorque2 = -MAX_TORQUE;
        }
       }

        if (counter)
        {
            counter--;
        }
        else
        { // do this at 1 Hz
            counter = FREQUENCY;
            counter = 100;

     
            // check for master state (optional)
            check_master_state();
            if(master_state.al_states == 0x08)
            {
              float angle1 = (actualPos1%30000)/30000.0 * 360.0;
              float angle2 = (actualPos2%30000)/30000.0 * 360.0;
              //printf("actualPos: l %6d %3.3f° r %6d %3.3f°\n",actualPos1,angle1,actualPos2,angle2);
              printf("actualtorque: %f v%f a%f ds1%f %d\n",currentTorque1,v1,a,ds1,countDiff1);
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

        // write process data
        EC_WRITE_U16(domain1_pd + offset.ctrl_word, 0x000f);
        EC_WRITE_U16(domain2_pd + offset.ctrl_word, 0x000f);
        int currentTorque = currentTorque1;
        EC_WRITE_S16(domain1_pd + offset.target_torque, currentTorque); // torque is actually a signed 16 bit value in 1/1000 or rated torque
        currentTorque = currentTorque2;
        EC_WRITE_S16(domain2_pd + offset.target_torque, currentTorque); // torque is actually a signed 16 bit value in 1/1000 or rated torque
        //EC_WRITE_U8(domain1_pd + offset.op_mode, 10);
        EC_WRITE_U8(domain1_pd + offset.op_mode, 10);
        EC_WRITE_U8(domain2_pd + offset.op_mode, 10);


        if (sync_ref_counter)
        {
            sync_ref_counter--;
        }
        else
        {
            sync_ref_counter = 1; // sync every cycle

            clock_gettime(CLOCK_TO_USE, &time);
            ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));
        }
        ecrt_master_sync_slave_clocks(master);

        // send process data
        ecrt_domain_queue(domain1);
        ecrt_domain_queue(domain2);
        ecrt_master_send(master);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &endTime);
#endif
    }
}

    ec_slave_config_t *scs[2];
void setSDO8(ec_slave_config_t *sc ,int index, int subindex, uint8_t data)
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
void setSDO16(ec_slave_config_t *sc ,int index, int subindex, uint16_t data)
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
void setSDO32(ec_slave_config_t *sc ,int index, int subindex, uint32_t data)
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
void configureSlaves(ec_slave_config_t *sc)
{
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

    domain2 = ecrt_master_create_domain(master);
    if (!domain2)
        return -1;

    // Create configuration for bus coupler
    scs[0] = ecrt_master_slave_config(master, JMCSlave1Pos, JMCDrive);
    if (!scs[0])
        return -1;
    scs[1] = ecrt_master_slave_config(master, JMCSlave2Pos, JMCDrive);
    if (!scs[1])
        return -1;

    /* Configure AKD flexible PDO */
    printf("Configuring flexible PDO...\n");
    configureSlaves(scs[0]);
    configureSlaves(scs[1]);

    printf("Configuring PDOs...\n");
    for(int i=0;i<2;i++)
    {
        if ( ecrt_slave_config_pdos( scs[i], EC_END, jmc_syncs ) )
        {
            fprintf( stderr, "Failed to configure JMC PDOs of drive %d.\n",i );
            exit( EXIT_FAILURE );
        }
    }
    

    printf("registering PDOs...\n");
    if ( ecrt_domain_reg_pdo_entry_list( domain1, domain1_regs ) )
    {
        fprintf( stderr, "PDO entry registration failed!\n" );
        exit( EXIT_FAILURE );
    }
    if ( ecrt_domain_reg_pdo_entry_list( domain2, domain2_regs ) )
    {
        fprintf( stderr, "PDO entry registration failed!\n" );
        exit( EXIT_FAILURE );
    }


    printf("Activating master...\n");
    if (ecrt_master_activate(master))
    {
        printf("activation failed\n");
        return -1;
    }

    printf("done Activating master...\n");
    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }
    if (!(domain2_pd = ecrt_domain_data(domain2))) {
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
