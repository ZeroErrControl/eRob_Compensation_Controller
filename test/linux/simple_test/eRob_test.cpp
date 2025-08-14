//#include <QCoreApplication>

#include <stdio.h>
#include <string.h>
#include "ethercat.h"
#include <iostream>
#include <inttypes.h>
#include <time.h>
#include <unistd.h>

#include <sys/time.h>
#include <pthread.h>
#include <math.h>

#include <chrono>
#include <ctime>

#include <iostream>
#include <cstdint>

#include <sched.h>


char IOmap[4096];
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
int dorun = 0;
bool start_ecatthread_thread;
int ctime_thread;

int64 toff, gl_delta;

OSAL_THREAD_FUNC ecatcheck(void *ptr);
OSAL_THREAD_FUNC ecatthread(void *ptr);

OSAL_THREAD_HANDLE thread1;
OSAL_THREAD_HANDLE thread2;

void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime);
void add_timespec(struct timespec *ts, int64 addtime);

#define stack64k (64 * 1024)
#define NSEC_PER_SEC 1000000000   // 1s
#define EC_TIMEOUTMON 5000        // 5 us

// Conversion units from the servomotor
float Cnt_to_deg = 0.000686645;
int8_t SLAVE_ID;

//##################################################################################################
// 函数：设置线程的 CPU 亲和性
void set_thread_affinity(pthread_t thread, int cpu_core) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset);

    int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0) {
        printf("无法为线程设置 CPU %d 的亲和性\n", cpu_core);
    } else {
        printf("线程成功绑定到 CPU %d\n", cpu_core);
    }
}

//##################################################################################################
int erob_test();

uint16_t data_R;

int erob_test() {
    int rdl;
    SLAVE_ID = 1;
    int i, j, oloop, iloop, chk;
    // 1. Call ec_config_init() to move from INIT to PRE-OP state.
    printf("__________STEP 1___________________\n");
    // Init EtherCAT master
    if (ec_init("eno1") <= 0) {
        printf("Error: Could not initialize EtherCAT master!\n");
        printf("No socket connection on Ethernet port. Execute as root.\n");
        printf("___________________________________________\n");
        return -1;
    }
    printf("EtherCAT master initialized successfully.\n");
    printf("___________________________________________\n");

    // Search for EtherCAT slaves on the network
    if (ec_config_init(FALSE) <= 0) {
        printf("Error: Cannot find EtherCAT slaves!\n");
        printf("___________________________________________\n");
        ec_close();
        return -1;
    }
    printf("%d slaves found and configured.\n", ec_slavecount);
    printf("___________________________________________\n");

    //2.- Change to pre operational state to config the PDO registers
    printf("__________STEP 2___________________\n");

    //Check if the slave is ready to map
    ec_readstate();

    for(int i = 1; i<=ec_slavecount ; i++)
    {
        if(ec_slave[i].state != EC_STATE_PRE_OP)
        {
            printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                   i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            printf("\nRequest init state for slave %d\n", i);
            ec_slave[i].state = EC_STATE_INIT;
            printf("___________________________________________\n");
        }
        else //request operational mode
        {
            ec_slave[0].state = EC_STATE_PRE_OP;
            /* request EC_STATE_PRE_OP state for all slaves */
            ec_writestate(0);
            /* wait for all slaves to reach state */
            if ((ec_statecheck(0, EC_STATE_PRE_OP,  3*EC_TIMEOUTSTATE)) == EC_STATE_PRE_OP)
            {
                printf("State changed to EC_STATE_PRE_OP: %d \n", EC_STATE_PRE_OP );
                printf("___________________________________________\n");
            }else { printf("State EC_STATE_PRE_OP can not changed step 2 to\n"); return -1;}

        }

    }

//##################################################################################################
    //3.- Map RXPOD
    printf("__________STEP 3___________________\n");

    //clear the register 0x1c12, subindex
    int retval = 0;
    uint16 clear_val=  0x0000;
    retval = 0;
    uint16 map_1c12;
    for(int i = 1; i<=ec_slavecount ; i++) {

        retval = ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        map_1c12 =  0x1606 ;
        retval += ec_SDOwrite(i, 0x1c12, 0x01, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
        map_1c12 =  0x0001 ;
        retval += ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
    }
    printf("slave %d set, retval = %d\n", SLAVE_ID, retval);
    if ( retval <1)
    {
        printf("RXPOD Mapping can no set.\n");
        printf("___________________________________________\n");
        return -1;

    }

    printf("RXPOD Mapping set correct1ly.\n");
    printf("___________________________________________\n");

    //........................................................................................
    //Map TXPOD
    retval = 0;
    uint16 map_1c13;
    for(int i = 1; i<=ec_slavecount ; i++) {
        //clear the register 0x1c13, subindex
        clear_val=  0x0000;

        retval += ec_SDOwrite(i, 0x1c13, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);

        map_1c13 = 0x1A03;
        retval += ec_SDOwrite(i, 0x1c13, 0x01, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);

        map_1c13 = 0x0001;
        retval += ec_SDOwrite(i, 0x1c13, 0x00, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);
    }
    printf(" slave %d set, retval = %d\n", SLAVE_ID, retval);

   if ( retval <1  )
   {
       printf("TXPOD Mapping can no set.\n");
       printf("___________________________________________\n");
       return -1;
   }

   printf("TXPOD Mapping set correct1ly.\n");
   printf("___________________________________________\n");

   //.............................................................................................

   //##################################################################################################

    //4.- Set ecx_context.manualstatechange = 1. Map PDOs for all slaves by calling ec_config_map().
   printf("__________STEP 4___________________\n");

   ecx_context.manualstatechange = 1; //Automatic state changed disable
   osal_usleep(1e6);

    uint8 WA = 0;
    uint8 my_RA = 0;
    uint32 TIME_RA;

    // Print the information of the slaves found
    for (int i = 1; i <= ec_slavecount; i++) {
       // (void)ecx_FPWR(ecx_context.port, i, ECT_REG_DCSYNCACT, sizeof(WA), &WA, 5 * EC_TIMEOUTRET);
        printf("Name: %s\n", ec_slave[i].name);
        printf("Slave %d: Type %d, Address 0x%02x, State Machine actual %d, required %d\n", i, ec_slave[i].eep_id, ec_slave[i].configadr, ec_slave[i].state, EC_STATE_INIT);
        printf("___________________________________________\n");
        ecx_dcsync0(&ecx_context, i, TRUE, 1000000, 0);  ///!!!!!!!  DC
    }

    // Set the IOMap
    ec_config_map(&IOmap);

    printf("__________STEP 5___________________\n");


    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    if ((ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 5)) == EC_STATE_SAFE_OP) {
        printf("State changed to EC_STATE_SAFE_OP: %d\n", EC_STATE_SAFE_OP);
        printf("___________________________________________\n");
    } else {
        printf("State could not be changed to EC_STATE_SAFE_OP\n");
        return -1;
    }
    // Call ec_configdc() in EC_STATE_SAFE_OP state
    ec_configdc();   // 9%
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("Calculated workcounter %d\n", expectedWKC);
    printf("___________________________________________\n");

    ec_readstate();
    for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
        printf("Slave %d\nName: %s\nOutput size: %3d bits\nInput size: %3d bits\nState: %2d\ndelay: %d.%d\n", cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits, ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
        printf("Out: %p, %4d\nIn: %p, %4d\n", ec_slave[cnt].outputs, ec_slave[cnt].Obytes, ec_slave[cnt].inputs, ec_slave[cnt].Ibytes);
    }
    printf("___________________________________________\n");

    // 6. Start the regular PDO transfer at the proper interval
    printf("__________STEP 6___________________\n");

    start_ecatthread_thread = TRUE;
    osal_thread_create_rt(&thread1, stack64k * 2, (void *)&ecatthread, (void *)&ctime_thread);
    // 将 thread1 绑定到 CPU 4
    //set_thread_affinity(*thread1, 4);
    osal_thread_create(&thread2, stack64k * 2, (void *)&ecatcheck, NULL);
    //set_thread_affinity(*thread2, 5);
    printf("___________________________________________\n");
    my_RA = 0;
    for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
        ec_SDOread(cnt, 0x1c32, 0x01, FALSE, &rdl, &data_R, EC_TIMEOUTSAFE);
        printf("Slave %d DC synchronized 0x1c32: %d\n", cnt, data_R);
    }


    // 8. Transition to OP state
    printf("__________STEP 8___________________\n");

    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);


    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);


    if ((ec_statecheck(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE)) == EC_STATE_OPERATIONAL) {
        printf("State changed to EC_STATE_OPERATIONAL: %d\n", EC_STATE_OPERATIONAL);
        printf("___________________________________________\n");
    } else {
        printf("State could not be changed to EC_STATE_OPERATIONAL\n");
        for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
            printf("ALstatuscode: %d\n", ecx_context.slavelist[cnt].ALstatuscode);
        }
    }

    ec_readstate();
    for (int i = 1; i <= ec_slavecount; i++) {
        printf("Slave %d: Type %d, Address 0x%02x, State Machine actual %d, required %d\n", i, ec_slave[i].eep_id, ec_slave[i].configadr, ec_slave[i].state, EC_STATE_OPERATIONAL);
        printf("Name: %s\n", ec_slave[i].name);
        printf("___________________________________________\n");
    }

    // 9. Configure servomotor and mode operation
    printf("__________STEP 9___________________\n");

    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        printf("######################################################################################\n");
        printf("################# All slaves entered into OP state! ##################################\n");
        // Profile velocity

        uint16_t  Control_Word;
        Control_Word = 128;

        uint8    operation_mode = 9;
        for (int i = 1; i <= ec_slavecount; i++) {
            ec_SDOwrite(i, 0x6040, 0x00, FALSE, sizeof(Control_Word), &Control_Word, 5*EC_TIMEOUTSAFE);
            ec_SDOwrite(i, 0x6060, 0x00, FALSE, sizeof(operation_mode), &operation_mode, EC_TIMEOUTSAFE);
        }
        // Record start time

        auto start = std::chrono::high_resolution_clock::now();
        //osal_usleep(1000000);
        int step = 0;
        for(i = 1; i <= 3*60*60*1000; i++)
        {

            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);


            if(wkc >= expectedWKC)
            {
               printf("Processdata cycle %4d, WKC %d \r , O:", i, wkc);
/************
                for(j = 0 ; j < oloop; j++)
                {
                    printf(" %2.2x", *(ec_slave[0].outputs + j));
                }

                printf(" I:");
                for(j = 0 ; j < iloop; j++)
                {
                    printf(" %2.2x", *(ec_slave[0].inputs + j));
                }
                //printf(" T:%"PRId64"\r",ec_DCtime);
***88*******/
                needlf = TRUE;
            }

                uint32_t target_position = 0; //
                uint32_t target_velocity = 0; //
                uint16 output_data[10] = {0};
            if (step > 600)
            {

                output_data[0] = target_position & 0xFFFF;          //
                output_data[1] = (target_position >> 16) & 0xFFFF; //
                output_data[2] = 0x00;          //
                output_data[3] = 0x00; //
                output_data[4] = target_velocity & 0xFFFF;          //
                output_data[5] = (target_velocity >> 16) & 0xFFFF; //
                output_data[6] = 0x00; //
                output_data[7] = 0x00; //
                output_data[8] = 0x00; //
                output_data[9] = 0x0f; //

                //step = 1;
            }
            if (step <= 600 && step > 400)
            {

                output_data[0] = target_position & 0xFFFF;          //
                output_data[1] = (target_position >> 16) & 0xFFFF; //
                output_data[2] = 0x00;          //
                output_data[3] = 0x00; //
                output_data[4] = target_velocity & 0xFFFF;          //
                output_data[5] = (target_velocity >> 16) & 0xFFFF; //
                output_data[6] = 0x00; //
                output_data[7] = 0x00; //
                output_data[8] = 0x00; //
                output_data[9] = 0x07; //
               // memcpy(ec_slave[0].outputs, output_data, sizeof(output_data)); //
                //step = 3;

            }
            if (step <= 400 && step >200 )
            {

                output_data[0] = target_position & 0xFFFF;          //
                output_data[1] = (target_position >> 16) & 0xFFFF; //
                output_data[2] = 0x00;          //
                output_data[3] = 0x00; //
                output_data[4] = 0x00;          //
                output_data[5] = 0x00; //
                output_data[6] = 0x00; //
                output_data[7] = 0x00; //
                output_data[8] = 0x00; //
                output_data[9] = 0x06; //
               // memcpy(ec_slave[0].outputs, output_data, sizeof(output_data)); //
                //step = 2;
            }
            if (step <= 200)
            {
                output_data[0] = target_position & 0xFFFF;          //
                output_data[1] = (target_position >> 16) & 0xFFFF; //
                output_data[2] = 0x00;          //
                output_data[3] = 0x00; //
                output_data[4] = 0x00;          //
                output_data[5] = 0x00; //
                output_data[6] = 0x00; //
                output_data[7] = 0x00; //
                output_data[8] = 0x00; //
                output_data[9] = 0xf0; //
                //memcpy(ec_slave[0].outputs, output_data, sizeof(output_data)); //
            }

            for (int i = 1; i <= ec_slavecount; i++)
            {
            memcpy(ec_slave[i].outputs, output_data, sizeof(output_data)); //
            }
            if(step <900) {
                step += 1;/* code */
            }

            osal_usleep(1000);
            // Record end time

            auto end = std::chrono::high_resolution_clock::now();

            // Calculate the duration in different units
            //auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
            auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            auto duration_s = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
            auto duration_m = std::chrono::duration_cast<std::chrono::minutes>(end - start).count();
            auto duration_h = std::chrono::duration_cast<std::chrono::hours>(end - start).count();
            // Output the runtime
            // Output the runtime in the correct format, keeping the columns intact
            std::cout << "Program runtime: " << duration_h << "h:" << duration_m % 60 << "m:"
                      << duration_s % 60 << "s:" << duration_ms % 1000 << "ms"<< duration_us%1000<<"us"<<std::flush;
            //ec_send_processdata();
            //wkc = ec_receive_processdata(EC_TIMEOUTRET);


        }

        osal_usleep(1e6);

        ec_close();

         printf("\nRequest init state for all slaves\n");
         ec_slave[0].state = EC_STATE_INIT;
         /* request INIT state for all slaves */
         ec_writestate(0);

        printf("EtherCAT master closed.\n");
    } else {
        printf("Not all slaves reached operational state.\n");

        ec_readstate();
        for (int i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                printf("Requesting INIT state for slave %d\n", i);
                ec_slave[i].state = EC_STATE_INIT;
                printf("___________________________________________\n");
            }
        }
    }

    return 0;
}

/* PI calculation to get Linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime) {
    static int64 integral = 0;
    int64 delta;
    delta = (reftime) % cycletime;
    if (delta > (cycletime / 2)) {
        delta = delta - cycletime;
    }
    if (delta > 0) {
        integral++;
    }
    if (delta < 0) {
        integral--;
    }
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta = delta;
}

/* Add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime) {
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec >= NSEC_PER_SEC) {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

OSAL_THREAD_FUNC ecatcheck(void *ptr) {
    int slave;
    (void)ptr; // Not used

    while (1) {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
            if (needlf) {
                needlf = FALSE;
                printf("\n");
            }
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++) {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        printf("ERROR: Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
                        printf("WARNING: Slave %d is in SAFE_OP, changing to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    } else if (ec_slave[slave].state > EC_STATE_NONE) {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE: Slave %d reconfigured\n", slave);
                        }
                    } else if (!ec_slave[slave].islost) {
                        ec_statecheck(slave, EC_STATE_OPERATIONAL,  EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE) {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR: Slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost) {
                    if (ec_slave[slave].state == EC_STATE_NONE) {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE: Slave %d recovered\n", slave);
                        }
                    } else {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE: Slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate) {
                printf("OK: All slaves resumed OPERATIONAL.\n");
            }
        }
    }
}

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr) {
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;
    struct timeval tp;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    if (ts.tv_nsec >= NSEC_PER_SEC) {
        ts.tv_sec++;
        ts.tv_nsec -= NSEC_PER_SEC;
    }
    cycletime = *(int *)ptr * 1000; /* cycletime in ns */

    toff = 0;
    dorun = 0;
    ec_send_processdata();
    while (1) {
        dorun++;
        /* calculate next cycle start */
        add_timespec(&ts, cycletime + toff);
        /* wait to cycle start */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);

        if (start_ecatthread_thread) {
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            if (ec_slave[0].hasdc) {
                /* calculate toff to get Linux time and DC synced */
                ec_sync(ec_DCtime, cycletime, &toff);
            }

            ec_send_processdata();
        }
    }
}

int correct_count = 0;
int incorrect_count = 0;
int test_count_sum = 100;
int test_count = 0;
float correct_rate = 0;

int main(int argc, char **argv) {
    needlf = FALSE;
    inOP = FALSE;
    start_ecatthread_thread = FALSE;
    dorun = 0;
    ctime_thread = 1500; // 1ms cycle time

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset); // Clear the CPU set
    CPU_SET(3, &cpuset); // Set CPU 0 (change this to a valid core number)

    // Set the CPU affinity for the current process
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
        perror("sched_setaffinity");
        return EXIT_FAILURE;
    }

    // Your program logic here
    printf("Running on CPU core 0\n");

    erob_test();



    printf("End program\n");

    return EXIT_SUCCESS;
}
