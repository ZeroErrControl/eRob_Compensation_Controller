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

OSAL_THREAD_FUNC ecatcheck( void *ptr );
OSAL_THREAD_FUNC ecatthread( void *ptr );


OSAL_THREAD_HANDLE thread1;
OSAL_THREAD_HANDLE thread2;

void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime);
void add_timespec(struct timespec *ts, int64 addtime);


#define stack64k (64 * 1024)
#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

//convertion units from the servomotor
float Cnt_to_deg = 0.000686645;
int8_t SLAVE_ID;

//##################################################################################################
//create struct according motor datasshet
typedef struct PACKED
{
    int32_t         Target_Position;
    int32_t         Target_Velocity;
    int16_t         Target_Torque;
    int16_t         Max_Torque;
    uint16_t        Control_Word;
    int8_t          Modes_Operation;
    int8_t          Modes_Operation1;

} Motor_input;



typedef struct PACKED
{
    uint16_t       Error_Code;
    uint16_t       Status_Word;
    int32_t        Position_Actual_Value;
    int32_t        Velocity_Actual_Value;
    int16_t        Torque_Actual_Value;
    int8_t         Modes_Operation_Dsiplay;
    int8_t         Modes_Operation_Dsiplay1;

} Motor_output;

//asig the structer to a pointer
Motor_input * set_motor; //target set poit
Motor_output * read_motor; // feedback


//##################################################################################################
int erob_test();

int erob_test()
{

    //##################################################################################################
    SLAVE_ID = 1;
    //ecx_context.manualstatechange = 0;
    //osal_usleep(1e6);

    //1.- Call ec_config_init() to move from INIT to PRE-OP state.
      printf("__________STEP 1___________________\n");
    // Init EtherCAT master
    if (ec_init("ethEtherCAT") <= 0)
    {
        printf("Error: No se pudo inicializar el maestro EtherCAT!\n");
        printf("No socket connection on ethernt port \nExecute as root\n");
        printf("___________________________________________\n");


        return -1;
    }
    printf("Init EtherCAT master OK.\n");
    printf("___________________________________________\n");

    //.............................................................................................
    // shearch Ethercat slaves on network
    if (ec_config_init(FALSE) <= 0) //ST PReop
    {
        printf("Error: can not find EtherCAT slaves!\n");
        printf("___________________________________________\n");
        ec_close();
        return -1;
    }

    printf("%d slaves found and configured.\n \n",ec_slavecount);

    printf("___________________________________________\n");


    //.............................................................................................
    // Print the information of the slaves founded
    for (int i = 1; i <= ec_slavecount; i++)
    {
        printf("-Slave %d: Type %d, Adress 0x%02x\n, State MAchine actual %d; required %d \n", i, ec_slave[i].eep_id,
               ec_slave[i].configadr, ec_slave[i].state,EC_STATE_INIT );
        printf("%s \n", ec_slave[i].name);
        printf("___________________________________________\n");

    }
    printf("__________STEP 5___________________\n");

    for (int i = 1; i <= ec_slavecount; i++) {
        ecx_dcsync0(&ecx_context, i, TRUE, 1000000, 0);
    }


    //##################################################################################################

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


    //Map RXPOD

    //clear the register 0x1c12, subindex
    int retval = 0;
    uint16 clear_val=  0x0000;
    retval = ec_SDOwrite(SLAVE_ID, 0x1c12, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
    if ( retval < 0)  printf("POD reset.\n");

    //map the register
    retval = 0;
    uint16 map_1c12 =  0x1600 ;
    retval += ec_SDOwrite(SLAVE_ID, 0x1c12, 0x01, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);

    map_1c12 =  0x0001 ;
    retval += ec_SDOwrite(SLAVE_ID, 0x1c12, 0x00, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);


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

    //clear the register 0x1c13, subindex
    clear_val=  0x0000;
    retval += ec_SDOwrite(SLAVE_ID, 0x1c13, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);

    map_1c13 = 0x1A00;
    retval += ec_SDOwrite(SLAVE_ID, 0x1c13, 0x01, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);

    map_1c13 = 0x0001;
    retval += ec_SDOwrite(SLAVE_ID, 0x1c13, 0x00, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);

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

    //.............................................................................................
   //set the IOMap
   ulong  usedmem;
   usedmem = ec_config_overlap_map(&IOmap);
   //usedmem = ec_config_map(&IOmap);
   if (usedmem > sizeof(IOmap))
   {
       printf("Slaves can not mapped. Chech The IO memory config. Memory size %lu \n",usedmem );
       printf("___________________________________________\n");
       return -1;
   }
   else
   {
       printf("Slaves mapped. Memory size %lu \n",usedmem );
       printf("___________________________________________\n");
   }
  // osal_usleep(1e6);
   //.............................................................................................

    printf("__________STEP 5___________________\n");

    for (int i = 1; i <= ec_slavecount; i++) {
        ecx_dcsync0(&ecx_context, i, TRUE, 1000000, 0);
    }


  //##################################################################################################

//5.- Manually call the transition to SAFE-OP (some slaves require starting sync0 here,
   //but this is a violation of the EtherCAT protocol spec so it is not the default)

    printf("__________STEP 5___________________\n");
//   printf("step 4 actual state %d = %d\n", ec_slave[SLAVE_ID].state, EC_STATE_SAFE_OP );

  ec_slave[0].state = EC_STATE_SAFE_OP;
   /* request EC_STATE_SAFE_OP state for all slaves */
  ec_writestate(0);
   /* wait for all slaves to reach state */
   if ((ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE*4)) == EC_STATE_SAFE_OP)
   {
       printf("Slave on EC_STATE_PRE_OP: %d \n", EC_STATE_SAFE_OP );
       printf("___________________________________________\n");
   }else { printf("Slave can not changed step 5 to EC_STATE_SAFE_OP\n"); return -1;}




  usleep(10000);
   expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
   printf("Calculated workcounter %d\n", expectedWKC);
   printf("___________________________________________\n");

   /* read individual slave state and store */
   ec_readstate();
   for(int cnt = 1; cnt <= ec_slavecount ; cnt++)
   {
       printf("Slave:%d \nName:%s \nOutput size:%3dbits \nInput size:%3dbits \nState:%2d \ndelay:%d.%d\n",
              cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
              ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
       printf("Out:%p,%4d \nIn:%p,%4d\n",
              ec_slave[cnt].outputs, ec_slave[cnt].Obytes, ec_slave[cnt].inputs, ec_slave[cnt].Ibytes);
   }
   printf("___________________________________________\n");




   //##################################################################################################

    //6.-  start the regular PDO transfer at the proper interval

     printf("__________STEP 6___________________\n");

  // init PDO mapping read_write
    start_ecatthread_thread= TRUE;



    osal_thread_create_rt(&thread1, stack64k * 2, (void *)&ecatthread, (void *)&ctime_thread);

    osal_thread_create(&thread2, stack64k * 2, (void*)&ecatcheck, NULL);

 //##################################################################################################

    //   //9.- Call ec_dcsync0() for the reference clock slave
    //    printf("__________STEP 9___________________\n");
    //   ;
    //  // ec_dcsync01(1, TRUE, 1000000,1000000,0);
    //   //ec_dcsync0(SLAVE_ID, TRUE, SYNC0TIME,0,0);

    //   //Read the register of the type of sincronization.
    //   int rdl;
    //   uint16_t data_R;
    //   ec_SDOread(SLAVE_ID, 0x1c32, 0x01,FALSE,&rdl,&data_R, EC_TIMEOUTSAFE);
    //   printf("DC SINCRONIZED 0x1c32 %d\n",data_R);
    //   printf("___________________________________________\n");

    //   ec_SDOread(SLAVE_ID, 0x1c33, 0x01,FALSE,&rdl,&data_R, EC_TIMEOUTSAFE);
    //    printf("DC SINCRONIZED 0x1c33 %d\n",data_R);
    //    printf("___________________________________________\n");




   //7.- /* connect struct pointers PDO TDX RTX to slave I/O pointers */
   printf("__________STEP 7___________________\n");

       set_motor = (Motor_input*) ec_slave[1].inputs;
       read_motor = (Motor_output*) ec_slave[1].outputs;


       printf("IOMAP conected\n");

   //##################################################################################################

  //8.- Transition to OP state

 printf("__________STEP 8___________________\n");


 /* wait for all slaves to reach EC_STATE_PRE_OP */

//// ec_slave[0].state = EC_STATE_PRE_OP;
// /* request EC_STATE_PRE_OP state for all slaves */
//// ec_writestate(0);
//  if ((ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE*4)) == EC_STATE_PRE_OP)
// {
//     printf("Slave on EC_STATE_PRE_OP: %d \n", EC_STATE_PRE_OP );
//     printf("___________________________________________\n");
// }else { printf("Slave can not changed step 8 to EC_STATE_PRE_OP\n"); return -1;}

//osal_usleep(0.5*1e6);

///* request EC_STATE_PRE_OP state for all slaves */
////ec_slave[0].state = EC_STATE_SAFE_OP;
////ec_writestate(0);
// /* wait for all slaves to reach EC_STATE_SAFE_OP */
// if ((ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE*4)) == EC_STATE_SAFE_OP)
// {
//     printf("Slave on EC_STATE_SAFE_OP: %d \n", EC_STATE_SAFE_OP );
//     printf("___________________________________________\n");
// }else { printf("Slave can not changed step 8 to EC_STATE_SAFE_OP\n"); return -1;}


 //send one request data
 ec_send_processdata();
 wkc = ec_receive_processdata(EC_TIMEOUTRET);


   /* request EC_STATE_OPERATIONAL state for all slaves */
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_writestate(0);

  /* wait for all slaves to reach state */
  if ((ec_statecheck(0, EC_STATE_OPERATIONAL, 5* EC_TIMEOUTSTATE)) == EC_STATE_OPERATIONAL)
  {
      printf("State changed to EC_STATE_OPERATIONAL: %d \n", EC_STATE_OPERATIONAL );
      printf("___________________________________________\n");
  }else { printf("State can not changed step 8 to\n"); return -1;}



  /* read individual slave state and store */
  ec_readstate();
  for (int i = 1; i <= ec_slavecount; i++)
  {
      printf("-Slave %d: Type %d, Adress 0x%02x\n, State MAchine actual %d; required %d \n", i, ec_slave[i].eep_id,
             ec_slave[i].configadr, ec_slave[i].state,EC_STATE_OPERATIONAL );
      printf("%s \n", ec_slave[i].name);
      printf("___________________________________________\n");

  }



  //##################################################################################################

   //9.- Config servomotor and  mode operation
   printf("__________STEP 9___________________\n");

  if (ec_slave[0].state == EC_STATE_OPERATIONAL )
  {

      printf("######################################################################################\n");
      printf("#################set profile position##################################\n");

      inOP = TRUE; //init check error
      needlf = TRUE;
      start_ecatthread_thread= TRUE;




        //set profile position


      int8_t data_W= 9;
      uint16 control_word,state_word;
      int rdl =sizeof(control_word);
      ec_SDOwrite(SLAVE_ID, 0x6060, 0x00, FALSE, sizeof(data_W), &data_W, 5*EC_TIMEOUTSAFE);
      data_W= 6;
      ec_SDOwrite(SLAVE_ID, 0x6040, 0x00, FALSE, sizeof(data_W), &data_W, 5*EC_TIMEOUTSAFE);
      data_W= 7;
      ec_SDOwrite(SLAVE_ID, 0x6040, 0x00, FALSE, sizeof(data_W), &data_W, 5*EC_TIMEOUTSAFE);
      data_W= 15;
      ec_SDOwrite(SLAVE_ID, 0x6040, 0x00, FALSE, sizeof(data_W), &data_W, 5*EC_TIMEOUTSAFE);
      data_W= 0;
      rdl =sizeof(control_word);
      ec_SDOread(SLAVE_ID, 0x6040, 0x00, FALSE, &rdl, &control_word, 5*EC_TIMEOUTSAFE);
      ec_SDOread(SLAVE_ID, 0x6041, 0x00, FALSE, &rdl, &state_word, 5*EC_TIMEOUTSAFE);


       int32 ck =  set_motor->Modes_Operation;
       printf("control_word: %d \n", control_word);
      printf("state_word: %d \n", state_word);



      //.............................................................................................

      //Profile settings


    //.............................................................................................

      // Profile velocity
      uint16_t        Control_Word = 0;
      //ec_SDOwrite(SLAVE_ID, 0x6040, 0x00, FALSE, sizeof(Control_Word), &Control_Word, EC_TIMEOUTSAFE);
      Control_Word = 6;
      ec_SDOwrite(SLAVE_ID, 0x6040, 0x00, FALSE, sizeof(Control_Word), &Control_Word, 5*EC_TIMEOUTSAFE);
      Control_Word = 7;
      ec_SDOwrite(SLAVE_ID, 0x6040, 0x00, FALSE, sizeof(Control_Word), &Control_Word, 5*EC_TIMEOUTSAFE);
      Control_Word = 15;
      ec_SDOwrite(SLAVE_ID, 0x6040, 0x00, FALSE, sizeof(Control_Word), &Control_Word, 5*EC_TIMEOUTSAFE);

      //read data by 10 seconds and stop
      for (int i = 1; i<=10;i++)
      {
          printf("___________________________________________\n");

          osal_usleep(1e6);

      }


      //close SOEM
      ec_close();
      printf("EDNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN STPO\n");

  }
  else
  {
      printf("Not all slaves reached operational state.\n");

      ec_readstate();
      for(int i = 1; i<=ec_slavecount ; i++)
      {
          if(ec_slave[i].state != EC_STATE_OPERATIONAL)
          {
              printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                     i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
              printf("\nRequest init state for slave %d\n", i);
              ec_slave[i].state = EC_STATE_INIT;
              printf("___________________________________________\n");
          }

          return -1;

      }



   }


return 0;
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;
    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime) % cycletime;
    if(delta> (cycletime / 2)) { delta= delta - cycletime; }
    if(delta>0){ integral++; }
    if(delta<0){ integral--; }
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta = delta;
}


/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if ( ts->tv_nsec >= NSEC_PER_SEC )
    {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}




OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n",slave);
                        }
                    }
                    else if(!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n",slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if(ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n",slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n",slave);
                    }
                }
            }
            if(!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
    struct timespec   ts, tleft;
    int ht;
    int64 cycletime;
    struct timeval    tp;


    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    if (ts.tv_nsec >= NSEC_PER_SEC) {
        ts.tv_sec++;
        ts.tv_nsec -= NSEC_PER_SEC;
    }
    cycletime = *(int*)ptr * 1000; /* cycletime in ns */

    toff = 0;
    dorun = 0;
    ec_send_processdata();
    while(1)
    {

        dorun++;
        /* calculate next cycle start */
        add_timespec(&ts, cycletime + toff);
        /* wait to cycle start */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
        //pthread_cond_timedwait(&cond, &mutex, &ts);

        if(start_ecatthread_thread)
        {


            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            if (ec_slave[0].hasdc)
            {
                /* calulate toff to get linux time and DC synced */
                ec_sync(ec_DCtime, cycletime, &toff);
            }

            ec_send_processdata();
        }


    }
}


int main(int argc, char **argv)
{

    needlf = FALSE;
    inOP = FALSE;
    dorun =0 ;
    ctime_thread = 1500;// 1ms cycle time
    start_ecatthread_thread = FALSE;


    erob_test();


    printf("End program\n");
    ec_close();
    return 0;

}



