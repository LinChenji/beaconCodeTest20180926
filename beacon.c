/******************************************************************************
 *  Copyright (C) Cambridge Silicon Radio Limited, 2014
 *
 *  FILE
 *      beacon.c
 *
 *  DESCRIPTION
 *      This file defines an advertising node implementation
 *
 *****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <main.h>
#include <gap_app_if.h>
#include <config_store.h>
#include <pio.h>
#include <random.h>
#include <timer.h>
#include <mem.h>
#include <nvm.h>
#include <reset.h>
/*============================================================================*
 *  Local Header File
 *============================================================================*/

#include "beacon.h"
#include "esurl_beacon_service.h"

uint8 ibeaconName[]="CSR";
uint8 ibeaconName1[]="Apple";
#define timers_num  1
#define TOGGLE  11
#define ADVERT_SIZE                     (28)
timer_id button_press_tid;
uint16 *app_timer[SIZEOF_APP_TIMER* timers_num];
/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

static void startAdvertising(timer_id tid);
extern void HandlePIOChangedEvent(pio_changed_data *pio_data);
void BeaconStart(timer_id tid);
//static void appSetRandomAddress(void);
void reintoOther(timer_id tid);
void handleExtraLongButtonPress(timer_id tid);
/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/


/*----------------------------------------------------------------------------*
 *  NAME
 *      startAdvertising
 *
 *  DESCRIPTION
 *      This function is called to start advertisements.
 *
 *      Advertisement packet will contain Flags AD and Manufacturer-specific
 *      AD with Manufacturer id set to CSR and payload set to the value of
 *      the User Key 0. The payload size is set by the User Key 1.
 *
 *      +--------+-------------------------------------------------+
 *      |FLAGS AD|MANUFACTURER AD                                  |
 *      +--------+-------------------------------------------------+
 *       0      2 3
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
void BeaconStart(timer_id tid)
{
    uint8 advData[ADVERT_SIZE];
    uint8 scanRspData[31];
    MemSet(advData,0xff,sizeof(advData));
    MemSet(scanRspData,0xff,sizeof(scanRspData)); 
    EsurlBeaconInitChipReset();
    uint16 offset = 0;
    uint8* beacon_data;
    uint8 beacon_data_size;
    uint8 i;
    uint8 len_i = 0;
    uint8 adv_parameter_len = 0;
    uint32 beacon_interval = EsurlBeaconGetPeriodMillis();//beacon period:adv interval    
    
    /* Stop broadcasting */
    LsStartStopAdvertise(FALSE, whitelist_disabled, ls_addr_type_public);

    /* beacon_interval of zero overrides and stops beaconning */
    if (TRUE && (beacon_interval != 0)) 
    {
        /* set the GAP Broadcaster role */
        GapSetMode(gap_role_broadcaster,
                   gap_mode_discover_no,
                   gap_mode_connect_no,
                   gap_mode_bond_no,
                   gap_mode_security_none);
        //broadcaster/no/no/no/none
        /*GapSetMode(gap_role_broadcaster, 
                   gap_mode_discover_general,
                   gap_mode_connect_undirected,
                   gap_mode_bond_no,
                   gap_mode_security_unauthenticate);*/
        #if 0
#ifdef USE_STATIC_RANDOM_ADDRESS
    uint16 connect_flags = L2CAP_CONNECTION_SLAVE_UNDIRECTED |
                      L2CAP_OWN_ADDR_TYPE_RANDOM;//0x04/0x0010
#else
    uint16 connect_flags = L2CAP_CONNECTION_SLAVE_UNDIRECTED |
                          L2CAP_OWN_ADDR_TYPE_PUBLIC;//0x04/0x0000
#endif        
        #endif
        /* clear the existing advertisement and scan response data */
        LsStoreAdvScanData(0, NULL, ad_src_advertise);
        LsStoreAdvScanData(0, NULL, ad_src_scan_rsp);
    
        /* set the advertisement interval */

        GapSetAdvInterval(beacon_interval, beacon_interval);
        
        /* get the beaconing data USING SERVICE */
        EsurlBeaconGetData(&beacon_data, &beacon_data_size);//get real data
        
        if(beacon_data_size > 0)
        {
            adv_parameter_len = beacon_data[0];
            len_i = adv_parameter_len - 1;
            
            /* and store in the packet */
            for(i = 1; (i < beacon_data_size) && (offset < ADVERT_SIZE); i++,offset++, len_i--)
            {
                advData[offset] = beacon_data[i];//=*(beacon+i)??
                
                if(len_i == 0)
                {
                    /* store the advertisement parameter and get length for the next parameter */
                    LsStoreAdvScanData(adv_parameter_len, &advData[offset - adv_parameter_len + 1], ad_src_advertise);//
    
                    adv_parameter_len = beacon_data[i+1];
                    len_i = adv_parameter_len;
                    i++;
                }
            }
            scanRspData[0]=AD_TYPE_LOCAL_NAME_COMPLETE;
            uint8 eddystoneName[]="Google";
            
      
            MemCopy(scanRspData+1,eddystoneName,sizeof(eddystoneName));//
            LsStoreAdvScanData(sizeof(eddystoneName)+1 , scanRspData, ad_src_scan_rsp);
        }
        else
        {
            /* store the advertisement data */
            LsStoreAdvScanData(offset, advData, ad_src_advertise);
        }
        
        /* Start broadcasting */
        LsStartStopAdvertise(TRUE, whitelist_disabled, ls_addr_type_public);//random
        //GattConnectReq(NULL, connect_flags);
    }
    
    //uint16 bra=0x02;
    //NvmWrite(&bra,1,0);    
    //TimerCreate(15*SECOND,TRUE,reintoOther);
}

void reintoOther(timer_id tid)
{
    TimerDelete(tid);tid=TIMER_INVALID;
    WarmReset();
}

void startAdvertising(timer_id tid)
{
    TimerDelete(tid);tid=TIMER_INVALID;
    uint8 advData[27];//MAX_ADVERT_PACKET_SIZE:31=0x1f
    uint8 scanRspData[31];
    uint16 advInterval;
    uint8 advPayloadSize = DEFAULT_ADVERT_PAYLOAD_SIZE;//24
    /* read User key 2 for the advertising interval */
    advInterval = 1000;//CSReadUserKey(2);//1000ms
    
    /* range check */
    if((advInterval < MIN_ADVERTISING_INTERVAL) ||
       (advInterval > MAX_ADVERTISING_INTERVAL))    //[0.1,10.24](s)
    {
        /* revert to default advertising interval */
        advInterval = DEFAULT_ADVERTISING_INTERVAL;//1s
    }
    
    //if(start)
    {
        /* set the GAP Broadcaster role */
        GapSetMode(gap_role_broadcaster,
                   gap_mode_discover_no,
                   gap_mode_connect_no,
                   gap_mode_bond_no,
                   gap_mode_security_none);
        
        /* clear the existing advertisement data, if any */
        LsStoreAdvScanData(0, NULL, ad_src_advertise);
        LsStoreAdvScanData(0, NULL, ad_src_scan_rsp);
    
        /* set the advertisement interval, API accepts the value in microseconds */
        GapSetAdvInterval(advInterval * MILLISECOND, advInterval * MILLISECOND);
    
        /*uint8 localName[14]={AD_TYPE_LOCAL_NAME_COMPLETE,'A','p','p','l','e',
                        ' ','i','B','e','a','c','o','n'};
        LsStoreAdvScanData(14,localName,ad_src_advertise);*/
        
        /* manufacturer-specific data */
        advData[0] = AD_TYPE_MANUF;//0xff
    
        /* Apple company code, little endian */
        advData[1] = 0x4c;
        advData[2] = 0x00;
        advData[3] = 0x02;//Magic No
        advData[4] = 0x15;//Length:21    
        
        uint8 uuid[16]={    //af2f1d73-92b8-4671-bb21-45ebff64a894
            //0xaf,0x2f,0x1d,0x73,0x92,0xb8,0x46,0x71,
            //0xbb,0x21,0x45,0xeb,0xff,0x64,0xa8,0x94
            //AB8190D5D11E4941ACC442F30510B408和2731/659C //巴士头条
            0xAB,0x81,0x90,0xD5,0xD1,0x1E,0x49,0x41,
            0xAC,0xC4,0x42,0xF3,0x05,0x10,0xB4,0x08            
        };
        uint8 major[2]={0x27,0x31};//{0x15,0x01};//big-endian
        uint8 minor[2]={0x65,0x9c};//{0x15,0x08};
        MemCopy(advData+5,uuid,16);
        MemCopy(advData+21,major,2);
        MemCopy(advData+23,minor,2);
        advData[advPayloadSize+1]=-74;//Ref RSSI(txPower 1m)
    
        /* store the advertisement data */
        LsStoreAdvScanData(advPayloadSize+2 , advData, ad_src_advertise);
        
        scanRspData[0]=AD_TYPE_LOCAL_NAME_COMPLETE;
        MemCopy(scanRspData+1,ibeaconName1,sizeof(ibeaconName1));
        LsStoreAdvScanData(sizeof(ibeaconName1)+1 , scanRspData, ad_src_scan_rsp);        
    }

    /* decide whether continue to advertise*/
    LsStartStopAdvertise(TRUE, whitelist_disabled, ls_addr_type_public);
    //uint16 bra=0x01;
    //NvmWrite(&bra,1,0);    
    //TimerCreate(15*SECOND,TRUE,reintoOther);    
}
/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppPowerOnReset
 *
 *  DESCRIPTION
 *      This function is called just after a power-on reset (including after
 *      a firmware panic).
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

void AppPowerOnReset(void)
{
    /* empty */    
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppInit
 *
 *  DESCRIPTION
 *      This function is called after a power-on reset (including after a
 *      firmware panic) or after an HCI Reset has been requested.
 *
 *      NOTE: In the case of a power-on reset, this function is called
 *      after AppPowerOnReset().
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

void AppInit(sleep_state last_sleep_state)
{
    /* set all PIOs to inputs and pull them down */
    //PioSetModes(0xFFFFFFFFUL, pio_mode_user);
    //PioSetDirs(0xFFFFFFFFUL, FALSE);
    //PioSetPullModes(0xFFFFFFFFUL, pio_mode_strong_pull_down);
    PioSetMode(TOGGLE, pio_mode_user);
    PioSetDir(TOGGLE, 1L<<TOGGLE);
    PioSetPullModes(1L<<TOGGLE, pio_mode_strong_pull_up);
    PioSetEventMask(1L<<TOGGLE,pio_event_mode_falling);
    /* disable wake up on UART RX */
    SleepWakeOnUartRX(FALSE);
    TimerInit(timers_num,app_timer);
    /* pull down the I2C lines */
    PioSetI2CPullMode(pio_i2c_pull_mode_strong_pull_down);
    NvmConfigureI2cEeprom();
    uint16 bra=0x00;
    NvmRead(&bra,1,0);
    if(bra==0xffff)
    {
        bra=0x01;
        NvmWrite(&bra,1,0);
    }
    if(bra==0x01)
        TimerCreate(0*SECOND,TRUE,BeaconStart);//Eddystone
    if(bra==0x02)
        TimerCreate(0*SECOND,TRUE,startAdvertising);//iBeacon    
}

extern void HandlePIOChangedEvent(pio_changed_data *pio_data)
{
    uint32 pios = PioGets();
    
    if(pio_data->pio_cause & (1L<<TOGGLE))   //caused by button pressed
    {
        if(!(pios & (1L<<TOGGLE)))   //pressed
        {

            button_press_tid = 
                TimerCreate(1*SECOND,TRUE, handleExtraLongButtonPress);
       
        }
        else                          //released
        {
            if(button_press_tid != TIMER_INVALID)
            {
                TimerDelete(button_press_tid);
                button_press_tid = TIMER_INVALID;
            }
        }
    }
   
}
void handleExtraLongButtonPress(timer_id tid)
{
    TimerDelete(tid);tid=TIMER_INVALID;    
    uint16 bra=0;
    NvmRead(&bra,1,0);
    switch(bra)
    {
        case 0x01:bra=0x02;break;
        case 0x02:bra=0x01;break;          
    }
    NvmWrite(&bra,1,0);
    WarmReset();
}/**/
/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessSystemEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a system event, such
 *      as a battery low notification, is received by the system.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

void AppProcessSystemEvent(sys_event_id id, void *data)
{
    switch(id)
    {
        case sys_event_pio_changed:
        {
             HandlePIOChangedEvent((pio_changed_data*)data);
        }
        break;/**/
        default:
        break;
    }
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessLmEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a LM-specific event is
 *      received by the system.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

bool AppProcessLmEvent(lm_event_code event_code, 
                       LM_EVENT_T *p_event_data)
{
    return TRUE;
}
