/******************************************************************************
 *    Copyright (c) 2015 Cambridge Silicon Radio Limited 
 *    All rights reserved.
 * 
 *    Redistribution and use in source and binary forms, with or without modification, 
 *    are permitted (subject to the limitations in the disclaimer below) provided that the
 *    following conditions are met:
 *
 *    Redistributions of source code must retain the above copyright notice, this list of 
 *    conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright notice, this list of conditions 
 *    and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *    Neither the name of copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * 
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER 
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE 
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  FILE
 *      hw_access.c
 *
 *  DESCRIPTION
 *      This file defines the application hardware specific routines.
 *
 
 *****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <pio.h>            /* PIO configuration and control functions */
#include <pio_ctrlr.h>      /* Access to the PIO controller */
#include <timer.h>          /* Chip timer functions */
//#include <gatt_access.h>
#include <aio.h>    //判定有无光
#include <sleep.h>  //省电模式
#include <ls_app_if.h>
#include <nvm.h>
#include <reset.h>

/*============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "hw_access.h"      /* Interface to this file */
#include "beacon.h"    /* Definitions used throughout the GATT server */
//#include "buzzer.h"         /* Buzzer functions */
//#include "led.h"            /* LED functions */
#include "nvm_access.h"
#include "eddystone\eddystone_adv.h"
#include "ibeacon\ibeacon_adv.h"
#include "ibeacon\ibeacon_service.h"

#include "sunlight_detect_io.h"
#include "app_gatt_db.h"
/*============================================================================*
 *  Private Definitions
 *============================================================================*/

/* Extra long button press timer */
#define EXTRA_LONG_BUTTON_PRESS_TIMER   (1*SECOND)//(4*SECOND)


timer_id button_press_tid;
bool sunLight=FALSE;

/*============================================================================*
 *  Public data type
 *============================================================================*/
/*typedef enum{
    beacon_type_ibeacon=0,
    beacon_type_eddystone_tlm=1
    //beacon_type_eddystone_uid=2
}beacon_type;*/
typedef enum{
    sunlight_init=0x0000,
    sunlight_still=0x0001
} sunlight_state;

/* Application Hardware data structure */
typedef struct _APP_HW_DATA_T
{
    //beacon_type beacon_entity;
    sunlight_state currentSunlightState;
    /* Timer for button press */
    timer_id                    button_press_tid;

} APP_HW_DATA_T;

/*============================================================================*
 *  Public data
 *============================================================================*/

/* Application hardware data instance */
static APP_HW_DATA_T            g_app_hw_data;

/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

/* Handle extra long button press */
static void handleExtraLongButtonPress(timer_id tid);


/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleExtraLongButtonPress
 *
 *  DESCRIPTION
 *      This function contains handling of extra long button press, which
 *      triggers pairing / bonding removal.
 *
 *  PARAMETERS
 *      tid [in]                ID of timer that has expired
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleExtraLongButtonPress(timer_id tid)
{
    TimerDelete(button_press_tid);button_press_tid=TIMER_INVALID;
    TimerDelete(aioReadSolar_tid);aioReadSolar_tid=TIMER_INVALID;
    //PioSetDir(PIO_SIDE_CIRCURT,0);//E:1/INT X/USER V
    //PioSetEventMask(1L<<PIO_SOLAR_DETECT,pio_event_mode_disable);    
    PioSetEventMask(BUTTON_PIO_MASK, pio_event_mode_falling);

    //BeaconGetConfigFlagFromNvm(&BeaconConfig,1);
    //uint8 branchConfigTemp=(BeaconConfig>>8)&0xff;
    BeaconConfig|=0xff00;
    BeaconConfig&=0x03ff;//config mode
    /*switch(branchConfigTemp)
    {
        case 0x01:           
            BeaconConfig&=0X02ff;
            break;
        case 0x02:
            BeaconConfig&=0x03ff;
            break;
        case 0x03:
            BeaconConfig&=0x01ff;
            break;
        default:
            BeaconConfig&=0x01ff;//出厂模式:default
            break;
    }*/
    BeaconSaveConfigFlagToNvm(&BeaconConfig,1); 
    //SetState(app_state_fast_advertising);//beaconing
    WarmReset();
}

/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      InitHardware
 *
 *  DESCRIPTION
 *      This function is called to initialise the application hardware.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void InitHardware(void)
{
    /* Setup PIOs
     * PIO11 - Button
     */
    /* Set the button PIO to user mode */
    PioSetModes(BUTTON_PIO_MASK, pio_mode_user);

    /* Set the PIO direction as input */
    PioSetDir(BUTTON_PIO, PIO_DIRECTION_INPUT);

    /* Pull up the PIO */
    PioSetPullModes(BUTTON_PIO_MASK, pio_mode_strong_pull_up);
    
    /*Enable the eeprom WP*/
    /*PioSetMode(WP_PIO,pio_mode_user);
    PioSetPullModes(WP_PIO_MASK,pio_mode_strong_pull_up);
    PioSetDir(WP_PIO,1);//output
    PioSet(WP_PIO,0);//disable the WP function
    */
    /* Initialise buzzer hardware */
    //BuzzerInitHardware();
    
    /* Initialise LED hardware */
    //LedInitHardware();
    
    sunlightDetectIoInit();
    g_app_hw_data.currentSunlightState=sunlight_init;

    /* Request an event when the button PIO changes state */
    PioSetEventMask(BUTTON_PIO_MASK, pio_event_mode_disable);

    /* Save power by changing the I2C pull mode to pull down.*/
    PioSetI2CPullMode(pio_i2c_pull_mode_strong_pull_down);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      HwDataInit
 *
 *  DESCRIPTION
 *      This function initialises the hardware data to a known state. It is
 *      intended to be called once, for example after a power-on reset or HCI
 *      reset.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void HwDataInit(void)
{
    /* Initialise button press timer */
    g_app_hw_data.button_press_tid = TIMER_INVALID;

    //g_app_hw_data.currentState=EsurlGetAppState();
    /* Initialise buzzer data */
    //BuzzerInitData();
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      HwDataReset
 *
 *  DESCRIPTION
 *      This function resets the hardware data. It is intended to be called when
 *      the data needs to be reset to a clean state, for example, whenever a
 *      device connects or disconnects.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void HwDataReset(void)
{
    /* Delete button press timer */
    if (g_app_hw_data.button_press_tid != TIMER_INVALID)
    {
        TimerDelete(g_app_hw_data.button_press_tid);
        g_app_hw_data.button_press_tid = TIMER_INVALID;
    }

    /* Reset buzzer data */
    //BuzzerResetData();
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      HandlePIOChangedEvent
 *
 *  DESCRIPTION
 *      This function handles the PIO Changed event.
 *
 *  PARAMETERS
 *      pio_data [in]           State of the PIOs when the event occurred
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void HandlePIOChangedEvent(pio_changed_data *pio_data)
{
    uint32 pios = PioGets();
    
    if(pio_data->pio_cause & BUTTON_PIO_MASK)   //caused by button pressed
    {
        if(!(pios & BUTTON_PIO_MASK))   //pressed
        {   //E:1/INT X/USER X
            LsStartStopAdvertise(FALSE, whitelist_disabled, ls_addr_type_public);
            PioSetDir(PIO_SIDE_CIRCURT,0);
            PioSetEventMask(1L<<PIO_SOLAR_DETECT,pio_event_mode_disable);
            PioSetEventMask(BUTTON_PIO_MASK, pio_event_mode_disable);
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
    if(pio_data->pio_cause & (1L<<PIO_SOLAR_DETECT))
    {   //E:1/INT X/USER V
        if(PioGet(PIO_SOLAR_DETECT)==0)     //solar charged        
        {   //pressed
            BeaconGetConfigFlagFromNvm(&BeaconConfig,1);
            if( ((BeaconConfig>>8)&0xff)==0x01)
                BeaconStart(TRUE);//EddystoneAdv(TRUE);//Eddystone
            if( ((BeaconConfig>>8)&0xff)==0x02)
                startAdvertising(TRUE);//ibeaconAdv();//iBeacon
            timesCtrl=0;
            g_app_data.state=app_state_beaconing;
            PioSetDir(PIO_SIDE_CIRCURT,0);//E:1/INT X/USER V
            PioSetEventMask(1L<<PIO_SOLAR_DETECT,pio_event_mode_disable);
            PioSetEventMask(BUTTON_PIO_MASK, pio_event_mode_falling);
            aioReadSolar_tid=TimerCreate(20*SECOND,TRUE,handleAioReadSolarLevel);
        }
    }    
}

void handleAioReadSolarLevel(timer_id tid)  //to detect no-sunlight state
{
    if(aioReadSolar_tid!=TIMER_INVALID)
    {
        TimerDelete(aioReadSolar_tid);
        aioReadSolar_tid=TIMER_INVALID;
    }
    //PioSetEventMask(BUTTON_PIO_MASK,pio_event_mode_falling);
    //BeaconGetConfigFlagFromNvm(&BeaconConfig,1);

    //if(((BeaconConfig>>4)&0x000f)==BEACON_SAVING_POWER_ON)//saving power model
    {
        uint16 solarLevel=AioRead(AIO0);//AIO2
        if(solarLevel<800)  
        {
            enableBtnIntEventMask();//E:0/INT V/USER V
            SetState(app_state_sleep);
        }
        else 
        {
            //if(timesCtrl==0)
            {
                switch(BeaconConfig&0xff00)
                {
                    case 0x0100://eddystone
                        enableBtnEventMask();//E:1/INT X/USER V
                        g_app_data.state=app_state_beaconing;
                        BeaconStart(TRUE);//Eddystone 
                    break;
                    
                    case 0x0200://ibeacon
                        enableBtnEventMask();//E:1/INT X/USER V
                        g_app_data.state=app_state_beaconing;
                        startAdvertising(TRUE);//ibeacon 
                    break;
                }
                //timesCtrl++;
            }
        }               
            /*//有光:first time
            if( ((BeaconConfig>>8)&0xff)==0x01)
                BeaconStart(TRUE);//EddystoneAdv(TRUE);//Eddystone
            if( ((BeaconConfig>>8)&0xff)==0x02)
                startAdvertising(TRUE);//ibeaconAdv();//iBeacon
            g_app_data.state=app_state_beaconing;
            PioSetDir(PIO_SIDE_CIRCURT,0);//E:1/INT X/USER V
            PioSetEventMask(1L<<PIO_SOLAR_DETECT,pio_event_mode_disable);
            */
        aioReadSolar_tid=TimerCreate(20*SECOND,TRUE,handleAioReadSolarLevel);
    }
}
