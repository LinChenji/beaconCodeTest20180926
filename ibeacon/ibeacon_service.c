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
 * FILE
 *     battery_service.c
 *
 * DESCRIPTION
 *     This file defines routines for using the Battery Service.
 *
 
 ****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *===========================================================================*/

#include <gatt.h>           /* GATT application interface */
#include <battery.h>        /* Read the battery voltage */
#include <buf_utils.h>      /* Buffer functions */
#include <mem.h>
/*============================================================================*
 *  Local Header Files
 *===========================================================================*/

#include "beacon.h"    /* Definitions used throughout the GATT server */
#include "battery_service.h"/* Interface to this file */
#include "nvm_access.h"     /* Non-volatile memory access */
#include "app_gatt_db.h"    /* GATT database definitions */
#include "ibeacon\ibeacon_adv.h"
#include "eddystone\eddystone_service.h"
/*============================================================================*
 *  Private Data
 *===========================================================================*/

/* Esurl Beacon nvm write flag indicates if the esurl_beacon_data is dirty */
bool g_ibeacon_nvm_write_flag=FALSE;//write protect switch

/*============================================================================*
 *  Private Function Implementations
 *===========================================================================*/


/*============================================================================*
 *  Public Function Implementations
 *===========================================================================*/
/*----------------------------------------------------------------------------*
 *  NAME
 *      EsurlBeaconDataInit
 *
 *  DESCRIPTION
 *      This function is used to initialise the Beacon Service data structure.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void iBeaconDataInit(void)
{
    /* Data initialized from NVM during readPersistentStore */
    
}
/*----------------------------------------------------------------------------*
 *  NAME
 *      EsurlBeaconInitChipReset
 *
 *  DESCRIPTION
 *      This function is used to initialise the Beacon Service data structure
 *      at chip reset.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void iBeaconInitChipReset(void)
{
    g_ibeacon_data.active_slot=0x00;
    g_ibeacon_data.adv_interval=DEFAULT_ADVERTISING_INTERVAL;//500ms
    g_ibeacon_data.tx_power=0xbc;//-68dBm
    uint8 pkt[20]={
        0xaf,0x2f,0x1d,0x73,0x92,0xb8,0x46,0x71,
        0xbb,0x21,0x45,0xeb,0xff,0x64,0xa8,0x94,//uuid:af2f1d73-92b8-4671-bb21-45ebff64a894
        0x15,0x01,//major
        0x15,0x08 //minor
    };
    MemCopy(g_ibeacon_data.payload,pkt,sizeof(pkt));
    g_ibeacon_data.extra_byte=0x00;//bat_level %:inactive
    g_ibeacon_data.savingPowerSwitch=BEACON_SAVING_POWER_ON;//1
    g_ibeacon_data.on_state=0x01;//default:ON
    g_ibeacon_nvm_write_flag=TRUE;
    //为应付灾难性的掉电，打开写保护开关让其当前状态得以保存，这样能够从当前恢复过来
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      BatteryHandleAccessRead
 *
 *  DESCRIPTION
 *      This function handles read operations on Battery Service attributes
 *      maintained by the application and responds with the GATT_ACCESS_RSP
 *      message.
 *
 *  PARAMETERS
 *      p_ind [in]              Data received in GATT_ACCESS_IND message.
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void iBeaconHandleAccessRead(GATT_ACCESS_IND_T *p_ind)
{
    uint16 length = 0;                  /* Length of attribute data, octets */
    uint8  value[20];                    /* Attribute value */
    uint8 *p_val = NULL;                /* Pointer to attribute value */
    sys_status rc = sys_status_success; /* Function status */
    //iBeaconReadDataFromNVM(&g_ibeacon_nvm_offset);//return last updated status
    
    switch(p_ind->handle)
    {
        case HANDLE_SLOT_ACTIVE:
        {
            length = 1; /* One Octet */
            value[0] = g_ibeacon_data.active_slot;
        }
        break;

        case HANDLE_ADVERTISING_INTREVAL:
        {
            uint8 valueUnoack[2];
            length = 2; /* Two Octets */
            p_val = value;
            valueUnoack[0]=(g_ibeacon_data.adv_interval>>8)&0xff;
            valueUnoack[1]=g_ibeacon_data.adv_interval&0xff;
            MemCopy(p_val,valueUnoack,sizeof(valueUnoack));
        }
        break;
        
        case HANDLE_TX_POWER:
        {
            length = 1;
            value[0]=g_ibeacon_data.tx_power;
        }
        break;
        
        case HANDLE_CALCULATOR_POWER:
        {
            length = 1;
            value[0]=g_ibeacon_data.cal_power;
        }
        break;  
        
        case HANDLE_UUID_MAJOR_MINOR:
        {
            length = 20;
            MemCopy(value,g_ibeacon_data.payload,length);            
        }
        break; 
        
        case HANDLE_EXTRA_BYTE:
        {
            length = 1;
            value[0]=g_ibeacon_data.cal_power;
        }
        break;         

        case HANDLE_SAVING_POWER_SWITCH:
        {
            length=1;
            value[0]=g_ibeacon_data.savingPowerSwitch;
        }
        break;
        
        case HANDLE_ON_STATE:
        {
            length=1;
            value[0]=g_ibeacon_data.on_state;
        }
        break;
        
        default:
            /* No more IRQ characteristics */
            rc = gatt_status_read_not_permitted;
        break;

    }

    /* Send ACCESS RESPONSE */
    GattAccessRsp(p_ind->cid, p_ind->handle, rc, length, value);

}

/*----------------------------------------------------------------------------*
 *  NAME
 *      HandleAccessWrite
 *
 *  DESCRIPTION
 *      This function handles write operations on Battery Service attributes
 *      maintained by the application and responds with the GATT_ACCESS_RSP
 *      message.
 *
 *  PARAMETERS
 *      p_ind [in]              Data received in GATT_ACCESS_IND message.
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void iBeaconHandleAccessWrite(GATT_ACCESS_IND_T *p_ind)
{   
    uint8 *p_value = p_ind->value;      /* New attribute value */
    //uint16 client_config;               /* Client configuration descriptor */
    sys_status rc = sys_status_success; /* Function status */
    g_ibeacon_data.active_slot++;
   
    //bool isChanged=FALSE;
    switch(p_ind->handle)
    {
        case HANDLE_SLOT_ACTIVE:
        {       
            MemCopy(&g_ibeacon_data.active_slot,p_value,p_ind->size_value);
            g_ibeacon_nvm_write_flag=TRUE;
        }
        break;

        case HANDLE_ADVERTISING_INTREVAL:
        {
            uint8 valueUnpack[2];
            MemCopy(valueUnpack,p_value,p_ind->size_value);
            g_ibeacon_data.adv_interval=valueUnpack[1]|((uint16)valueUnpack[0]<<8);
            g_ibeacon_nvm_write_flag=TRUE;
            //BufWriteUint16((uint8 **)&p_val, g_ibeacon_data.adv_interval);
        }
        break;
        
        case HANDLE_TX_POWER:
        {
            MemCopy(&g_ibeacon_data.tx_power,p_value,p_ind->size_value);
            g_ibeacon_nvm_write_flag=TRUE;
        }
        break;
        
        case HANDLE_CALCULATOR_POWER:
        {
            MemCopy(&g_ibeacon_data.cal_power,p_value,p_ind->size_value);
            g_ibeacon_nvm_write_flag=TRUE;
        }
        break;  
        
        case HANDLE_UUID_MAJOR_MINOR:
        {
            MemSet(g_ibeacon_data.payload,0x00,sizeof(g_ibeacon_data.payload));
            MemCopy(g_ibeacon_data.payload,p_value,p_ind->size_value); 
            g_ibeacon_nvm_write_flag=TRUE;
        }
        break; 
        
        case HANDLE_EXTRA_BYTE:
        {
            MemCopy(&g_ibeacon_data.extra_byte,p_value,p_ind->size_value);
            g_ibeacon_nvm_write_flag=TRUE;
        }
        break;    

        case HANDLE_SAVING_POWER_SWITCH:
        {
            MemCopy(&g_ibeacon_data.savingPowerSwitch,p_value,p_ind->size_value);
            if(g_ibeacon_data.savingPowerSwitch==0x00)
                BeaconConfig&=0xff0f;
            else
                BeaconConfig|=0x0010;
            BeaconSaveConfigFlagToNvm(&BeaconConfig,1);/**/
            g_ibeacon_nvm_write_flag=TRUE;
        }
        break;
        
        case HANDLE_ON_STATE:
        {
            uint8 newWrittenVal=0;          
            MemCopy(&newWrittenVal,p_value,p_ind->size_value);
            //if(newWrittenVal!=g_ibeacon_data.on_state)
            //    isChanged=TRUE;
            //else isChanged=FALSE;
            //if(isChanged==TRUE)
            {
                g_ibeacon_data.on_state=newWrittenVal;
                if( g_ibeacon_data.on_state==0x01 )
                    g_esurl_beacon_data.on_state=0x00;
                else g_esurl_beacon_data.on_state=0x01;/**/
                //g_esurl_beacon_data.on_state=!(g_esurl_beacon_data.on_state);
                g_ibeacon_nvm_write_flag=TRUE;
            }
        }
        break;
        
        default:
            rc = gatt_status_write_not_permitted;
        break;
    }
    if (g_ibeacon_nvm_write_flag) //提前存盘
    {
        /* Write all ibeacon service data into NVM */
        Nvm_Write((uint16*)&g_ibeacon_data, sizeof(g_ibeacon_data),
                  g_ibeacon_nvm_offset);//该偏移在开机PS保存动作求得，未做掉电保存处理
        /* Write all esurl beacon service data into NVM */
        Nvm_Write((uint16*)&g_esurl_beacon_data, sizeof(g_esurl_beacon_data),
                  g_esurl_beacon_nvm_offset);        
        g_ibeacon_nvm_write_flag = FALSE;
        BeaconGetConfigFlagFromNvm(&BeaconConfig,1);
        
        BeaconConfig|=0x000f;
        BeaconConfig&=0x00f1;
        if( g_ibeacon_data.on_state==0x01 )
            BeaconConfig|=0x0200;
        else
            BeaconConfig|=0x0100;
    }
    BeaconSaveConfigFlagToNvm(&BeaconConfig,1);
    /* Send ACCESS RESPONSE */
    GattAccessRsp(p_ind->cid, p_ind->handle, rc, 0, NULL);

    /* Send an update as soon as notifications are configured */
    //if(g_ibeacon_data.level_client_config == gatt_client_config_notification)
    {

    }

}


/*----------------------------------------------------------------------------*
 *  NAME
 *      iBeaconCheckHandleRange
 *
 *  DESCRIPTION
 *      This function is used to check if the handle belongs to the Battery 
 *      Service.
 *
 *  PARAMETERS
 *      handle [in]             Handle to check
 *
 *  RETURNS
 *      TRUE if handle belongs to the Battery Service, FALSE otherwise
 *----------------------------------------------------------------------------*/
extern bool iBeaconCheckHandleRange(uint16 handle)
{
    return ((handle >= HANDLE_IBEACON_SERVICE) &&
            (handle <= HANDLE_IBEACON_SERVICE_END))
            ? TRUE : FALSE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      EsurlBeaconReadDataFromNVM
 *
 *  DESCRIPTION
 *      This function is used to read Beacon Service specific data stored in 
 *      NVM.
 *
 *  PARAMETERS
 *      p_offset [in]           Offset to Beacon Service data in NVM
 *               [out]          Offset to next entry in NVM
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void iBeaconReadDataFromNVM(uint16 *p_offset)
{
    g_ibeacon_nvm_offset = *p_offset;
    
    /* Read beacon data size */
    Nvm_Read((uint16*)&g_ibeacon_data, sizeof(g_ibeacon_data),
             g_ibeacon_nvm_offset);//整个NVM倒出
    
    *p_offset += sizeof(g_ibeacon_data);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      EsurlBeaconWriteDataToNVM
 *
 *  DESCRIPTION
 *      This function is used to write Beacon Service specific data in memory to 
 *      NVM.
 *
 *  PARAMETERS
*      p_offset  [in]           Offset to EsurlBeacon Service data in NVM
 *               [out]          Offset to next entry in NVM
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void iBeaconWriteDataToNVM(uint16 *p_offset)
{   //该函数只能应对断连存盘，而不能应对灾难性的断电存盘问题
    if (*p_offset != NULL) 
    { /* Update the stored nvm offset with the pointer arg */
        g_ibeacon_nvm_offset = *p_offset;//指定确定位置    
    } else {
        /* Null arg, so the .nvm_offset is already set correctly */      
        *p_offset = g_ibeacon_nvm_offset;//自己原位置确定
    }
    
    /* Only write out the esurl beacon data if it is flagged dirty */
    if (g_ibeacon_nvm_write_flag) 
    {
        /* Write all esurl beacon service data into NVM */
        Nvm_Write((uint16*)&g_ibeacon_data, sizeof(g_ibeacon_data),
                  g_ibeacon_nvm_offset); 
        g_ibeacon_nvm_write_flag = FALSE;
    }
    
    *p_offset += sizeof(g_ibeacon_data); 
}

uint16 GetIbeaconAdvertisingInterval()
{
    return g_ibeacon_data.adv_interval;
}

bool GetBeaconCurrentPowerModel()
{
    return g_ibeacon_data.savingPowerSwitch;
}
uint8 *GetIbeaconUuidMajorMinor()
{
    return g_ibeacon_data.payload;
}
int8 GetIbeaconTxPower()
{
    return g_ibeacon_data.tx_power;
}