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

 *  Copyright Google 2014
 *
 * FILE
 *     esurl_beacon_service.c
 *
 * DESCRIPTION
 *     This file defines routines for using the Beacon Configuration Service.
 *
  *
 ****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *===========================================================================*/

#include <gatt.h>           /* GATT application interface */
#include <buf_utils.h>      /* Buffer functions */
#include <mem.h>            /* Memory routines */
#include <ls_app_if.h>      /* Link supervisor interface e.g. TX Power */
#include <thermometer.h>    //20170414
#include <battery.h>
#include <time.h>
/*============================================================================*
 *  Local Header Files
 *===========================================================================*/
#include "esurl_beacon_service.h" /* Interface to this file */
//#include "battery_service.h"    //20170414

/* Note AD Types referenced below can be found at the site below */
/* www.bluetooth.org/en-us/specification/assigned-numbers/generic-access-profile */

/* Esurl Beacon ADV header */
unsigned char adv_service_hdr[] =
{
    0x03, // Length of Service List
            0x03, // AD Type: Service List:Complete List of 16-bit Service Class UUIDs 
            0xAA, // Esurl Beacon Service Data UUID LSB
            0xFE // Esurl Beacon Service Data UUID MSB:little-endian      
};

/* Esurl Beacon Service Data Param and Bluetooth SIG assigned 16-bit UUID */
unsigned char adv_service_data_hdr[] = 
{
    0x16, // AD Type: Service Data
            0xAA, // Esurl Beacon Service Data UUID LSB:0XFEAA-Google(16-bit UUID Assigned Numbers)
            0xFE  // Esurl Beacon Service Data UUID MSB(MFG ID)
};

/*============================================================================*
 *  Constants Arrays  
 *============================================================================*/ 

/* Initialise Uri to http://eddystone.org for a new beacon (using compression) */
unsigned char initial_uri[ESURL_BEACON_DATA_MAX] =   //18
{
    //0x02, 'p', 'h', 'y', 's', 'i', 'c', 'a', 'l', '-', 'w', 'e', 'b', 0x08 //0x02:http://,0x08:.org
    //0x00,'i','n','t','e','r','q','u','i','p',0x07    //http://www.interquip.com
    //0x24,0x9d,0xc8,0x7b,0x01,0x8c,0xc5,0x3d,0x2f,0x51,//namespace(10B)/1.SHA1 hash of FQDM
    //0xfc,0x7e,0xc0,0xd3,0x1c,0x13,0xd0,0x6f,0x1f,0x30,//2.elided version 4 uuid
    //0x11,0x22,0x33,0x44,0x55,0x66,0x00,0x00          //distance(6B)+resved
    /**/
    //0X0A,0X51,0XAA,0X27,0X00,0X97,0XC9,0XE8,//AES128(EID)
};
   
/* Esurl Beacon Adv TX calibration for packets Low to high */
unsigned char adv_tx_power_levels[] =
{    ADV_TX_POWER_FOR_NEG_18,  // 0 LOWEST:-22
     ADV_TX_POWER_FOR_NEG_10,  // 1 LOW (DEFAULT):-14
     ADV_TX_POWER_FOR_NEG_2,   // 2 MEDIUM:-6
     ADV_TX_POWER_FOR_POS_6    // 3 HIGH:+2
};

/* Esurl Beacon Radio TX calibration  range low to high */
unsigned char radio_tx_power_levels[] =
{    RADIO_TX_POWER_NEG_18,    // 0 LOWEST:0
     RADIO_TX_POWER_NEG_10,    // 1 LOW (DEFAULT):2
     RADIO_TX_POWER_NEG_2,     // 2 MEDIUM:4 
     RADIO_TX_POWER_POS_6      // 3 HIGH:6 
};     


/* Esurl Beacon nvm write flag indicates if the esurl_beacon_data is dirty */
uint8 g_esurl_beacon_nvm_write_flag = FALSE;

/* Temporary buffer used for read/write characteristics */
uint8 g_esurl_beacon_buf[ESURL_BEACON_PERIOD_SIZE];//2

    
char* short2fixedpoint(int16 dat);
char* short2fixedpoint(int16 dat)   //8.8 fixed point
{
    bool isSigned=FALSE;
    static char retChar[2];
    if((dat&0x8000)==0x8000) isSigned=TRUE;
    if(isSigned==TRUE) retChar[0]=(0x80-(dat&0x7f));
    else retChar[0]=dat&0xff;
    retChar[1]=0x00;//input no decimal part
    return retChar;
}
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
void EsurlBeaconDataInit(void)
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
void EsurlBeaconInitChipReset(void)
{
    /* Initialize memory at reset before first NVM write */
    /* set the adv size to be the minimum (Just the header) */
    g_esurl_beacon_data.adv_length = BEACON_DATA_HDR_SIZE;//10    
    
    /* Init the ADV data */
    
    /* Init Beacon data hdr: 10 bytes*/
    MemCopy(g_esurl_beacon_data.adv.service_hdr, adv_service_hdr, sizeof(adv_service_hdr));//4
    g_esurl_beacon_data.adv.service_data_length = SERVICE_DATA_PRE_URI_SIZE;//5
    MemCopy(g_esurl_beacon_data.adv.service_data_hdr, adv_service_data_hdr, sizeof(adv_service_data_hdr));//3    
    g_esurl_beacon_data.adv.flags = FLAGS_DEFAULT;//0x20:TLM
    g_esurl_beacon_data.adv.tx_power = 0x00;//ADV_TX_POWER_DEFAULT;////0xfff2:-14dBm
    g_esurl_beacon_data.tlm_set=0x01;
    
    uint8 rawTlmData[12];char *tempT;
    uint16 batLevelPacked;
    int16 tempBeaconPacked;
    uint32 adv_count,sec_count;
    batLevelPacked=BatteryReadVoltage();
    tempBeaconPacked=ThermometerReadTemperature();
    tempT=short2fixedpoint(tempBeaconPacked);
    adv_count=0x01;
    sec_count=TimeGet32();
    rawTlmData[0] =(batLevelPacked>>8)&0xff;  //VBatt(big_endian)
    rawTlmData[1] =(batLevelPacked&0xff);      
    //rawTlmData[2] =(tempBeaconPacked>>8)&0xff;//TEMP
    //rawTlmData[3] =(tempBeaconPacked&0xff);    
    MemCopy(rawTlmData+2,tempT,2);
    rawTlmData[4] =(adv_count>>24)&0xff;      //ADV_CNT
    rawTlmData[5] =(adv_count>>16)&0xff;
    rawTlmData[6] =(adv_count>>8 )&0xff;
    rawTlmData[7] =(adv_count&0xff);
    rawTlmData[8] =(sec_count>>24)&0xff;      //SEC_CNT
    rawTlmData[9] =(sec_count>>16)&0xff;
    rawTlmData[10]=(sec_count>>8 )&0xff;
    rawTlmData[11]=(sec_count&0xff);/**/

    /* Initialize uri_data memory in adv: 0 - 18 bytes */
    MemSet(g_esurl_beacon_data.adv.uri_data, 0, ESURL_BEACON_DATA_MAX);//0<14<18 
    MemCopy(initial_uri,rawTlmData,sizeof(rawTlmData));//12
    /* Initialize uri_data with a URI:  http://physical-web.org */
    MemCopy(g_esurl_beacon_data.adv.uri_data, initial_uri, sizeof(rawTlmData));//14
    g_esurl_beacon_data.adv.service_data_length = 0x11;//SERVICE_DATA_PRE_URI_SIZE + sizeof(initial_uri);
    //0x11;(TLM)//5+8=0x0D;//0x17;(UID)
    g_esurl_beacon_data.adv_length = BEACON_DATA_HDR_SIZE + sizeof(rawTlmData);//10+14=0x18    
    
    /* Managagement Data: Not included in transmitted ADV packet */
    
    /* Inialized lock to be unlocked */
    g_esurl_beacon_data.lock_state = FALSE;       
    
    /* Init memory in esurl beacon lock_code */
    MemSet(g_esurl_beacon_data.lock_code, 0, ESURL_BEACON_LOCK_CODE_SIZE);//16  
    
    /* tx mode values 0-3: this is looked up in the calibration tabs below */    
    g_esurl_beacon_data.tx_power_mode = TX_POWER_MODE_DEFAULT;//TX_POWER_MODE_LOW=1  
    
    MemCopy(g_esurl_beacon_data.adv_tx_power_levels, adv_tx_power_levels, ESURL_BEACON_ADV_TX_POWER_LEVELS_SIZE);//4
    
    MemCopy(g_esurl_beacon_data.radio_tx_power_levels, radio_tx_power_levels, ESURL_BEACON_RADIO_TX_POWER_LEVELS_SIZE);//4 
    
    /* Update ADV pkt and RADIO based on the TX power mode default */
    EsurlBeaconUpdateTxPowerFromMode(TX_POWER_MODE_DEFAULT);//1    
    
    /* Set default period = 1000 milliseconds */
    g_esurl_beacon_data.period = 1000;//0x3e8
    g_esurl_beacon_data.on_state = 0x00;//default:OFF
    
    /* Flag data structure needs writing to NVM */
    g_esurl_beacon_nvm_write_flag = TRUE;    
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      EsurlBeaconGetData
 *
 *  DESCRIPTION
 *      This function returns the current value of the beacon data
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void EsurlBeaconGetData(uint8** data, uint8* data_size)
{
    /* return current values */
    *data = (uint8*) &g_esurl_beacon_data.adv;
    *data_size = g_esurl_beacon_data.adv_length;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      EsurlBeaconGetPeriod
 *
 *  DESCRIPTION
 *      This function returns the current value of the beacon period (1-65kms)
 *
 *  RETURNS
 *      Beacon Period (unit32)
 *----------------------------------------------------------------------------*/
extern uint32 EsurlBeaconGetPeriodMillis(void)
{
    /* return current value */
    return (uint32) g_esurl_beacon_data.period * (SECOND / 1000);//MILLISECOND 
}


/*----------------------------------------------------------------------------*
  *  NAME
  *      EsurlBeaconBondingNotify
  *
  *  DESCRIPTION
  *      This function is used by application to notify bonding status to the
  *      Beacon Service.
  *
  *  PARAMETERS
  *      None
  *
  *  RETURNS
  *      Nothing
  *----------------------------------------------------------------------------*/
extern void EsurlBeaconBondingNotify(void)
{
    /* Empty */
}

/*----------------------------------------------------------------------------*
  *  NAME
  *      EsurlBeaconUpdateTxFromMode
  *
  *  DESCRIPTION
  *      This function is used to set both the ADV and RADIO TX param in the
  *      Esurl Beacon packet
  *
  *  PARAMETERS
  *      None
  *
  *  RETURNS
  *      Nothing
  *----------------------------------------------------------------------------*/
extern void EsurlBeaconUpdateTxPowerFromMode(uint8 tx_power_mode)
{
    /* Update the pkt tx level here */
    /*g_esurl_beacon_data.adv.tx_power =
            g_esurl_beacon_data.adv_tx_power_levels[tx_power_mode];*///only to TLM
    /* Update the radio tx level here */
    LsSetTransmitPowerLevel(
            g_esurl_beacon_data.radio_tx_power_levels[tx_power_mode]);
}

/*----------------------------------------------------------------------------*
  *  NAME
  *      EsurlBeaconGetTxPowerMode
  *
  *  DESCRIPTION
  *      This function is used find the last Tx Power Mode set by a client
  *
  *  PARAMETERS
  *      None
  *
  *  RETURNS
  *      uint8 : a power mode 0 - 3
  *----------------------------------------------------------------------------*/
extern uint8 EsurlBeaconGetTxPowerMode(void) 
{
    return g_esurl_beacon_data.tx_power_mode;
}