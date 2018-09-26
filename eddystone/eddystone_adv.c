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
 *     beaconing.c
 *
 * DESCRIPTION
 *     This file defines beaconing routines.
 *

 ****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *===========================================================================*/

#include <gatt.h>           /* GATT application interface */
#include <buf_utils.h>      /* Buffer functions */
#include <mem.h>            /* Memory routines */
#include <gatt_prim.h>
#include <gatt_uuid.h>
#include <ls_app_if.h>
#include <gap_app_if.h>

/*============================================================================*
 *  Local Header Files
 *===========================================================================*/

#include "eddystone_service.h" /* Interface to this file */
#include "eddystone_adv.h"      /* Beaconing routines */

/*=============================================================================*
 *  Private Definitions
 *============================================================================*/

/* Advertisement payload size:
 *      31
 *      - 3 octets for mandatory Flags AD (added automatically by the firmware)
 *      - 1 octet for manufacturer specific AD length field (added by the firmware)
 */
#define ADVERT_SIZE                     (28)

/*============================================================================*
 *  Public Function Implementations
 *===========================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      BeaconStart
 *
 *  DESCRIPTION
 *      This function is used to start or stop beaconing
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void EddystoneAdv(bool start)
{
    uint8 advData[ADVERT_SIZE];
    uint16 offset = 0;
    uint8* beacon_data;
    uint8 beacon_data_size;
    uint8 i;
    uint8 len_i = 0;
    uint8 adv_parameter_len = 0;
    EsurlBeaconInitChipReset();
    uint32 beacon_interval = EsurlBeaconGetPeriodMillis();    
    
    /* Stop broadcasting */
    LsStartStopAdvertise(FALSE, whitelist_disabled, ls_addr_type_public);
    
    /* beacon_interval of zero overrides and stops beaconning */
    if (start && (beacon_interval != 0)) 
    {
        /* prepare the advertisement packet */
   
        /* set the GAP Broadcaster role */
        GapSetMode(gap_role_broadcaster,
                   gap_mode_discover_no,
                   gap_mode_connect_no,
                   gap_mode_bond_no,
                   gap_mode_security_none);//mask AppProcessLmEvent
        
        /* clear the existing advertisement and scan response data */
        LsStoreAdvScanData(0, NULL, ad_src_advertise);
        LsStoreAdvScanData(0, NULL, ad_src_scan_rsp);
    
        /* set the advertisement interval */

        GapSetAdvInterval(beacon_interval, beacon_interval);
        
        /* get the beaconing data USING SERVICE */
        EsurlBeaconGetData(&beacon_data, &beacon_data_size);
        
        if(beacon_data_size > 0)
        {
            adv_parameter_len = beacon_data[0];
            len_i = adv_parameter_len - 1;
            
            /* and store in the packet */
            for(i = 1; (i < beacon_data_size) && (offset < ADVERT_SIZE); i++,offset++, len_i--)
            {
                advData[offset] = beacon_data[i];
                
                if(len_i == 0)
                {
                    /* store the advertisement parameter and get length for the next parameter */
                    LsStoreAdvScanData(adv_parameter_len, &advData[offset - adv_parameter_len + 1], ad_src_advertise);
    
                    adv_parameter_len = beacon_data[i+1];
                    len_i = adv_parameter_len;
                    i++;
                }
            }
        }
        else
        {
            /* store the advertisement data */
            LsStoreAdvScanData(offset, advData, ad_src_advertise);
        }
        
        /* Start broadcasting */
        LsStartStopAdvertise(TRUE, whitelist_disabled, ls_addr_type_public);
    }
}

void BeaconStart(bool start)
{
    uint8 advData[ADVERT_SIZE];
    uint8 scanRspData[31];
    MemSet(advData,0xff,sizeof(advData));
    MemSet(scanRspData,0xff,sizeof(scanRspData));    
    uint16 offset = 0;
    uint8* beacon_data;
    uint8 beacon_data_size;
    uint8 i;
    uint8 len_i = 0;
    uint8 adv_parameter_len = 0;
    uint32 beacon_interval = EsurlBeaconGetPeriodMillis();//beacon period:adv interval    
    
    /* Stop broadcasting */
    LsStartStopAdvertise(FALSE, whitelist_disabled, ls_addr_type_public);
    beaconState=0;
    /* beacon_interval of zero overrides and stops beaconning */
    if (start && (beacon_interval != 0)) 
    {
        /* prepare the advertisement packet */
        beaconState|=0x01;
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
            uint8 eddystoneName[13];
            
            switch( (*(beacon_data+8)) )
            {
                case FLAGS_UID:
                {
                    uint8 esName[]="UID";
                    MemCopy(eddystoneName,esName,sizeof(esName));
                    break;
                }
                case FLAGS_URL:
                {
                    uint8 esName[]="URL";
                    MemCopy(eddystoneName,esName,sizeof(esName));
                    break;
                }
                case FLAGS_TLM:
                {
                    uint8 esName[]="TLM";
                    MemCopy(eddystoneName,esName,sizeof(esName));
                    break;
                }                
            }           
            MemCopy(scanRspData+1,eddystoneName,3);//sizeof(esName)
            //LsStoreAdvScanData(3+1 , scanRspData, ad_src_scan_rsp);/**/
        }
        else
        {
            /* store the advertisement data */
            LsStoreAdvScanData(offset, advData, ad_src_advertise);
        }
        
        /* Start broadcasting */
        LsStartStopAdvertise(start, whitelist_disabled, ls_addr_type_public);//random
        //GattConnectReq(NULL, connect_flags);
    }
}
