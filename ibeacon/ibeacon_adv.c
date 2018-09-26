#include <gap_app_if.h>
#include <config_store.h>
#include <random.h>
#include <mem.h>

#include "ibeacon_adv.h"
#include "ibeacon_service.h"

static void appSetRandomAddress(void);

/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/
static void appSetRandomAddress(void)
{
    BD_ADDR_T addr;

    /* "completely" random MAC addresses by default: */
    for(;;)
    {
        uint32 now = TimeGet32();
        /* Random32() is just two of them, no use */
        uint32 rnd = Random16();
        addr.uap = 0xff & (rnd ^ now);//nap uap lap(2B 1B 3B)
        /* No sub-part may be zero or all-1s */
        if ( 0 == addr.uap || 0xff == addr.uap ) continue;
        addr.lap = 0xffffff & ((now >> 8) ^ (73 * rnd));
        if ( 0 == addr.lap || 0xffffff == addr.lap ) continue;
        addr.nap = 0x3fff & rnd;
        if ( 0 == addr.nap || 0x3fff == addr.nap ) continue;
        break;
    }

    /* Set it to actually be an acceptable random address */
    addr.nap &= ~BD_ADDR_NAP_RANDOM_TYPE_MASK;      //bluetooth.h:0xc000
    addr.nap |=  BD_ADDR_NAP_RANDOM_TYPE_NONRESOLV; //0x0000
    GapSetRandomAddress(&addr);
}

void ibeaconAdv(void)
{
    uint8 advData[MAX_ADVERT_PACKET_SIZE];//31
    uint16 offset = 0;
    uint8 filler;
    uint16 advInterval;
    uint8 advPayloadSize;
    ls_addr_type addressType = ls_addr_type_public;     /* use public address */
    
    /* initialise values from User CsKeys */
    
    /* read User key 0 for the payload filler */
    filler = (uint8)(CSReadUserKey(0) & 0x00FF);
    
    /* read User key 1 for the payload size */
    advPayloadSize = (uint8)(CSReadUserKey(1) & 0x00FF);
    
    /* range check */
    if((advPayloadSize < 1) || (advPayloadSize > MAX_ADVERT_PAYLOAD_SIZE))//24
    {
        /* revert to default payload size */
        advPayloadSize = DEFAULT_ADVERT_PAYLOAD_SIZE;//24
    }
    
    /* read User key 2 for the advertising interval */
    advInterval = g_ibeacon_data.adv_interval;//CSReadUserKey(2);
    
    /* range check */
    if((advInterval < MIN_ADVERTISING_INTERVAL) ||
       (advInterval > MAX_ADVERTISING_INTERVAL))    //[0.1,10.24](s)
    {
        /* revert to default advertising interval */
        advInterval = DEFAULT_ADVERTISING_INTERVAL;//1s
    }
    
    /* read address type from User key 3 */
    if(CSReadUserKey(3))
    {
        /* use random address type */
        addressType = ls_addr_type_random;

        /* generate and set the random address */
        appSetRandomAddress();
    }

    /* set the GAP Broadcaster role */
    GapSetMode(gap_role_broadcaster,
               gap_mode_discover_no,
               gap_mode_connect_no,
               gap_mode_bond_no,
               gap_mode_security_none);
    
    /* clear the existing advertisement data, if any */
    LsStoreAdvScanData(0, NULL, ad_src_advertise);

    /* set the advertisement interval, API accepts the value in microseconds */
    GapSetAdvInterval(advInterval * MILLISECOND, advInterval * MILLISECOND);
    
    /* manufacturer-specific data */
    advData[0] = AD_TYPE_MANUF;

    /* CSR company code, little endian */
    advData[1] = 0x0A;
    advData[2] = 0x00;
    
    /* fill in the rest of the advertisement */
    for(offset = 0; offset < advPayloadSize; offset++)
    {
        advData[3 + offset] = filler;
    }

    /* store the advertisement data */
    LsStoreAdvScanData(advPayloadSize + 3, advData, ad_src_advertise);
    
    /* Start broadcasting */
    LsStartStopAdvertise(TRUE, whitelist_disabled, addressType);    
}

void startAdvertising(bool start)
{
    uint8 advData[27];//MAX_ADVERT_PACKET_SIZE:31=0x1f
    uint8 scanRspData[31];
    MemSet(advData,0xff,sizeof(advData));
    MemSet(scanRspData,0xff,sizeof(scanRspData));
    uint16 advInterval;
    uint8 advPayloadSize;
    ls_addr_type addressType = ls_addr_type_public;     /* use public address */
    
    /* initialise values from User CsKeys */

        /* revert to default payload size */
    advPayloadSize = DEFAULT_ADVERT_PAYLOAD_SIZE;//24
    
    /* read User key 2 for the advertising interval */
    advInterval = GetIbeaconAdvertisingInterval();//1000ms
    
    /* range check */
    if((advInterval < MIN_ADVERTISING_INTERVAL) ||
       (advInterval > MAX_ADVERTISING_INTERVAL))    //[0.1,10.24](s)
    {
        /* revert to default advertising interval */
        advInterval = DEFAULT_ADVERTISING_INTERVAL;//1s
    }
    
    if(start)
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
        
        /* manufacturer-specific data */
        advData[0] = AD_TYPE_MANUF;//0xff
    
        /* Apple company code, little endian */
        advData[1] = 0x4c;
        advData[2] = 0x00;
        advData[3] = 0x02;//Magic No
        advData[4] = 0x15;//Length:21    
        MemCopy(advData+5,g_ibeacon_data.payload,sizeof(g_ibeacon_data.payload));
        advData[advPayloadSize+1]=GetIbeaconTxPower();//Ref RSSI(txPower 1m)
    
        /* store the advertisement data */
        LsStoreAdvScanData(advPayloadSize+2 , advData, ad_src_advertise);
        
        uint8 ibeaconName[]="IQ";
        scanRspData[0]=AD_TYPE_LOCAL_NAME_COMPLETE;
        MemCopy(scanRspData+1,ibeaconName,sizeof(ibeaconName));
        //LsStoreAdvScanData(sizeof(ibeaconName)+1 , scanRspData, ad_src_scan_rsp);/**/ 
    }

    /* decide whether continue to advertise*/
    LsStartStopAdvertise(start, whitelist_disabled, addressType);          
}