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
 *      hw_access.h
 *
 *  DESCRIPTION
 *      Header definitions for HW setup.
 *

 *
 *****************************************************************************/

#ifndef __HW_ACCESS_H__
#define __HW_ACCESS_H__

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <sys_events.h>     /* System Event definitions and declarations */
#include <timer.h>
#include "gatt_access.h"
#include "sunlight_detect_io.h"
#include "beacon.h"

/*============================================================================*
 *  Private Data types
 *============================================================================*/
/* Application data structure */
typedef struct _APP_DATA_T
{
    /* Current state of application */
    app_state                  state;

    /* TYPED_BD_ADDR_T of the host to which device is connected */
    TYPED_BD_ADDR_T            con_bd_addr;

    /* Track the Connection Identifier (UCID) as Clients connect and
     * disconnect
     */
    uint16                     st_ucid;

    /* Boolean flag to indicate whether the device is bonded */
    bool                       bonded;

    /* TYPED_BD_ADDR_T of the host to which device is bonded */
    TYPED_BD_ADDR_T            bonded_bd_addr;//5B

    /* Diversifier associated with the Long Term Key (LTK) of the bonded
     * device
     */
    uint16                     diversifier;

    /* Timer ID for Connection Parameter Update timer in Connected state */
    timer_id                   con_param_update_tid;
    
    /* Central Private Address Resolution IRK. Will only be used when
     * central device used resolvable random address.
     */
    uint16                     irk[MAX_WORDS_IRK];//8

    /* Number of connection parameter update requests made */
    uint8                      num_conn_update_req;

    /* Boolean flag to indicate pairing button press */
    bool                       pairing_button_pressed;

    /* Timer ID for 'UNDIRECTED ADVERTS' and activity on the sensor device like
     * measurements or user intervention in CONNECTED state.
     */
    timer_id                   app_tid;

    /* Boolean flag to indicate whether to set white list with the bonded
     * device. This flag is used in an interim basis while configuring 
     * advertisements.
     */
    bool                       enable_white_list;

#ifdef PAIRING_SUPPORT
    /* Boolean flag to indicate whether encryption is enabled with the bonded
     * host
     */
    bool                       encrypt_enabled;

    /* This timer will be used if the application is already bonded to the 
     * remote host address but the remote device wanted to rebond which we had 
     * declined. In this scenario, we give ample time to the remote device to 
     * encrypt the link using old keys. If remote device doesn't encrypt the 
     * link, we will disconnect the link when this timer expires.
     */
    timer_id                   bonding_reattempt_tid;
#endif /* PAIRING_SUPPORT */

    /* Current connection interval */
    uint16                     conn_interval;

    /* Current slave latency */
    uint16                     conn_latency;

    /* Current connection timeout value */
    uint16                     conn_timeout;

} APP_DATA_T;

/* Application data instance */
APP_DATA_T g_app_data;

/*============================================================================*
 *  Private Data types
 *============================================================================*/
/* Convert a PIO number into a bit mask */
#define PIO_BIT_MASK(pio)       (0x01UL << (pio))

/* PIO direction */
#define PIO_DIRECTION_INPUT     (FALSE)
#define PIO_DIRECTION_OUTPUT    (TRUE)

/* PIO state */
#define PIO_STATE_HIGH          (TRUE)
#define PIO_STATE_LOW           (FALSE)


/* Setup PIO 11 as Button PIO */
#define BUTTON_PIO                  (11)
//#define WP_PIO                      (11)

#define BUTTON_PIO_MASK             (PIO_BIT_MASK(BUTTON_PIO))
//#define WP_PIO_MASK                 (PIO_BIT_MASK(WP_PIO))

uint16 selfStart_tid;
uint16 aioReadSolar_tid;

uint16 timesCtrl;
/*============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* Initialise the application hardware */
extern void InitHardware(void);

/* Initialise the application hardware data structure */
extern void HwDataInit(void);

/* Reset the application hardware data structure */
extern void HwDataReset(void);

/* Handle the PIO changed event */
extern void HandlePIOChangedEvent(pio_changed_data *pio_data);

void handleAioReadSolarLevel(timer_id tid);

void reconfigReadyForSunlightHandler(timer_id tid);

#endif /* __HW_ACCESS_H__ */
