#include <pio.h>
#include "sunlight_detect_io.h"


void sunlightDetectIoInit()
{   //E:1/INT X/USER X
    PioSetModes((1L<<PIO_SIDE_CIRCURT)|(1L<<PIO_SOLAR_DETECT),pio_mode_user);   
    PioSetDir(PIO_SOLAR_DETECT,0);//input(sense)
    PioSetPullModes((1L<<PIO_SIDE_CIRCURT)|(1L<<PIO_SOLAR_DETECT),\
                    pio_mode_strong_pull_up);    
    //PioSetDirs((1L<<PIO_SIDE_CIRCURT),(1L<<PIO_SIDE_CIRCURT));//output 1
    //PioSet(PIO_SIDE_CIRCURT,1);
    PioSetDirs(PIO_SIDE_CIRCURT,0);

    PioSetEventMask(1L<<PIO_SOLAR_DETECT,pio_event_mode_disable);      
    PioSetEventMask(1L<<PIO_SIDE_CIRCURT,pio_event_mode_disable);

    /* pull down the I2C lines */
    PioSetI2CPullMode(pio_i2c_pull_mode_strong_pull_down);
}
    