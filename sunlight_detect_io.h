#ifndef _SUNLIGHT_DETECT_IO_H_
#define _SUNLIGHT_DETECT_IO_H_
#include <pio.h>

#define PIO_SOLAR_DETECT    9   //三极管共射极电路io口配置
#define PIO_SIDE_CIRCURT    10

void sunlightDetectIoInit(void);

#endif