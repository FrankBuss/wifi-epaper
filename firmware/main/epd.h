#ifndef EPD_H
#define EPD_h

#include <stdint.h>

void epdInitHardware(void);

void epdShowImage(const uint8_t* newImage);

#endif
