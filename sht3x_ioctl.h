
#ifndef SHT3X_IOCTL_H
#define SHT3X_IOCTL_H

#include <linux/ioctl.h>


// IOCTL commands for the SHT3X driver
#define SHT3X_HEATER_CONTROL        _IOW('x',0x00,int)
#define SHT3X_MEASUREMENT_MODE      _IOW('x',0x01,int)
#define SHT3X_BREAK                 _IOW('x',0x02,int)
#define SHT3X_STATUS                _IOW('x',0x03,int)
#define SHT3X_CRC_CHECK             _IOW('x',0x04,int)

// SHT3X heater control argument
#define SHT3X_HEATER_DISABLE        0UL
#define SHT3X_HEATER_ENABLE         1UL

// SHT3X measurement mode argument
#define SHT3X_SINGLE_SHOT_LOW       0UL
#define SHT3X_SINGLE_SHOT_MED       1UL
#define SHT3X_SINGLE_SHOT_HIGH      2UL
#define SHT3X_PERIODIC_0P5_LOW      3UL
#define SHT3X_PERIODIC_0P5_MED      4UL
#define SHT3X_PERIODIC_0P5_HIGH     5UL
#define SHT3X_PERIODIC_1_LOW        6UL
#define SHT3X_PERIODIC_1_MED        7UL
#define SHT3X_PERIODIC_1_HIGH       8UL
#define SHT3X_PERIODIC_2_LOW        9UL
#define SHT3X_PERIODIC_2_MED       10UL
#define SHT3X_PERIODIC_2_HIGH      11UL
#define SHT3X_PERIODIC_4_LOW       12UL
#define SHT3X_PERIODIC_4_MED       13UL
#define SHT3X_PERIODIC_4_HIGH      14UL
#define SHT3X_PERIODIC_10_LOW      15UL
#define SHT3X_PERIODIC_10_MED      16UL
#define SHT3X_PERIODIC_10_HIGH     17UL

// SHT3X status command argument
#define SHT3X_STATUS_READ           0UL
#define SHT3X_STATUS_CLEAR          1UL

// SHT3X CRC check argument
#define SHT3X_CRC_CHECK_DISABLE     0UL
#define SHT3X_CRC_CHECK_ENABLE      1UL


#endif
