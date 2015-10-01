#ifndef __MPU9250_DMP_H
#define __MPU9250_DMP_H

#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"

/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
//#define PRINT_COMPASS   (0x08)
//#define PRINT_EULER     (0x10)
//#define PRINT_ROT_MAT   (0x20)
//#define PRINT_HEADING   (0x40)
//#define PRINT_PEDO      (0x80)
//#define PRINT_LINEAR_ACCEL (0x100)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
//#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

//#define PEDO_READ_MS    (1000)
//#define TEMP_READ_MS    (500)
//#define COMPASS_READ_MS (100)
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};

enum packet_type_e {
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_TAP,
    PACKET_TYPE_ANDROID_ORIENT,
    PACKET_TYPE_PEDO,
    PACKET_TYPE_MISC
};

/******************************************************************************
*                                 外部函数申明
******************************************************************************/
void MPU9250_GetDMPData();

void run_self_test(void);

void MPU9250_Dmp_Init(void);

#endif
