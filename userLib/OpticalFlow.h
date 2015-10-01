/*
 * OpticalFlow.h
 *
 *  Created on: 2014Äê8ÔÂ14ÈÕ
 *      Author: Ljm
 */

#ifndef OPTICALFLOW_H_
#define OPTICALFLOW_H_

#include "board.h"

typedef struct
{
	int motion;
	int dx, dy;
	float x, y;
	int squal;
	float change_x, change_y;
	float x_cm, y_cm;
	float compensatedx, compensatedy;
}OpticalFlow_Data;

extern OpticalFlow_Data OpticalFlow;

// field of view of ADNS3080 sensor lenses
#define AP_OPTICALFLOW_ADNS3080_08_FOV 0.349065f        // 11.6 degrees

// scaler - value returned when sensor is moved equivalent of 1 pixel
#define AP_OPTICALFLOW_ADNS3080_SCALER_400   1.1f       // when resolution set to 400
#define AP_OPTICALFLOW_ADNS3080_SCALER_1600  4.4f       // when resolution set to 1600

#define ADNS3080_PIXELS_X               30
#define ADNS3080_PIXELS_Y               30

#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

void OpticalFlow_GetMotion();

void OpticalFlow_Init();

float OpticalFlow_Distance(float x, float y);

#endif /* OPTICALFLOW_H_ */
