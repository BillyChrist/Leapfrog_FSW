/*
 * alt_imu_coupling.h
 *
 *  Created on: Mar 7, 2025
 *      Author: Viserion
 */

#ifndef INC_ALT_IMU_COUPLING_H_
#define INC_ALT_IMU_COUPLING_H_

#include "altimeter.h" // For AltimeterData*

// Extern declaration for the adjusted altitude
extern float adjusted_altitude_cm;

float altimeterGetAdjustedData(AltimeterData*, float, float, float);
float getTrueHeight(float, float, float, float);

void rotate3DVector(float[3], float[4], float[3]);
void multQuats(float[4], float[4], float[4]);

float dotProd3D(float[3], float[3]);

void crossProd3D(float[3], float[3], float[3]);
void multByScalar3D(float[3], float, float[3]);

#endif /* INC_ALT_IMU_COUPLING_H_ */
