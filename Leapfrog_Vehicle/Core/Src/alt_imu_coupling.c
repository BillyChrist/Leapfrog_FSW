/* ******************************************************************************
 * @file   : alt_imu_coupling.c
 *
 * @brief  : This module provides sensor fusion between LIDAR and IMU sensors to compensate for the vehicle's tilt angle
 *           when calculating altitude.
 * 
 *  Created on: Mar 7, 2025
 *      Author: Brad Barakat and Matthew Basha on 6/30/22.
 *      Updated by Billy Christ on 3/07/2025
 *  ******************************************************************************
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "queue.h"
#include "alt_imu_coupling.h"
#include "altimeter.h"


float pi = 3.14159265358979323846; // VSCode showed a warning about M_PI not being defined

// Global variable to store the adjusted altitude
float adjusted_altitude_cm = 0.0f;

/**
 * @brief  Gets the true height of the vehicle's center by
 * using the altimeter's data package and IMU data
 * @param  altMsg: the data package returned from the altimeter
 * @param  roll: the roll angle from the IMU
 * @param  pitch: the pitch angle from the IMU
 * @param  yaw: the yaw angle from the IMU
 * @retval  float: the true height of the vehicle's center
 */
float altimeterGetAdjustedData(AltimeterData* altMsg, float roll, float pitch, float yaw) {
    // Retrieve the latest data from the altimeter
    altimeterGetLatestData(altMsg);
    float rawAlt = (float)(altMsg->altitude);

    // Calculate adjusted altitude and store it in the global variable
    adjusted_altitude_cm = getTrueHeight(rawAlt, roll, pitch, yaw);

    // Return the adjusted altitude
    return adjusted_altitude_cm;
}



/**
 * @brief  Gets the true height of the vehicle's center by
 * using the altimeter's distance measurement and IMU data
 * @param  laser: the distance returned from the altimeter
 * @param  roll: the roll angle from the IMU
 * @param  pitch: the pitch angle from the IMU
 * @param  yaw: the yaw angle from the IMU
 * @retval  float: the true height of the vehicle's center
 */
float getTrueHeight(float laser, float roll, float pitch, float yaw) {

    float z0[3] = {0, 0, -1}; // Vector pointing down (direction altimeter is facing)
    float r0[3] = {0, 0.44, 0}; // Vector from center to altimeter, in meters

    // Convert rpy to radians
    float r = (pi/180)*roll;
    float p = (pi/180)*pitch;
    float y = (pi/180)*yaw;

    // Save trig values in variables since trig operations are expensive in embedded systems
    float sr = sin(r/2);
    float cr = cos(r/2);
    float sp = sin(p/2);
    float cp = cos(p/2);
    float sy = sin(y/2);
    float cy = cos(y/2);

    // Convert rpy to quaternion
    float q[4];
    q[0] = cr*cp*cy + sr*sp*sy;
    q[1] = sr*cp*cy - cr*sp*sy;
    q[2] = cr*sp*cy + sr*cp*sy;
    q[3] = cr*cp*sy - sr*sp*cy;

    // TODO: New - TEST
    // Normalize to unit quaternion before rotation
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (int i = 0; i < 4; ++i) {
        q[i] /= norm;
    }

    // Initialize resultant vectors
    float z_[3] = {0,0,0};
    float r_[3] = {0,0,0};
    // Find rotated vectors
    rotate3DVector(z0, q, z_);
    rotate3DVector(r0, q, r_);

    // Turn z_ into the vector from the altimeter to the ground
    multByScalar3D(z_, laser, z_);

    // Find the vector from the center to the point on the ground that the altimeter sees
    float sum[3] = {r_[0] + z_[0], r_[1] + z_[1], r_[2] + z_[2]};
    // Find the projection of that vector onto the z-azis (remember, z0 is a unit vector)
    return dotProd3D(z0, sum);
}

/**
 * @brief  Rotates a 3D vector using a quaternion
 * @param  v: the 3D vector
 * @param  q: the second quaternion
 * @param  res: the resultant vector
 * @retval  None
 */
void rotate3DVector(float v[3], float q[4], float res[3]) {
    // Find conjugate of q
    float q_[4] = {q[0], -1*q[1], -1*q[2], -1*q[3]};

    // Turn the vector into a quaternion
    float p[4] = {0, v[0], v[1], v[2]};

    // Transformation: p' = q*p*q_
    float res0[4] = {0,0,0,0};
    multQuats(q, p, res0);
    multQuats(res0, q_, res0);

    // Turn quaternion result into vector (the real part is 0)
    res[0] = res0[1];
    res[1] = res0[2];
    res[2] = res0[3];
}

/**
 * @brief  Multiplies two quaternions
 * @param  q1: the first quaternion
 * @param  q2: the second quaternion
 * @param  res: the resultant quaternion
 * @retval  None
 */

void multQuats(float q1[4], float q2[4], float res[4]) {
    // (r1,v1)*(r2,v2) = (r1*r2 - (v1•v2), r1*v2 + r2*v1 + (v1xv2))
    float r1 = q1[0];
    float v1[3] = {q1[1], q1[2], q1[3]};
    float r2 = q2[0];
    float v2[3] = {q2[1], q2[2], q2[3]};

    float pxq[3] = {0,0,0};
    crossProd3D(v1, v2, pxq);
    float p0q[3] = {0,0,0};
    multByScalar3D(v2, r1, p0q);
    float q0p[3] = {0,0,0};
    multByScalar3D(v1, r2, q0p);
    res[0] = r1*r2 - dotProd3D(v1, v2);
    res[1] = p0q[0] + q0p[0] + pxq[0];
    res[2] = p0q[1] + q0p[1] + pxq[1];
    res[3] = p0q[2] + q0p[2] + pxq[2];
}

/**
 * @brief  Performs a dot product between two 3D vectors
 * @param  v1: the first 3D vector
 * @param  v2: the second 3D vector
 * @retval  float: the result
 */
float dotProd3D(float v1[3], float v2[3]) {
    return (v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]);
}

/**
 * @brief  Performs a cross product between two 3D vectors
 * @param  v1: the first 3D vector
 * @param  v2: the second 3D vector
 * @param  res: the resultant 3D vector
 * @retval  None
 */
void crossProd3D(float v1[3], float v2[3], float res[3]) {
    // axb = <a2b3 - a3b2, a3b1 - a1b3, a1b2 - a2b1>
    res[0] = v1[1]*v2[2] - v1[2]*v2[1];
    res[1] = v1[2]*v2[0] - v1[0]*v2[2];
    res[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

/**
 * @brief  Multiplies a 3D vector by a scalar
 * @param  v: the 3D vector
 * @param  s: the scalar
 * @param  res: the resultant 3D vector
 * @retval  None
 */
void multByScalar3D(float v[3], float s, float res[3]) {
    res[0] = v[0]*s;
    res[1] = v[1]*s;
    res[2] = v[2]*s;
}
