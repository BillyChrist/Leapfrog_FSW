/* ******************************************************************************
 * @file engine.c
 *
 * @brief This module handles engine operation and telemetry. All throttle control called from this code, including PID variables for tuning.
 * 
 *  Created on: Nov 19, 2024
 *      Author: Julia Schatz
 *      Updated by: Billy Christ on 3/07/2025
 *  ******************************************************************************
 */

// TODO Add calculation to compensate for vehicle mass... tune only with PID control, or add variable?

#include <engine_PID.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stdbool.h"
#include "queue.h"
#include <stdio.h>
#include "printf.h"
#include "main.h"
#include "heartbeat.h"
#include "engine.h"
#include "engine_PID.h"
#include "altimeter.h"
#include "ACS_PID.h"
#include "processIMU.h"
#include "alt_imu_coupling.h"


// Engine PID Values
float engine_KP = 75.00;
float engine_KI = 00.10;
float engine_KD = 00.10;

extern UART_HandleTypeDef huart1, huart2;
extern SubsystemState engineState;
extern float engine_thrust_p;
extern float engine_hover_m;
extern uint16_t engine_pwm_scale;
extern TIM_HandleTypeDef htim2;
static AltimeterData altimeterData;

// These extern variables are from ACS_PID.c
extern float current_roll_deg;
extern float current_pitch_deg;
extern float current_yaw_deg;

// Define Engine Safe and Activate Pins
#define Engine_Safe_Pin      GPIO_PIN_8  // Engine Safe connected to Pin 8
#define Engine_Activate_Pin  GPIO_PIN_9  // Example: Engine Activate on Pin 9
#define Engine_GPIO_Port     GPIOC       // Port C used (Adjust as needed)

static TickType_t engineLastWakeTime;

// Define global state variable for "lastState"
SubsystemState lastState = SystemDisabled;  // Initialize to a default state

// Define local variables
osMessageQueueId_t engineRxQueueHandle = NULL;
uint8_t engineSafe = 0;
uint8_t enginePwr = 0;
uint8_t rxBuf[ENGINE_RX_BUF_SIZE];
uint8_t engineRxBuffPtr = 0;

PID_TypeDef altPID;


void readCt(uint8_t* src, uint8_t* offset, uint8_t* dst, uint8_t count) {
  for (uint8_t i = 0; i < count; ++i) {
    dst[i] = *(src + *offset + i);
  }
  *offset += count;
}


uint8_t readStrToComma(uint8_t* src, uint8_t* offset, uint8_t* dst, uint8_t bufsize) {
  uint8_t i = 0;
  while (*(src + *offset + i) != ',' && *(src + *offset + i) != '\r') {
    dst[i] = *(src + *offset + i);
    ++i;
    if (i >= bufsize) { // Break out early and return an empty string if we go too long
      dst[0] = '\0';
      *offset += 1;
      return 0;
    }
  }
  *offset += i+1;
  dst[i] = '\0';
  return i;
}


// Define global variables used by the engine system
float engine_hover_m = 1.5f;   // Hover altitude in meters
float engine_thrust_p = 0.0f;   // Thrust percentage (0 to 100)

// Global variable definitions
double altPIDOut;          		// PID output (throttle percentage)
double engine_setpoint = 1.00; 	// Target altitude [m]
double altitude;
float throttle;

// Engine state variables
EngineMessageType state = IDLE;
uint8_t turbine_state, off_condition, throttle_percent;
uint16_t last_run_time;

//TODO Make sure these terms point to data collected from engine
int turbineRPM;
int rpm_setpoint;        		// Maintains specific engine speed
int egt_c;

float pump_voltage;
float engine_current_a;
float fuel_flow, fuel_volume, battery_voltage, fuel_consumed;



void initEngineControl(void) {
    engineLastWakeTime = xTaskGetTickCount();

    PID(&altPID, &altitude, &altPIDOut, &engine_setpoint, engine_KP, engine_KI, engine_KD, _PID_P_ON_E, _PID_CD_DIRECT);
    PID_SetMode(&altPID, _PID_MODE_AUTOMATIC);

    // Sample and output limit time must reflect the RTOS task timing!
    PID_SetSampleTime(&altPID, 100);
    PID_SetOutputLimits(&altPID, 0, 100);
}


void runEngineControl_Test(void) {
    // Fetch latest altitude reading and compute throttle via PID
    switch (engineState) {

        case SystemAutomatic:
            engine_setpoint = engine_hover_m;
            altitude = altimeterGetAdjustedData(&altimeterData, current_roll_deg, current_pitch_deg, current_yaw_deg) / 100.0f; // convert cm to meters
            PID_Compute(&altPID);
            throttle = altPIDOut;
            break;

        case SystemManual:
            throttle = engine_thrust_p;
            break;

        case SystemDisabled:
        	//TODO Ensure "disabled mode" triggers auto cooldown
            throttle = 0;
            break;
    }

    // This value is captured by debugOutput()
    throttle_percent = throttle;


}



void runEngineControl(void) {
	bool gotByte = false;

    {
        while (xQueueReceive(engineRxQueueHandle, &rxBuf[engineRxBuffPtr], 0) == pdTRUE) {
          gotByte = true;
          if (rxBuf[engineRxBuffPtr] == '\r') {
            // Process message
            uint8_t tempRxPtr = 0;
            uint8_t tempRxBuf[TEMP_RX_BUFSIZE];
            // First we're going to get the command echoed back to us
            // Read the first two strings to see if this is an echo or a handshake
            readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE); // First one is address, don't care
            readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);

            if (strcmp((char*)tempRxBuf, "HS") == 0) {
              // This is a result from the engine
              // The next two bytes are the return code
              readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);

              if (strcmp((char*)tempRxBuf, "OK") == 0) {
            	  // now we start reading actual values
            	  if (state == RAC) {
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     turbineRPM = atoi((char*)tempRxBuf);
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     egt_c = atoi((char*)tempRxBuf);
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     pump_voltage = atof((char*)tempRxBuf);
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     turbine_state = atoi((char*)tempRxBuf);
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     uint8_t pwm_throttle_percent = atoi((char*)tempRxBuf);
            	     (void)pwm_throttle_percent;
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     engine_current_a = atof((char*)tempRxBuf);
            	  }
            	  else if (state == DHC) {
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     uint8_t dhc_response = atoi((char*)tempRxBuf);
            	     (void)dhc_response;
            	  }
            	  else if (state == RHC) {
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     uint8_t starter_health = atoi((char*)tempRxBuf);
            	     (void)starter_health;
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     uint8_t main_valve_health = atoi((char*)tempRxBuf);
            	     (void)main_valve_health;
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     uint8_t starter_valve_health = atoi((char*)tempRxBuf);
            	     (void)starter_valve_health;
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     uint8_t rpm_sensor_health = atoi((char*)tempRxBuf);
            	     (void)rpm_sensor_health;
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     uint8_t pump_health = atoi((char*)tempRxBuf);
            	     (void)pump_health;
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     uint8_t glowplug_health = atoi((char*)tempRxBuf);
            	     (void)glowplug_health;
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     uint8_t egt_health = atoi((char*)tempRxBuf);
            	     (void)egt_health;
            	  }
            	  else if (state == RFI) {
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     fuel_flow = atof((char*)tempRxBuf);
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     fuel_volume = atof((char*)tempRxBuf);
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     rpm_setpoint = atoi((char*)tempRxBuf);
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     battery_voltage = atof((char*)tempRxBuf);
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     last_run_time = atoi((char*)tempRxBuf);
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     fuel_consumed = atof((char*)tempRxBuf);
            	  }
            	  else if (state == RSS) {
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     uint8_t dummy_parameter = atoi((char*)tempRxBuf);
            	     (void)dummy_parameter;
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     off_condition = atoi((char*)tempRxBuf);
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     uint8_t actual_flight_speed = atoi((char*)tempRxBuf);
            	     (void)actual_flight_speed;
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     uint8_t prop_speed_regulator = atoi((char*)tempRxBuf);
            	     (void)prop_speed_regulator;
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     uint8_t ad_speed = atoi((char*)tempRxBuf);
            	     (void)ad_speed;
            	     readStrToComma(rxBuf, &tempRxPtr, tempRxBuf, TEMP_RX_BUFSIZE);
            	     uint8_t ad_zero_speed = atoi((char*)tempRxBuf);
            	     (void)ad_zero_speed;
                }

                state = IDLE;
              }
            }

            // Reset pointer
            engineRxBuffPtr = 0;
          }
          else {
            engineRxBuffPtr++;
            if (engineRxBuffPtr >= ENGINE_RX_BUF_SIZE) {
              // this should never happen, but it's going to be a weird error if it does
              engineRxBuffPtr = 0;
            }
          }
        }
        static uint8_t rxTimeoutCounter = 0;
        if (!gotByte) {
          if (rxTimeoutCounter++ > 10) {
            state = IDLE;
            engineRxBuffPtr = 0;
            rxTimeoutCounter = 0;
          }
        }
        else {
          rxTimeoutCounter = 0;
        }

        if (engineSafe > 0) {
          HAL_GPIO_WritePin(GPIOC, Engine_Safe_Pin, GPIO_PIN_RESET);
        }
        else {
          HAL_GPIO_WritePin(GPIOC, Engine_Safe_Pin, GPIO_PIN_SET);
        }

        if (enginePwr > 0 && engineSafe > 0) { //do not enable the engine if the safety is not on
          HAL_GPIO_WritePin(GPIOC, Engine_Activate_Pin, GPIO_PIN_SET);
        }
        else {
          HAL_GPIO_WritePin(GPIOC, Engine_Activate_Pin, GPIO_PIN_RESET);
        }

        switch(engineState){

          case SystemAutomatic:

            engine_setpoint = engine_hover_m;
            altitude = altimeterGetAdjustedData(&altimeterData, current_roll_deg, current_pitch_deg, current_yaw_deg)/100; // the "/100" converts to m from cm
            //altimeterGetLatestData(&altimeterData);
            //altitude = (float)altimeterData.distance / 100;
            PID_Compute(&altPID);
            throttle = altPIDOut;

          break;

          case SystemManual:

            throttle = engine_thrust_p;

          break;

          case SystemDisabled:

            throttle = 0;

          break;
        }

        throttle_percent = throttle;

        static bool needToSendTCO = false;
        needToSendTCO = needToSendTCO || (lastState != engineState);
        //setThrottlePWM(0.0);


        // Send messages in a cycle
        static uint8_t msgState = 0;
        if (state == IDLE) {
          if (needToSendTCO) {
            // 0 = disable, 1 = enable with RS232
    //        uint8_t turbine_state = (engineState != DISABLE) ? 1 : 0;
            uint8_t turbine_state = (engineState != SystemDisabled) ? 1 : 0;

            sendTCO(turbine_state);
            needToSendTCO = false;
          }
          else if (msgState == 0) {
            sendRAC();
            ++msgState;
          }
          else if (msgState == 1) {
            sendRFI();
            ++msgState;
          }
          else if (msgState == 2) {
            sendRSS();
            ++msgState;
          }
          else if (msgState == 3) {
            // Only send a throttle command if the engine should be enabled
            // if (engineState != DISABLE) {
            if (engineState != SystemDisabled) {

              sendWPE(throttle);
            }
            msgState = 0;
          }
        }


        lastState = engineState;
    }
}



void sendRAC() {
  ENGINE_TRANSMIT_STR("1,RAC,1\r");
  state = RAC;
}

void sendDHC() {
  ENGINE_TRANSMIT_STR("1,DHC,1\r");
  state = DHC;
}

void sendRHC() {
  ENGINE_TRANSMIT_STR("1,RHC,1\r");
  state = RHC;
}

void sendRFI() {
  ENGINE_TRANSMIT_STR("1,RFI,1\r");
  state = RFI;
}

void sendRSS() {
  ENGINE_TRANSMIT_STR("1,RSS,1\r");
  state = RSS;
}

void sendWPE(float throttle) {
  char buffer[16];
  uint8_t throttle_u = (uint8_t) (throttle);
  sprintf_(buffer, "1,WPE,%u\r", throttle_u);
  ENGINE_TRANSMIT_STR(buffer);
  state = WPE;
}

void sendTCO(uint8_t state) {
  char buffer[16];
  sprintf_(buffer, "1,TCO,%u\r", state);
  ENGINE_TRANSMIT_STR(buffer);
  state = TCO;
}

/**
 * Set throttle using PWM. Input is [0.0, 1.0]
 */

// TODO Reset the timer channel to the corresponding engine PWM.TIM3 is currently used for the EDF...
void setThrottlePWM(float throttle) {
  const uint16_t minThrottle_us = 1000;
  const uint16_t maxThrottle_us = 2000; // TODO values from real engine, or teach these ones
  uint16_t throttle_us = throttle * (maxThrottle_us - minThrottle_us) + minThrottle_us;
  uint16_t compareVal = engine_pwm_scale * throttle_us / 1000.0;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, compareVal);
}
