#ifndef __ENGINE_H
#define __ENGINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "heartbeat.h"
#include "engine_PID.h"


//#define ENGINE_TRANSMIT_STR(str_) HAL_UART_Transmit(&huart1, str_, strlen(str_), 10)
#define ENGINE_TRANSMIT_STR(str_) HAL_UART_Transmit(&huart1, (const uint8_t*)(str_), strlen((const char*)(str_)), 10)

#define ENGINE_RX_BUF_SIZE 255
typedef enum {
  IDLE = 0,
  RAC,
  RSS,
  RFI,
  RTY,
  TCO,
  RHC,
  DHC,
  WPE
} EngineMessageType;

#define TEMP_RX_BUFSIZE 20

void sendRAC();
void sendRFI();
void sendWPE(float throttle);
void sendTCO(uint8_t state);
void sendDHC();
void sendRHC();
void sendRSS();

void setThrottlePWM(float throttle);
void initEngineControl(void);
void runEngineControl(void);
void runEngineControl_Test(void);


// Expose global variables
extern double engine_setpoint; // Target altitude [m]
extern double altPIDOut;       // PID output (throttle percentage)
extern int turbineRPM;         // Current measured turbine RPM

extern osMessageQueueId_t engineRxQueueHandle;
extern uint8_t engineSafe;
extern uint8_t enginePwr;

extern PID_TypeDef altPID;
extern uint8_t throttle_percent;
extern SubsystemState lastState;


extern double altitude;
extern float throttle;
extern float engine_hover_m;
extern float engine_thrust_p;

extern int rpm_setpoint;
extern int egt_c;
extern float pump_voltage;
extern float engine_current_a;
extern uint8_t turbine_state;
extern uint8_t off_condition;



#ifdef __cplusplus
}

#endif
#endif
