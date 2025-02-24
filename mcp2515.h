#ifndef APPLICATION_USER_INC_MCP2515_H_
#define APPLICATION_USER_INC_MCP2515_H_

#include <stdint.h>
#include <stm32f0xx_hal.h>

typedef enum {
    ERROR_OK = 0,
    ERROR_FAIL = 1,
    ERROR_ALLTXBUSY = 2,
    ERROR_FAILINIT = 3,
    ERROR_FAILTX = 4,
    ERROR_NOMSG = 5
} CAN_Error;

typedef enum {
    CLKOUT_DISABLE = -1,
    CLKOUT_DIV1 = 0x0,
    CLKOUT_DIV2 = 0x1,
    CLKOUT_DIV4 = 0x2,
    CLKOUT_DIV8 = 0x3,
} CAN_CLKOUT;

typedef enum { MCP_20MHZ, MCP_16MHZ, MCP_8MHZ } CAN_CLOCK;

typedef enum {
    CAN_5KBPS,
    CAN_10KBPS,
    CAN_20KBPS,
    CAN_31K25BPS,
    CAN_33KBPS,
    CAN_40KBPS,
    CAN_50KBPS,
    CAN_80KBPS,
    CAN_83K3BPS,
    CAN_95KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_200KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
} CAN_SPEED;

typedef enum { MASK0, MASK1 } MASK;

typedef enum { RXF0 = 0, RXF1 = 1, RXF2 = 2, RXF3 = 3, RXF4 = 4, RXF5 = 5 } RXF;

typedef enum { TXB0 = 0, TXB1 = 1, TXB2 = 2 } TXBn;

typedef enum { RXB0 = 0, RXB1 = 1 } RXBn;

typedef enum {
    MODE_LISTEN_ONLY,
    MODE_SLEEP,
    MODE_LOOPBACK,
    MODE_NORMAL
} MCP2515_Mode_t;

typedef struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
} MCP2515_HandleTypeDef;

typedef struct can_frame_s {
    uint32_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
    uint8_t  can_dlc;
    uint8_t  data[8];
} can_frame_t;

void MCP2515_Init(MCP2515_HandleTypeDef* hcan, SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin);
CAN_Error MCP2515_Reset(MCP2515_HandleTypeDef* hcan);
uint8_t MCP2515_Get_Status(MCP2515_HandleTypeDef* hcan);

CAN_Error MCP2515_Set_Mode(MCP2515_HandleTypeDef* hcan, MCP2515_Mode_t mode);

CAN_Error MCP2515_Set_Bitrate_Clock(MCP2515_HandleTypeDef* hcan, CAN_SPEED canSpeed, CAN_CLOCK canClock);

CAN_Error MCP2515_Set_Filter_Mask(MCP2515_HandleTypeDef* hcan, MASK mask, uint8_t ext, uint32_t ulData);
CAN_Error MCP2515_Set_Filter(MCP2515_HandleTypeDef* hcan, RXF num, uint8_t ext, uint32_t ulData);

CAN_Error MCP2515_Send_Message_To(MCP2515_HandleTypeDef* hcan, TXBn txbn, can_frame_t* frame);
CAN_Error MCP2515_Send_Message(MCP2515_HandleTypeDef* hcan, can_frame_t* frame);

CAN_Error MCP2515_Read_Message_From(MCP2515_HandleTypeDef* hcan, RXBn rxbn, can_frame_t* frame);
CAN_Error MCP2515_Read_Message(MCP2515_HandleTypeDef* hcan, can_frame_t* frame);

uint8_t MCP2515_Check_Receive(MCP2515_HandleTypeDef* hcan);
uint8_t MCP2515_Get_Error_Flags(MCP2515_HandleTypeDef* hcan);
uint8_t MCP2515_Check_Error(MCP2515_HandleTypeDef* hcan);

void MCP2515_Clear_RXn_OVR_Flags(MCP2515_HandleTypeDef* hcan);
uint8_t MCP2515_Get_Interrupts(MCP2515_HandleTypeDef* hcan);
void MCP2515_Clear_Interrupts(MCP2515_HandleTypeDef* hcan);
uint8_t MCP2515_Get_Interrupt_Mask(MCP2515_HandleTypeDef* hcan);
void MCP2515_Clear_TX_Interrupts(MCP2515_HandleTypeDef* hcan);
void MCP2515_Clear_RXn_OVR(MCP2515_HandleTypeDef* hcan);
void MCP2515_Clear_MERR(MCP2515_HandleTypeDef* hcan);
void MCP2515_Clear_ERRIF(MCP2515_HandleTypeDef* hcan);

#endif /* APPLICATION_USER_INC_MCP2515_H_ */
