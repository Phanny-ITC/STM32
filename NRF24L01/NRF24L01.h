/*
 * NRF24L01.h
 *
 *  Created on: Mar 24, 2021
 *  Author: TPN
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "stm32f1xx_hal.h"

void NRF24_Init (void);

void NRF24_TxMode (uint8_t *Address, uint8_t channel, uint8_t payload_size);

void NRF24_RxMode (uint8_t *Address, uint8_t channel, uint8_t payload_size);

uint8_t NRF24_Transmit (uint8_t *data, uint16_t size);

void NRF24_Receive (uint8_t *data, uint16_t size);

uint8_t isDataAvailable (uint8_t Pipenum);

void nrf24_reset(uint8_t REG);
uint8_t RxFifoEmpty(void);
uint8_t DataReady(uint8_t Pipenum);
void SetRF(uint8_t DataRate, uint8_t OutPwr);

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0a
#define RX_ADDR_P1  0x0b
#define RX_ADDR_P2  0x0c
#define RX_ADDR_P3  0x0d
#define RX_ADDR_P4  0x0e
#define RX_ADDR_P5  0x0f
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1c
#define FEATURE	    0x1d

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1f
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xa0
#define W_ACK_PAYLOAD 0xa8
#define FLUSH_TX      0xe1
#define FLUSH_RX      0xe2
#define REUSE_TX_PL   0xe3
#define NOP           0xff






#endif /* INC_NRF24L01_H_ */
