/*
 * NRF24L01.c
 *
 *  Created on: Mar 24, 2021
 *      Author: meh
 */



#include "NRF24L01.h"

extern SPI_HandleTypeDef hspi1;
#define NRF24_SPI &hspi1

#define NRF24_CSN_PORT 	GPIOB
#define NRF24_CSN_PIN 	GPIO_PIN_10
#define NRF24_CE_PORT 	GPIOB
#define NRF24_CE_PIN 	GPIO_PIN_11

/* Pins configuration */
#define NRF24_CE_LOW			HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET)
#define NRF24_CE_HIGH			HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET)
#define NRF24_CSN_LOW			HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET)
#define NRF24_CSN_HIGH			HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET)

/* Write the data to the register */
static void nrf24_WriteReg (uint8_t Reg, uint8_t data)
{
	uint8_t buf[2];
	buf[0] = Reg|(1<<5);
	buf[1] = data;
	NRF24_CSN_LOW;

	HAL_SPI_Transmit(NRF24_SPI, buf, 2, 100);

	NRF24_CSN_HIGH;
}

static void nrf24_WriteReg_Multi (uint8_t Reg, uint8_t *data, uint16_t size)
{
	uint8_t buf[2];
	buf[0] = Reg|(1<<5);
//	buf[1] = data;
	NRF24_CSN_LOW;

	HAL_SPI_Transmit(NRF24_SPI, buf, 1, 10);
	HAL_SPI_Transmit(NRF24_SPI, data, size, 100);

	NRF24_CSN_HIGH;
}


/* Read single byte from the register */
static uint8_t nrf24_ReadReg(uint8_t Reg)
{
	uint8_t data;

	NRF24_CSN_LOW;

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 10);
	HAL_SPI_Receive(NRF24_SPI, &data, 1, 10);

	NRF24_CSN_HIGH;

	return data;
}


/* Read multiple bytes from the register */
static void nrf24_ReadReg_Multi (uint8_t Reg, uint8_t *data, uint16_t size)
{
	NRF24_CSN_LOW;

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 10);
	HAL_SPI_Receive(NRF24_SPI, data, size, 100);

	NRF24_CSN_HIGH;
}


static void nrfsendCmd (uint8_t cmd)
{

	NRF24_CSN_LOW;
	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 10);
	NRF24_CSN_HIGH;
}

void nrf24_reset(uint8_t REG)
{
	if (REG == STATUS)
	{
		nrf24_WriteReg(STATUS, 0x0e);
	}

	else if (REG == FIFO_STATUS)
	{
		nrf24_WriteReg(FIFO_STATUS, 0x11);
	}

	else {
	nrf24_WriteReg(CONFIG, 0x08);
	nrf24_WriteReg(EN_AA, 0x3f);
	nrf24_WriteReg(EN_RXADDR, 0x03);
	nrf24_WriteReg(SETUP_AW, 0x03);
	nrf24_WriteReg(SETUP_RETR, 0x03);
	nrf24_WriteReg(RF_CH, 0x02);
	nrf24_WriteReg(RF_SETUP, 0x0e);
	nrf24_WriteReg(STATUS, 0x0e);
	nrf24_WriteReg(OBSERVE_TX, 0x00);
	nrf24_WriteReg(CD, 0x00);
	uint8_t rx_addr_p0_def[5] = {0xe7, 0xe7, 0xe7, 0xe7, 0xe7};
	nrf24_WriteReg_Multi(RX_ADDR_P0, rx_addr_p0_def, 5);
	uint8_t rx_addr_p1_def[5] = {0xc2, 0xc2, 0xc2, 0xc2, 0xc2};
	nrf24_WriteReg_Multi(RX_ADDR_P1, rx_addr_p1_def, 5);
	nrf24_WriteReg(RX_ADDR_P2, 0xc3);
	nrf24_WriteReg(RX_ADDR_P3, 0xc4);
	nrf24_WriteReg(RX_ADDR_P4, 0xc5);
	nrf24_WriteReg(RX_ADDR_P5, 0xc6);
	uint8_t tx_addr_def[5] = {0xc7, 0xc7, 0xc7, 0xc7, 0xc7};
	nrf24_WriteReg_Multi(TX_ADDR, tx_addr_def, 5);
	nrf24_WriteReg(RX_PW_P0, 0);
	nrf24_WriteReg(RX_PW_P1, 0);
	nrf24_WriteReg(RX_PW_P2, 0);
	nrf24_WriteReg(RX_PW_P3, 0);
	nrf24_WriteReg(RX_PW_P4, 0);
	nrf24_WriteReg(RX_PW_P5, 0);
	nrf24_WriteReg(FIFO_STATUS, 0x11);
	nrf24_WriteReg(DYNPD, 0);
	nrf24_WriteReg(FEATURE, 0);
	}
}



void NRF24_Init (void)
{
	// idle state for the pins
	NRF24_CSN_HIGH;
	NRF24_CE_LOW;
	HAL_Delay (1);

	// reset the nrf
	nrf24_reset(0);

	// config register
	nrf24_WriteReg(CONFIG, 0x00);

	// Disable Auto ACK for all data pipes
	nrf24_WriteReg(EN_AA, 0x00);

	// set up address width
	//nrf24_WriteReg(SETUP_AW, 0x01);

	//set the  Automatic Retransmission
	nrf24_WriteReg(SETUP_RETR, 0xff);

	// set RF Register
	//nrf24_WriteReg(RF_SETUP , 0x06);
	SetRF(250, 3);

	// disable dynamic payload
	nrf24_WriteReg(FEATURE, 0);
}
void SetRF(uint8_t DataRate, uint8_t OutPwr) {
	uint8_t tmp = 0;

	if (DataRate == 2) {//NRF24L01_DataRate_2M
		tmp |= 1 << 3;
	} else if (DataRate == 250) {//NRF24L01_DataRate_250k
		tmp |= 1 << 5;
	}
	/* If 1Mbps, all bits set to 0 */

	if (OutPwr == 3) {//NRF24L01_OutputPower_0dBm
		tmp |= 3 << 1;
	} else if (OutPwr == 2) {//NRF24L01_OutputPower_-6dBm
		tmp |= 2 << 1;
	} else if (OutPwr == 1) {//NRF24L01_OutputPower_-12dBm
		tmp |= 1 << 1;
	}else if(OutPwr == 0){////NRF24L01_OutputPower_-18dBm
		tmp |= 0 << 1;
	}

	nrf24_WriteReg(RF_SETUP, tmp);
}

void NRF24_TxMode (uint8_t *Address, uint8_t channel, uint8_t payload_size)
{
	uint8_t cmdtosend;

	// Pull the CE LOW
	NRF24_CE_LOW;

	// set the tx address and also set the same address for RX_p0
	nrf24_WriteReg_Multi(TX_ADDR, Address, 5);
	nrf24_WriteReg_Multi(RX_ADDR_P0, Address, 5);
	nrf24_WriteReg(RX_PW_P0, payload_size);
	// flush the TX FIFO
	cmdtosend = FLUSH_TX;
	nrfsendCmd(cmdtosend);

	// set the channel
	nrf24_WriteReg(RF_CH, channel);

	// Power on in the TX MODE

	uint8_t config = nrf24_ReadReg(CONFIG);
	config|= (1<<1);   // enable the PWR_UP bit
	nrf24_WriteReg(CONFIG, config);  // power on in Tx mode

	// PULL the CE Pin High
	NRF24_CE_HIGH;
}


uint8_t NRF24_Transmit (uint8_t *data, uint16_t size)
{
	uint8_t cmdtosend;

	// pull the CS low
	NRF24_CSN_LOW;

	// send the payload write command
	cmdtosend = W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 10);
	
	// send the payload
	HAL_SPI_Transmit(NRF24_SPI, data, size, 100);

	// pull the CS high
	NRF24_CSN_HIGH;

	// some delay to settle the pin
	HAL_Delay (1);

	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

	// check if the transmit was success
	if (fifostatus&(1<<4))  // TX_FIFO Empty flag
	{
		cmdtosend = FLUSH_TX;
		nrfsendCmd(cmdtosend);

		return 1;
	}

	return 0;
}


void NRF24_RxMode (uint8_t *Address, uint8_t channel, uint8_t payload_size)
{
	uint8_t cmdtosend;

	// Pull the CE LOW
	NRF24_CE_LOW;

	// Flush the RX FIFO
	cmdtosend = FLUSH_RX;
	nrfsendCmd(cmdtosend);

	// Reset the status Register
	nrf24_reset(STATUS);

	// set the payload size for pipe 1
	nrf24_WriteReg(RX_PW_P1, payload_size);

	//set address for rx pipe
	nrf24_WriteReg_Multi(RX_ADDR_P1, Address, 5);

	nrf24_WriteReg(RF_CH, channel);

	// enable Pipe 1 for RX
	//nrf24_WriteReg(EN_RXADDR, nrf24_ReadReg(EN_RXADDR)|(1<<1));
	// enable Pipe 0 for RX
	//nrf24_WriteReg(EN_RXADDR, 0x01);

	uint8_t config = nrf24_ReadReg(CONFIG);
	config|= (1<<1) | (1<<0);   // enable the PWR_UP bit and PRIM_RX bit
	nrf24_WriteReg(CONFIG, config);  // power on in rx mode

	NRF24_CE_HIGH;
}


uint8_t isDataAvailable (uint8_t Pipenum)
{
	// read status register
	uint8_t status=nrf24_ReadReg(STATUS);
	//uint8_t status=nrf24_ReadReg(NOP);

	// check for RX_DR bit and the pipe number
	if ((status&(1<<6)) && (status&(Pipenum<<1)))
	{
		// clear the RX_DR bit
		//nrf24_WriteReg(STATUS, status|(1<<6));
		return 1;
	}

	else return 0;
}
uint8_t DataReady(uint8_t Pipenum) {
	uint8_t status=nrf24_ReadReg(STATUS);
	if ((status&(1<<6)) && (status&(Pipenum<<1))){
		// clear the RX_DR bit
		//nrf24_WriteReg(STATUS, status|(1<<6));
		return 1;
	}
	return !RxFifoEmpty();
}

uint8_t RxFifoEmpty(void) {
	uint8_t data = nrf24_ReadReg(FIFO_STATUS);
	return data;
}



void NRF24_Receive (uint8_t *data, uint16_t size)
{
	uint8_t cmdtosend;

	// Pull the CSN pin LOW
	NRF24_CSN_LOW;

	// Send the read command
	cmdtosend = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 10);

	// read the data
	HAL_SPI_Receive(NRF24_SPI, data, size, 100);

	// pull the CS pin HIGH
	NRF24_CSN_HIGH;
	// clear the RX_DR bit
	nrf24_WriteReg(STATUS, (1<<6));
	// Flush the RX FIFO
	cmdtosend = FLUSH_RX;
	nrfsendCmd(cmdtosend);
}

