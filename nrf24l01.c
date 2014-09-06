#include "nrf24l01.h"

void NRF_CS_ENABLE(NRF24L01* dev) {
	HAL_GPIO_WritePin(dev->NRF_CSN_GPIOx, dev->NRF_CSN_GPIO_PIN,
			GPIO_PIN_RESET);
}

void NRF_CS_DISABLE(NRF24L01* dev) {
	HAL_GPIO_WritePin(dev->NRF_CSN_GPIOx, dev->NRF_CSN_GPIO_PIN, GPIO_PIN_SET);
}

void NRF_CE_ENABLE(NRF24L01* dev) {
	HAL_GPIO_WritePin(dev->NRF_CE_GPIOx, dev->NRF_CE_GPIO_PIN, GPIO_PIN_SET);
}

void NRF_CE_DISABLE(NRF24L01* dev) {
	HAL_GPIO_WritePin(dev->NRF_CE_GPIOx, dev->NRF_CE_GPIO_PIN, GPIO_PIN_RESET);
}

NRF_RESULT NRF_SetupGPIO(NRF24L01* dev) {

	GPIO_InitTypeDef GPIO_InitStructure;

	// CE pin
	GPIO_InitStructure.Pin = dev->NRF_CE_GPIO_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
	GPIO_InitStructure.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(dev->NRF_CE_GPIOx, &GPIO_InitStructure);
	// end CE pin

	// IRQ pin
	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Pin = dev->NRF_IRQ_GPIO_PIN;
	HAL_GPIO_Init(dev->NRF_IRQ_GPIOx, &GPIO_InitStructure);

	/* Enable and set EXTI Line Interrupt to the given priority */
	HAL_NVIC_SetPriority(dev->NRF_IRQn, dev->NRF_IRQ_preempt_priority,
			dev->NRF_IRQ_sub_priority);
	HAL_NVIC_EnableIRQ(dev->NRF_IRQn);
	// end IRQ pin

	NRF_CS_DISABLE(dev);
	NRF_CE_DISABLE(dev);

	return NRF_OK;
}

NRF_RESULT NRF_Init(NRF24L01* dev) {

	NRF_SetupGPIO(dev);

	NRF_PowerUp(dev, 1);

	uint8_t config=0;

	while((config&2)==0){	// wait for powerup
		NRF_ReadRegister(dev,NRF_CONFIG,&config);
	}

	NRF_SetRXPayloadWidth_P0(dev, dev->PayloadLength);
	NRF_SetRXAddress_P0(dev, dev->RX_ADDRESS);
	NRF_SetTXAddress(dev, dev->TX_ADDRESS);
	NRF_EnableRXDataReadyIRQ(dev, 1);
	NRF_EnableTXDataSentIRQ(dev,1);
	NRF_EnableMaxRetransmitIRQ(dev,1);
	NRF_EnableCRC(dev, 1);
	NRF_SetCRCWidth(dev, dev->CRC_WIDTH);
	NRF_SetAddressWidth(dev, dev->ADDR_WIDTH);
	NRF_SetRFChannel(dev, dev->RF_CHANNEL);
	NRF_SetDataRate(dev, dev->DATA_RATE);
	NRF_SetRetransmittionCount(dev, dev->RetransmitCount);
	NRF_SetRetransmittionDelay(dev, dev->RetransmitDelay);

	NRF_EnableRXPipe(dev, 0);
	NRF_EnableAutoAcknowledgement(dev, 0);

	NRF_ClearInterrupts(dev);

	NRF_RXTXControl(dev, NRF_STATE_RX);

	return NRF_OK;
}

NRF_RESULT NRF_SendCommand(NRF24L01* dev, uint8_t cmd, uint8_t* tx, uint8_t* rx,
		uint8_t len) {
	uint8_t myTX[len + 1];
	uint8_t myRX[len + 1];
	myTX[0] = cmd;

	int i = 0;
	for (i = 0; i < len; i++) {
		myTX[1 + i] = tx[i];
		myRX[i] = 0;
	}

	NRF_CS_ENABLE(dev);
	if (HAL_SPI_TransmitReceive(dev->spi, myTX, myRX, 1 + len, NRF_SPI_TIMEOUT)
			!= HAL_OK) {
		return NRF_ERROR;
	}

	for (i = 0; i < len; i++) {
		rx[i] = myRX[1 + i];
	}

	NRF_CS_DISABLE(dev);

	return NRF_OK;
}

void NRF_IRQ_Handler(NRF24L01* dev) {
	uint8_t status = 0;
	if (NRF_ReadRegister(dev, NRF_STATUS, &status) != NRF_OK) {
		return;
	}

	if ((status & (1 << 6))) {	// RX FIFO Interrupt
		uint8_t fifo_status = 0;

		if (dev->RX_FLAG == 0) {
			NRF_ReadRXPayload(dev, dev->RX_BUFFER);
			status |= 1 << 6;
			NRF_WriteRegister(dev, NRF_STATUS, &status);

			NRF_ReadRegister(dev, NRF_FIFO_STATUS, &fifo_status);

			while ((fifo_status & 1) == 0) {	//data in RX FIFO
				NRF_ReadRXPayload(dev, dev->RX_BUFFER);
				NRF_ReadRegister(dev, NRF_FIFO_STATUS, &fifo_status);
			}
			dev->RX_FLAG=1;
		}

	}
	if ((status & (1 << 5))) {	// TX Data Sent Interrupt
		status |= 1 << 5;	// clear the interrupt flag
		NRF_CE_DISABLE(dev);
		NRF_RXTXControl(dev, NRF_STATE_RX);
		dev->STATE = NRF_STATE_RX;
		NRF_CE_ENABLE(dev);
	}
	if ((status & (1 << 4))) {	// MaxRetransmits reached
		status |= 1 << 4;
		NRF_CE_DISABLE(dev);
		NRF_RXTXControl(dev, NRF_STATE_RX);
		dev->STATE = NRF_STATE_RX;
		NRF_CE_ENABLE(dev);
	}

	NRF_WriteRegister(dev, NRF_STATUS, &status);

}

NRF_RESULT NRF_ReadRegister(NRF24L01* dev, uint8_t reg, uint8_t* data) {
	uint8_t tx = 0;
	if (NRF_SendCommand(dev, NRF_CMD_R_REGISTER | reg, &tx, data, 1)
			!= NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_WriteRegister(NRF24L01* dev, uint8_t reg, uint8_t* data) {
	uint8_t rx = 0;
	if (NRF_SendCommand(dev, NRF_CMD_W_REGISTER | reg, data, &rx, 1)
			!= NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_ReadRXPayload(NRF24L01* dev, uint8_t* data) {
	uint8_t tx[dev->PayloadLength];
	if (NRF_SendCommand(dev, NRF_CMD_R_RX_PAYLOAD, tx, data, dev->PayloadLength)
			!= NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_WriteTXPayload(NRF24L01* dev, uint8_t* data) {
	uint8_t rx[dev->PayloadLength];
	if (NRF_SendCommand(dev, NRF_CMD_W_TX_PAYLOAD, data, rx, dev->PayloadLength)
			!= NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_FlushTX(NRF24L01* dev) {
	uint8_t rx = 0;
	uint8_t tx = 0;
	if (NRF_SendCommand(dev, NRF_CMD_FLUSH_TX, &tx, &rx, 0) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_FlushRX(NRF24L01* dev) {
	uint8_t rx = 0;
	uint8_t tx = 0;
	if (NRF_SendCommand(dev, NRF_CMD_FLUSH_RX, &tx, &rx, 0) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_SetDataRate(NRF24L01* dev, NRF_DATA_RATE rate) {
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_RF_SETUP, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	if (rate & 1) {	// low bit set
		reg |= 1 << 5;
	} else {	// low bit clear
		reg &= ~(1 << 5);
	}

	if (rate & 2) {	// high bit set
		reg |= 1 << 3;
	} else {	// high bit clear
		reg &= ~(1 << 3);
	}
	if (NRF_WriteRegister(dev, NRF_RF_SETUP, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_SetTXPower(NRF24L01* dev, NRF_TX_PWR pwr) {
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_RF_SETUP, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	reg &= 0xF9;	// clear bits 1,2
	reg |= pwr << 1;	// set bits 1,2
	if (NRF_WriteRegister(dev, NRF_RF_SETUP, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_SetCCW(NRF24L01* dev, uint8_t activate) {
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_RF_SETUP, &reg) != NRF_OK) {
		return NRF_ERROR;
	}

	if (activate) {
		reg |= 0x80;
	} else {
		reg &= 0x7F;
	}

	if (NRF_WriteRegister(dev, NRF_RF_SETUP, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_ClearInterrupts(NRF24L01* dev) {
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_STATUS, &reg) != NRF_OK) {
		return NRF_ERROR;
	}

	reg |= 7 << 4;	// setting bits 4,5,6

	if (NRF_WriteRegister(dev, NRF_STATUS, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_SetRFChannel(NRF24L01* dev, uint8_t ch) {
	ch &= 0x7F;
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_RF_CH, &reg) != NRF_OK) {
		return NRF_ERROR;
	}

	reg |= ch;	// setting channel

	if (NRF_WriteRegister(dev, NRF_RF_CH, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_SetRetransmittionCount(NRF24L01* dev, uint8_t count) {
	count &= 0x0F;
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_SETUP_RETR, &reg) != NRF_OK) {
		return NRF_ERROR;
	}

	reg &= 0xF0;	// clearing bits 0,1,2,3
	reg |= count;	// setting count

	if (NRF_WriteRegister(dev, NRF_SETUP_RETR, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_SetRetransmittionDelay(NRF24L01* dev, uint8_t delay) {
	delay &= 0x0F;
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_SETUP_RETR, &reg) != NRF_OK) {
		return NRF_ERROR;
	}

	reg &= 0x0F;	// clearing bits 1,2,6,7
	reg |= delay << 4;	// setting delay

	if (NRF_WriteRegister(dev, NRF_SETUP_RETR, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_SetAddressWidth(NRF24L01* dev, NRF_ADDR_WIDTH width) {
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_SETUP_AW, &reg) != NRF_OK) {
		return NRF_ERROR;
	}

	reg &= 0x03;	// clearing bits 0,1
	reg |= width;	// setting delay

	if (NRF_WriteRegister(dev, NRF_SETUP_AW, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_EnableRXPipe(NRF24L01* dev, uint8_t pipe) {
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_EN_RXADDR, &reg) != NRF_OK) {
		return NRF_ERROR;
	}

	reg |= 1 << pipe;

	if (NRF_WriteRegister(dev, NRF_EN_RXADDR, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_EnableAutoAcknowledgement(NRF24L01* dev, uint8_t pipe) {
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_EN_AA, &reg) != NRF_OK) {
		return NRF_ERROR;
	}

	reg |= 1 << pipe;

	if (NRF_WriteRegister(dev, NRF_EN_AA, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_EnableCRC(NRF24L01* dev, uint8_t activate) {
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_CONFIG, &reg) != NRF_OK) {
		return NRF_ERROR;
	}

	if (activate) {
		reg |= 1 << 3;
	} else {
		reg &= ~(1 << 3);
	}

	if (NRF_WriteRegister(dev, NRF_CONFIG, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_SetCRCWidth(NRF24L01* dev, NRF_CRC_WIDTH width) {
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_CONFIG, &reg) != NRF_OK) {
		return NRF_ERROR;
	}

	if (width == NRF_CRC_WIDTH_2B) {
		reg |= 1 << 2;
	} else {
		reg &= ~(1 << 3);
	}

	if (NRF_WriteRegister(dev, NRF_CONFIG, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_PowerUp(NRF24L01* dev, uint8_t powerUp) {
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_CONFIG, &reg) != NRF_OK) {
		return NRF_ERROR;
	}

	if (powerUp) {
		reg |= 1 << 1;
	} else {
		reg &= ~(1 << 1);
	}

	if (NRF_WriteRegister(dev, NRF_CONFIG, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_RXTXControl(NRF24L01* dev, NRF_TXRX_STATE rx) {
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_CONFIG, &reg) != NRF_OK) {
		return NRF_ERROR;
	}

	if (rx) {
		reg |= 1;
	} else {
		reg &= ~(1);
	}

	if (NRF_WriteRegister(dev, NRF_CONFIG, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_EnableRXDataReadyIRQ(NRF24L01* dev, uint8_t activate) {
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_CONFIG, &reg) != NRF_OK) {
		return NRF_ERROR;
	}

	if (!activate) {
		reg |= 1 << 6;
	} else {
		reg &= ~(1 << 6);
	}

	if (NRF_WriteRegister(dev, NRF_CONFIG, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_EnableTXDataSentIRQ(NRF24L01* dev, uint8_t activate) {
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_CONFIG, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	if (!activate) {
		reg |= 1 << 5;
	} else {
		reg &= ~(1 << 5);
	}
	if (NRF_WriteRegister(dev, NRF_CONFIG, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_EnableMaxRetransmitIRQ(NRF24L01* dev, uint8_t activate) {
	uint8_t reg = 0;
	if (NRF_ReadRegister(dev, NRF_CONFIG, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	if (!activate) {
		reg |= 1 << 4;
	} else {
		reg &= ~(1 << 4);
	}
	if (NRF_WriteRegister(dev, NRF_CONFIG, &reg) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_SetRXAddress_P0(NRF24L01* dev, uint8_t* address) {
	uint8_t rx[5];
	if (NRF_SendCommand(dev, NRF_CMD_W_REGISTER | NRF_RX_ADDR_P0, address, rx,
			5) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_SetTXAddress(NRF24L01* dev, uint8_t* address) {
	uint8_t rx[5];
	if (NRF_SendCommand(dev, NRF_CMD_W_REGISTER | NRF_TX_ADDR, address, rx, 5)
			!= NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_SetRXPayloadWidth_P0(NRF24L01* dev, uint8_t width) {
	width &= 0x3F;
	if (NRF_WriteRegister(dev, NRF_RX_PW_P0, &width) != NRF_OK) {
		return NRF_ERROR;
	}
	return NRF_OK;
}

NRF_RESULT NRF_SendPacket(NRF24L01* dev, uint8_t* data) {
	NRF_CE_DISABLE(dev);
	while (dev->STATE == NRF_STATE_TX)
		;	//waiting for the previous tx
	NRF_RXTXControl(dev, NRF_STATE_TX);
	NRF_WriteTXPayload(dev, data);
	NRF_CE_ENABLE(dev);
	return NRF_OK;
}

NRF_RESULT NRF_ReceivePacket(NRF24L01* dev, uint8_t* data) {
	NRF_CE_DISABLE(dev);
	NRF_RXTXControl(dev, NRF_STATE_RX);
	NRF_CE_ENABLE(dev);

	while (dev->RX_FLAG == 0)
		;	// wait for reception

	int i = 0;
	for (i = 0; i < dev->PayloadLength; i++) {
		data[i] = dev->RX_BUFFER[i];
	}

	dev->RX_FLAG = 0;

	return NRF_OK;
}

