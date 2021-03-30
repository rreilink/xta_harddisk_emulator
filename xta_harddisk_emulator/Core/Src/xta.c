/*
 * xta.c
 *
 *  Created on: 30 Mar 2021
 *      Author: rob
 *
 *  This file implements the XTA (XT-Attachement) register interface
 */

#include <stdint.h>
#include "main.h"

uint16_t sample=0;
uint8_t regs[9]={0};
volatile uint8_t irq_drq_reg = 0;
volatile uint8_t irq_drq_val = 0;
uint8_t command_block[6];
uint8_t command_block_idx = 0;

#define DRIVE_TYPE 4 // see https://www.win.tue.nl/~aeb/linux/hdtypes/hdtypes-3.html section 3.2

uint8_t sense_block[0xe] = {0,0,0,0,0,0,0,0,0,0,0,0,DRIVE_TYPE,0};
uint8_t sense_block_idx = 0;

volatile uint8_t command_execute = 0;
volatile int data_idx = 0;

const uint32_t MODER_INPUTS = 0;
const uint32_t MODER_OUTPUTS = 0x5555;


static inline void irq_drq_update() {
	GPIOB->ODR = (GPIOB->ODR & ~0x300) | (irq_drq_reg & irq_drq_val)<<8;
}

static const char data[512] =
		"\xb8\x52\x0e\xbb\x00\x00\xcd\x10\xeb\xfe"
		"                                                  "
		"                                                  "
		"                                                  "
		"                                                  "
		"                                                  "
		"                                                  "
		"                                                  "
		"                                                  "
		"                                                  "
		"                                                  "
		"\x55\xaa"

		;


void xta_init(void) {
	  regs[0] = 0; // data

	  //322 read:
	  regs[2] = 0x12; // busy/sync bit4=ready for data write to 320/read from 320. bit2=command busy   bit1=command accepted

	  //322 write: dreq / int mask

	  regs[4] = 0; // result; bit 7 = error

	  regs[8] = 0x52; // DMA register

	  nIORDY_GPIO_Port->ODR |=nIORDY_Pin;
}

void xta_dma_transfer(void) {
	  data_idx = 0;
	  regs[8] = data[data_idx++];

	  irq_drq_val |= 1; // set DRQ
	  irq_drq_update();

	  while(data_idx<512); // TODO: clear DRQ from interrupt handler

	  irq_drq_val &= ~1; // clear DRQ
	  irq_drq_update();
}

void xta_set_irq(void) {
	  irq_drq_val |= 2; // set IRQ
	  irq_drq_update();
}




/* IO access interrupt
 *
 */
void EXTI1_IRQHandler(void)
{


	sample = GPIOC->IDR;
	uint8_t address = (sample >> 8) & 7;
	if (!(sample & nDACK_Pin)) address = 8;

	if (sample & nIORD_Pin) {
		nIORDY_GPIO_Port->ODR &= ~nIORDY_Pin; // IO is ready

		// write access
		GPIOA->BSRR = (1<<5);


		if (address == 0) {
			if ((command_block_idx<6) && (!command_execute)) {
				command_block[command_block_idx++] = sample;
				if (command_block_idx==6) {
					command_execute = 1;
				}
			}
		}

		if (address == 2) { // irq/drq mask register
			if (sample & 0x80) { // reset
				// reset. Probably cannot do a processor reset since the host will start writing to us right-away
				command_block_idx = 0;
				irq_drq_val = 2; // BIOS expects an interrupt upon reset completion

			}
			irq_drq_reg = sample;
			irq_drq_update();
			// todo: set irq/drq
		} else if(address == 4) {
			if (sample & 0x80) { // start of write_command_block_to_controller
				command_block_idx = 0;
				irq_drq_val = 0;
				irq_drq_update();
			}
			if (sample & 0x20) { // start of sense operation
				sense_block_idx = 0;
				regs[0] = sense_block[sense_block_idx++];

			}


		}


		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
		GPIOA->BSRR = (0x10000<<5);


		while(!(nIOACCESS_GPIO_Port->IDR & nIOACCESS_Pin));

		nIORDY_GPIO_Port->ODR |=nIORDY_Pin;  // IO is not ready (next access)


	} else {
		// read access

		GPIOC->MODER = MODER_OUTPUTS;
		GPIOC->ODR=regs[address];

		nIORDY_GPIO_Port->ODR &= ~nIORDY_Pin; // IO is ready

		GPIOA->BSRR = (1<<5);

		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);

		while(!(nIOACCESS_GPIO_Port->IDR & nIOACCESS_Pin));
		GPIOC->MODER = MODER_INPUTS;

		nIORDY_GPIO_Port->ODR |=nIORDY_Pin; // IO is not ready (next access)


		if (address == 8) { // DMA data transfer
			if (data_idx==512) { // transfer completed
				irq_drq_val &=~1; // clear DRQ
				irq_drq_update();
			} else {
				regs[8] = data[data_idx++];
			}

		} else if (address == 0) { // sense register
			if (sense_block_idx<0xe) {
				regs[0] = sense_block[sense_block_idx++];
			}
		}



		GPIOA->BSRR = (0x10000<<5);

	}










}
