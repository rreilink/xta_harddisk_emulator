/*
 * xta.c
 *
 *  Created on: 30 Mar 2021
 *      Author: rob
 *
 *  This file implements the XTA (XT-Attachment) register interface
 */

#include <stdint.h>
#include "main.h"

uint16_t sample=0;
uint8_t regs[9]={0};
volatile uint8_t irq_drq_reg = 0;
volatile uint8_t irq_drq_val = 0;
uint8_t command_block[6];
uint8_t command_block_idx = 0;

// This controller emulates drive type 4
// see https://www.win.tue.nl/~aeb/linux/hdtypes/hdtypes-3.html section 3.2
#define DRIVE_TYPE 4
#define CYLINDERS 940
#define HEADS 8
#define SECTORS 17


uint8_t sense_block[0xe] = {0,0,0,0,0,0,0,0,0,0,0,0,DRIVE_TYPE,0};
uint8_t sense_block_idx = 0;

volatile uint8_t command_execute = 0;
volatile int dma_cnt = 0;

const uint32_t MODER_INPUTS = 0;
const uint32_t MODER_OUTPUTS = 0x5555;


static inline void irq_drq_update() {
	GPIOB->ODR = (GPIOB->ODR & ~0x300) | (irq_drq_reg & irq_drq_val)<<8;
}

static volatile unsigned char* dma_ptr;


void xta_init(void) {
	  regs[0] = 0; // data

	  //322 read:
	  regs[2] = 0x12; // busy/sync bit4=ready for data write to 320/read from 320. bit2=command busy   bit1=command accepted

	  //322 write: dreq / int mask

	  regs[4] = 0; // result; bit 7 = error

	  regs[8] = 0x52; // DMA register

	  nIORDY_GPIO_Port->ODR |=nIORDY_Pin;
}



/* Get the command that was written to the controller, if any
 *
 * returns:
 *  -1 if no command was written
 *  the command (0-255) if a command was written. In that case:
 *    *lba   is written the linear block address (start block, 0-based)
 *    *count is written the number of blocks to be transferred
 *
 */
int xta_get_command(uint32_t *lba, uint32_t *count) {
	if (!command_execute) {
		return -1;
	}

	/* Command block:
	 byte 0 = command
     byte 1 = head (high nibble), cylinder bit 9&8 in bit 1&0
     byte 2 = cylinder bit 7..0
     byte 3 = sector (6 bits, 0..63)
     byte 4 = always 2
     byte 5 = sector count
     */


	uint32_t cylinder = ((command_block[1]&3)<<8) | command_block[2];
	uint32_t head = command_block[1]>>4;
	uint32_t sector = command_block[3];

	if ((cylinder>=CYLINDERS) || (head>=HEADS) || (sector==0) || (sector>SECTORS)) {
		// TODO: process the error
		command_execute = 0;
		return -1;
	}

	*lba = (sector-1) + SECTORS* (head + HEADS*cylinder);

	*count = command_block[5];

	command_execute = 0;
	return command_block[0];

}


void xta_dma_read_transfer_start(unsigned char * data) {
	  dma_ptr = data;
	  regs[8] = *dma_ptr++;
	  dma_cnt = 511; // the fist byte has just been written, 511 remaining

	  irq_drq_val |= 1; // set DRQ
	  irq_drq_update();

}

void xta_dma_waitfinish(void) {
	  while(irq_drq_val & 1) {
		  // DRQ is cleared from IRQHandler upon reading the last byte
	  }
}

void xta_dma_write_transfer_start(unsigned char * data) {
	  dma_ptr = data;
	  dma_cnt = 512; // 512 bytes remain to be written

	  irq_drq_val |= 1; // set DRQ
	  irq_drq_update();




}



void xta_set_irq(void) {
	  irq_drq_val |= 2; // set IRQ
	  irq_drq_update();
}

void xta_clear_irq(void) {
	  irq_drq_val &= ~2; // set IRQ
	  irq_drq_update();
}



void xta_set_cmdbusy(int busy) {
	if (busy) {
		regs[2]|=1<<2;
	} else {
		regs[2]&=~1<<2;
	}

}


#define PINHIGH(x) x ## _GPIO_Port->BSRR = x## _Pin;
#define PINLOW(x) x ## _GPIO_Port->BSRR = x## _Pin<<16;


/* IO access interrupt
 *
 */
void EXTI1_IRQHandler(void)
{

	// first thing to do: sample the data
	sample = GPIOC->IDR;




	uint8_t address;


	if (!(sample & (nDACK_Pin | nIORD_Pin))) {
		GPIOC->MODER = MODER_OUTPUTS;
		GPIOC->ODR=regs[8];


		// DRQ low, to make sure no transfer is started after the last byte of a block
		// As a speed optimization, we set the pin level here (before IO ready)
		// and update the irq_drq_val flag later
		if(dma_cnt==0) 	GPIOB->BSRR = DRQ_Pin << 16;

		PINLOW(nIORDY);  // IO is ready


		// Wait for I/O cycle to finish
		while(!(nIOACCESS_GPIO_Port->IDR & nIOACCESS_Pin));

		GPIOC->MODER = MODER_INPUTS;


		PINHIGH(nIORDY); // IO is not ready (next access)

		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);

		if (dma_cnt==0) { // transfer completed
			irq_drq_val &=~1; // clear DRQ flag (pin is already lowered upfront)
		} else {
			regs[8] = *(dma_ptr++);
			dma_cnt--;
		}


	} else if (sample & nIORD_Pin) { // write access (DMA or non-DMA)
		if (!(sample & nDACK_Pin)) {
			// DMA write
			if (dma_cnt) {
				*(dma_ptr++) = sample;
				dma_cnt--;
			}

			if (!dma_cnt) {
				GPIOB->BSRR = DRQ_Pin << 16; // DRQ low, to make sure no transfer is started after the last byte of a block

				irq_drq_val &=~1; // clear DRQ
			}

		} else {
			// Non-DMA write
			address = (sample >> 8) & 7;
			if (address == 0) {
				if ((command_block_idx<6) && (!command_execute)) {
					command_block[command_block_idx++] = sample;
					if (command_block_idx==6) {
						command_execute = 1;
					}
				}
			} else if (address == 2) { // irq/drq mask register
				if (sample & 0x80) { // reset
					// reset. Probably cannot do a processor reset since the host will start writing to us right-away
					// TODO: reset main loop and DMA status
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
		}


		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);


		nIORDY_GPIO_Port->ODR &= ~nIORDY_Pin; // IO is ready todo: optimize, move forward, but ensure DRQ is lowered before


		while(!(nIOACCESS_GPIO_Port->IDR & nIOACCESS_Pin));

		nIORDY_GPIO_Port->ODR |=nIORDY_Pin;  // IO is not ready (next access)


	} else {
		// read access (non-DMA)
		address = (sample >> 8) & 7;

		GPIOC->MODER = MODER_OUTPUTS;
		GPIOC->ODR=regs[address];

		PINLOW(nIORDY);  // IO is ready

		// Wait for I/O cycle to finish
		while(!(nIOACCESS_GPIO_Port->IDR & nIOACCESS_Pin));

		GPIOC->MODER = MODER_INPUTS;

		PINHIGH(nIORDY);  // IO is not ready (next access)

		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);

		if (address == 0) { // sense register
			if (sense_block_idx<0xe) {
				regs[0] = sense_block[sense_block_idx++];
			}
		}



	}

	irq_drq_update(); // Set DRQ to the required value


}
