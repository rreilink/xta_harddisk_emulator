/*
 * xta.h
 *
 *  Created on: 30 Mar 2021
 *      Author: rob
 */

#ifndef SRC_XTA_H_
#define SRC_XTA_H_

void xta_init(void);
void xta_dma_read_transfer_start(unsigned char * data);
void xta_dma_write_transfer_start(unsigned char * data);
void xta_dma_waitfinish(void);
void xta_set_irq(void);
void xta_clear_irq(void);
int xta_get_command(uint32_t *lba, uint32_t *count);


/*
 * Commands
 */

#define CMD_READ    0x15
#define CMD_VERIFY  0x25
#define CMD_WRITE   0x95


#endif /* SRC_XTA_H_ */
