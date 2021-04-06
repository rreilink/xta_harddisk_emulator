XT-Attachment interface
=======================

This document describes the XT-Attachemnt hard disk interface as found on the IBM model 30-8086. Other computers from the same era may (or may not) work the same or similar.

About the only information about this interface that I could find online was the [connector pinout](<https://www.ardent-tool.com/misc/IBM_HD_pinouts.html>). All other information was found by disassembling the BIOS INT13 handler, which handles hard drive access.

As a consequence, the names and terms that are used in this document are mostly conceived by me. I have tried to use terminology that is commonly used in similar fields, e.g. in the ATA interface, in the ST506/ST412 interface or that is used in the comments of the listings of the original PC-XT BIOS.

Convections used in this document
---------------------------------

When referring to individiual bits, the bit number always means the position of that bit in the byte, not the bit mask. I.e. the bit numbers range from 0-7, with bit 0 corresponding to value 0x01 and bit 7 corresponding to 0x80.

When referring to interrupts (e.g. INT13) the value is in hexadecimal (i.e. 0x13=19 dec). All other numbers are decimal unless preceded by 0x .

Several sections refer to the BIOS hard drive interrupt handler INT13. Check [Wikipedia](<https://en.wikipedia.org/wiki/INT_13H>) for a description of the commands and return codes.


Pinout & physical interface
--------------------------

The XTA interface on the IBM model30-8086 is found here: <https://www.ardent-tool.com/misc/IBM_HD_pinouts.html>
For the BIOS to detect the hard disk, pin 2 (nDiskInstalled) must be grounded. In this pinout discription pin 23 is named nCS1FX which would suggest that it is a chip select line which is active (low) when address 0x1fx is selected. However, that adress range is what is used in the ATA interface. Actually, it is active when 0x32x (x=0..7) is selected; thus the pin would be more apropriately referred to as nCS32x.

Furthermore, this nCS32x is gated. In order to allow access to the drive, IO register 0x65, bit 0 must be written high. Otherwise, any IO access to 0x320..0x327 does not activate nCS32x


Registers
---------

Once the chip select gate has been opened (i.e. bit 0 of IO 0x65 is written 1), the physical interface exposes the following registers:

Reading (data from drive to host):
| Address |  Name   | Function |
| ------- | ------  | -------- |
| 0x320   | sense   | reading of sense bytes; the result of an operation like error and status |
| 0x322   | sync_in | synchronization of host to drive, e.g. wait until the drive is ready to accept a new command |
| 0x324   | status  | status/error after an operation |

Writing (data from processor to drive):
| Address | Name        | Function |
| ------- | ---------   | -------- |
| 0x320   | command     | command and arguments to the drive |
| 0x322   | irq_drq_ena | Mask IRQ and DRQ signals to the host; reset the drive |
| 0x324   | sync_out    | synchronization of drive to the host, e.g. signal start of a command |

### sync_in register
| Bit | Name         | Function |
| ----| ---------    | ----- |
|  4  | data_rdy     | The host waits for this bit to be set before reading each sense byte or writing each command byte |
|  2  | drive_bsy    | The host waits for this bit to be clear before sending a command block or reading a sense block  |
|  1  | data_cmd_rdy | The host waits for this bit to be set after writing the command and before starting the DMA transfer |

### status register
| Bit | Name         | Function |
| ----| ---------    | ----- |
| 7   | error        | If this bit is set after a command is given, the host will perform error handling |
| 6,5 | invalid_cmd  | If these bits are 00 and the error bit is set, the host will perform a sense operation to find the source of the error. Otherwise, AH=0x01 (invalid command) will be returned from the BIOS INT13 handler.



### sync_out register
| Bit | Name          | Function |
| --- | ------------- | -------- |
| 7   | command_start | Written 1 before the host starts sending the command block |
| 5   | sense_start   | Written 1 before the host starts reading the sense block |
| 4   | dma_start?    | Written 1 when waiting for interrupt in a command with DMA, and in the INT13,AH=0x05 format track routine |

### irq_drq_ena register
| Bit | Name    | Function |
| --- | ------  | -------  |
| 7   | reset   | Reset the drive, written 1 from the reset routine (BIOS INT13, AH=0x00) |
| 1   | irq_ena | Written 1 when the host waits for an IRQ |
| 0   | drq_ena | Written 1 when the host wants to transfer data via DMA |

Command, Sense and Data transfer
-------------------------------

### Command block
Commands are written to the drive as a series of 6 bytes which are written in succession to the command register (0x320). 

A command that does *not* require DMA data transfer is sent by the host using the following procedure:

- Write 0x2 to the irq_drq_ena register (bit irq_ena high, drq_ena low)
- Wait until bit drive_bsy in the sync_in register is cleared
- For 6 bytes:
  - wait until bit data_rdy in the sync_in register is set
  - output the byte to the command register
- Enable the IRQ5 interrupt on the PIC interrupt controller
- Wait for IRQ5
- Check the error bit in status. If set:
  - If bits invalid_cmd in the status register are 00, perform a sense operation to find the cause. Otherwise, return error=01H Invalid command

A command that *does* require DMA data transfer is sent by the host using the following procedure:

- Wait until bit drive_bsy in the sync_in register is cleared
- For 6 bytes:
  - wait until bit data_rdy in the sync_in register is set
  - output the byte to the command register
- Wait until bit data_cmd_rdy in the sync_in register is set
- Check the error bit in status. If set:
  - If bits invalid_cmd in the status register are 00, perform a sense operation to find the cause. Otherwise, return error=01H Invalid command
- Setup the DMA controller to transfer the apropriate amount of bytes
- Write 0x3 to the irq_drq_ena register (bits irq_ena and drq_ena high)
- Enable DRQ3 mask of the DMA controller
- Enable the IRQ5 interrupt on the PIC interrupt controller
- Write 0x10 to the sync_out register (bit dma_start)
- Wait for IRQ5
- Check the error bit in status. If set:
  - If bits invalid_cmd in the status register are 00, perform a sense operation to find the cause. Otherwise, return error=01H Invalid command


| Byte | Contents |
| -----| -------- |
| 0    | Command |
| 1, bit 7..4| Head (0-15) |
| 1, bit 1..0| Cylinder bits 9..8 |
| 2    | Cylinder bits 7..0 |
| 3    | Sector (1-63) |
| 4    | Always contains value 0x2 |
| 5    | Sector count |

### Sense block
The sense block (14 bytes) is read from the drive at startup to determine the drive parameters (cylinders, heads, sectors), at then INT13, AH0x10 (test whether drive is ready) command, and whenever a command resulted in an error indicated by the error bit in the status register. Sense bytes are read from the sense register (0x320). 

The sense block is read using the following procedure:

- Wait until bit drive_bsy is cleared
- Write 0x20 to sync_out (sense_start = 1)
- for 14 bytes:
  - wait until bit data_rdy is set
  - read sense register
- Wait until bit data_cmd_rdy is set
- Read status register; if bit 7 is set then an error occurred during the sense operation

If an error occurred during the sense operation, the INT13 handler will return AH=0xff (sense failed)

The contents of the sense block is as follows. The other bits do not seem to be used in the BIOS INT13 handler, and thus their meaning (if any) cannot be deduced from it.

| Byte | Contents| BIOS INT13 error code AH |
| ----- | ------ | ------- |
| 0    | Status, this byte is stored in the BIOS data area address 0x40:0x8c after a sense operation |
| 0, bit 7 | Drive not ready | 0xaa |
| 0, bit 4 | Write fault | 0xcc |
| 1    | Error, this byte is stored in the BIOS data area address 0x40:0x8d after a sense operation |
| 1, bit 6 | crc/ecc error | 0x10 |
| 1, bit 5 | could not find address mark | 0x02 |
| 1, bit 4 | bad cylinder detected | 0x0b |
| 1, bit 3 | seek failiure | 0x40 |
| 1, bit 2 | undefined error | 0xbb |
| 1, bit 0 | sector not found | 0x04 |
| 2, bit 6 | controller failiure | 0x20 |
| 2, bit 4 | bad sector detected BIOS error|  0x0a |
| 9 | any bit set = ECC corrected data. This is not necessarily treated as an error | 0x11 |
| 12 | drive type, See [these hard drive tables, section 3.2](<https://www.win.tue.nl/~aeb/linux/hdtypes/hdtypes-3.html>) | |

The drive type is read once when the PC boots; the pointer to the table is stored in the INT41 interrupt vector, and data can be retrieved using INT13, AH=08 (read drive params).

### Data
In the IBM model30-8086, all data transfer (i.e. the bytes that go to and come from the actual disk) is done using DMA. It is not known whether XTA drives could alternatively use register access to transfer data.





