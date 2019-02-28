#ifndef __spi_H
#define __spi_H

#define WRITE 0
#define READ  1

void spi_init(void);
u8 spi_rw_byte(unsigned char dt);
void spi_nss_low(void);
void spi_nss_high(void);
void spi_rw(u8 *data_buff, u8 byte_quantity, u8 reg_address, u8 control_Byte);
#endif
