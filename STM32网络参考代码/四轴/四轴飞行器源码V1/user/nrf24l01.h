#ifndef __nrf24l01_H
#define __nrf24l01_H

void nrf24l01_init(void);
void nrf24l01_rx_mode(void);
u8 nrf24l01_rx_data(u8 *tx_data_buf);
#endif
