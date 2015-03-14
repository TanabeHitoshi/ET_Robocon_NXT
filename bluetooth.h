/*
 *bluetooth.h
 *
 *  Created on: 2015/03/14
 *      Author: ‚ˆ‚”
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

extern char rx_buf[BT_MAX_RX_BUF_SIZE];
extern char tx_buf[128];

int remote_start(void);
int strlen(const char *s);

#endif
