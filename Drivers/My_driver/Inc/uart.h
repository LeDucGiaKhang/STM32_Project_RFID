/*
 * uart.h
 *
 *  Created on: Oct 11, 2023
 *      Author: ADMIN
 */

#ifndef INC_UART_H_
#define INC_UART_H_

void UART_init();
void UART_send_1_byte(char data);
void UART_send_string(char* str);
char UART_recieve_data();

#endif /* INC_UART_H_ */
