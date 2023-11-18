/*
 * spi.h
 *
 *  Created on: Oct 6, 2023
 *      Author: ADMIN
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#ifndef CMSIS_MY_DRIVER_INC_SPI_H_
#define CMSIS_MY_DRIVER_INC_SPI_H_

#define WHO_AM_I 	0X0F
#define CTRL_REG1	0X20
#define STATUS_REG  0X27

#define OUT_X_L 	0x28
#define OUT_X_H 	0x29

#define OUT_Y_L 	0x2A
#define OUT_Y_H 	0x2B

#define OUT_Z_L 	0x2C
#define OUT_Z_H 	0x2D


typedef enum
{
    SLAVE_ACTIVE,
    SLAVE_INACTIVE
}slave_ctrl_t;

void spi_init();
uint8_t spi_read(uint8_t data_type);
void spi_write(uint8_t data_type, uint8_t data);

#endif /* CMSIS_MY_DRIVER_INC_SPI_H_ */



#endif /* INC_SPI_H_ */
