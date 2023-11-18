/*
 * spi.c
 *
 *  Created on: Oct 6, 2023
 *      Author: ADMIN
 */

#include <stdint.h>
#include "spi.h"
#include "main.h"


static void slave_ctrl(slave_ctrl_t state) //"static void" for privacy
{
    uint32_t* GPIOD_ODR = (uint32_t*)(0x40020C00 + 0x14);
    if(state == SLAVE_ACTIVE)
    {
        *GPIOD_ODR &= ~(1 << 12);
    }
    else
    {
        *GPIOD_ODR |= (1 << 12);
    }
}

void spi_init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    uint32_t* GPIOA_MODER = (uint32_t*)(0x40020000 + 0x00);
    *GPIOA_MODER &= ~((0b11 << 10)|(0b11 << 12)|(0b11 << 14));
    *GPIOA_MODER |= (0b10 << 10)|(0b10 << 12)|(0b10 << 14);
    uint32_t* GPIOA_AFRL = (uint32_t*)(0x40020000 + 0x20);
    *GPIOA_AFRL &= ~(0xfff << 20);
    *GPIOA_AFRL |= (5 << 20) | (5 << 24) | (5 << 28);

    __HAL_RCC_GPIOD_CLK_ENABLE(); // PD12 is Slave Select (SS)
    uint32_t* GPIOD_MODER = (uint32_t*)(0x40020C00 + 0x00);
    *GPIOD_MODER &= ~(0b11 << 24);
    *GPIOD_MODER |= (0b01 << 24);

    __HAL_RCC_GPIOE_CLK_ENABLE(); // PD12 is Slave Select (SS)
    uint32_t* GPIOE_MODER = (uint32_t*)(0x40021000 + 0x00);
    *GPIOE_MODER &= ~(0b11 << 6);
    *GPIOE_MODER |= (0b01 << 6);

    uint32_t* GPIOE_ODR = (uint32_t*)(0x40021000 + 0x14);
    *GPIOE_ODR |= (1 << 3);

    slave_ctrl(SLAVE_INACTIVE);
     __HAL_RCC_SPI1_CLK_ENABLE();
     uint32_t* SPI1_CR1 = (uint32_t*)(0x40013000 + 0x00);
     *SPI1_CR1 |= (1 << 2)|(0b100 << 3)|(1 << 9)|(1 << 8); // have some mistake in there
     *SPI1_CR1 |= (1 << 6);  // should be config SPI first before enable SPI
}

uint8_t spi_read(uint8_t data_type)
{
    uint32_t* SPI_SR = (uint32_t*)(0x40013000 + 0x08);
    uint32_t* SPI_DR = (uint32_t*)(0x40013000 + 0x0C);

    slave_ctrl(SLAVE_ACTIVE);
    while(((*SPI_SR >> 1) &1) !=1);
    *SPI_DR = data_type;
    while(((*SPI_SR >> 7) &1) ==1);
    while(((*SPI_SR >> 0) &1) !=1);
    uint32_t temp = *SPI_DR;

    while(((*SPI_SR >> 1) &1) !=1);
    *SPI_DR = 0xFF; // sending dummy data
    while(((*SPI_SR >> 7) &1) ==1);
    while(((*SPI_SR >> 0) &1) !=1);
    temp = *SPI_DR;

    slave_ctrl(SLAVE_INACTIVE);
    return temp;
}
void spi_write(uint8_t data_type, uint8_t data)
{
	uint32_t* SPI_SR = (uint32_t*)(0x40013000 + 0x08);
	uint32_t* SPI_DR = (uint32_t*)(0x40013000 + 0x0C);

	slave_ctrl(SLAVE_ACTIVE);
    while(((*SPI_SR >> 1) &1) !=1);
    *SPI_DR = data_type;
    while(((*SPI_SR >> 7) &1) ==1);
    while(((*SPI_SR >> 0) &1) !=1);
    uint32_t temp = *SPI_DR;

    while(((*SPI_SR >> 1) &1) !=1);
    *SPI_DR = data;
    while(((*SPI_SR >> 7) &1) ==1);
    while(((*SPI_SR >> 0) &1) !=1);
    temp = *SPI_DR;

    (void) temp;
    slave_ctrl(SLAVE_INACTIVE);
    return temp;
}


