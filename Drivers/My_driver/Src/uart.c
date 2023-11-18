/*
 * uart.c
 *
 *  Created on: Oct 11, 2023
 *      Author: ADMIN
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "MFRC522.h"
#include "main.h"

char rx_buf[256];
int rx_index;


