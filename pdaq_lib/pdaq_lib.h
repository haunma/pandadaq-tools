/* pdaq_lib.h - PandaDAQ board access library                                */
/* Based on bfpga_lib.h, Copyright 2010 Eric Brombaugh                       */
/*  This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/

#ifndef __pdaq_lib__
#define __pdaq_lib__

#include <stdint.h>
#include <linux/types.h>
#include <linux/i2c-dev.h>


typedef struct
{
    int i2c_file;       /* I2C filehandle */
    char prom_addr;          /* I2C bus address of ID EEPROM */
    char pport_addr;         /* I2C bus address of parallel port */
    int spi_file;       /* SPI filehandle */
    int verbose;        /* Verbose level */
} pdaq;

#define READBUFSIZE 512

int pdaq_i2c_get_prom(pdaq *s, unsigned char *buf, int saddr, int size);
int pdaq_i2c_set_prom(pdaq *s, unsigned char *buf, int saddr, int size);
int pdaq_i2c_parport_rd_byte(pdaq *s, int reg);
int pdaq_i2c_parport_wr_byte(pdaq *s, int reg, __u8 dat);
int pdaq_i2c_parport_rd(pdaq *s, int bit);
void pdaq_i2c_parport_wr(pdaq *s, int bit, int val);
int pdaq_spi_txrx(pdaq *s, uint8_t *tx, uint8_t *rx, __u32 len);
pdaq *pdaq_init(int i2c_bus, char prom_addr, char pport_addr, int spi_bus, int spi_add, int verbose);
int pdaq_cfg(pdaq *s, char *bitfile);
void pdaq_delete(pdaq *s);

#endif
