/* pdaq_lib.c - PandaDAQ board access library             */
/* Based on bfpga_lib.c, copyright 2010 Eric Brombaugh    */
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

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <errno.h>
#include <math.h>
#include "pdaq_lib.h"

const unsigned char idprom[] =
{
    0x00, 0x06, 0x00, 0x02, 0x00, 0x00
};

// .bit file header
const char bit_hdr[] =
{
    0x00, 0x09, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x00, 0x00, 0x01
};

const char *bit_hdr_strings[] =
{
    "filename",
    "device",
    "date",
    "time"
};

const char *deviceid = "6slx9tqg144";

// wrapper for fprintf(stderr, ...) to support verbose control
void qprintf(pdaq *s, char *fmt, ...)
{
    va_list args;
    
    if(s->verbose)
    {
        va_start(args, fmt);
        vfprintf(stderr, fmt, args);
        va_end(args);
    }
}

// set I2C slave address
int i2c_set_slave_addr(int file, int address)
{
    if(ioctl(file, I2C_SLAVE, address) < 0)
        return -errno;
    else
        return 0;
}

// low-level interface to I2C read/write
/*
static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command, 
                                     int size, union i2c_smbus_data *data)
{
    struct i2c_smbus_ioctl_data args;

    args.read_write = read_write;
    args.command = command;
    args.size = size;
    args.data = data;
    return ioctl(file,I2C_SMBUS,&args);
}
*/



//
// get <count> bytes from the i2c eeprom starting at <addr>
//
int pdaq_i2c_get_prom(pdaq *s, unsigned char *buf, int addr, int size)
{
    int j;
    __s32 tmp;
        
    // set address
    if(i2c_set_slave_addr(s->i2c_file, s->prom_addr))
        return -1;

    // get data
    for (j = 0 ; j < size ; j++)
    {
        if ((tmp = i2c_smbus_read_byte_data(s->i2c_file, addr+j)) < 0)
            return -1;
        else
            buf[j] = 0x0FF & tmp;
    }
        
    return j;  // return number of bytes successfully read
}


//
// write <size> bytes to the i2c eeprom starting at <addr>
//
int pdaq_i2c_set_prom(pdaq *s, unsigned char *buf, int addr, int size)
{
    int j;
    
    // set address
    if(i2c_set_slave_addr(s->i2c_file, s->prom_addr))
        return -1;

    // send data
    for (j = 0 ; j < size ; j++)
    {
        if (i2c_smbus_write_byte_data(s->i2c_file, addr+j, buf[j]) != 0)
            return -1;
    }
    
    return j;  // return number of bytes successfully written
}


//
// read an i2c parport register (entire byte)
//
int pdaq_i2c_parport_rd_byte(pdaq *s, int reg)
{
    __s32 tmp;
    
    // set address
    if(i2c_set_slave_addr(s->i2c_file, s->pport_addr))
        return -1;

    // get byte at register address <reg>
    if ((tmp = i2c_smbus_read_byte_data(s->i2c_file, reg)) < 0)
        return -1;
    else
        return 0x0FF & tmp;
}


//
// write an i2c parport register (entire byte)
//
int pdaq_i2c_parport_wr_byte(pdaq *s, int reg, __u8 dat)
{
    // set address
    if (i2c_set_slave_addr(s->i2c_file, s->pport_addr))
        return -1;

    // send register address and byte data
    return i2c_smbus_write_byte_data(s->i2c_file, reg, dat);
}


//
// get i2c parport bit
//
int pdaq_i2c_parport_rd(pdaq *s, int bit)
{
    return (pdaq_i2c_parport_rd_byte(s, 0x00) >> bit) & 1;
}


//
// set i2c parport bit
//
void pdaq_i2c_parport_wr(pdaq *s, int bit, int val)
{
    int oldval = pdaq_i2c_parport_rd_byte(s, 0x01);
    int hmask = 1<<bit;
    int lmask = ~hmask & 0xFF;
    int newval = (oldval & lmask) | ((val & 1) << bit);
    pdaq_i2c_parport_wr_byte(s, 0x01, newval);
}


//
// SPI Transmit/Receive
//
int pdaq_spi_txrx(pdaq *s, uint8_t *tx, uint8_t *rx, __u32 len)
{
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len,
        .delay_usecs = 0,
        .speed_hz = 500000,
        .bits_per_word = 8,
    };
    
    return ioctl(s->spi_file, SPI_IOC_MESSAGE(1), &tr);
}

// initialize our FPGA interface
pdaq *pdaq_init(int i2c_bus, char prom_addr, char pport_addr, int spi_bus, int spi_addr, int verbose)
{
    pdaq *s;
    char filename[20];
    unsigned char buf[6];
    uint32_t speed = 500000;
    uint8_t mode = 0;
    //uint8_t mode = SPI_CPHA;
    //uint8_t mode = SPI_CPOL;
    int i;
    
    // allocate the object
    if((s = calloc(1, sizeof(pdaq))) == NULL)
    {
        qprintf(s, "pdaq_init: Couldn't allocate pdaq object\n");
        goto fail0;
    }
    
    // set verbose level and other info
    s->verbose = verbose;
    s->prom_addr = prom_addr;
    s->pport_addr = pport_addr;
    
    // open I2C bus
    sprintf(filename, "/dev/i2c-%d", i2c_bus);
    s->i2c_file = open(filename, O_RDWR);

    if(s->i2c_file < 0)
    {
        qprintf(s, "pdaq_init: Couldn't open I2C device %s\n", filename);
        goto fail1;
    }
    else
    {
        qprintf(s, "pdaq_init: opened I2C device %s\n", filename);
    }   
    
    // Check for the PandaDAQ ID PROM
    if (pdaq_i2c_get_prom(s, buf, 0, 6) != 6)
    {
        qprintf(s, "pdaq_init: Failed to read IDPROM - giving up\n");
        goto fail2;
    }
    else
    {
        for (i = 0 ; i < 6 ; i++)
        {
            qprintf(s, "pdaq_init: ID[%1d] = 0x%02X\n", i, buf[i]);
            if(buf[i] != idprom[i])                                          
            {
                qprintf(s, "pdaq_init: IDPROM mismatch - giving up\n");
                goto fail2;
            }
        }
        
        qprintf(s, "pdaq_init: found IDPROM\n");
    }
    
    // diagnostic - check status of port expander
    qprintf(s, "pdaq_init: parport reads 0x%02X\n", pdaq_i2c_parport_rd_byte(s, 0x00));

    // Open the SPI port
    sprintf(filename, "/dev/spidev%d.%d", spi_bus, spi_addr);
    s->spi_file = open(filename, O_RDWR);
    
    if(s->spi_file < 0)
    {
        qprintf(s, "pdaq_init: Couldn't open spi device %s\n", filename);
        goto fail2;
    }
    else
    {
        qprintf(s, "pdaq_init: opened spi device %s\n", filename);
    }

    if(ioctl(s->spi_file, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1)
    {
        qprintf(s, "pdaq_init: Couldn't set SPI clock to %d Hz\n", speed);
        goto fail2;
    }
    else
    {
        qprintf(s, "pdaq_init: Set SPI clock to %d Hz\n", speed);
    }
    
    if(ioctl(s->spi_file, SPI_IOC_WR_MODE, &mode) == -1)
    {
        qprintf(s, "pdaq_init: Couldn't set SPI mode\n");
        goto fail2;
    }
    else
    {
        qprintf(s, "pdaq_init: Set SPI mode\n");
    }
    
    // Initialize parport GPIOs.
    // Set POWER_EN = 1, FPGA_PROG = 1 (won't happen until direction reg is written)
    pdaq_i2c_parport_wr_byte(s, 0x01, 0x21);
    // Set FPGA_AWAKE, FPGA_DONE, and FPGA_INIT as inputs; all others as outputs
    pdaq_i2c_parport_wr_byte(s, 0x03, 0xd0);
    
    qprintf(s, "pdaq_init: Parport initialized; red LED on\n");
    qprintf(s, "pdaq_init: Wait one second for power-supply stabilization\n");
    sleep(1);
            
    // success
    return s;

    // failure modes
//fail3:
//  close(s->spi_file);     // close the SPI device
fail2:
    close(s->i2c_file);     // close the I2C device
fail1:
    free(s);                // free the structure
fail0:
    return NULL;
}

// Send a bitstream to the FPGA
int pdaq_cfg(pdaq *s, char *bitfile)
{
    FILE *fd;
    int read;
    int j, d, header, len;
    long ct, n;
    char *cp;
    unsigned char byte, rxbyte, dummybuf[READBUFSIZE];
    char readbuf[READBUFSIZE];

    // open file or return error*/
    if(!(fd = fopen(bitfile, "r")))
    {
        qprintf(s, "pdaq_cfg: open file %s failed\n\r", bitfile);
        return 1;
    }
    else
    {
        qprintf(s, "pdaq_cfg: found bitstream file %s\n\r", bitfile);
    }

    // Read file & send bitstream via SPI1
    ct = 0;
    header = 1;
    qprintf(s, "pdaq_cfg: parsing bitstream\n\r");
    while( (read=fread((char*)readbuf, sizeof(char), READBUFSIZE,fd)) > 0 )
    {
        // init pointer to keep track
        cp = readbuf;
        
        // are we parsing the header?
        if(header)
        {
            // check / skip .bit header
            for(j=0;j<13;j++)
            {
                if(bit_hdr[j] != *cp++)
                {
                    qprintf(s, "pdaq_cfg: .bit header mismatch\n\r");
                    fclose(fd);
                    return 1;
                }
            }
            qprintf(s, "pdaq_cfg: found header\n\r");
        
            // Skip File header chunks
            for(j=0;j<4;j++)
            {
                // get 1 byte chunk desginator (a,b,c,d)
                d = *cp++;
                
                // compute chunksize
                n = *cp++;
                n <<= 8;
                n += *cp++;
            
                // print chunk
                qprintf(s, "pdaq_cfg: chunk %c length %ld %s %s\n\r", d, n, bit_hdr_strings[j], cp);
            
                // Check device type
                if(j==1)
                {
                    if(strcmp(cp, deviceid))
                        qprintf(s, "pdaq_cfg: Device != %s\n\r", deviceid);
                    else
                        qprintf(s, "pdaq_cfg: Device == %s\n\r", deviceid);
                }
            
                // skip chunk
                cp += n;
            }
    
            // Skip final chunk designator
            cp++;
        
            // compute config data size - modified for 16-bit int & char
            n = *cp++;
            n <<= 8;
            n += *cp++;
            n <<= 8;
            n += *cp++;
            n <<= 8;
            n += *cp++;
            qprintf(s, "pdaq_cfg: config size = %ld\n\r", n);
            
            // no longer processing header
            header = 0;


            // pulse PROG_B low min 500 ns
            pdaq_i2c_parport_wr(s,5,0);
            usleep(1);          // wait a bit
    
            // Wait for INIT low
            qprintf(s, "pdaq_cfg: PROG low; waiting for INIT low\n\r");
            while(pdaq_i2c_parport_rd(s,7)==1)
            {
                asm volatile ("nop");   //"nop" means no-operation.  We don't want to do anything during the delay
            }
    
            // Release PROG
            pdaq_i2c_parport_wr(s,5,1);
    
            // Wait for INIT high
            qprintf(s, "pdaq_cfg: PROG high; waiting for INIT high\n\r");
            while(pdaq_i2c_parport_rd(s,7)==0)
            {
                asm volatile ("nop");   //"nop" means no-operation.  We don't want to do anything during the delay
            }

            // wait 5us
            usleep(5);
            
            qprintf(s, "pdaq_cfg: Sending bitstream\n\r");
        }

        // Send bitstream
        len = read - (cp - readbuf);
        pdaq_spi_txrx(s, (unsigned char *)cp, dummybuf, len);
        ct += len;
        
        // diagnostic to track buffers
        qprintf(s, ".");
        if(s->verbose)
            fflush(stdout);
        
        // Check INIT - if low then fail
        if(pdaq_i2c_parport_rd(s,7)==0)
        {
            qprintf(s, "\n\rpdaq_cfg: INIT low during bitstream send\n\r");
            fclose(fd);
            return 1;
        }
    }
    
    // close file
    qprintf(s, "\n\rpdaq_cfg: sent %ld of %ld bytes\n\r", ct, n);
    qprintf(s, "pdaq_cfg: bitstream sent, closing file\n\r");
    fclose(fd);
    
    // send dummy data while waiting for DONE or !INIT
    qprintf(s, "pdaq_cfg: sending dummy clocks, waiting for DONE or fail\n\r");
    byte = 0xFF;
    ct = 0;
    while((pdaq_i2c_parport_rd(s,6)==0) && (pdaq_i2c_parport_rd(s,7)==1))
    {
        // Dummy - all ones
        pdaq_spi_txrx(s, &byte, &rxbyte, 1);
        ct++;
    }
    qprintf(s, "pdaq_cfg: %d dummy clocks sent\n\r", ct*8);
    
    // return status
    if(pdaq_i2c_parport_rd(s,6)==0)
    {
        qprintf(s, "pdaq_cfg: cfg failed - DONE not high\n\r");
        return 1;   // Done = 0 - error
    }
    else    
    {
        qprintf(s, "pdaq_cfg: success\n\r");
        return 0;   // Done = 1 - OK
    }
}

// Clean shutdown of our FPGA interface
void pdaq_delete(pdaq *s)
{
    close(s->spi_file);     // close the SPI device
    close(s->i2c_file);     // close the I2C device
    free(s);            // free the structure
}
