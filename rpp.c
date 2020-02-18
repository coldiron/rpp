/*
 * Raspberry Pi PIC Programmer using GPIO connector
 * Copyright 2012 Giorgio Vazzana
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Compile: gcc -Wall -O rpp.c -o rpp */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

/* GPIO registers address */
#define BCM2708_PERI_BASE  0x20000000
#define GPIO_BASE          (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define BLOCK_SIZE         (256)

/* GPIO setup macros. Always use GPIO_IN(x) before using GPIO_OUT(x) or GPIO_ALT(x,y) */
#define GPIO_IN(g)    *(gpio+((g)/10))   &= ~(7<<(((g)%10)*3))
#define GPIO_OUT(g)   *(gpio+((g)/10))   |=  (1<<(((g)%10)*3))
#define GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET(g)   *(gpio+7)  = 1<<(g)  /* sets   bit which are 1, ignores bit which are 0 */
#define GPIO_CLR(g)   *(gpio+10) = 1<<(g)  /* clears bit which are 1, ignores bit which are 0 */
#define GPIO_LEV(g)  (*(gpio+13) >> (g)) & 0x00000001

/* GPIO <-> PIC connections */
#define PIC_CLK    4	/* Output */
#define PIC_DATA   7	/* Output */
#define PIC_DATAIN 8	/* Input  */
#define PIC_MCLR   9	/* Output */
#define DELAY      40	/* microseconds */

/* 8K program memory + configuration memory + eeprom data */
#define PICMEMSIZE (0x2100 + 0xFF)

struct picmemory {
	uint16_t  program_memory_used_cells;
	uint16_t  program_memory_max_used_address;

	uint8_t   has_configuration_data;
	uint8_t   has_eeprom_data;

	uint16_t *data;		/* 14-bit data */
	uint8_t  *filled;	/* 1 if this cell is used */
};

struct picmicro {
	uint16_t device_id;
	char     name[16];
	size_t   program_memory_size;
	size_t   data_memory_size;

	int program_cycle_time;		/* in microseconds */
	int eeprom_program_cycle_time;
	int erase_and_program_cycle_time;
	int bulk_erase_cycle_time;

	uint8_t load_configuration_cmd;
	uint8_t load_data_for_program_memory_cmd;
	uint8_t load_data_for_data_memory_cmd;
	uint8_t read_data_from_program_memory_cmd;
	uint8_t read_data_from_data_memory_cmd;
	uint8_t increment_address_cmd;
	uint8_t begin_erase_programming_cycle_cmd;
	uint8_t begin_programming_only_cycle_cmd;
	uint8_t bulk_erase_program_memory_cmd;
	uint8_t bulk_erase_data_memory_cmd;
};

//#define PIC16F84   0xFFFF	/* cannot be autodetected */
#define PIC16F84A  0x0560
#define PIC16F627A 0x1040
#define PIC16F628A 0x1060
#define PIC16F648A 0x1100
#define PIC16F870  0x0D00
#define PIC16F871  0x0D20
#define PIC16F872  0x08E0
#define PIC16F873  0x0960
#define PIC16F874  0x0920
#define PIC16F876  0x09E0
#define PIC16F877  0x09A0

//const struct picmicro pic16f84   = {PIC16F84,   "pic16f84",    0x400,  64, 20000, 20000, 20000, 10000, 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08, 0xFF, 0x09, 0x0B};
const struct picmicro pic16f84a  = {PIC16F84A,  "pic16f84a",   0x400,  64,  4000,  4000,  8000, 10000, 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08, 0x18, 0x09, 0x0B};
const struct picmicro pic16f627a = {PIC16F627A, "pic16f627a",  0x400, 128,  4000,  6000,  4000,  6000, 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0xFF, 0x08, 0x09, 0x0B};
const struct picmicro pic16f628a = {PIC16F628A, "pic16f628a",  0x800, 128,  4000,  6000,  4000,  6000, 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0xFF, 0x08, 0x09, 0x0B};
const struct picmicro pic16f648a = {PIC16F648A, "pic16f648a", 0x1000, 256,  4000,  6000,  4000,  6000, 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0xFF, 0x08, 0x09, 0x0B};
const struct picmicro pic16f870  = {PIC16F870,  "pic16f870",   0x800,  64,  4000,  4000,  8000,  8000, 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08, 0x18, 0x01, 0x07};
const struct picmicro pic16f871  = {PIC16F871,  "pic16f871",   0x800,  64,  4000,  4000,  8000,  8000, 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08, 0x18, 0x01, 0x07};
const struct picmicro pic16f872  = {PIC16F872,  "pic16f872",   0x800,  64,  4000,  4000,  8000,  8000, 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08, 0x18, 0x01, 0x07};
const struct picmicro pic16f873  = {PIC16F873,  "pic16f873",  0x1000, 128,  4000,  4000,  8000,  8000, 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08, 0x18, 0x01, 0x07};
const struct picmicro pic16f874  = {PIC16F874,  "pic16f874",  0x1000, 128,  4000,  4000,  8000,  8000, 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08, 0x18, 0x01, 0x07};
const struct picmicro pic16f876  = {PIC16F876,  "pic16f876",  0x2000, 256,  4000,  4000,  8000,  8000, 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08, 0x18, 0x01, 0x07};
const struct picmicro pic16f877  = {PIC16F877,  "pic16f877",  0x2000, 256,  4000,  4000,  8000,  8000, 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08, 0x18, 0x01, 0x07};

const struct picmicro *piclist[] = {&pic16f84a, &pic16f627a, &pic16f628a, &pic16f648a, &pic16f870, &pic16f871, &pic16f872, &pic16f873, &pic16f874, &pic16f876, &pic16f877, NULL};

int                mem_fd;
void              *gpio_map;
volatile uint32_t *gpio;


/* Set up a memory regions to access GPIO */
void setup_io()
{
	/* open /dev/mem */
	mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
	if (mem_fd == -1) {
		perror("Cannot open /dev/mem");
		exit(1);
	}

	/* mmap GPIO */
	gpio_map = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIO_BASE);
	if (gpio_map == MAP_FAILED) {
		perror("mmap() failed");
		exit(1);
	}

	/* Always use volatile pointer! */
	gpio = (volatile uint32_t *)gpio_map;

	/* Configure GPIOs */
	GPIO_IN(PIC_CLK); /* must use GPIO_IN before we can use GPIO_OUT */
	GPIO_OUT(PIC_CLK);

	GPIO_IN(PIC_DATA);
	GPIO_OUT(PIC_DATA);

	GPIO_IN(PIC_DATAIN);

	GPIO_IN(PIC_MCLR);
	GPIO_OUT(PIC_MCLR);

	GPIO_CLR(PIC_CLK);
	GPIO_CLR(PIC_DATA);
	GPIO_CLR(PIC_DATAIN);
	GPIO_CLR(PIC_MCLR);
	usleep(DELAY);
}

/* Release GPIO memory region */
void close_io()
{
	int ret;

	/* munmap GPIO */
	ret = munmap(gpio_map, BLOCK_SIZE);
	if (ret == -1) {
		perror("munmap() failed");
		exit(1);
	}

	/* close /dev/mem */
	ret = close(mem_fd);
	if (ret == -1) {
		perror("Cannot close /dev/mem");
		exit(1);
	}
}

void free_picmemory(struct picmemory **ppm)
{
	free((*ppm)->data);
	free((*ppm)->filled);
	free(*ppm);
}

/* Read a file in Intel HEX 16-bit format and return a pointer to the picmemory
   struct on success, or NULL on error */
struct picmemory *read_inhx16(char *infile, int debug)
{
	FILE *fp;
	int linenum;
	char line[256], *ptr;
	size_t linelen;
	int nread;

	uint16_t i;
	uint8_t  start_code;
	uint8_t  byte_count;
	uint16_t address;
	uint8_t  record_type;
	uint16_t data;
	uint8_t  checksum_calculated;
	uint8_t  checksum_read;

	struct picmemory *pm;

	fp = fopen(infile, "r");
	if (fp == NULL) {
		fprintf(stderr, "Error: cannot open source file %s.\n", infile);
		return NULL;
	}

	pm = calloc(1, sizeof(*pm));
	if (pm) {
		pm->data   = calloc(PICMEMSIZE, sizeof(*pm->data));
		pm->filled = calloc(PICMEMSIZE, sizeof(*pm->filled));
	}
	if (!pm || !pm->data || !pm->filled) {
		fprintf(stderr, "Error: calloc() failed.\n");
		return NULL;
	}

	fprintf(stderr, "Reading hex file...\n");

	linenum = 0;
	while (1) {
		ptr = fgets(line, 256, fp);

		if (ptr != NULL) {
			linenum++;
			linelen = strlen(line);
			if (debug) {
				fprintf(stderr, "  line %d (%zd bytes): '", linenum, linelen);
				for (i = 0; i < linelen; i++) {
					if (line[i] == '\n')
						fprintf(stderr, "\\n");
					else if (line[i] == '\r')
						fprintf(stderr, "\\r");
					else
						fprintf(stderr, "%c", line[i]);
				}
			fprintf(stderr, "'\n");
			}

			start_code = line[0];
			if (start_code != ':') {
				fprintf(stderr, "Error: invalid start code.\n");
				free_picmemory(&pm);
				return NULL;
			}

			nread = sscanf(&line[1], "%2hhx", &byte_count);
			if (nread != 1) {
				fprintf(stderr, "Error: cannot read byte count.\n");
				free_picmemory(&pm);
				return NULL;
			}
			if (debug)
				fprintf(stderr, "  byte_count  = 0x%02X\n", byte_count);


			nread = sscanf(&line[3], "%4hx", &address);
			if (nread != 1) {
				fprintf(stderr, "Error: cannot read address.\n");
				free_picmemory(&pm);
				return NULL;
			}
			if (debug)
				fprintf(stderr, "  address     = 0x%04X\n", address);

			nread = sscanf(&line[7], "%2hhx", &record_type);
			if (nread != 1) {
				fprintf(stderr, "Error: cannot read record type.\n");
				free_picmemory(&pm);
				return NULL;
			}
			if (debug)
				fprintf(stderr, "  record_type = 0x%02X (%s)\n", record_type, record_type == 0 ? "data" : (record_type == 1 ? "EOF" : "Unknown"));
			if (record_type != 0 && record_type != 1) {
				fprintf(stderr, "Error: unknown record type.\n");
				free_picmemory(&pm);
				return NULL;
			}

			checksum_calculated  = byte_count;
			checksum_calculated += (address >> 8) & 0xFF;
			checksum_calculated += address & 0xFF;
			checksum_calculated += record_type;

			for (i = 0; i < byte_count; i++) {
				nread = sscanf(&line[9+4*i], "%4hx", &data);
				if (nread != 1) {
					fprintf(stderr, "Error: cannot read data.\n");
					free_picmemory(&pm);
					return NULL;
				}
				if (debug)
					fprintf(stderr, "  data        = 0x%04X\n", data);
				checksum_calculated += (data >> 8) & 0xFF;
				checksum_calculated += data & 0xFF;

				if (address + i < 0x2000) {
					pm->program_memory_used_cells       += 1;
					pm->program_memory_max_used_address  = address + i;
				} else if (0x2000 <= address + i && address + i < 0x2008)
					pm->has_configuration_data = 1;
				else if (address + i >= 0x2100)
					pm->has_eeprom_data = 1;

				pm->data[address + i]   = data;
				pm->filled[address + i] = 1;
			}

			checksum_calculated = (checksum_calculated ^ 0xFF) + 1;

			nread = sscanf(&line[9+4*i], "%2hhx", &checksum_read);
			if (nread != 1) {
				fprintf(stderr, "Error: cannot read checksum.\n");
				free_picmemory(&pm);
				return NULL;
			}
			if (debug)
				fprintf(stderr, "  checksum    = 0x%02X\n", checksum_read);

			if (checksum_calculated != checksum_read) {
				fprintf(stderr, "Error: checksum does not match.\n");
				free_picmemory(&pm);
				return NULL;
			}

			if (debug)
				fprintf(stderr, "\n");

			if (record_type == 1)
				break;
		} else {
			fprintf(stderr, "Error: unexpected EOF.\n");
			free_picmemory(&pm);
			return NULL;
		}
	}

	fclose(fp);

	return pm;
}

/* Write the filled cells in struct picmemory to a Intel HEX 16-bit file */
void write_inhx16(struct picmemory *pm, char *outfile)
{
	FILE *fp;
	uint16_t i, j, k, start, stop;
	uint8_t  byte_count;
	uint16_t address;
	uint8_t  record_type;
	uint16_t data;
	uint8_t  checksum_calculated;

	fp = fopen(outfile, "w");
	if (fp == NULL) {
		fprintf(stderr, "Error: cannot open destination file %s.\n", outfile);
		return;
	}

	fprintf(stderr, "Writing hex file...\n");

	for (i = 0; i < PICMEMSIZE; i += 8) {
		j = 0;
		while (j != 8) {
			for ( ; j < 8; j++) {
				if (pm->filled[i+j])
					break;
			}
			start = j;

			for ( ; j < 8; j++) {
				if (!pm->filled[i+j])
					break;
			}
			stop = j;

			byte_count  = stop - start;
			if (byte_count > 0) {
				address     = i + start;
				record_type = 0x00;
				fprintf(fp, ":%02X%04X%02X", byte_count, address, record_type);

				checksum_calculated  = byte_count;
				checksum_calculated += (address >> 8) & 0xFF;
				checksum_calculated += address & 0xFF;
				checksum_calculated += record_type;

				for (k = start; k < stop; k++) {
					data = pm->data[i+k];
					fprintf(fp, "%04X", data);
					checksum_calculated += (data >> 8) & 0xFF;
					checksum_calculated += data & 0xFF;
				}

				checksum_calculated = (checksum_calculated ^ 0xFF) + 1;
				fprintf(fp, "%02X\n", checksum_calculated);
			}
		}
	}
	fprintf(fp, ":00000001FF\n");
	fclose(fp);
}

/* Send a 6-bit command to the PIC */
void pic_send_cmd(uint8_t cmd)
{
	int i;

	for (i = 0; i < 6; i++) {
		GPIO_SET(PIC_CLK);
		if ((cmd >> i) & 0x01)
			GPIO_SET(PIC_DATA);
		else
			GPIO_CLR(PIC_DATA);
		usleep(DELAY);	/* Setup time */
		GPIO_CLR(PIC_CLK);
		usleep(DELAY);	/* Hold time */
	}
	GPIO_CLR(PIC_DATA);
	usleep(DELAY);
}

/* Read 14-bit data from the PIC (start bit, 14-bit data, stop bit. lsb first) */
uint16_t pic_read_data(void)
{
	int i;
	uint16_t data = 0;

	for (i = 0; i < 16; i++) {
		GPIO_SET(PIC_CLK);
		usleep(DELAY);	/* Wait for data to be valid */
		data |= (GPIO_LEV(PIC_DATAIN)) << i;
		GPIO_CLR(PIC_CLK);
		usleep(DELAY);
	}
	data = (data >> 1) & 0x3FFF;

	return data;
}

/* Load 14-bit data to the PIC (start bit, 14-bit data, stop bit. lsb first) */
void pic_load_data(uint16_t data)
{
	int i;

	/* Insert start and stop bit (0s) */
	data = (data << 1) & 0x7FFE;
	for (i = 0; i < 16; i++) {
		GPIO_SET(PIC_CLK);
		if ((data >> i) & 0x01)
			GPIO_SET(PIC_DATA);
		else
			GPIO_CLR(PIC_DATA);
		usleep(DELAY);	/* Setup time */
		GPIO_CLR(PIC_CLK);
		usleep(DELAY);	/* Hold time */
	}
	GPIO_CLR(PIC_DATA);
	usleep(DELAY);
}

/* Read PIC device id word, located at 0x2006. Used to autodetect the chip */
uint16_t pic_read_device_id_word(const struct picmicro *pic)
{
	int i;
	uint16_t data;

	GPIO_SET(PIC_MCLR);	/* Enter Program/Verify Mode */
	usleep(DELAY);

	pic_send_cmd(pic->load_configuration_cmd);
	pic_load_data(0x0000);
	for (i = 0; i < 6; i++)
		pic_send_cmd(pic->increment_address_cmd);
	pic_send_cmd(pic->read_data_from_program_memory_cmd);
	data = pic_read_data();

	GPIO_CLR(PIC_MCLR);	/* Exit Program/Verify Mode */
	usleep(DELAY);

	return data;
}

/* Bulk erase the chip */
void pic_bulk_erase(const struct picmicro *pic, int debug)
{
	fprintf(stderr, "Bulk erasing chip...\n");

	if (pic->device_id == PIC16F870 || pic->device_id == PIC16F871 ||
	    pic->device_id == PIC16F872 || pic->device_id == PIC16F873 ||
	    pic->device_id == PIC16F874 || pic->device_id == PIC16F876 ||
	    pic->device_id == PIC16F877) {
		if (debug)
			fprintf(stderr, "  Erasing program memory...\n");
		GPIO_SET(PIC_MCLR);
		usleep(DELAY);
		pic_send_cmd(pic->load_data_for_program_memory_cmd);	/* Clear program memory only */
		pic_load_data(0x3FFF);
		pic_send_cmd(pic->bulk_erase_program_memory_cmd);
		pic_send_cmd(pic->bulk_erase_data_memory_cmd);
		pic_send_cmd(pic->begin_erase_programming_cycle_cmd);
		usleep(pic->bulk_erase_cycle_time);
		pic_send_cmd(pic->bulk_erase_program_memory_cmd);
		pic_send_cmd(pic->bulk_erase_data_memory_cmd);
		GPIO_CLR(PIC_MCLR);
		usleep(DELAY);

		if (debug)
			fprintf(stderr, "  Erasing data memory...\n");
		GPIO_SET(PIC_MCLR);
		usleep(DELAY);
		pic_send_cmd(pic->load_data_for_data_memory_cmd);	/* Clear data memory */
		pic_load_data(0x3FFF);
		pic_send_cmd(pic->bulk_erase_program_memory_cmd);
		pic_send_cmd(pic->bulk_erase_data_memory_cmd);
		pic_send_cmd(pic->begin_erase_programming_cycle_cmd);
		usleep(pic->bulk_erase_cycle_time);
		pic_send_cmd(pic->bulk_erase_program_memory_cmd);
		pic_send_cmd(pic->bulk_erase_data_memory_cmd);
		GPIO_CLR(PIC_MCLR);
		usleep(DELAY);

		return;
	}

	if (debug)
		fprintf(stderr, "  Erasing program and configuration memory...\n");
	GPIO_SET(PIC_MCLR);
	usleep(DELAY);
	pic_send_cmd(pic->load_configuration_cmd);	/* Clear both program and configuration memory */
	pic_load_data(0x3FFF);
#if 0
	pic_send_cmd(pic->load_data_for_program_memory_cmd); /* Clear program memory only */
	pic_load_data(0x3FFF);
#endif
	pic_send_cmd(pic->bulk_erase_program_memory_cmd);
	pic_send_cmd(pic->begin_programming_only_cycle_cmd);
	usleep(pic->bulk_erase_cycle_time);
	GPIO_CLR(PIC_MCLR);
	usleep(DELAY);

	if (debug)
		fprintf(stderr, "  Erasing data memory...\n");
	GPIO_SET(PIC_MCLR);
	usleep(DELAY);
	pic_send_cmd(pic->load_data_for_data_memory_cmd);	/* Clear data memory */
	pic_load_data(0x3FFF);
	pic_send_cmd(pic->bulk_erase_data_memory_cmd);
	pic_send_cmd(pic->begin_programming_only_cycle_cmd);
	usleep(pic->bulk_erase_cycle_time);
	GPIO_CLR(PIC_MCLR);
	usleep(DELAY);
}

/* Read program memory, configuration memory and data memory of the PIC
   and write the contents to a .hex file */
void pic_read(const struct picmicro *pic, char *outfile, int skipones, int debug)
{
	struct picmemory *pm;
	uint16_t addr, data;

	pm = calloc(1, sizeof(*pm));
	if (pm) {
		pm->data   = calloc(PICMEMSIZE, sizeof(*pm->data));
		pm->filled = calloc(PICMEMSIZE, sizeof(*pm->filled));
	}
	if (!pm || !pm->data || !pm->filled) {
		fprintf(stderr, "Error: calloc() failed.\n");
		return;
	}

	fprintf(stderr, "Reading chip...\n");

	GPIO_SET(PIC_MCLR);
	usleep(DELAY);
	/* Read Program Memory */
	if (debug)
		fprintf(stderr, "  Program memory:\n");
	for (addr = 0; addr < pic->program_memory_size; addr++) {
		pic_send_cmd(pic->read_data_from_program_memory_cmd);
		data = pic_read_data();
		if (debug)
			fprintf(stderr, "  addr = 0x%04X  data = 0x%04X\n", addr, data);

		if (!skipones || data != 0x3FFF) {
			pm->program_memory_used_cells       += 1;
			pm->program_memory_max_used_address  = addr;
			pm->data[addr]                       = data;
			pm->filled[addr]                     = 1;
		}
		pic_send_cmd(pic->increment_address_cmd);
	}

	/* Read Configuration Memory */
	if (debug)
		fprintf(stderr, "  Configuration memory:\n");
	pic_send_cmd(pic->load_configuration_cmd);
	pic_load_data(0x0000);
	for (addr = 0x2000; addr < 0x2008; addr++) {
		if (addr <= 0x2003 /*|| addr == 0x2006*/ || addr == 0x2007) {
			pic_send_cmd(pic->read_data_from_program_memory_cmd);
			data = pic_read_data();
			if (debug)
				fprintf(stderr, "  addr = 0x%04X  data = 0x%04X\n", addr, data);

			if (!skipones || (addr <= 0x2003 && data != 0x3FFF) || addr == 0x2007) {
				pm->has_configuration_data = 1;
				pm->data[addr]             = data;
				pm->filled[addr]           = 1;
			}
		}
		pic_send_cmd(pic->increment_address_cmd);
	}

	/* Read Data Memory    (but first reset PC, at most only the 8 least
	   significant bits of PC are decoded when reading data memory) */
	if (debug)
		fprintf(stderr, "  Data memory:\n");
	pic_send_cmd(pic->load_configuration_cmd);
	pic_load_data(0x0000);
	for (addr = 0x2100; addr < 0x2100 + pic->data_memory_size; addr++) {
		pic_send_cmd(pic->read_data_from_data_memory_cmd);
		data = pic_read_data();
		if (debug)
			fprintf(stderr, "  addr = 0x%04X  data = 0x%04X\n", addr, data);

		data &= 0x00FF;
		if (!skipones || data != 0x00FF) {
			pm->has_eeprom_data = 1;
			pm->data[addr]      = data;
			pm->filled[addr]    = 1;
		}
		pic_send_cmd(pic->increment_address_cmd);
	}
	GPIO_CLR(PIC_MCLR);
	usleep(DELAY);

	write_inhx16(pm, outfile);
	free_picmemory(&pm);
}

/* Bulk erase the chip, and then write contents of the .hex file to the PIC */
void pic_write(const struct picmicro *pic, char *infile, int debug)
{
	int error = 0;
	uint16_t addr, data;
	struct picmemory *pm;

	pm = read_inhx16(infile, debug);
	if (!pm)
		return;

	/* Bulk erase the chip first */
	pic_bulk_erase(pic, debug);

	fprintf(stderr, "Writing chip...\n");

	/* Write Program Memory */
	if (debug)
		fprintf(stderr, "  Program memory:\n");
	GPIO_SET(PIC_MCLR);
	usleep(DELAY);
	for (addr = 0; addr <= pm->program_memory_max_used_address; addr++) {
		if (pm->filled[addr]) {
			pic_send_cmd(pic->load_data_for_program_memory_cmd);
			pic_load_data(pm->data[addr]);
			pic_send_cmd(pic->begin_programming_only_cycle_cmd);
			usleep(pic->program_cycle_time);

			pic_send_cmd(pic->read_data_from_program_memory_cmd);
			data = pic_read_data();

			if (debug)
				fprintf(stderr, "  addr = 0x%04X  written = 0x%04X  read = 0x%04X\n", addr, pm->data[addr], data);

			if (pm->data[addr] != data) {
				fprintf(stderr, "Error: addr = 0x%04X, written = 0x%04X, read = 0x%04X\n", addr, pm->data[addr], data);
				error = 1;
				break;
			}
		}
		pic_send_cmd(pic->increment_address_cmd);
	}
	GPIO_CLR(PIC_MCLR);
	usleep(DELAY);

	if (error)
		return;

	/* Write Configuration Memory */
	if (pm->has_configuration_data) {
		if (debug)
			fprintf(stderr, "  Configuration memory:\n");
		GPIO_SET(PIC_MCLR);
		usleep(DELAY);
		pic_send_cmd(pic->load_configuration_cmd);
		pic_load_data(pm->data[0x2000]);
		for (addr = 0x2000; addr < 0x2008; addr++) {
			if ((addr <= 0x2003 || addr == 0x2007) && pm->filled[addr]) {
				pic_send_cmd(pic->load_data_for_program_memory_cmd);
				pic_load_data(pm->data[addr]);

				if ((pic->device_id == PIC16F84A && addr == 0x2007) ||
				    pic->device_id == PIC16F870 || pic->device_id == PIC16F871 ||
				    pic->device_id == PIC16F872 || pic->device_id == PIC16F873 ||
				    pic->device_id == PIC16F874 || pic->device_id == PIC16F876 ||
				    pic->device_id == PIC16F877) {
					pic_send_cmd(pic->begin_erase_programming_cycle_cmd);
					usleep(pic->erase_and_program_cycle_time);
				} else {
					pic_send_cmd(pic->begin_programming_only_cycle_cmd);
					usleep(pic->program_cycle_time);
				}

				pic_send_cmd(pic->read_data_from_program_memory_cmd);
				data = pic_read_data();

				if (debug)
					fprintf(stderr, "  addr = 0x%04X  written = 0x%04X  read = 0x%04X\n", addr, pm->data[addr], data);

				if (pm->data[addr] != data) {
					fprintf(stderr, "Error: addr = 0x%04X, written = 0x%04X, read = 0x%04X\n", addr, pm->data[addr], data);
					error = 1;
					break;
				}
			}
			pic_send_cmd(pic->increment_address_cmd);
		}
		GPIO_CLR(PIC_MCLR);
		usleep(DELAY);
	}
	if (error)
		return;

	/* Write Data Memory */
	if (pm->has_eeprom_data) {
		if (debug)
			fprintf(stderr, "  Data memory:\n");
		GPIO_SET(PIC_MCLR);
		usleep(DELAY);
		for (addr = 0x2100; addr < 0x2100 + pic->data_memory_size; addr++) {
			if (pm->filled[addr]) {
				pic_send_cmd(pic->load_data_for_data_memory_cmd);
				pic_load_data(pm->data[addr]);
				pic_send_cmd(pic->begin_programming_only_cycle_cmd);
				usleep(pic->eeprom_program_cycle_time);

				pic_send_cmd(pic->read_data_from_data_memory_cmd);
				data = pic_read_data();

				if (debug)
					fprintf(stderr, "  addr = 0x%04X  written = 0x%04X  read = 0x%04X\n", addr, pm->data[addr], data);

				if (pm->data[addr] != (data & 0x00FF)) {
					fprintf(stderr, "Error: addr = 0x%04X, written = 0x%04X, read = 0x%04X\n", addr, pm->data[addr], data);
					error = 1;
					break;
				}
			}
			pic_send_cmd(pic->increment_address_cmd);
		}
		GPIO_CLR(PIC_MCLR);
		usleep(DELAY);
	}
}

void usage(void)
{
	const struct picmicro **ppic;
	uint8_t comma = 0;

	fprintf(stderr,
"Usage: rpp [options]\n"
"       -h          print help\n"
"       -D          turn debug on\n"
"       -i file     input file\n"
"       -o file     output file (ofile.hex)\n"
"       -r          read chip\n"
"       -w          bulk erase and write chip\n"
"       -e          bulk erase chip\n"
"       -s          skip all-ones memory locations\n"
"\n"
"Supported PICs:"
	);
	for (ppic = piclist; *ppic; ppic++) {
		fprintf(stderr, "%s %s", comma ? "," : "", (*ppic)->name);
		comma = 1;
	}
	fprintf(stderr, "\n");
}

int main(int argc, char *argv[])
{
	int opt, debug = 0, skipones = 0, function = 0;
	char *infile  = NULL;
	char *outfile = "ofile.hex";
	uint16_t device_id;
	const struct picmicro *pic, **ppic;

	fprintf(stderr, "Raspberry Pi PIC Programmer, v0.1\n\n");

	while ((opt = getopt(argc, argv, "hDi:o:rwes")) != -1) {
		switch (opt) {
		case 'h':
			usage();
			exit(0);
			break;
		case 'D':
			debug = 1;
			break;
		case 'i':
			infile = optarg;
			break;
		case 'o':
			outfile = optarg;
			break;
		case 'r':
			function |= 0x01;
			break;
		case 'w':
			function |= 0x02;
			break;
		case 'e':
			function |= 0x04;
			break;
		case 's':
			skipones = 1;
			break;
		default:
			fprintf(stderr, "\n");
			usage();
			exit(1);
		}
	}
#if 0
	fprintf(stderr, "debug = %d, infile = %s, outfile = %s, function = 0x%02x, skipones = %d\n",
	        debug, infile == NULL ? "NULL" : infile, outfile, function, skipones);
#endif

	if (function == 0x02 && !infile) {
		fprintf(stderr, "Please specify an input file with -i option.\n");
		exit(1);
	}

	/* Setup gpio pointer for direct register access */
	setup_io();

	/* Read PIC device id word */
	pic = &pic16f84a;
	device_id = pic_read_device_id_word(pic);
	fprintf(stderr, "device_id = 0x%04x\n", device_id);
	for (ppic = piclist; *ppic; ppic++) {
		if ((*ppic)->device_id == (device_id & 0xFFE0)) {
			pic = *ppic;
			break;
		}
	}
	if (*ppic == NULL) {
		fprintf(stderr, "Error: unknown device or programmer not connected.\n");
		exit(1);
	} else
		fprintf(stderr, "%s detected, revision 0x%02x\n", pic->name, device_id & 0x001F);

	switch (function) {
	case 0x00:
		/* no function selected, exit */
		break;
	case 0x01:
		pic_read(pic, outfile, skipones, debug);
		break;
	case 0x02:
		pic_write(pic, infile, debug);
		break;
	case 0x04:
		pic_bulk_erase(pic, debug);
		break;
	default:
		fprintf(stderr, "\nPlease select only one option in -r, -w, -e.\n");
	};

	close_io();

	return 0;
}
