/*
 *  iceprog -- simple programming tool for FTDI-based Lattice iCE programmers
 *
 *  Copyright (C) 2015  Clifford Wolf <clifford@clifford.at>
 *  
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *  Relevant Documents:
 *  -------------------
 *  http://www.latticesemi.com/~/media/Documents/UserManuals/EI/icestickusermanual.pdf
 *  http://www.micron.com/~/media/documents/products/data-sheet/nor-flash/serial-nor/n25q/n25q_32mb_3v_65nm.pdf
 *  http://www.ftdichip.com/Support/Documents/AppNotes/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
 */

#define _GNU_SOURCE

#include <ftdi.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

#define NUM_VECTORS 10	// Este número puede obtenerse del tamaño del applet, ya que depende del número definido en icemulti con el que se haya generado la imagen grabada en la flash.

//--------------------- predefiniciones ----------------
void flash_4kB_subsector_erase(int addr);
void dump_buffer (unsigned int begin_addr, uint8_t *buffer, unsigned int size);
void test_change_vectors (unsigned int vector1, unsigned int vector2);
void test_get_vectors ();
void get_comment (unsigned int vector);
void change_vector (unsigned int vector, unsigned int boot, uint8_t *buffer);

//--------- globales -----------
struct ftdi_context ftdic;
bool ftdic_open = false;
bool verbose = false;
bool ftdic_latency_set = false;
unsigned char ftdi_latency;

//------------------------------
void check_rx()
{
	while (1) {
		uint8_t data;
		int rc = ftdi_read_data(&ftdic, &data, 1);
		if (rc <= 0) break;
		fprintf(stderr, "unexpected rx byte: %02X\n", data);
	}
}

void error()
{
	check_rx();
	fprintf(stderr, "ABORT.\n");
	if (ftdic_open) {
		if (ftdic_latency_set)
			ftdi_set_latency_timer(&ftdic, ftdi_latency);
	if (ftdic_open)
		ftdi_usb_close(&ftdic);
	}
	ftdi_deinit(&ftdic);
	exit(1);
}

uint8_t recv_byte()
{
	uint8_t data;
	while (1) {
		int rc = ftdi_read_data(&ftdic, &data, 1);
		if (rc < 0) {
			fprintf(stderr, "Read error.\n");
			error();
		}
		if (rc == 1)
			break;
		usleep(100);
	}
	return data;
}

void send_byte(uint8_t data)
{
	int rc = ftdi_write_data(&ftdic, &data, 1);
	if (rc != 1) {
		fprintf(stderr, "Write error (single byte, rc=%d, expected %d).\n", rc, 1);
		error();
	}
}

void send_spi(uint8_t *data, int n)
{
	if (n < 1)
		return;

	send_byte(0x11);
	send_byte(n-1);
	send_byte((n-1) >> 8);

	int rc = ftdi_write_data(&ftdic, data, n);
	if (rc != n) {
		fprintf(stderr, "Write error (chunk, rc=%d, expected %d).\n", rc, n);
		error();
	}
}

void xfer_spi(uint8_t *data, int n)
{
	if (n < 1)
		return;

	send_byte(0x31);
	send_byte(n-1);
	send_byte((n-1) >> 8);

	int rc = ftdi_write_data(&ftdic, data, n);
	if (rc != n) {
		fprintf(stderr, "Write error (chunk, rc=%d, expected %d).\n", rc, n);
		error();
	}

	for (int i = 0; i < n; i++)
		data[i] = recv_byte();
}

void set_gpio(int slavesel_b, int creset_b)
{
	uint8_t gpio = 1;

	if (slavesel_b) {
		// ADBUS4 (GPIOL0)
		gpio |= 0x10;
	}

	if (creset_b) {
		// ADBUS7 (GPIOL3)
		gpio |= 0x80;
	}

	send_byte(0x80);
	send_byte(gpio);
	send_byte(0x93);
}

int get_cdone()
{
	uint8_t data;
	send_byte(0x81);
	data = recv_byte();
	// ADBUS6 (GPIOL2)
	return (data & 0x40) != 0;
}

void flash_read_id()
{
	// fprintf(stderr, "read flash ID..\n");

	uint8_t data[21] = { 0x9F };
	set_gpio(0, 0);
	xfer_spi(data, 21);
	set_gpio(1, 0);

	fprintf(stderr, "flash ID:");
	for (int i = 1; i < 21; i++)
		fprintf(stderr, " 0x%02X", data[i]);
	fprintf(stderr, "\n");
}

void flash_power_up()
{
	uint8_t data[1] = { 0xAB };
	set_gpio(0, 0);
	xfer_spi(data, 1);
	set_gpio(1, 0);
}

void flash_power_down()
{
	uint8_t data[1] = { 0xB9 };
	set_gpio(0, 0);
	xfer_spi(data, 1);
	set_gpio(1, 0);
}

void flash_write_enable()
{
	if (verbose)
		fprintf(stderr, "write enable..\n");

	uint8_t data[1] = { 0x06 };
	set_gpio(0, 0);
	xfer_spi(data, 1);
	set_gpio(1, 0);
}

void flash_bulk_erase()
{
	fprintf(stderr, "bulk erase..\n");

	uint8_t data[1] = { 0xc7 };
	set_gpio(0, 0);
	xfer_spi(data, 1);
	set_gpio(1, 0);
}

void flash_64kB_sector_erase(int addr)
{
	fprintf(stderr, "erase 64kB sector at 0x%06X..\n", addr);

	uint8_t command[4] = { 0xd8, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
	set_gpio(0, 0);
	send_spi(command, 4);
	set_gpio(1, 0);
}

void flash_prog(int addr, uint8_t *data, int n)
{
	if (verbose)
		fprintf(stderr, "prog 0x%06X +0x%03X..\n", addr, n);

	uint8_t command[4] = { 0x02, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
	set_gpio(0, 0);
	send_spi(command, 4);
	send_spi(data, n);
	set_gpio(1, 0);

	if (verbose)
		for (int i = 0; i < n; i++)
			fprintf(stderr, "%02x%c", data[i], i == n-1 || i % 32 == 31 ? '\n' : ' ');
}

void flash_read(int addr, uint8_t *data, int n)
{
	if (verbose)
		fprintf(stderr, "read 0x%06X +0x%03X..\n", addr, n);

	uint8_t command[4] = { 0x03, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
	set_gpio(0, 0);
	send_spi(command, 4);
	memset(data, 0, n);
	xfer_spi(data, n);
	set_gpio(1, 0);

	if (verbose)
		for (int i = 0; i < n; i++)
			fprintf(stderr, "%02x%c", data[i], i == n-1 || i % 32 == 31 ? '\n' : ' ');
}

void flash_wait()
{
	if (verbose)
		fprintf(stderr, "waiting..");

	while (1)
	{
		uint8_t data[2] = { 0x05 };

		set_gpio(0, 0);
		xfer_spi(data, 2);
		set_gpio(1, 0);

		if ((data[1] & 0x01) == 0)
			break;

		if (verbose) {
			fprintf(stderr, ".");
			fflush(stdout);
		}
		usleep(1000);
	}

	if (verbose)
		fprintf(stderr, "\n");
}

void help(const char *progname)
{
	fprintf(stderr, "\n");
	fprintf(stderr, "iceprog -- simple programming tool for FTDI-based Lattice iCE programmers\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Notes for iCEstick (iCE40HX-1k devel board):\n");
	fprintf(stderr, "  An unmodified iCEstick can only be programmed via the serial flash.\n");
	fprintf(stderr, "  Direct programming of the SRAM is not supported. For direct SRAM\n");
	fprintf(stderr, "  programming the flash chip and one zero ohm resistor must be desoldered\n");
	fprintf(stderr, "  and the FT2232H SI pin must be connected to the iCE SPI_SI pin, as shown\n");
	fprintf(stderr, "  in this picture: http://www.clifford.at/gallery/2014-elektronik/IMG_20141115_183838\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Notes for the iCE40-HX8K Breakout Board:\n");
	fprintf(stderr, "  Make sure that the jumper settings on the board match the selected\n");
	fprintf(stderr, "  mode (SRAM or FLASH). See the iCE40-HX8K user manual for details.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Usage: %s [options] <filename>\n", progname);
	fprintf(stderr, "\n");
	fprintf(stderr, "    -d <device-string>\n");
	fprintf(stderr, "        use the specified USB device:\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "            d:<devicenode>                (e.g. d:002/005)\n");
	fprintf(stderr, "            i:<vendor>:<product>          (e.g. i:0x0403:0x6010)\n");
	fprintf(stderr, "            i:<vendor>:<product>:<index>  (e.g. i:0x0403:0x6010:0)\n");
	fprintf(stderr, "            s:<vendor>:<product>:<serial-string>\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    -I [ABCD]\n");
	fprintf(stderr, "        connect to the specified interface on the FTDI chip\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    -r\n");
	fprintf(stderr, "        read first 256 kB from flash and write to file\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    -R <size_in_bytes>\n");
	fprintf(stderr, "        read the specified number of bytes from flash\n");
	fprintf(stderr, "        (append 'k' to the argument for size in kilobytes, or\n");
	fprintf(stderr, "        'M' for size in megabytes)\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    -o <offset_in_bytes>\n");
	fprintf(stderr, "        start address for read/write (instead of zero)\n");
	fprintf(stderr, "        (append 'k' to the argument for size in kilobytes, or\n");
	fprintf(stderr, "        'M' for size in megabytes)\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    -c\n");
	fprintf(stderr, "        do not write flash, only verify (check)\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    -b\n");
	fprintf(stderr, "        bulk erase entire flash before writing\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    -n\n");
	fprintf(stderr, "        do not erase flash before writing\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    -S\n");
	fprintf(stderr, "        perform SRAM programming\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    -t\n");
	fprintf(stderr, "        just read the flash ID sequence\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    -x <vector1> <vector2>\n");
	fprintf(stderr, "        interchange two vectors.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    -l\n");
	fprintf(stderr, "        list vectors from flash.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    -v\n");
	fprintf(stderr, "        verbose output\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Without -b or -n, iceprog will erase aligned chunks of 64kB in write mode.\n");
	fprintf(stderr, "This means that some data after the written data (or even before when -o is\n");
	fprintf(stderr, "used) may be erased as well.\n");
	fprintf(stderr, "\n");
	exit(1);
}

int main(int argc, char **argv)
{
	int read_size = 256 * 1024;
	int rw_offset = 0;

	bool read_mode = false;
	bool check_mode = false;
	bool bulk_erase = false;
	bool dont_erase = false;
	bool prog_sram = false;
	bool test_mode = false;
	bool get_vectors = false;
	bool change_vectors = false;
    unsigned int vector1, vector2;
	const char *filename = NULL;
	const char *devstr = NULL;
	enum ftdi_interface ifnum = INTERFACE_A;

	int opt;
	char *endptr=NULL;
	while ((opt = getopt(argc, argv, "d:I:rR:o:cbnStgvx:l")) != -1)
	{
		switch (opt)
		{
		case 'd':
			devstr = optarg;
			break;
		case 'I':
			if (!strcmp(optarg, "A")) ifnum = INTERFACE_A;
			else if (!strcmp(optarg, "B")) ifnum = INTERFACE_B;
			else if (!strcmp(optarg, "C")) ifnum = INTERFACE_C;
			else if (!strcmp(optarg, "D")) ifnum = INTERFACE_D;
			else help(argv[0]);
			break;
		case 'r':
			read_mode = true;
			break;
		case 'R':
			read_mode = true;
			read_size = strtol(optarg, &endptr, 0);
			if (!strcmp(endptr, "k")) read_size *= 1024;
			if (!strcmp(endptr, "M")) read_size *= 1024 * 1024;
			break;
		case 'o':
			rw_offset = strtol(optarg, &endptr, 0);
			if (!strcmp(endptr, "k")) rw_offset *= 1024;
			if (!strcmp(endptr, "M")) rw_offset *= 1024 * 1024;
			break;
		case 'c':
			check_mode = true;
			break;
		case 'b':
			bulk_erase = true;
			break;
		case 'n':
			dont_erase = true;
			break;
		case 'S':
			prog_sram = true;
			break;
		case 't':
			test_mode = true;
			break;
		case 'v':
			verbose = true;
			break;

		case 'x':
    		vector1 = strtol(argv[optind-1], NULL, 0);
			vector2 = strtol(argv[optind], NULL, 0);
			change_vectors = true;
            break;

    	case 'l':
			get_vectors = true;
            break;
        
		default:
			help(argv[0]);
		}
	}

	if (read_mode + check_mode + prog_sram + test_mode > 1)
		help(argv[0]);

	if (bulk_erase && dont_erase)
		help(argv[0]);

	if (optind+1 != argc && !test_mode) {
		if (bulk_erase && optind == argc)
			filename = "/dev/null";
		else
			help(argv[0]);
	} else
		filename = argv[optind];

	// ---------------------------------------------------------
	// Initialize USB connection to FT2232H
	// ---------------------------------------------------------

	fprintf(stderr, "init..\n");

	ftdi_init(&ftdic);
	ftdi_set_interface(&ftdic, ifnum);

	if (devstr != NULL) {
		if (ftdi_usb_open_string(&ftdic, devstr)) {
			fprintf(stderr, "Can't find iCE FTDI USB device (device string %s).\n", devstr);
			error();
		}
	} else {
		if (ftdi_usb_open(&ftdic, 0x0403, 0x6010)) {
			fprintf(stderr, "Can't find iCE FTDI USB device (vedor_id 0x0403, device_id 0x6010).\n");
			error();
		}
	}

	ftdic_open = true;

	if (ftdi_usb_reset(&ftdic)) {
		fprintf(stderr, "Failed to reset iCE FTDI USB device.\n");
		error();
	}

	if (ftdi_usb_purge_buffers(&ftdic)) {
		fprintf(stderr, "Failed to purge buffers on iCE FTDI USB device.\n");
		error();
	}

	if (ftdi_get_latency_timer(&ftdic, &ftdi_latency) < 0) {
		fprintf(stderr, "Failed to get latency timer (%s).\n", ftdi_get_error_string(&ftdic));
		error();
	}

	/* 1 is the fastest polling, it means 1 kHz polling */
	if (ftdi_set_latency_timer(&ftdic, 1) < 0) {
		fprintf(stderr, "Failed to set latency timer (%s).\n", ftdi_get_error_string(&ftdic));
		error();
	}

	ftdic_latency_set = true;

	if (ftdi_set_bitmode(&ftdic, 0xff, BITMODE_MPSSE) < 0) {
		fprintf(stderr, "Failed set BITMODE_MPSSE on iCE FTDI USB device.\n");
		error();
	}

	// enable clock divide by 5
	send_byte(0x8b);

	// set 6 MHz clock
	send_byte(0x86);
	send_byte(0x00);
	send_byte(0x00);

	fprintf(stderr, "cdone: %s\n", get_cdone() ? "high" : "low");

	set_gpio(1, 1);
	usleep(100000);


	if (test_mode)
	{
		fprintf(stderr, "reset..\n");

		set_gpio(1, 0);
		usleep(250000);

		fprintf(stderr, "cdone: %s\n", get_cdone() ? "high" : "low");

		flash_power_up();

		flash_read_id();

		flash_power_down();

		set_gpio(1, 1);
		usleep(250000);

		fprintf(stderr, "cdone: %s\n", get_cdone() ? "high" : "low");
	}
	else if (prog_sram)
	{
		// ---------------------------------------------------------
		// Reset
		// ---------------------------------------------------------

		fprintf(stderr, "reset..\n");

		set_gpio(0, 0);
		usleep(100);

		set_gpio(0, 1);
		usleep(2000);

		fprintf(stderr, "cdone: %s\n", get_cdone() ? "high" : "low");


		// ---------------------------------------------------------
		// Program
		// ---------------------------------------------------------

		FILE *f = (strcmp(filename, "-") == 0) ? stdin :
			fopen(filename, "rb");
		if (f == NULL) {
			fprintf(stderr, "Error: Can't open '%s' for reading: %s\n", filename, strerror(errno));
			error();
		}

		fprintf(stderr, "Programming SRAM..\n");
		while (1)
		{
			static unsigned char buffer[4096];
			int rc = fread(buffer, 1, 4096, f);
			if (rc <= 0) break;
			if (verbose)
				fprintf(stderr, "sending %d bytes.\n", rc);
			send_spi(buffer, rc);
		}

		if (f != stdin)
			fclose(f);

		// add 48 dummy bits
		send_byte(0x8f);
		send_byte(0x05);
		send_byte(0x00);

		// add 1 more dummy bit
		send_byte(0x8e);
		send_byte(0x00);

		fprintf(stderr, "cdone: %s\n", get_cdone() ? "high" : "low");
	}
	else if (change_vectors)
	{
		if ((vector1 <= NUM_VECTORS) && (vector2 <= NUM_VECTORS)) {
			if (verbose) fprintf(stderr, "Intercambiar: %d, %d\n", vector1, vector2);
			test_change_vectors (vector1, vector2);					
		}		
	}			
	else if (get_vectors)
	{

		test_get_vectors ();
	}
	else
	{
		// ---------------------------------------------------------
		// Reset
		// ---------------------------------------------------------

		fprintf(stderr, "reset..\n");

		set_gpio(1, 0);
		usleep(250000);

		fprintf(stderr, "cdone: %s\n", get_cdone() ? "high" : "low");

		flash_power_up();

		flash_read_id();


		// ---------------------------------------------------------
		// Program
		// ---------------------------------------------------------

		if (!read_mode && !check_mode)
		{
			FILE *f = (strcmp(filename, "-") == 0) ? stdin :
				fopen(filename, "rb");
			if (f == NULL) {
				fprintf(stderr, "Error: Can't open '%s' for reading: %s\n", filename, strerror(errno));
				error();
			}

			if (!dont_erase)
			{
				if (bulk_erase)
				{
					flash_write_enable();
					flash_bulk_erase();
					flash_wait();
				}
				else
				{
					struct stat st_buf;
					if (stat(filename, &st_buf)) {
						fprintf(stderr, "Error: Can't stat '%s': %s\n", filename, strerror(errno));
						error();
					}

					fprintf(stderr, "file size: %d\n", (int)st_buf.st_size);

					int begin_addr = rw_offset & ~0xffff;
					int end_addr = (rw_offset + (int)st_buf.st_size + 0xffff) & ~0xffff;

					for (int addr = begin_addr; addr < end_addr; addr += 0x10000) {
						flash_write_enable();
						flash_64kB_sector_erase(addr);
						flash_wait();
					}
				}
			}

			fprintf(stderr, "programming..\n");

			for (int rc, addr = 0; true; addr += rc) {
				uint8_t buffer[256];
				int page_size = 256 - (rw_offset + addr) % 256;
				rc = fread(buffer, 1, page_size, f);
				if (rc <= 0) break;
				flash_write_enable();
				flash_prog(rw_offset + addr, buffer, rc);
				flash_wait();
			}

			if (f != stdin)
				fclose(f);
		}


		// ---------------------------------------------------------
		// Read/Verify
		// ---------------------------------------------------------

		if (read_mode)
		{
			FILE *f = (strcmp(filename, "-") == 0) ? stdout :
				fopen(filename, "wb");
			if (f == NULL) {
				fprintf(stderr, "Error: Can't open '%s' for writing: %s\n", filename, strerror(errno));
				error();
			}

			fprintf(stderr, "reading..\n");
			for (int addr = 0; addr < read_size; addr += 256) {
				uint8_t buffer[256];
				flash_read(rw_offset + addr, buffer, 256);
				fwrite(buffer, 256, 1, f);
			}

			if (f != stdout)
				fclose(f);
		}
		else
		{
			FILE *f = (strcmp(filename, "-") == 0) ? stdin :
				fopen(filename, "rb");
			if (f == NULL) {
				fprintf(stderr, "Error: Can't open '%s' for reading: %s\n", filename, strerror(errno));
				error();
			}

			fprintf(stderr, "reading..\n");
			for (int addr = 0; true; addr += 256) {
				uint8_t buffer_flash[256], buffer_file[256];
				int rc = fread(buffer_file, 1, 256, f);
				if (rc <= 0) break;
				flash_read(rw_offset + addr, buffer_flash, rc);
				if (memcmp(buffer_file, buffer_flash, rc)) {
					fprintf(stderr, "Found difference between flash and file!\n");
					error();
				}
			}

			fprintf(stderr, "VERIFY OK\n");

			if (f != stdin)
				fclose(f);
		}


		// ---------------------------------------------------------
		// Reset
		// ---------------------------------------------------------

		flash_power_down();

		set_gpio(1, 1);
		usleep(250000);

		fprintf(stderr, "cdone: %s\n", get_cdone() ? "high" : "low");
	}


	// ---------------------------------------------------------
	// Exit
	// ---------------------------------------------------------

	fprintf(stderr, "Bye.\n");
	ftdi_set_latency_timer(&ftdic, ftdi_latency);
	ftdi_disable_bitbang(&ftdic);
	ftdi_usb_close(&ftdic);
	ftdi_deinit(&ftdic);
	return 0;
}

// ---------------------------------------------------------
// Test multi image change second image vector.
// ---------------------------------------------------------
// Prueba para comprobar si modificando un vector del applet de iCE40HX
// directamente desde la flash, se puede modificar el comportamiento de
// la FPGA al reiniciar con warmboot.
// 
// Se modifica el vector de reset (address 0x09 - 3 bytes) por la dirección de
// otra imagen de síntesis que contiene la flash y que se encuentra en la dirección 0x100.
//
void test_change_vectors (unsigned int vector1, unsigned int vector2)
{
        uint8_t buffer[0x001100]; // Tamaño del buffer a grabar 4kB (4096 bytes - 0x1000 bytes).

		// ---------------------------------------------------------
		// Reset de la flash.
		// ---------------------------------------------------------
		fprintf(stderr, "reset..\n");
		set_gpio(1, 0);
    	usleep(250000);
		fprintf(stderr, "cdone: %s\n", get_cdone() ? "high" : "low");
		flash_power_up();
		flash_read_id();

        // Leemos un subsector completo de la flash (4kB).
    	fprintf(stderr, "Leyendo el primer subsector...\n");
		flash_read (0, buffer, 0x1000);

		// Se muestra lo leido.		
		if (verbose) dump_buffer(0, buffer, 256);

		// Se modifica el vector en el buffer leido.
		change_vector (vector1, vector2, buffer);		

        // Se borra el primer subsector (4kb) en flash.
    	fprintf(stderr, "Borrando el primer subsector...\n");
		flash_write_enable();
		flash_4kB_subsector_erase(0x00);
		flash_wait();

		// Se programa el subsector por páginas de 256 bytes.
		fprintf(stderr, "Programando...\n");
        unsigned int page_size = 256;
		unsigned int subsector_size = 0x1000; // 4kB = 4096 bytes = 0x1000 bytes
		for ( int addr = 0; addr < subsector_size; addr += page_size) {
			if (verbose) fprintf(stderr, "Grabar flash en 0x%04X\n", addr);
			flash_write_enable();
			flash_prog(addr, buffer+addr, page_size);
			flash_wait();
		}

		// ---------------------------------------------------------
		// Reset general.
		// ---------------------------------------------------------
		flash_power_down();
		set_gpio(1, 1);
		usleep(250000);
		fprintf(stderr, "cdone: %s\n", get_cdone() ? "high" : "low");
}

//------------------------------------------------------------
// Muestra el contenido de un buffer en formato hexadecimal.
//------------------------------------------------------------
void dump_buffer (unsigned int begin_addr, uint8_t *buffer, unsigned int size)
{
		// Para comprobar se imprime el buffer en pantalla.
    	fprintf(stderr, "Buffer...\n");
		int end_addr = begin_addr + size;
		int size_line = 16;		
		for (int addr = begin_addr; addr < end_addr; addr += size_line) {
			fprintf(stderr, "%06X: |", addr);
            for (int b=0; b<8; b++) fprintf(stderr," %02X", buffer[addr+b]);
			fprintf(stderr, " |");
            for (int b=0; b<8; b++) fprintf(stderr," %02X", buffer[addr+b+8]);           
			fprintf(stderr, " |\n");
		}
}

//------------------------------------------------------------
// Borra un subsector (4kB) de la flash.
//------------------------------------------------------------
void flash_4kB_subsector_erase(int addr)
{
	fprintf(stderr, "erase 4kB subsector at 0x%06X..\n", addr);

	uint8_t command[4] = { 0x20, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
	set_gpio(0, 0);
	send_spi(command, 4);
	set_gpio(1, 0);
}

// ---------------------------------------------------------
// Test multi image change second image vector.
// ---------------------------------------------------------
// Se conecta a la flash y extrae los vectores del applet
// y los muestra por pantalla.
// El siguiente paso es poder modificar el orden de esta lista
// de vectores para que puedan seleccionarse otras imágnes con
// la opción del warmboot.
//
void test_get_vectors ()
{
/*
Se toma el applet del 'pack.bin' generado con la orden:

$ icemulti -p1 -o pack.bin blink.bin hardware.bin

Luego se tiene en el reset la síntesis de 'hardware.bin' (en la direción 0x8000) y
en la dirección 0x100 tenemos 'blink.bin'. Y su applet es:

$ hexdump -C -n256 pack.bin

00000000  7e aa 99 7e 92 00 00 44  03 00 80 00 82 00 00 01  |~..~...D........|
00000010  08 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
00000020  7e aa 99 7e 92 00 00 44  03 00 01 00 82 00 00 01  |~..~...D........|
00000030  08 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
00000040  7e aa 99 7e 92 00 00 44  03 00 80 00 82 00 00 01  |~..~...D........|
00000050  08 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
00000060  7e aa 99 7e 92 00 00 44  03 00 80 00 82 00 00 01  |~..~...D........|
00000070  08 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
00000080  7e aa 99 7e 92 00 00 44  03 00 80 00 82 00 00 01  |~..~...D........|
00000090  08 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
000000a0  ff ff ff ff ff ff ff ff  ff ff ff ff ff ff ff ff  |................|

Cambiando los valores de los 3 bytes del vector de reset (dirección 0x09) por {0x00, 0x01, 0x00}
se desvía el vector a la síntesis de 'blink.bin' y grabando 4kB (subsector) en la flash se
selecciona otra síntesis.
*/
        unsigned int vector;
        uint8_t buffer[0x001100]; // Tamaño del buffer a grabar 4kB (4096 bytes - 0x1000 bytes).

		// ---------------------------------------------------------
		// Reset de la flash.
		// ---------------------------------------------------------
		fprintf(stderr, "reset..\n");
		set_gpio(1, 0);
    	usleep(250000);
		fprintf(stderr, "cdone: %s\n", get_cdone() ? "high" : "low");
		flash_power_up();
		flash_read_id();

        // Leemos un subsector completo de la flash (4kB).
    	fprintf(stderr, "Leyendo el primer subsector...\n");
		flash_read (0, buffer, 0x1000);

		// Se muestra lo leido.		
		if (verbose) dump_buffer(0, buffer, 256);

		// Mostrar los vectores (máximo 50).
        for (unsigned int i=0, offset=0; i<50; i++, offset+=0x20) {
            // Se comprueba antes de imprimir si es un vector válido.
            if (buffer[offset]==0x7E && buffer[offset+1]==0xAA) { 
				vector = (buffer[offset+9] << 16) + (buffer[offset+10] << 8) + buffer[offset+11];						
				fprintf(stderr, "Vector %02d: 0x%06X - ", i, (unsigned int) vector);
				get_comment (vector);				
				switch (i){
					case 0:  fprintf(stderr," (reset)\n");  break;
					case 1:  fprintf(stderr," (boot 0)\n"); break;
					case 2:  fprintf(stderr," (boot 1)\n"); break;
					case 3:  fprintf(stderr," (boot 2)\n"); break;
					case 4:  fprintf(stderr," (boot 3)\n"); break;
					default: fprintf(stderr,"\n"); break;
				}
			}
			else {
				break;
 			}
        }

		// ---------------------------------------------------------
		// Reset general.
		// ---------------------------------------------------------
		flash_power_down();
		set_gpio(1, 1);
		usleep(250000);
		fprintf(stderr, "cdone: %s\n", get_cdone() ? "high" : "low");
}

//---------------------------------------------
// Muestra el comentario del bitstream.
//---------------------------------------------
void get_comment (unsigned int vector)
{
	char comment[30]="";

	vector += 2;
	flash_read (vector, (uint8_t *) comment, 25);
	fprintf(stderr, "%s", comment);
}

//--------------------------------------------------------
// Cambia un vector boot por otro apuntado en el applet.
// @param vector número de vector a cambiar.
// @param boot   posición boot a intercambiar (0-reset, 1-boot0,....)
//--------------------------------------------------------
void change_vector (unsigned int vector, unsigned int boot, uint8_t *buffer)
{
	// Calculate address.
    unsigned int addr_boot = boot*0x20;
    unsigned int addr_vector = vector*0x20;

	// Save vector boot in a temporal vector.
    uint8_t vector_temp[3];
    vector_temp[0] = buffer[addr_boot+9];
    vector_temp[1] = buffer[addr_boot+10];
    vector_temp[2] = buffer[addr_boot+11];

	// Change boot vector.
    buffer[addr_boot+9]  = buffer[addr_vector+9];
    buffer[addr_boot+10] = buffer[addr_vector+10];
    buffer[addr_boot+11] = buffer[addr_vector+11];

	// Change vector.
    buffer[addr_vector+9]  = vector_temp[0];
    buffer[addr_vector+10]  = vector_temp[1];
    buffer[addr_vector+11]  = vector_temp[2];
	
	// Info.
    fprintf(stderr, "Intercambiados vectores: 0x%06X por 0x%06X\n",
				    (buffer[addr_boot+9] << 16) + (buffer[addr_boot+10] << 8) + buffer[addr_boot+11],
					(buffer[addr_vector+9] << 16) + (buffer[addr_vector+10] << 8) + buffer[addr_vector+11]);			
		
}

