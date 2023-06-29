/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <assert.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *master_device = "/dev/spidev0.0";
static const char *slave_device = "/dev/spidev9.0";
static const char *srdypath = "/sys/class/spi_slave/spi9/device/slave_ready";
static uint32_t mode;
static uint8_t bits = 32;
static char *input_file;
static char *output_file;
static uint32_t speed = 24000000;
static uint16_t delay;
static int verbose;
int mfd, sfd;
long numtr = 10;
double avg_dur = 0;
int tr_size = 32;

uint8_t *master_tx;
uint8_t *master_rx;
uint8_t *slave_tx;
uint8_t *slave_rx;

static pthread_mutex_t printf_mutex;
char *input_tx;

long getmicros(){
	struct timeval currentTime;
	gettimeofday(&currentTime, NULL);
	return currentTime.tv_sec*1000000  + currentTime.tv_usec;
}

static void hex_dump(const void *src, size_t length, size_t line_size,
		     char *prefix)
{
	int i = 0;
	const unsigned char *address = src;
	const unsigned char *line = address;
	unsigned char c;

	printf("%s | ", prefix);
	while (length-- > 0) {
		printf("%02X ", *address++);
		if (!(++i % line_size) || (length == 0 && i % line_size)) {
			if (length == 0) {
				while (i++ % line_size)
					printf("__ ");
			}
			printf(" | ");  /* right close */
			while (line < address) {
				c = *line++;
				printf("%c", (c < 33 || c == 255) ? 0x2E : c);
			}
			printf("\n");
			if (length > 0)
				printf("%s | ", prefix);
		}
	}
}

static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int ret;
	int out_fd;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	if (mode & SPI_TX_QUAD)
		tr.tx_nbits = 4;
	else if (mode & SPI_TX_DUAL)
		tr.tx_nbits = 2;
	if (mode & SPI_RX_QUAD)
		tr.rx_nbits = 4;
	else if (mode & SPI_RX_DUAL)
		tr.rx_nbits = 2;
	if (!(mode & SPI_LOOP)) {
		if (mode & (SPI_TX_QUAD | SPI_TX_DUAL))
			tr.rx_buf = 0;
		else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
			tr.tx_buf = 0;
	}

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	if (verbose)
		hex_dump(tx, len, 32, "TX");

	if (output_file) {
		out_fd = open(output_file, O_WRONLY | O_CREAT | O_TRUNC, 0666);
		if (out_fd < 0)
			pabort("could not open output file");

		ret = write(out_fd, rx, len);
		if (ret != len)
			pabort("not all bytes written to output file");

		close(out_fd);
	}

	if (verbose)
		hex_dump(rx, len, 32, "RX");
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev0.0)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word\n"
	     "  -i --input    input data from a file (e.g. \"test.bin\")\n"
	     "  -o --output   output data to a file (e.g. \"results.bin\")\n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n"
	     "  -v --verbose  Verbose (show tx buffer)\n"
	     "  -p            Send data (e.g. \"1234\\xde\\xad\")\n"
	     "  -N --no-cs    no chip select\n"
	     "  -R --ready    slave pulls low to pause\n"
	     "  -2 --dual     dual transfer\n"
	     "  -4 --quad     quad transfer\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "Size",   1, 0, 'S' },
			{ "numtransactions",   1, 0, 'n' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "input",   1, 0, 'i' },
			{ "output",  1, 0, 'o' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "no-cs",   0, 0, 'N' },
			{ "ready",   0, 0, 'R' },
			{ "dual",    0, 0, '2' },
			{ "verbose", 0, 0, 'v' },
			{ "quad",    0, 0, '4' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:S:n:d:b:i:o:lHOLC3NR24p:v",
				lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 's':
			speed = atoi(optarg);
			break;
		case 'S':
			tr_size = atoi(optarg);
			break;
		case 'n':
			numtr = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'i':
			input_file = optarg;
			break;
		case 'o':
			output_file = optarg;
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'N':
			mode |= SPI_NO_CS;
			break;
		case 'v':
			verbose = 1;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		case 'p':
			input_tx = optarg;
			break;
		case '2':
			mode |= SPI_TX_DUAL;
			break;
		case '4':
			mode |= SPI_TX_QUAD;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
	if (mode & SPI_LOOP) {
		if (mode & SPI_TX_DUAL)
			mode |= SPI_RX_DUAL;
		if (mode & SPI_TX_QUAD)
			mode |= SPI_RX_QUAD;
	}
}

static int check_slave_ready()                   
{                                                                                                  
        FILE *srdy_fd;                                        
	int srdy_status = 0;                                 
                                                                                 
        srdy_fd = fopen(srdypath, "r");
        if (!srdy_fd) {
		printf("failed to open file slave ready");
		return -21;
	}
	if (!fscanf(srdy_fd, "%d", &srdy_status))
		printf("Failed to get slave ready status");
	fclose(srdy_fd);
	return srdy_status;
}

void *slave_transfer(void *vargp)
{
	int iter;
	int i;

	for(iter = 1; iter < numtr; iter++) {
		slave_tx[0] = (iter % 255) + 1;
		transfer(sfd, slave_tx, slave_rx, tr_size);
	    
		if(slave_rx[0] != ((iter % 255) + 1)){
			pthread_mutex_lock(&printf_mutex);
			printf("slave RX data index with master TX data [%d] != [%d]\n", slave_rx[0], iter);
		    pthread_mutex_unlock(&printf_mutex);
		}
		if (strncmp(slave_rx + 1, master_tx + 1, numtr - 1)) {
			pthread_mutex_lock(&printf_mutex);
			printf("slave RX data mismatch with master TX data  [%d], iter = %d\n", strncmp(slave_rx + 1, master_tx + 1, numtr - 1), iter);
			hex_dump(slave_rx, tr_size, 32, "Slave RX Data:");
			hex_dump(master_tx, tr_size, 32, "Master TX Data:");
			pthread_mutex_unlock(&printf_mutex);
			return NULL;
		}
	}
    return NULL;
}

void *master_transfer(void *vargp)
{
	long start, stop;
	int iter;
	int i;

	for(iter = 1; iter < numtr; iter++) {
		master_tx[0] = (iter % 255) + 1;
		start = getmicros();
		while(!check_slave_ready());
		transfer(mfd, master_tx, master_rx, tr_size);
		stop = getmicros();
		avg_dur += (double)(stop - start)/numtr;
	    
		if(master_rx[0] != ((iter % 255) + 1)) {
			pthread_mutex_lock(&printf_mutex);
			printf("master RX data index with slave TX data [%d] != [%d]\n", master_rx[0], iter);
		    pthread_mutex_unlock(&printf_mutex);
		}
		if (strncmp(master_rx + 1, slave_tx + 1, numtr - 1)){
			pthread_mutex_lock(&printf_mutex);
			printf("master RX data mismatch with slave TX data [%d], iter = %d\n",	strncmp(master_rx + 1, slave_tx + 1, numtr - 1), iter);
			hex_dump(master_rx, tr_size, 32, "Master RX Data:");
			hex_dump(slave_tx, tr_size, 32, "Slave TX Data:");
		    pthread_mutex_unlock(&printf_mutex);
			return NULL;
		}
	}
	return NULL;
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int iter;
	pthread_t slave_transfer_thread, master_transfer_thread;

	parse_opts(argc, argv);

	mfd = open(master_device, O_RDWR);
	if (mfd < 0)
		pabort("can't open device");

	sfd = open(slave_device, O_RDWR);
	if (sfd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(mfd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(mfd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	ret = ioctl(sfd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(sfd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(mfd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(mfd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");
	
	ret = ioctl(sfd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(sfd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(mfd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(mfd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");
	
	ret = ioctl(sfd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(sfd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	// printf("spi mode: 0x%x\n", mode);
	// printf("bits per word: %d\n", bits);
	// printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	/* 
	 * populate TX buffers for Master/Slave
	 * later set buf[0] to identify tr_index.
	 */
	master_tx = malloc(tr_size);
	master_rx = malloc(tr_size);
	slave_tx = malloc(tr_size);
	slave_rx = malloc(tr_size);
	memset(master_rx, 0, tr_size);
	memset(slave_rx, 0, tr_size);
	for (iter = 0; iter < tr_size; iter++) {
    	master_tx[iter] = (rand() % 255) + 1;
		slave_tx[iter] = (rand() % 255) + 1;
	}
	if(verbose){
		hex_dump(master_tx, tr_size, 32, "MASTER TX");
		hex_dump(slave_tx, tr_size, 32, "SLAVE TX");
	}

	pthread_create(&slave_transfer_thread, NULL, slave_transfer, NULL);
	pthread_create(&master_transfer_thread, NULL, master_transfer, NULL);
	pthread_mutex_init(&printf_mutex, NULL);

    pthread_join(slave_transfer_thread, NULL);
    pthread_join(master_transfer_thread, NULL);
	
	printf("Average time taken per transaction (averaged over %d transactions of size %d bytes @ %f MHz) is %f us\n", numtr, tr_size, (float)speed/1000000, avg_dur);
	free(master_tx);
	free(master_rx);
	free(slave_tx);
	free(slave_rx);
	close(mfd);
	close(sfd);

	return ret;
}

