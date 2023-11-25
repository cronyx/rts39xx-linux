/*function:
 *connect ssi and flash, test operate flash using ssi.
 *compile tip:
 *mips-linux-gcc -W spidev_flash.c -o spidev_flash
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <linux/types.h>
#include <linux/spi/spidev.h>


static int verbose;

static unsigned char mode;
static unsigned char bits = 8;
static unsigned int speed = 15000000;
static int delay;

static void do_read(int fd, int len)
{
	unsigned char	buf[32], *bp;
	int		status;

	/* read at least 2 bytes, no more than 32 */
	if (len < 2)
		len = 2;
	else if (len > sizeof(buf))
		len = sizeof(buf);
	memset(buf, 0, sizeof(buf));

	status = read(fd, buf, len);
	if (status < 0) {
		perror("read");
		return;
	}
	if (status != len) {
		fprintf(stderr, "short read\n");
		return;
	}

	printf("read(%2d, %2d): %02x %02x,", len, status,
		buf[0], buf[1]);
	status -= 2;
	bp = buf + 2;
	while (status-- > 0)
		printf(" %02x", *bp++);
	printf("\n");
}

static void write_enable(int fd)
{
	struct spi_ioc_transfer	xfer[1];
	unsigned char		buftx[32];
	int			status;

	memset(xfer, 0, sizeof(xfer));
	memset(buftx, 0, sizeof(buftx));

	buftx[0] = 0x06;
	xfer[0].tx_buf = (unsigned long)buftx;
	xfer[0].len = 1;

	status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);

	if (status < 0) {
		perror("SPI_IOC_MESSAGE");
		return;
	}
	printf("response(%2d, %2d): ", xfer[0].len, status);
	printf("\n");
}

static void flash_wait_wel(int fd)
{
	struct spi_ioc_transfer	xfer[1];
	unsigned char		buftx[32], *bp, bufrx[32];
	int			status;

	memset(xfer, 0, sizeof(xfer));
	memset(buftx, 0, sizeof(buftx));
	memset(bufrx, 0, sizeof(bufrx));

	buftx[0] = 0x05;
	xfer[0].tx_buf = (unsigned long)buftx;
	xfer[0].len = 2;
	xfer[0].rx_buf = (unsigned long)bufrx;
	while (1) {
		status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
		if (status < 0) {
			perror("SPI_IOC_MESSAGE");
			return;
		}
		printf("response(%2d, %2d): ", xfer[0].len, status);
		bp = bufrx + 1;
		printf(" %02x", *bp);
		if (*bp & 0x2)
			break;
		printf("\n");
	}
	printf("\n");
}

static void flash_wait_busy(int fd)
{
	struct spi_ioc_transfer	xfer[1];
	unsigned char		buftx[32], *bp, bufrx[32];
	int			status;

	memset(xfer, 0, sizeof(xfer));
	memset(buftx, 0, sizeof(buftx));
	memset(bufrx, 0, sizeof(bufrx));

	buftx[0] = 0x05;
	xfer[0].tx_buf = (unsigned long)buftx;
	xfer[0].len = 2;
	xfer[0].rx_buf = (unsigned long)bufrx;
	while (1) {
		status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
		if (status < 0) {
			perror("SPI_IOC_MESSAGE");
			return;
		}
		printf("response(%2d, %2d): ", xfer[0].len, status);
		bp = bufrx + 1;
		printf(" %02x", *bp);
		if (!(*bp & 0x1))
			break;
		printf("\n");
	}
	printf("\n");
}

static void do_msg(int fd, int len, int commandnum)
{
	struct spi_ioc_transfer	xfer[2];
	unsigned char		buftx[32], *bp, bufrx[32];
	int			status;

	memset(xfer, 0, sizeof(xfer));
	memset(buftx, 0, sizeof(buftx));
	memset(bufrx, 0, sizeof(bufrx));

	if (len > sizeof(buftx))
		len = sizeof(buftx);
	switch (commandnum) {
		/*read flash id*/
		/*1byte cmd + 3byte byte*/
	case 1:
		buftx[0] = 0x9f;
		xfer[0].tx_buf = (unsigned long)buftx;
		xfer[0].len = 4;
		xfer[0].rx_buf = (unsigned long)bufrx;
		break;
		/*write register*/
		/*1byte cmd + 2byte data*/
	case 2:
		buftx[0] = 0x01;
		buftx[1] = 0x00;
		buftx[2] = 0x07;
		xfer[0].tx_buf = (unsigned long)buftx;
		xfer[0].len = 3;
		/*before write register need to send write enable*/
		write_enable(fd);
		/*wait wel is ready*/
		flash_wait_wel(fd);
		/*wait flash ready*/
		flash_wait_busy(fd);
		break;
		/*read configuration register*/
		/*1byte cmd + 1byte data*/
	case 3:
		buftx[0] = 0x15;
		xfer[0].tx_buf = (unsigned long)buftx;
		xfer[0].len = 2;
		xfer[0].rx_buf = (unsigned long)bufrx;
		break;
		/*read flash data*/
	case 4:
		/* 1byte cmd + 3byte address*/
		buftx[0] = 0x03;
		buftx[1] = 0x00;
		buftx[2] = 0x00;
		buftx[3] = 0x55;
		xfer[0].tx_buf = (unsigned long)buftx;
		xfer[0].len = 8;
		xfer[0].rx_buf = (unsigned long)bufrx;
		break;
		/*write flash data*/
	case 5:
		/*1byte cmd + 3byte address + 4byte data*/
		buftx[0] = 0x02;
		buftx[1] = 0x00;
		buftx[2] = 0x00;
		buftx[3] = 0x55;
		buftx[4] = 0x11;
		buftx[5] = 0x22;
		buftx[6] = 0x33;
		buftx[7] = 0x44;
		xfer[0].tx_buf = (unsigned long)buftx;
		xfer[0].len = 8;
		/*write flow first is write enable*/
		write_enable(fd);
		/*wait wel is ready*/
		flash_wait_wel(fd);
		break;
		/*erase flash*/
	case 6:
		/*1byte cmd + 3byte address*/
		buftx[0] = 0xd8;
		buftx[1] = 0x00;
		buftx[2] = 0x00;
		buftx[3] = 0x55;
		xfer[0].tx_buf = (unsigned long)buftx;
		xfer[0].len = 4;
		/*write flow first is write enable*/
		write_enable(fd);
		/*wait wel is ready*/
		flash_wait_wel(fd);
		break;
	default:
			printf("not supported operation\n");
			return;
	}

	status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSAGE");
		return;
	}

	printf("response(%2d, %2d): ", len, status);
	for (bp = bufrx; len; len--)
		printf(" %02x", *bp++);
	printf("\n");
	if ((commandnum == 5) || (commandnum == 2) || (commandnum == 6))
		flash_wait_busy(fd);
}

static void dumpstat(const char *name, int fd)
{
	__u8	mode, lsb, bits;
	__u32	speed;

	if (ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0) {
		perror("SPI rd_mode");
		return;
	}
	if (ioctl(fd, SPI_IOC_RD_LSB_FIRST, &lsb) < 0) {
		perror("SPI rd_lsb_fist");
		return;
	}
	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
		perror("SPI bits_per_word");
		return;
	}
	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
		perror("SPI max_speed_hz");
		return;
	}

	printf("%s: spi mode %d, %d bits %sper word, %d Hz max\n",
		name, mode, bits, lsb ? "(lsb first) " : "", speed);
}

int main(int argc, char **argv)
{
	int		c;
	int commandnum = 0;
	int		readcount = 0;
	int		msglen = 0;
	int		fd;
	const char	*name;
	int ret = 0;

	while ((c = getopt(argc, argv, "ho:m:r:v")) != EOF) {
		switch (c) {
		case 'o':
			commandnum = atoi(optarg);
			if (commandnum < 0)
				goto usage;
			continue;
		case 'm':
			msglen = atoi(optarg);
			if (msglen < 0)
				goto usage;
			continue;
		case 'r':
			readcount = atoi(optarg);
			if (readcount < 0)
				goto usage;
			continue;
		case 'v':
			verbose++;
			continue;
		case 'h':
		case '?':
usage:
			fprintf(stderr,
	"usage: %s [-h] [-o N] [-m N] [-r N] /dev/spidevB.D\n"
					"[-o 1: read id] [-m 4]\n"
					"[-o 2: write register] [-m 3]\n"
					"[-o 3: read register] [-m 2]\n"
					"[-o 4: read data] [-m 8]\n"
					"[-o 5: write data][-m 8]\n"
					"[-o 6: erase][-m 4]\n",
				argv[0]);
			return 1;
		}
	}

	if ((optind + 1) != argc)
		goto usage;
	name = argv[optind];

	fd = open(name, O_RDWR);
	if (fd < 0) {
		perror("open");
		return 1;
	}

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		perror("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		perror("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		perror("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		perror("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		perror("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		perror("can't get max speed hz");

	dumpstat(name, fd);

	if (msglen)
		do_msg(fd, msglen, commandnum);

	if (readcount)
		do_read(fd, readcount);

	close(fd);
	return 0;
}
