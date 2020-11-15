/* (C) 2011 by Duncan Salerno <duncan.salerno@googlemail.com>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "gsmtap.h"

#define FBUS_BAUD B115200

#define FBUS2_FRAME_ID       	0x1e
#define FBUS2_DEVICE_PHONE   	0x00 /* Nokia mobile phone */
#define FBUS2_DEVICE_PC      	0x0c /* Our PC */
#define FBUS2_ACK_BYTE	     	0x7f /* Acknowledge of the received frame */
#define FBUS2_MAX_TRANSMIT_LENGTH 120

#define FBUS_TIMEOUT 1000000

#define CHANNEL_SDCCH 128
#define CHANNEL_SACCH 112
#define CHANNEL_FACCH 176
#define CHANNEL_CCCH 96
#define CHANNEL_BCCH 80
#define CHANNEL_SCH 64

//static const unsigned char msg_0xd1[] = { 0x00, 0x01, 0x00, 0x03, 0x00, 0x01, 0x60 };
static const unsigned char msg_1_0x40[] = { 0x00, 0x01, 0x64, 0x01, 0x01 };
static const unsigned char msg_2_0x40[] = { 0x00, 0x01, 0x70, 0x00, 0x00, 0x00,
  0xc0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };
static const unsigned char reqDisable[] = { 0x01, 0x01, 0x71, 0x01 };
static const unsigned char reqVersion[] = { 0x00, 0x01, 0x00, 0x03, 0x00, 0x01 };

static volatile int run = 1;
static int gsmtap_fd;
static int serial_fd;


/*
 * Serial comms functions
 */

static int serial_read(void *buf, size_t nbytes)
{
    	struct timeval timeout = { 0, FBUS_TIMEOUT };
    	fd_set readfds;
    	int actual;

    	FD_ZERO(&readfds);
    	FD_SET(serial_fd, &readfds);

    	actual = select(serial_fd+1, &readfds, NULL, NULL, &timeout);
	if( actual > 0 )
	{
		actual = read(serial_fd, buf, nbytes);
		if( actual == 0 )
		{
			/* EOF - Terminate program */
			run = 0;
			return -1;
		}
	}
	
	if( actual == -1 && errno != EINTR )
	{
		/* Serious problem, terminate program */
		perror("serial_read");
		run = 0;
	}

    	return actual;
}


static int serial_init(const char *serial_port)
{
	struct termios t;
	int fd;

	/* O_NONBLOCK is required to avoid waiting for DCD */
	fd = open(serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0)
	{
		perror("open");
		return -1;
	}

#ifdef TIOCEXCL
	/* open() calls from other applications shall fail now */
	ioctl(fd, TIOCEXCL, (char *) 0);
#endif
#ifdef HAVE_I_SETSIG
	/* Disable any signals from this file */
	ioctl(fd, I_SETSIG, (char *) 0);
#endif

	/* Use previous settings as start */
	if (tcgetattr(fd, &t) == -1)
	{
		perror("tcgetattr");
		return -1;
    	}

	/* Opening without parity */
    	t.c_iflag       = IGNPAR;
    	t.c_oflag       = 0;
    	/* disconnect line, 8 bits, enable receiver,
     	 * ignore modem lines,lower modem line after disconnect
     	 */
    	t.c_cflag       = B0 | CS8 | CREAD | CLOCAL | HUPCL;
    	t.c_lflag       = 0;
    	t.c_cc[VMIN]    = 1;
    	t.c_cc[VTIME]   = 0;

#ifdef CRTSCTS
    	/* Disabling hardware flow control */
    	t.c_cflag &= ~CRTSCTS;
#endif

	unsigned int v24 = TIOCM_DTR | TIOCM_RTS;
	if (ioctl(fd, TIOCMBIS, &v24) < 0)
	{
		perror("ioctl(TIOCMBIS)");
		return -1;
	}

    	cfsetispeed(&t, FBUS_BAUD);
    	cfsetospeed(&t, FBUS_BAUD);

    	if (tcsetattr(fd, TCSADRAIN, &t) == -1)
	{
		perror("tcsetattr");
		return -1;
    	}

	return fd;
}


/*
 * FBUS functions
 */


static void fbus_init()
{
	unsigned int count;
	unsigned char buff[255];
	static const unsigned char init_char = 0x55;

	/* Send init sequence */
	for (count = 0; count < 128; count++)
		write(serial_fd, &init_char, 1);

	/* Read any possible junk on the line */
	while (serial_read(buff, sizeof(buff)) > 0)
		usleep(1000);
}


/* FBUS docs: http://www.embedtronics.com/nokia/fbus.html */
static int fbus_write_frame(const unsigned char *msg, unsigned int msg_len, unsigned char type, unsigned char seq)
{
	unsigned char buffer[FBUS2_MAX_TRANSMIT_LENGTH + 10];
	unsigned const char *buf = buffer;
	unsigned char checksum;
	int i, length, sent = 0;

	if( msg_len > FBUS2_MAX_TRANSMIT_LENGTH )
		return -1;

	/* Add space for sequence */
	msg_len++;

	buffer[0] = FBUS2_FRAME_ID;
	buffer[1] = FBUS2_DEVICE_PHONE; /* destination */
	buffer[2] = FBUS2_DEVICE_PC; /* source */
	buffer[3] = type;
	buffer[4] = msg_len / 256;
	buffer[5] = msg_len % 256;
	memcpy(buffer + 6, msg, msg_len-1); /* Excluding Sequence */
	buffer[6+msg_len-1] = seq;
	length = msg_len + 6;

	/* Odd messages require additional 0x00 byte */
	if (msg_len % 2)
		buffer[length++] = 0x00;

	checksum = 0;
	for (i = 0; i < length; i+=2)
		checksum ^= buffer[i];

	buffer[length++] = checksum;

	checksum = 0;
	for (i = 1; i < length; i+=2)
		checksum ^= buffer[i];

	buffer[length++] = checksum;

	/* Sending to phone */
#ifdef DEBUG
	printf("TX frame: ");
	for( i = 0; i < length; i++ )
		printf("%02x ", buf[i]);
	printf("\n");
#endif
   	do
	{
		int ret = write(serial_fd, buf, length - sent);
		if (ret < 0)
		{
			if (errno == EAGAIN)
			{
				usleep(1000);
				continue;
			}
			break;
		}
		sent += ret;
		buf += ret;
    	} while (sent < length);

	if (sent != length)
		return -1;

	return 0;
}


static int fbus_write_message(const unsigned char *buf, int length, unsigned char type)
{
	static unsigned char seq = 0x4f; /* So first is 0x40 */
	seq = 0x40 | ((seq+1) & 0xf);
	return fbus_write_frame(buf, length, type, seq);
}


static int fbus_send_ack(unsigned char type, unsigned char seq)
{
	unsigned char buffer[1]={type};
	return fbus_write_frame(buffer, sizeof(buffer), FBUS2_ACK_BYTE, seq);
}


static int fbus_read_message(unsigned char* out_type, unsigned char** out_message, size_t *out_len)
{
	static unsigned char frame_buf[65535 + 11]; /* Theoretical max len, plus header, cksum etc */
	static size_t ptr = 0, remaining = 0;

	do
	{
		/* Resync if necessary */
		while( remaining && frame_buf[ptr] != FBUS2_FRAME_ID )
		{
			remaining--;
			ptr++;
#ifdef DEBUG
			fprintf(stderr, "Resyncing\n");
#endif
		}

		/* Do we have a frame? */
		if( remaining >= 6 )
		{
			size_t payload_len = frame_buf[ptr+4]*256 + frame_buf[ptr+5];
			size_t frame_len = payload_len + 8;

			if (payload_len % 2) /* Any padding? */
				frame_len++;
			if( frame_len <= remaining )
			{
				unsigned char *frame = frame_buf + ptr;
				remaining -= frame_len;
				ptr += frame_len;

				/* Calculate checksum */
				int i;
				unsigned char checksum1 = 0, checksum2 = 0;;
				for (i = 0; i < frame_len-2; i+=2)
					checksum1 ^= frame[i];

				for (i = 1; i < frame_len-1; i+=2)
					checksum2 ^= frame[i];

				/* Validate checksum */
				if( frame[frame_len-2] == checksum1 && frame[frame_len-1] == checksum2 )
				{
#ifdef DEBUG
					printf("RX frame: ");
					for( i = 0; i < frame_len; i++ )
						printf("%02x ", ((unsigned char*)frame)[i]);
					printf("\n");
#endif
					/* Check message is for us, and its not an ack */
					if( frame[1] == FBUS2_DEVICE_PC && frame[2] == FBUS2_DEVICE_PHONE && frame[3] != FBUS2_ACK_BYTE)
					{
						/* Send ack for non-debug frames */
						if( frame[3] != 0 )
							fbus_send_ack(frame[3], frame[6+payload_len-1] & 0x0f);
						*out_type = frame[3];
						*out_message = frame + 6;
						*out_len = payload_len - 1; /* Don't send sequence */
						return 0;
					}
				}
#ifdef DEBUG
				else
					fprintf(stderr, "Dropped frame with bad checksum\n");
#endif
			}

			/* Drop frame and continue */
		}

		/* Shift any remaining buffer contents to beginning */
		if( remaining )
			memmove(frame_buf, frame_buf+ptr, remaining);
		ptr = 0;

		/* Need to read more data */
		int res = serial_read(frame_buf+remaining, 65535-remaining);
		if( res > 0 )
			remaining += res;
		else if( res == 0 )
			return -1; /* Timeout */
		else
			return -2; /* Error or EINTR */
	} while(1);
}


static int fbus_receive_message(unsigned char expected_type, unsigned char** out_message, size_t *out_len)
{
	/* Keep receiving messages until correct type received, or timeout */
	while( run )
	{
		unsigned char out_type;	
		int ret = fbus_read_message(&out_type, out_message, out_len);
		if( ret == 0 )
		{
			if( out_type == expected_type )
				return 0;
#ifdef DEBUG
			else
				printf("Wanted msg 0x%x, got 0x%x\n", expected_type, out_type);
#endif
		}
		else
			break;
	}

	return -1;
}


static int fbus_send_message(const unsigned char *buf, int length, unsigned char type, unsigned char** out_message, size_t *out_len)
{
	/* Write message to phone */
	if( fbus_write_message(buf, length, type) == -1 )
		return -1;

	/* Wait for response to message */
	return fbus_receive_message(type, out_message, out_len);
}


/*
 * Message processing functions
 */


static void proc_sim(const unsigned char *apdu, unsigned char len)
{
	const size_t buf_len = sizeof(struct gsmtap_hdr) + len;
	unsigned char buf[buf_len];
	struct gsmtap_hdr *gh = (struct gsmtap_hdr *) buf;

	printf("SIM: ");
	int i;
	for( i = 0; i < len; i++ )
		printf("0x%02X ", apdu[i]);
	printf("\n");
	memset(gh, 0, sizeof(*gh));
	gh->version = GSMTAP_VERSION;
	gh->hdr_len = sizeof(*gh)/4;
	gh->type = GSMTAP_TYPE_SIM;

	memcpy(buf + sizeof(*gh), apdu, len);

	if (write(gsmtap_fd, buf, buf_len) < 0)
		perror("write gsmtap");
}


/* MDISND debug messages: http://nokix.pasjagsm.pl/help/blacksphere/sub_100hardware/sub_dsp/sub_mdi/sub_typestxd.htm */
static void proc_mdisnd(unsigned char subtype, unsigned const char *mdi, unsigned char mdi_len)
{
	struct gsmtap_hdr gh = { .version = GSMTAP_VERSION,
                                 .hdr_len = sizeof(struct gsmtap_hdr)/4,
                                 .type = GSMTAP_TYPE_UM,
                                 .sub_type = GSMTAP_CHANNEL_UNKNOWN,
				 .arfcn = htons(GSMTAP_ARFCN_F_UPLINK) };

	/* 2 byte MDI header, followed by subtype specific payload */
	if( mdi_len < 2 || mdi[1] != subtype )
		return;

	/* Subtype 0x0c - RACH */
	if( subtype == 0x0C && mdi_len > 4 && mdi[3] == 0x0)
	{
		printf("MDISND: RACH %02X\n", mdi[4]);
		gh.sub_type = GSMTAP_CHANNEL_RACH;
		mdi_len = 5; /* Truncate trailing stuff after CHANNEL REQUEST */
	}

	/* Subtype 0x1b - GSM L2 */
	if(subtype == 0x1B && mdi_len > 4) {
		printf("MDISND %02X: ch=%u (0x%02X) len=%u\n", mdi[2], mdi[3], mdi[3], mdi_len-4);
		switch(mdi[3])
		{
			case CHANNEL_SDCCH:
				gh.sub_type = GSMTAP_CHANNEL_SDCCH;
				break;
			case CHANNEL_FACCH:
				gh.sub_type = GSMTAP_CHANNEL_TCH_F; /* Assume full rate */
				break;
			case CHANNEL_SACCH:
				gh.sub_type = GSMTAP_CHANNEL_ACCH | GSMTAP_CHANNEL_SDCCH;
				break;
		}
	}

	if( gh.sub_type != GSMTAP_CHANNEL_UNKNOWN )
	{
		const size_t buf_len = sizeof(struct gsmtap_hdr) + mdi_len-4;
		unsigned char buf[buf_len];

		memcpy(buf, &gh, sizeof(struct gsmtap_hdr));
		memcpy(buf+sizeof(struct gsmtap_hdr), &mdi[4], mdi_len-4); 

		if (write(gsmtap_fd, buf, buf_len) < 0)
			perror("write gsmtap");
	}
}


/* MDIRCV debug messages: http://nokix.pasjagsm.pl/help/blacksphere/sub_100hardware/sub_dsp/sub_mdi/sub_typesrxd.htm */
static void proc_mdircv(unsigned char subtype, unsigned const char *mdi, unsigned char mdi_len)
{
	struct gsmtap_hdr gh = { .version = GSMTAP_VERSION,
                                 .hdr_len = sizeof(struct gsmtap_hdr)/4,
                                 .type = GSMTAP_TYPE_UM,
                                 .sub_type = GSMTAP_CHANNEL_UNKNOWN };

	/* 2 byte MDI header, followed by subtype specific payload */
	if( mdi_len < 2 || mdi[1] != subtype )
		return;

	/* Subtype 0x80 - GSM L2 */
	if( subtype == 0x80 && mdi_len > 11 )
	{
		gh.frame_number = htonl((mdi[5]<<16) | (mdi[6]<<8) | mdi[7]);
		gh.arfcn = htons((mdi[8]<<8) | mdi[9]);
		printf("MDIRCV %02X: ch=%u (0x%02X) bsic=%u err=%u fn=%u arfcn=%u shift=%u len=%u\n", subtype, mdi[2], mdi[2], mdi[3], mdi[4], (mdi[5]<<16)|(mdi[6]<<8)|(mdi[7]), (mdi[8]<<8)|mdi[9], (mdi[10]<<8)|mdi[11], mdi_len-2);
		switch( mdi[2] )
		{
			case CHANNEL_SACCH:
				gh.sub_type = GSMTAP_CHANNEL_ACCH | GSMTAP_CHANNEL_SDCCH;
				break;
			case CHANNEL_SDCCH:
				gh.sub_type = GSMTAP_CHANNEL_SDCCH;
				break;
			case CHANNEL_FACCH:
				gh.sub_type = GSMTAP_CHANNEL_TCH_F; /* Assume full rate */
				break;
			case CHANNEL_BCCH:
				if( (mdi[12] & 3) == 1 )
					gh.sub_type = GSMTAP_CHANNEL_BCCH;
				break;
			case CHANNEL_CCCH:
				if( (mdi[12] & 3) == 1 )
					gh.sub_type = GSMTAP_CHANNEL_CCCH;
				break;
		}
	}

	if( gh.sub_type != GSMTAP_CHANNEL_UNKNOWN )
	{
		const size_t buf_len = sizeof(struct gsmtap_hdr) + mdi_len-12;
		unsigned char buf[buf_len];

		memcpy(buf, &gh, sizeof(struct gsmtap_hdr));
		memcpy(buf+sizeof(struct gsmtap_hdr), &mdi[12], mdi_len-12); 

		if (write(gsmtap_fd, buf, buf_len) < 0)
			perror("write gsmtap");
	}
}


static void gsmtap_open(const char *gsmtap_host)
{
	struct sockaddr_in sin;

	sin.sin_family= AF_INET;
	sin.sin_port = htons(GSMTAP_UDP_PORT);

	if (inet_aton(gsmtap_host, &sin.sin_addr) < 0)
	{
		perror("parsing GSMTAP destination address");
		exit(2);
	}
	gsmtap_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (gsmtap_fd < 0)
	{
		perror("GSMTAP socket initialization");
		exit(2);
	}
	if (connect(gsmtap_fd, (struct sockaddr *)&sin, sizeof(sin)) < 0)
	{
		perror("connecting GSMTAP socket");
		exit(2);
	}
}


static void interrupt(int sign)
{
	signal(sign, SIG_IGN);
	run = 0;
}


int main(int argc, char **argv)
{
	if( argc != 3 )
	{
		fprintf(stderr, "Usage: %s device destination\n\n\teg. %s /dev/ttyUSB0 224.0.0.1\n\n", argv[0], argv[0]);
		return 1;
	}

	gsmtap_open(argv[2]);

	serial_fd = serial_init(argv[1]);
	if( serial_fd < 0 )
		return 1;

	fbus_init();

	unsigned char *msg;
	unsigned char type;
	size_t len;

	/* Try disabling debug, just in case it's already on to make things more sane */
	fbus_write_message(reqDisable, sizeof(reqDisable), 0x40);
	fbus_read_message(&type, &msg, &len);

	/* Get info about connected phone */
	if( fbus_write_message(reqVersion, sizeof(reqVersion), 0xd1) != 0)
	{
		fprintf(stderr, "Failed to request version\n");
		return 1;
	}
	if( fbus_receive_message(0xd2, &msg, &len) != 0 )
	{
		fprintf(stderr, "Failed get version response\n");
		return 1;
	}
	printf("Phone on %s:\n%s\n", argv[1], msg+4);

	/* Enable security */
	if( fbus_send_message(msg_1_0x40, sizeof(msg_1_0x40), 0x40, &msg, &len) != 0 )
	{
		fprintf(stderr, "Failed to enable security\n");
		return 1;
	}

	/* Enable debug */
	if( fbus_send_message(msg_2_0x40, sizeof(msg_2_0x40), 0x40, &msg, &len) != 0 )
	{
		fprintf(stderr, "Failed to enable debug mode\n");
		return 1;
	}

	signal(SIGINT, interrupt);
	printf("Press Ctrl+C to interrupt...\n");

	unsigned char seq = 0;
	while( run )
	{
		if( fbus_read_message(&type, &msg, &len) == 0 && type == 0x0 )
		{
			/* Anatomy of debug messages: http://nokix.pasjagsm.pl/help/blacksphere/sub_200nokiaos/sub_fbus/sub_zdebug.htm */
			/* Process 8 byte header, followed by payload */
			unsigned char type = msg[2];
			unsigned char subtype = msg[3];
			unsigned char seq_nr = msg[6];
			unsigned char payload_len = msg[7];
			unsigned const char *payload = msg+8;

			if( payload_len + 8 > len )
			{
				fprintf(stderr, "Truncated payload\n");
				continue;
			}
#ifdef DEBUG
			unsigned short timestamp = (msg[4]<<8) | msg[5];
			printf("<%02X%02X> t=%04x seq_nr=%02x payload_len=%u payload=", type, subtype, timestamp, seq_nr, payload_len);
			int ii;
			for( ii = 0; ii < payload_len; ii++ )
			{
				printf("%02x ", payload[ii]);
			}
			printf("\n");
#endif
			static int first = 1;
			if( seq != seq_nr )
			{
				if( first == 0 )
					fprintf(stderr, "Missed some packets expected 0x%x, got 0x%x\n", seq, seq_nr);
				seq = seq_nr;
			}
			first = 0;
			seq++;

			/* Debug types: http://nokix.pasjagsm.pl/help/blacksphere/sub_200nokiaos/sub_fbus/sub_zdebug/sub_types.htm */
			switch(type) {
				case 0x18:
					proc_mdisnd(subtype, payload, payload_len);
					break;
				case 0x19:
					proc_mdircv(subtype, payload, payload_len);
					break;
			 	case 0x20:
				case 0x23:
					proc_sim(payload, payload_len);
			 		break;
			}
		}
	}

	/* Disable debug before return */
	printf("Disabling debug\n");
	fbus_write_message(reqDisable, sizeof(reqDisable), 0x40);
	fbus_read_message(&type, &msg, &len);

	return 0;
}

