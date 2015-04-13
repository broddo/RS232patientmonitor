#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>

/* system commands */
#define NAK			0x15
#define EOT			0x04
#define SYNC_BYTE 		0xA5
#define PATIENT_DEMOGRAPHICS	0x56
#define PARAMETER_DATA		0x57
#define PARAMETER_DATA_2	0x77
#define SPO2_PULS_NIBP		0x75
#define STATUS_COMMAND		0x50

/* Device Types */
#define MODULAR_MONITOR		0x01
#define CONFIGURED_MONITOR	0x02
#define RESERVED		0x09

/* Languages codes */
#define	ENGLISH			0x01
#define	GERMAN			0x02
#define	FRENCH			0x03
#define	SPANISH			0x04
#define	ITALIAN			0x05
#define	DUTCH			0x06
#define	SWEDISH			0x07
#define	JAPANESE_KATAKANA	0x08
#define	JAPANESE_KANJI		0x09
#define	JAPANESE_HIRAGANA	0x0A
#define	DANISH			0x0B
#define	PORTUGUESE		0x0C
#define	GREEK			0x0D
#define	FINNISH			0x0E
#define	NORWEGIAN		0x0F
#define	POLISH			0x10
#define	HUNGARIAN		0x11
#define	ROMANIAN		0x12
#define	SLOVAKIAN		0x13
#define	RUSSIAN			0x14
#define	CHINESE			0x15
#define	ARABIC			0x16

/* Server-status */
#define ACTIVE_STATE			0x00
#define STANDBY_STATE			0x01
#define PATIENT_DISCHARGED_STATE	0x02

typedef struct
{
	uint8_t century;	// e.g. 20
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;		// 0 - 23
	uint8_t min;
	uint8_t sec;
	uint8_t spare;
} __attribute__((__packed__)) date_t;

typedef struct
{
	char hwsn[11];
	
	struct
	{
		uint8_t alarm;
		uint8_t param_cond;
		uint8_t percent;
	} __attribute__((__packed__)) SPO2;
	
	uint8_t pulse;
	
	struct
	{
		uint8_t units;
		
		uint8_t mean_alarm;
		uint8_t mean_param_cond;
		uint8_t mean_value;
		
		uint8_t sys_alarm;
		uint8_t sys_param_cond;
		uint8_t sys_value;
		
		uint8_t dia_alarm;
		uint8_t dia_param_cond;
		uint8_t dia_value;
		
	} __attribute__((__packed__)) NiBP;
	
	uint8_t checksum;
	
} __attribute__((__packed__)) ndp_spo2_t;

typedef struct
{
	uint8_t 	sync_byte;
	uint16_t 	length;
	uint8_t		transaction_code;
} __attribute__((__packed__)) packet_header_t;

static int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

static void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf("error %d setting term attributes", errno);
}

static void send_command(int fd, uint8_t command)
{
	int i;
	uint8_t msg[5];
	uint8_t msg_length = sizeof(msg);
	uint8_t checksum = 0;

	msg[0] = SYNC_BYTE;	// Sync byte
	msg[1] = 0x02;		// Length (LSB)
	msg[2] = 0x00;		// Length (MSB)
	msg[3] = command;	// Transaction code

	/* calculate checksum over all bytes in msg except for the checksum
	 * itself 
	 */
	for(i=0;i<msg_length-1;i++)
	{
		checksum += msg[i];
	}
	msg[4] = checksum;

	write(fd, msg, msg_length);
}

void decode_status_command(uint8_t *msg, uint8_t msg_length)
{
	int i = 0;
	int j = 0;
	char sw_version[16];
	char protocol_rev[6];
	date_t date;
	printf("Connection established to instrument\n");

	printf("\tMonitor type: ");
	switch (msg[0]) // device
	{
		case MODULAR_MONITOR:
			printf("modular");
		case CONFIGURED_MONITOR:
			printf("configured");
		default:
			printf("undefined");
	}
	printf("\n");

	printf("/tLanguage: ");
	switch(msg[1]) // monitor
	{
		case	ENGLISH:	printf("English	");
		case	GERMAN:		printf("German	");
		case	FRENCH:		printf("French	");
		case	SPANISH:	printf("Spanish	");
		case	ITALIAN:	printf("Italian	");
		case	DUTCH:		printf("Dutch	");
		case	SWEDISH:	printf("Swedish	");
		case	JAPANESE_KATAKANA:	printf("Japanese_Katakana	");
		case	JAPANESE_KANJI:		printf("Japanese_Kanji	");
		case	JAPANESE_HIRAGANA:	printf("Japanese_Hiragana	");
		case	DANISH:		printf("Danish	");
		case	PORTUGUESE:	printf("Portuguese	");
		case	GREEK:		printf("Greek	");
		case	FINNISH:	printf("Finnish	");
		case	NORWEGIAN:	printf("Norwegian	");
		case	POLISH:		printf("Polish	");
		case	HUNGARIAN:	printf("Hungarian	");
		case	ROMANIAN:	printf("Romanian	");
		case	SLOVAKIAN:	printf("Slovakian	");
		case	RUSSIAN:	printf("Russian	");
		case	CHINESE:	printf("Chinese	");
		case	ARABIC:		printf("Arabic	");
	}
	printf("\n");

	printf("\tSupport level: \n");
	if ((msg[2] >> 1) & 0x01)
		printf("\t\tServer supports Extended Data Mode\n");
	else
		printf("\t\tServer does not support Extended Data Mode\n");

	if ((msg[2] >> 2) & 0x02)
		printf("\t\tHigh-speed data is sampled at 200 Hz\n");
	else
		printf("\t\tHigh-speed data is sampled at 250 Hz\n");

	printf("\tServer status: ");
	switch(msg[3])
	{
		case ACTIVE_STATE: printf("Active");
		case STANDBY_STATE: printf("Standby");
		case PATIENT_DISCHARGED_STATE: printf("Discharged");
	}
	printf("\n");

	/* copy date data into date_t struct */
	memcpy(&date, &msg[4], 8);
	printf("\tServer time: %d:%d:%d %d-%d-%d\n", 
		date.hour,
		date.min,
		date.sec,
		date.day,
		date.month,
		date.year);

	/* software version */
	i = 12;
	while(1)
	{
		sw_version[j] = msg[i];
		if (msg[i] == '\0')
		{
			i++;
			j++;
			break;
		}
		i++;
		j++;
	}
	printf("\tSoftware Version: %s\n", sw_version);

	/* Protocol revision */
	j = 0;
	while(1)
        {
                protocol_rev[j] = msg[i];
                if (msg[i] == '\0')
                {
                        i++;
                        j++;
                        break;
                }
                i++;
                j++;
        }
	printf("\tProtocol rev: %s\n", protocol_rev);
}

void decode_patient_data(ndp_spo2_t patient_data)
{
	char units[6];
	uint8_t multiplier = 1;
	
	if (patient_data.NiBP.units == 0x0B)
	{
		strcpy(units, "mmHg\n");
	}
	else if (patient_data.NiBP.units == 0x0C)
	{
		strcpy(units, "kPa\n");
		multiplier = 0.1;
	}
	printf("NBP & SPO2 Data received:\n");
	printf("\tSPO2 = %d%%\n", patient_data.SPO2.percent);
	printf("\tPulse = %d BPM\n", patient_data.pulse);
	printf("\tMean BP = %.1f ", patient_data.NiBP.mean_value*multiplier);
	printf("%s", units);
	printf("\Systolic BP = %.1f ", patient_data.NiBP.sys_value*multiplier);
	printf("%s", units);
	printf("\tDiastolic BP = %.1f ", patient_data.NiBP.dia_value*multiplier);
	printf("%s", units);
}

void decode_parameter_data(uint8_t *serial_data, int serial_data_length)
{
	int i;
	unsigned int offset = 20;
	struct
	{
		uint8_t dummy1;
		uint8_t shutdnf;
		uint8_t waveform_id_bytes[8];
		uint8_t dummy2[8];
		uint8_t dummy3;
		uint8_t no_param_subpackets;
	} __attribute__((__packed__)) parameter_data_header_t;
	
	typedef struct
	{
		uint8_t length;
		uint8_t param_count;
		uint8_t reserved;
		uint8_t scale_ID;
		uint16_t top_val;
		uint16_t bot_val;
	} __attribute__((__packed__)) subpacket_header_t;
	
	parameter_data_header_t *parameter_data_header;
	subpacket_header_t *subpacket_header;

	parameter_data_header = *(parmeter_data_header_t*)serial_data;
	
	/* Check shutdown byte */
	if (parameter_data_header->shutdnf == 0x01)
	{
		printf("Monitor entering standby state\n");
		return;
	}
	else if(parameter_data_header->shutdnf == 0x02)
	{
		printf("Patient discharged\n");
		return;
	}
	
	/* Handle every subpacket */
	subpacket_header = malloc(parameter_data-header->no_param_subpackets*sizeof(subpacket_header));
	
	for (i=0;i<parameter_data_header->no_param_subpackets;i++)
	{	
		/* Grab the subpacket header */
		/* Note, if I'm dealing with all the subpackets now, then I probably don't need to store
		 * them all in a subpacket_header buffer
		 */
		memcpy(&subpacket_header[i], serial_data_length+offset, sizeof(subpacket_header_t));	
	}
	
	
	free(subpacket_header);
}

int read_data(int fd)
{
	packet_header_t serial_header;
	uint8_t *serial_data;
	int n;
	int i;
	uint8_t checksum = 0;
	
	n = read (fd, &serial_header, sizeof(packet_header_t));  
	
	if (n < 4)
	{
		return (1); 	// Header packet too small
	}
	
	/* Check the header for errors */
	if (serial_header.sync_byte != SYNC_BYTE)
	{
		return (2);	// Sync byte not received (out of sync)
	}
	
	/* Grab the rest of the data */
	serial_data = malloc(serial_header.lenth*sizeof(uint8_t));
	n = read (fd, serial_data, serial_header.length);
	if (n != serial_header.length)
	{
		free(serial_data);
		return (3); 	// Data packet too small
	}
	
	
	/* Checksum = byte wise sum of header packet + data packet (excluding checksum byte) */
	for (i=0;i<sizeof(packet_header_t);i++)
	{
		checksum += serial_header[i];	
	}
	for (i=0;i<sizeof(serial_data)-1;i++)
	{
		checksum += serial_data[i];
	}
	
	/* Compare checksum */
	if (checksum != serial_data[serial_header.length])
	{
		perror("Checksum failed on received data\n");
		free(serial_data);
		return (4);	// Checksum failed
	}

	/* Decode message */
	switch(serial_header.transaction_code)
	{
		case STATUS_COMMAND:
			decode_status_command(serial_data, serial_header.length);
			break;
		case SPO2_PULS_NIBP:
			ndp_spo2_t patient_data;
			memcpy(&patient_data, serial_data, serial_header.length)
			decode_patient_data(patient_data);
			break;
		case PARAMETER_DATA:
		case PARAMETER_DATA_2:
			decode_parameter_data(serial_data, serial_header.length);
			break;
		default:
			printf("Transaction code %x received - no handler available\n", serial_header.transaction_code);
			break;
	}
	
	free(serial_data);
	return (0);
}

int main(void)
{
	char *portname = "/dev/ttyUSB0";

	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
        	printf("error %d opening %s: %s", errno, portname, strerror (errno));
        	return;
	}

	set_interface_attribs (fd, B19200, 0);  // set speed to 19200bps, 8n1 (no parity)
	set_blocking (fd, 0);                	// set no blocking

	send_command(fd, STATUS_COMMAND);

	usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
                                     // receive 25:  approx 100 uS per char transmit
	
	read_data(fd);

}
