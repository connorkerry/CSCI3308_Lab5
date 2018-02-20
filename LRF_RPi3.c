/*
To get the serial comm to work, may need to use this command to disable the serial via GUI: sudo raspi-config -->advance settings --> serial --> disable-->reboot
To get user permission, may need to use: minicom -b 38400 -o -D /dev/ttyS0 THEN sudo chmod a+rw /dev/ttyS0 depending on which port is connected (S0 or AMA0)
NOTE:
-This demo assumes that the SF30 is set to a UART speed of 38400 Baud.
-This demo assumes that the SF30 is set to a 0.03m Snapshot Resolution. 
-This demo assumes that the SF30 is set to a 1144/sec Serial Port Update Rate
*/

#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

// Function Prototypes
void SetupUART(void);
void UARTread(void);

// Global Variables (ok for embedded systems)
int uart0_filestream;
float distance = 0;
int previousB = 0;
int currentB = 0;

int main(){
	
	// Setup UART
	SetupUART();//Initialize UART for RPi3

	// main loop: read LRF Data forever
	while (1) {
		UARTread();
		//delay(100);//delay .1s
		//delay() should be part of the wiringPi libraries. If delay() does not work, try this for loop:
		for (int i = 0; i < 1000; ++i)
		{
			asm("nop");
		}
	}

	return 0;
}

void SetupUART(){
//-------------------------
	//----- SETUP USART 0 -----
	//-------------------------
	//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
	uart0_filestream = -1;
	
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	uart0_filestream = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B38400 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
}

void UARTread(){
	//----- CHECK FOR ANY RX BYTES -----
	if (uart0_filestream != -1)
	{
		// Read up to 255 characters from the port if they are there
		unsigned char rx_buffer[256];
		int rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
		if (rx_length < 0)
		{
			//An error occured (will occur if there are no bytes)
			printf("Error: No bytes read.\n");
		}
		else if (rx_length == 0)
		{
			//No data waiting
			printf("Error: No data waiting.\n");
		}
		else
		{
			//Bytes received
			rx_buffer[rx_length] = '\0';
			currentB = rx_buffer[0];
			previousB = rx_buffer[1];
			distance = currentB * 256 + previousB;
     		distance = distance/256;
			printf("Buffer length: %i bytes. Distance = %.2f m\n", rx_length, distance);
		}
	}
}
