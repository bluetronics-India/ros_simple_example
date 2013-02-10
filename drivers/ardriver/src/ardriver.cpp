/* ardriver.cpp

   Example code to read serial data, process it, and publish a 
   message containing the processed data 

   The Arduino should be attached to ttyUBS0. It will wait for
   any serial data. When data is received it will acknowledge
   the data by sending "OK\n". It will then begin a full sweep
   (left or right) and return each point of data on a new line
   followed by the string "END\n". There will be 25 points of
   data. The Arduino will then wait for more serial data before
   starting the process again.

   Example output from arduino:

   ============= START ============
   OK
   0
   0
   500
   300
   10
   10
   16
   16
   15
   15
   14
   14
   10
   20
   21
   21
   21
   27
   33
   33
   50
   0
   0
   500
   10
   END
   ============= END ============

   This example code sends data to the arduino, waits for the
   response, processes the response, and then publishes it.

   Example published string:
   0,0,500,300,10,10,16,16,15,15,14,14,10,20,21,21,21,27,33,33,50,0,0,500,10

   Author: Daniel Davies

*/

#include <fcntl.h>   
#include <termios.h> 
#include "ros/ros.h"
#include "std_msgs/String.h"

int fd; 						/* File descriptor for the serial port */
char msgReceivedFlag = 0;		/* Flag indicating if a message was received */
int lfCounter = 0;				/* Counts the number of incoming line feeds. Used for basic
								   message integrity checks */
char messageBuffer[134];		/* Buffer to store incoming data */
int charCounter = 0;			/* Keeps track of our position in messageBuffer */
std_msgs::String lastMessage;	/* The last message received */

void read_port(void);
int open_port(void);
void config_port(void);
int write_port(unsigned char *, unsigned int);

/*****************************************************
** Reads the data from serial. Checks for messages	**
** If a message is found, sets msgReceivedFlag      **
*****************************************************/
void read_port(void){
	
	/* Read serial data into our buffer */
	char returnBuffer[134];
	int retval = read(fd,returnBuffer,sizeof(returnBuffer));
	char *bufPos;

	if (retval > 0) {
		bufPos = &returnBuffer[0];
		for (int i = 0; i < retval; i++) {

			/* Search for message data */
			if (bufPos[i] == 'O') { //Start of message
				charCounter = 0; 
				lfCounter = 0;
			}
			else if ((bufPos[i] >= '0') && (bufPos[i] <= '9')) { //Number
				messageBuffer[charCounter] = bufPos[i];
				charCounter = (charCounter + 1) % 134; //Prevent overflow
			}
			else if (bufPos[i] == (char)10) { //LF
				lfCounter++; //Count the lf's, to check message integrity
				if ((lfCounter > 1) && (lfCounter < 26)) {
					messageBuffer[charCounter] = ',';
					charCounter = (charCounter + 1) % 134; //Prevent overflow
				}
			}
			else if (bufPos[i] == 'D') { //End of message
				if (lfCounter == 26) {
					//Valid message received!
					messageBuffer[charCounter] = 0; //Add a null char

					lastMessage.data = std::string((char *)messageBuffer); //Store the message
					msgReceivedFlag = 1;
				}
			}
		}
	}
}

/*********************************
** Opens serial port ttyUSB0	**
*********************************/
int open_port(void){	

	// Open the USB port
	fd = open("/dev/ttyUSB0", O_RDWR | O_NDELAY | O_NOCTTY);
	if (fd == -1){
		ROS_ERROR("ARDUINO DRIVER Could not open port /dev/ttyUSB0");
		return 0;
	}
	else{
		fcntl(fd, F_SETFL, 0);
	}
	return (fd);
}

/*********************************
** Configures the serial port	**
*********************************/
void config_port(void){
	struct termios options;

	//Get the parameters associated with the terminal
	tcgetattr(fd, &options);

	// Use 9600 baud for both input and output
	cfsetispeed(&options, B9600);
	cfsetospeed(&options, B9600);
		
	// Ignore Modem Status Lines, Enable Receiver, 
	// 8 Bit Character Size, 1 Stop Bit, No Parity	
	options.c_cflag |= (CLOCAL | CREAD | CS8);

	options.c_iflag = IGNPAR; 	//Ignore Chacters with Parity Errors
	options.c_oflag = 0;		//No Output Options
	options.c_lflag = 0;		//No Terminal Options
	options.c_cc[VTIME] = 10;	
	options.c_cc[VMIN] = 0;
	tcflush(fd, TCIFLUSH);

	tcsetattr(fd, TCSANOW, &options); //Update attributes now

	return;
}


/*********************************
** Send some data over serial	**
*********************************/
int write_port(unsigned char sendBuffer[], unsigned int sendSize)
{
	/* Write to the serial port */
	int n = write(fd,sendBuffer, sendSize);

	if (n < 0) ROS_ERROR("ARDUINO DRIVER WRITE FAIL"); //Failed

	return (n);
}

int main(int argc, char **argv){ //we need argc and argv for the rosInit function

	/* Initialize ROS - This must be called before any other ROS functions work */
	ros::init(argc, argv, "ardriver");

	/* Create an access point to communicate with ROS */
	ros::NodeHandle ardriverN;

	/* Set up our publishers. In this case, we are publishing a string of data called
	   arduino_data. 100 is the number of messages to buffer before discarding them */
	ros::Publisher pubArduino = ardriverN.advertise<std_msgs::String>("arduino_data", 100);

	/* This is the loop rate (Hz) for our program to run - It's not required in all nodes.
	   We use this here instead of spin() or spin_once() since we are not listening to
	   any topics. */
	ros::Rate loop_rate(15);

	/* If we fail to open the serial port, end the node */
	if(!open_port()) return 0;

	/* Configure our serial port */
	config_port();
	ROS_INFO("ARDUINO DRIVER ONLINE");

	/* Send some data over the serial. This makes our servo move and collect data */
	if (write_port((unsigned char *)"a", 1) >= 0) {

		while (ros::ok()) {
		
			/* Read any data sent over the serial */		
			read_port();

			if (msgReceivedFlag == 1) { //We received some data
			
				/* Publish the data and reset the flag */
				pubArduino.publish(lastMessage); 
				msgReceivedFlag = 0; 

				/* Send some more data */
				if (write_port((unsigned char *)"a", 1) < 0) break;
			}

			/*Have a snooze*/
			loop_rate.sleep();

		}
	}

	/* Close the file, and exit */
	close(fd);
	ROS_INFO("ARDUINO DRIVER SHUTTING DOWN");

	return 0;
}

