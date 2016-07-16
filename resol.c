/* RESOL Interface program */

#include <stdio.h>	// for FILE
#include <stdlib.h>	// for timeval
#include <string.h>	// for strlen etc
#include <time.h>	// for ctime
#include <sys/types.h>	// for fd_set
// #include <sys/socket.h>
// #include <netinet/in.h>
#include <netdb.h>	// for sockaddr_in 
#include <fcntl.h>	// for O_RDWR
#include <termios.h>	// for termios
#include <unistd.h>		// for getopt
#ifdef linux
#include <errno.h>		// for Linux
#endif
#include <sys/mman.h>	// for PROT_READ
#include <sys/stat.h>	// for struct stat
#include <getopt.h>

#include "../Common/common.h"

#define PROGNAME "Resol"
const char progname[] = "resol";
#define LOGFILE "/tmp/resol%d.log"
#define LOGON "resol"
#define BAUD B9600

/*  1.1 19/11/2007 Initial version copied from Elster 1.3
    1.2 23/11/2008 Initial version deployed at Ilfraocmbe, modified for Resol ES+
    1.3 28/11/2008 Extended debugging to collect complete rows.
    1.4 12/02/2009 Parameterised to handle more types of Resol
    1.5 27/04/2009 Blink Red LED as data comes in
    1.6 19/07/2009 Add -f factor to scale meter readings
    1.7 28/08/2009 add -m to force modeltype 0=ES+ 1=BS+
    1.8 30/08/2009 Add read/write of file /root/resol.dat instead of static structure
    1.9 09/09/2009 Attempt to deal with E version with second data packet. Scrap resols[] for now.
    1.10 18/09/2009 Back to reading data until a pause of 100mSec. NOT TESTED on ES+ or E.
    1.11 23/10/2009 Modified for BS3 at Hewetson - type 4278
    1.12 15/04/2010 Modified for BS3 at Sturtridge - type 427B. (34 + 30 packets)
 	 Now gets 16 bytes, then calculates how much more to read. Less sensitive to timing, I hope.
    1.13 19/05/2010 -N output new format -O old format --cylinder | --solasyphon | --pool
    1.14 23/05/2010 Modified for BS3 at Woolfenden. w/kwh/mwh = 8/9/10 not 10/11/23. 
		Pump at 4 not 5. Id = 10004278 
	1.15 30/05/2010 Bugfix in InitialiseData - file not closed.
 --
	2.0  30/05/2010 Uses common.c
	2.1 Bugfix Sturtridge Found second word of kwh.
	2.2 Open file called resol.XXXX where XXXX is the signature word
	2.3 Continue trying to open serial device if not found or goes missing (for USB unplug)
		Implement Common Serial Framework
    2.4 Add Resol WMZ which has ID in the range 4010 to 401F depending on its subaddress.
		AND access to VBus/LAN on port 7053 using hostname:port syntax.
	2.5 29/10/2010 Bugfix change BAUD 9600 to BAUD B9600
    2.6 01/03/2012 Addition of BX/Plus for Barilla - PARTIAL support and BS/4 for Secon.
		Data driven - needs definitions in resol.XXXX
*/

static char* id="@(#)$Id: resol.c,v 2.6 2012/05/01 15:47:46 martin Exp $";
#define REVISION "$Revision: 2.6 $"

#define PORTNO 10010
#define SERIALNAME "/dev/ttyAM0"	/* although it MUST be supplied on command line */

// Severity levels.  ERROR and FATAL terminate program
#define INFO	0
#define	WARN	1
#define	ERROR	2
#define	FATAL	3
// Socket retry params
#define NUMRETRIES 3
int numretries = NUMRETRIES;
#define RETRYDELAY	1000000	/* microseconds */
int retrydelay = RETRYDELAY;
// Elster values expected
#define NUMPARAMS 15
// Set to if(0) to disable debugging
#define DEBUGFP stderr   /* set to stderr or logfp as required */
// Serial retry params
#define SERIALNUMRETRIES 10
#define SERIALRETRYDELAY 1000000 /*microseconds */
// If defined, use stdin
// #define DEBUGCOMMS

/* SOCKET CLIENT */

/* Command line params: 
1 - device name
2 - device timeout. Default to 60 seconds
*/

#ifndef linux
extern
#endif
int errno;  

enum ModelType {undefModel =-1} modelType = undefModel;

// Procedures in this file
int processSocket(void);			// process server message
void usage(void);					// standard usage message
char * getversion(void);
int getbuf(int fd, int max);	// get a buffer full of message
short int get1stval(unsigned char a[]);			// Reassemble value. C must be a flag, ie byte & 0x4 or similar.
short int get2ndval(unsigned char a[]);			// Reassemble value. C must be a flag, ie byte & 0x4 or similar.
short int getval(int n);		// Get n'th data from block
int blockcheck(unsigned char a[], int n);		// Validate checksum X on 6-byte block. A is the
							// address of the buffer; n is the n'th block to check
// int oldblockcheck(unsigned char a[]);		// Validate checksum on 6-byte block
void dumpbuf();		// Dump out a buffer for debug
enum ModelType initialiseData(int model);					// Load resol.XXXX
enum CommandType;
void pushint(enum CommandType type, int param, int val);	// push an int onto the queue
int sendCommand(int fd, int dest, int src, unsigned int cmd, unsigned int d1, unsigned int d2);
void paramValue(unsigned char * data, int len);
int index2row(int index);		// Return row of parameter specified by name or -1
int name2row(char * name);		// Return row of parameter specified by index or -1
int synctostart(int fd);		// wait for 0xAA in the input. 0 == success.
#define STARTSYNC 0xAA
void protocol2(void);			// Handle Protocol 2 response (parameter value)
int isPort(char *);				// Decide if connection to hostname:port
void vbuslogin(int sock, char * password);			// Login to VBus/LAN device
void analyse();					// print out a packet

/* GLOBALS */
FILE * logfp = NULL;
int sockfd[1];			// Because openSockets expects an array.
int debug = 0;
int noserver = 0;		// prevents socket connection when set to 1
float factor = 1.0;		// scaling for over-optimistic Resol devices

// Common Serial Framework
#define BUFSIZE 128	/* should be longer than max possible line of text */
struct data {	// The serial buffer
	int count;
	unsigned char buf[BUFSIZE];
	int status;
} data;

int controllernum = -1;	//	only used in logon message

#define debugfp stderr

// Struct to hold device-dependant data

enum UnitsType { UNITERROR = -1, INT8, INT16, INT32, WATTS48 };
	// INT8 - lower 8 bits of16 bit value
	// INT16 - the value is 16 bits
	// INT32 - two adjacent 16 bits values comprise 32 bit int
	// SECONDS32 - two adjacent 16 bits values comprise operating seconds
	// WATTS48 - three adjacent 16 values are Wh, kWh, MWh
enum UnitsType unitsFromStr(char * name);

#define MAXDEFS		(10)
struct {
	char name[10];	// Exactly what is sent in the data message
	int src;	// Usually 0x0010
	int dest;	
	int location;	// VAlue number in datablock zero-based
	float scale;	// Divide value by this
	enum UnitsType units;
} datadef[MAXDEFS];
int numdefs = 0;	

enum CommandType { CMD_INVALID = 0, CMD_GETPARAM, CMD_SETPARAM};
char *CommandName[] = {"INVALID", "CMD_GETPARAM", "CMD_SETPARAM", NULL};
int CommandIndex[] = {0, 0x0300, 0x0200};

#define QUEUESIZE 10
struct queue {
        int top, bottom;
        enum CommandType type[QUEUESIZE];
        int  param[QUEUESIZE];
		union {
        float f;
		int i;} val[QUEUESIZE];
		int inv[QUEUESIZE];
} queue;

#define MAXPARAMS 60
#define SHORTNAMELEN 10
#define FULLNAMELEN 70
struct {
	int index;
	char shortname[SHORTNAMELEN];
	char fullname[FULLNAMELEN];
	float min, max, def; 
	int units;} params[MAXPARAMS];
int numparams;
int param2read = -1;

char buffer[256];

char * serialName = SERIALNAME;
char * password = "vbus";

/********/
/* MAIN */
/********/
int main(int argc, char *argv[])
// arg1: serial device file
// arg2: optional timeout in seconds, default 60
{
    int commfd;
	int nolog = 0;
	int analyseflag = 0;

	int interval = 30;
	// time_t next;

	int run = 1;		// set to 0 to stop main loop
	fd_set readfd, errorfd; 
	int numfds;
	struct timeval timeout;
	int tmout = 90;
	int logerror = 0;
	int online = 1;		// used to prevent messages every minute in the event of disconnection
	int option; 
	int dest, src;
	queue.top = queue.bottom = 0;
	// Command line arguments
	bzero(data.buf, BUFSIZE);
	data.count = 0;
	int readyfornext = 0;	// Only send a command after a VBus clearance or Read Value
	// optind = -1;
	opterr = 0;
	
	while ((option = getopt(argc, argv, "adt:slVi:f:Z")) != -1) {
		switch (option) {
			case 'a': analyseflag = 1; break;
			case 's': noserver = 1; break;
			case 'l': nolog = 1; break;
			case '?': usage(); exit(1);
			case 't': tmout = atoi(optarg); break;
			case 'd': debug++; break;
			case 'i': interval = atoi(optarg); break;
			case 'f': factor = atof(optarg); break;
			case 'p': password = optarg; break;
			case 'V': printf("Version %s %s\n", getversion(), id); exit(0);
			case 'Z': decode("(b+#Gjv~z`mcx-@ndd`rxbwcl9Vox=,/\x10\x17\x0e\x11\x14\x15\x11\x0b\x1a" 
							 "\x19\x1a\x13\x0cx@NEEZ\\F\\ER\\\x19YTLDWQ'a-1d()#!/#(-9' >q\"!;=?51-??r"); exit(0);
		}
	}
	
	DEBUG printf("Debug on %d. optind %d argc %d Model=%d\n", debug, optind, argc, modelType);
	
	if (optind < argc) serialName = argv[optind];		// get seria/device name: parameter 1
	optind++;
	if (optind < argc) controllernum = atoi(argv[optind]);	// get optional controller number: parameter 2
	
	sprintf(buffer, LOGFILE, controllernum);
	
	if (!nolog) if ((logfp = fopen(buffer, "a")) == NULL) logerror = errno;	
	
	// There is no point in logging the failure to open the logfile
	// to the logfile, and the socket is not yet open.

	sprintf(buffer, "STARTED %s on %s as %d timeout %d %s", argv[0], serialName, controllernum, tmout, nolog ? "nolog" : "");
	logmsg(INFO, buffer);
	
	openSockets(0, 1, "thermal", REVISION, PROGNAME, 0);
	
	// read_resol();	// load in data
	
	// Open serial port
	if ((commfd = openSerial(serialName, BAUD, 0, CS8, 1)) < 0) {
		sprintf(buffer, "FATAL " PROGNAME " %d Failed to open %s: %s", controllernum, serialName, strerror(errno));
#ifdef DEBUGCOMMS
		logmsg(INFO, buffer);			// FIXME AFTER TEST
		fprintf(stderr, "Using stdio\n");
		commfd = 0;		// use stdin
#else
		logmsg(FATAL, buffer);
#endif
	}

#ifndef DEBUGCOMMS
	if (flock(commfd, LOCK_EX | LOCK_NB) == -1) {
		sprintf(buffer, "FATAL " PROGNAME " is already running, cannot start another one on %s", serialName);
		logmsg(FATAL, buffer);
	}
#endif

	// If we failed to open the logfile and were NOT called with nolog, warn server
	// Obviously don't use logmsg!
	if (logfp == NULL && nolog == 0) {
		sprintf(buffer, "event WARN " PROGNAME " %d could not open logfile %s: %s", controllernum, LOGFILE, strerror(logerror));
		sockSend(sockfd[0], buffer);
	}
		
	numfds = (sockfd[0] > commfd ? sockfd[0] : commfd) + 1;		// nfds parameter to select. One more than highest descriptor

	if (isPort(serialName)) {	// Connection to Vbus/LAN adapter
		vbuslogin(commfd, password);
	}
	
	// Main Loop
	FD_ZERO(&readfd); 
	FD_ZERO(&errorfd); 
	
//	next = 0;	// so we always get one at startup.
//	DEBUG2 fprintf(stderr, "Now is %zd next is %zd\n", time(NULL), next);
	while(run) {
		timeout.tv_sec = tmout;
		timeout.tv_usec = 0;
		FD_SET(sockfd[0], &readfd);
		FD_SET(commfd, &readfd);
		FD_SET(sockfd[0], &errorfd);
		FD_SET(commfd, &errorfd);
		DEBUG2 fprintf(stderr, "Before: Readfd %lx ", readfd);
		DEBUG2 fprintf(stderr, "ErrorFD %lx ", errorfd);
		blinkLED(0, REDLED);
		if (select(numfds, &readfd, NULL, &errorfd, &timeout) == 0) {	// select timed out. Bad news 
			if (online == 1) {
				logmsg(WARN, "WARN " PROGNAME " No data for last period");
				online = 0;	// Don't send a message every minute from now on
			}
			// For VBus, attempt to reconnect
			if (isPort(serialName)) {
				close(commfd);
				DEBUG fprintf(stderr, "REOPENING FD\n");
				if ((commfd = openSerial(serialName, BAUD, 0, CS8, 1)) < 0) {
					sprintf(buffer, "WARN " PROGNAME " Failed to reopen %s", serialName);
					logmsg(WARN, buffer);
				}
			}
			continue;
		}
		blinkLED(1, REDLED);
		DEBUG2 fprintf(stderr, "After: Readfd %lx ", readfd);
		DEBUG2 fprintf(stderr, "ErrorFD %lx ", errorfd);
		if (FD_ISSET(commfd, &readfd)) { 
			int num, got, sig, to_get, blocks, proto;
			DEBUG4 fprintf(stderr, "Commfd readable ..");
//			data.buf[0] = 0xAA;	// Initialise buffer since we don;'t read the AA into the buffer
			if (synctostart(commfd)) {
				sprintf(buffer, "WARN " PROGNAME " Couldn't synchronise to data stream");
				logmsg(WARN, buffer);
				sleep(10);		// prevent fast loops
				continue;
			}
			got = getbuf(commfd, 15);	// Get first 16 bytes.
			if (got == -1) {
				DEBUG fprintf(stderr, "Skipping ... ");
				continue;
			}
			online = 1;	// Back online
			if (data.buf[0] != STARTSYNC) {		// Can't happen !
				DEBUG fprintf(debugfp, "Initial byte is not AA - discarding packet of %d bytes\n", num);
				DEBUG dumpbuf();
				continue;
			}
			dest = (data.buf[2] << 8) | data.buf[1];
			src = (data.buf[4] << 8) | data.buf[3];
			sig = (data.buf[1] << 24) | (data.buf[2] << 16) | (data.buf[3] << 8) | data.buf[4];
			proto = data.buf[5];
			///cmd = data.buf[7];
			blocks = data.buf[8];
			DEBUG2 fprintf(debugfp, "\nGot packet %d long, Dest 0x%04x Src 0x%04x Protocol = %.1f\n", got, dest, src, proto / 16.0);
			// Byte 8 is the number of blocks of 6-byte data (plus 10-byte header) for protocol 1.0 blocks
			
			if (proto == 0x20) {		// Handle Protocol 2.0
				int cmd;
				DEBUG2 dumpbuf();
				protocol2();
				
				// If we have just had a Protocol V2 Bus Clearance we can pick a command off the the queueu
				cmd = data.buf[6] | (data.buf[7] << 8);
				readyfornext = 0;	// Block rapid transmission of requsts
				if (cmd == 0x500 || cmd == 0x100) readyfornext = 1;
				if (readyfornext) {		// Can read all params in one go.
					if (param2read > -1) {
						DEBUG fprintf(stderr, "Scanning row %d param 0x%x (%s): ", param2read, params[param2read].index, 
									  params[param2read].shortname);
						sendCommand(commfd, modelType, 0x0022, CommandIndex[CMD_GETPARAM], params[param2read].index, 0);
						param2read++;
						if (param2read >= numparams) param2read = -1;
					}
					else
						// Command from queue?
						if (queue.top != queue.bottom) {
							queue.bottom++;
							if (queue.bottom == QUEUESIZE) queue.bottom = 0;
							DEBUG fprintf(stderr,"Getting command from Queue index %d ", queue.bottom);
							DEBUG fprintf(stderr, "Command = %d '%s' %x ", queue.type[queue.bottom], CommandName[queue.type[queue.bottom]],
										  CommandIndex[queue.type[queue.bottom]]);
							sendCommand(commfd, modelType, 0x0022, CommandIndex[queue.type[queue.bottom]], queue.param[queue.bottom], 
										queue.val[queue.bottom].i);
						}
				}
				continue;
			}
			
			to_get = blocks * 6 + 10;
			if (got < to_get) {
				DEBUG2 fprintf(stderr, "Got %d, blocks=%d so total to get = %d\n", got, blocks, to_get);
				num = getbuf(commfd, to_get - got);
				if (num == -1) {
					DEBUG fprintf(stderr, "Skipping ..(1). ");
					continue;
				}
				got += num;
				DEBUG2 fprintf(stderr,"Getmore: read %d now packet is %d\n", num, got);
			}

			DEBUG2 dumpbuf();
			if (analyseflag)
				analyse();
			
			if (got < 0 && errno != 0) {
				fprintf(stderr, "Error reading %s : %s\n", serialName, strerror(errno));
				// run = 0;
				break;
			} else {
				// Initialise model data.
				if (modelType == undefModel) 
					modelType = initialiseData(src); 
		
				processPacket();
			}
		}
		if ((noserver == 0) && FD_ISSET(sockfd[0], &readfd))
			run = processSocket();	// the server may request a shutdown by setting run to 0
	}
	logmsg(INFO,"INFO " PROGNAME " Shutdown requested");
	close(sockfd[0]);
	closeSerial(commfd);

	return 0;
}

/*********/
/* USAGE */
/*********/
void usage(void) {
	printf("Usage: %s [-t timeout] [-l] [-s] [-f value] [-d] [-V] /[-a] dev/ttyname controllernum\n", progname);
	printf("-l: no log  -s: no server  -d: debug on\n -V: version -i: interval in seconds\n");
	printf("-f scalingfactor -p password -a analyse\n");

	return;
}

/**************/
/* NAME2ROW */
/**************/
int name2row(char * name) {
//	Given a name, finds its row number in Params
	int i;
	for (i = 0; i < numparams; i++) 
		if (strcasecmp(name, params[i].shortname) == 0) return i;
	return -1;	// not found
}

/************/
/* NAME2ROW */
/************/
int index2row(int index) {
	int i;
	for (i = 0; i < numparams; i++) 
		if (params[i].index == index) return i;
	return -1;	// not found
}

/*****************/
/* PROCESSSOCKET */
/*****************/
int processSocket(void){
	// Deal with commands from MCP.  Return to 0 to do a shutdown
	short int msglen, numread;
	char buffer2[192];	// about 128 is good but rather excessive since longest message is 'truncate'
	char * cp = &buffer[0];
	int retries = NUMRETRIES;
	
	if (read(sockfd[0], &msglen, 2) != 2) {
		logmsg(WARN, "WARN " PROGNAME " Failed to read length from socket");
		return 1;
	}
	msglen =  ntohs(msglen);
	while ((numread = read(sockfd[0], cp, msglen)) < msglen) {
		cp += numread;
		msglen -= numread;
		if (--retries == 0) {
			logmsg(WARN, "WARN " PROGNAME " Timed out reading from server");
			return 1;
		}
		usleep(RETRYDELAY);
	}
	cp[numread] = '\0';	// terminate the buffer 
	
	if (strcmp(buffer, "exit") == 0)
		return 0;	// Terminate program
	if (strcmp(buffer, "Ok") == 0)
		return 1;	// Just acknowledgement
	if (strcmp(buffer, "truncate") == 0) {
		if (logfp) {
			// ftruncate(logfp, 0L);
			// lseek(logfp, 0L, SEEK_SET);
			freopen(NULL, "w", logfp);
			logmsg(INFO, "INFO " PROGNAME " Truncated log file");
		} else
			logmsg(INFO, "INFO " PROGNAME " Log file not truncated as it is not open");
		return 1;
	}
	if (strcmp(buffer, "debug 0") == 0) {	// turn off debug
		debug = 0;
		return 1;
	}
	if (strcmp(buffer, "debug 1") == 0) {	// enable debugging
		debug = 1;
		return 1;
	}
	if (strcmp(buffer, "debug 2") == 0) {	// enable debugging
		debug = 2;
		return 1;
	}
	if (strcmp(buffer, "debug 3") == 0) {	// enable debugging
		debug = 3;
		return 1;
	}
	if (strcmp(buffer, "help") == 0) {
		strcpy(buffer2, "INFO " PROGNAME " Commands are: debug 0|1|2, exit, truncate, read param, write param value, readall");
		logmsg(INFO, buffer2);
		return 1;
	}
	
	if (strncmp(buffer, "readall", 7) == 0) {
		param2read = 0;	// start the scan process
		return 1;
	}
	if (strncmp(buffer, "read", 4) == 0) {
		int parsed, index, row;
		char name[SHORTNAMELEN];
		parsed = sscanf(buffer, "%*s %10s", &name[0]);
		
		if (parsed == 1) {
			index = strtol(name, NULL, 0);	// Param supplied as decimal or hex number
			if (index == 0)
				row = name2row(name);
			else row = index2row(index); 
			if (row == -1) {
				sprintf(buffer, "WARN " PROGNAME " Can't identify parameter by name or value '%s'", name);
				logmsg(WARN, buffer);
				return 1;
			}
			DEBUG fprintf(stderr,"Row is %d Scale is %d ", row, params[row].units);
			index = params[row].index;
			DEBUG fprintf(stderr,"\nSetting Queue entry %d to CMD %d index 0x%x\n", queue.top, CMD_GETPARAM, index);
			pushint(CMD_GETPARAM, index, 0);
		}
		else {
			sprintf(buffer, "WARN " PROGNAME " Didn't get parameter number for READ command (%d)", parsed);
			logmsg(WARN, buffer);
		}
		return 1;
	}
	if (strncmp(buffer, "write", 5) == 0) {
		int parsed, index, row;
		float val;
		char name[SHORTNAMELEN];
		parsed = sscanf(buffer, "%*s %10s %f", &name[0], &val);
		
		if (parsed == 2) {
			index = strtol(name, NULL, 0);	// Param supplied as decimal or hex number
			if (index == 0)
				row = name2row(name);
			else
				row = index2row(index);
			if (row == -1) {
				sprintf(buffer, "WARN " PROGNAME " Can't identify parameter by name or value '%s'", name);
				logmsg(WARN, buffer);
				return 1;
			}
			index = params[row].index;
			if (val < params[row].min) {
				sprintf(buffer, "WARN " PROGNAME " Can't set parameter %s below minimum of %f", params[row].shortname, params[row].min);
				logmsg (WARN, buffer);
				return 1;
			}
			if (val > params[row].max) {
				sprintf(buffer, "WARN " PROGNAME " Can't set parameter %s above maximum of %f", params[row].shortname, params[row].max);
				logmsg (WARN, buffer);
				return 1;
			}
			DEBUG fprintf(stderr,"Row is %d Scale is %d ", row, params[row].units);
			// If the units is -1, the value is in form hh:mm so read to be re-parsed.
			if (params[row].units == -1) {
				int hh, mm;
				parsed = sscanf(buffer, "%*s %*s %d:%d", &hh, &mm);
				if (parsed != 2) {
					sprintf(buffer, "WARN " PROGNAME " Can't read time in hh:mm format as required");
					logmsg(WARN, buffer);
					return 1;
				}
				val = hh * 60 + mm;
			}
			else val = val * params[row].units;
			
			DEBUG fprintf(stderr,"\nSetting Queue entry %d to CMD %d index 0x%x val %.2f\n", queue.top, CMD_SETPARAM, 
						  index, val);
			pushint(CMD_SETPARAM, index, val);
		}
		else {
			sprintf(buffer, "WARN " PROGNAME " Didn't get parameter number for WRITE command (%d)", parsed);
			logmsg(WARN, buffer);
		}
		return 1;
	}
	strcpy(buffer2, "INFO " PROGNAME " Unknown message from server: ");
	strcat(buffer2, buffer);
	logmsg(INFO, buffer2);	// Risk of loop: sending unknown message straight back to server
	
	return 1;	
};

/**************/
/* GETVERSION */
/**************/
char *getversion(void) {
// return pointer to version part of REVISION macro
	static char version[10] = "";	// Room for xxxx.yyyy
	if (!strlen(version)) {
		strcpy(version, REVISION+11);
		version[strlen(version)-2] = '\0';
	}
return version;
}

/********/
/* PUSHF */
/********/
void pushint(enum CommandType type, int param, int val) {
// Push a commnd onto the queue if there's room
	int prev = queue.top++;
    if (queue.top == QUEUESIZE) queue.top = 0;
        if (queue.top == queue.bottom) {
                logmsg(WARN, "WARN " PROGNAME " Queue full - ignoring command");
		queue.top = prev;
		return;
	}
	queue.type[queue.top] = type;
	queue.param[queue.top] = param;
	queue.val[queue.top].i = val;
}

/***********/
/* DUMPBUF */
/***********/
void dumpbuf() {
	int i;
	for (i = 0; i < data.count; i++) {
		fprintf(stderr, "%02x", data.buf[i]);
		if (i % 6 == 3)
			putc('-', stderr);
		else
			putc(' ', stderr);
	}
	putc('\n', stderr);
}

/**********/
/* GETBUF */
/**********/
int getbuf(int fd, int max) {
	// Read up to max chars into supplied buf. Return number
	// of chars read or negative error code if applicable
	
	int ready, numtoread, now;
	fd_set readfd; 
	struct timeval timeout;
	FD_ZERO(&readfd);
	// numread = 0;
	numtoread = max;
	DEBUG2 fprintf(stderr, "Getbuf %d count=%d ", max ,data.count);
	
	while(1) {
		FD_SET(fd, &readfd);
		timeout.tv_sec = 0;
		timeout.tv_usec = 500000;	 // 0.5sec
		ready = select(fd + 1, &readfd, NULL, NULL, &timeout);
		DEBUG4 {
			gettimeofday(&timeout, NULL);
			fprintf(stderr, "%03ld.%03d ", timeout.tv_sec%100, timeout.tv_usec / 1000);
		}
		if (ready == 0) 
			return data.count;		// timed out - return what we've got
		DEBUG4 fprintf(stderr, "Getbuf: before read1 ");
		now = read(fd, data.buf + data.count, 1);
		DEBUG4 fprintf(stderr, "After read1\n");
		DEBUG3 fprintf(stderr, "0x%02x ", data.buf[data.count]);
		if (now < 0)
			return now;
		if (now == 0) {
			fprintf(stderr, "ERROR fd was ready but got no data\n");
			// VBUs / LAN  - can't use standard Reopenserial as device name hostname: port is not valid
			if (isPort(serialName)) {
				if (fd) close(fd);
				DEBUG fprintf(stderr, "REOPENING FD\n");
				fd = openSerial(serialName, BAUD, 0, CS8, 1);
				vbuslogin(fd, password);
			}
			else
				fd = reopenSerial(fd, serialName, BAUD, 0, CS8, 1);
			
//			usleep(1000000); // 1 Sec
			continue;
		}
		if (data.buf[data.count] == STARTSYNC && data.count != 0) {
			sprintf(buffer, "WARN " PROGNAME " got 0xAA at position %d in packet", data.count);
			logmsg(WARN, buffer);
			return -1;
		}
		if (data.buf[data.count] == STARTSYNC)
			return 0;
		
		data.count += now;
		numtoread -= now;
		if (numtoread == 0) return data.count;
		if (numtoread < 0) {	// CANT HAPPEN
			fprintf(stderr, "ERROR buffer overflow - increase max from %d (numtoread = %d numread = %d)\n", 
					max, numtoread, data.count);
			return data.count;
			
		}
	}
}

/*****************/
/* SYNC TO Start */
/*****************/
int synctostart(int fd) {
	// This will wait for AA characters with a 10-sec timeout
	fd_set readfd, errorfd; 
	struct timeval timeout;
	int numread, ready;
	FD_ZERO(&readfd);
	FD_ZERO(&errorfd);
	numread = 0;
	while(1) {
		FD_SET(fd, &readfd);
		FD_SET(fd, &errorfd);
		timeout.tv_sec = 10;
		timeout.tv_usec = 0;	 // 10 sec
		errno = 0;
		ready = select(fd + 1, &readfd, NULL, &errorfd, &timeout);
		if (ready == 0) {	// timed out - return error
			DEBUG fprintf(stderr, "SyncToStart - timedout. errno %d\n", errno);
			return -1;
		}
		DEBUG4 fprintf(stderr, "Sync: readfd = %x ", readfd);
		DEBUG4 fprintf(stderr, "errorfd = %lx ", errorfd);
		DEBUG4 fprintf(stderr, "ready = %x ", ready);
		ready = read(fd, data.buf, 1);
		if (ready <= 0) {
			DEBUG fprintf(stderr, "FD ready but read returns %d (%s)\n", 
						  ready, strerror(ready));	// Comms error - missing device?
			// VBUs / LAN  - can't use standard Reopenserial as device name hostname: port is not valid
			if (isPort(serialName)) {
				if (fd) close(fd);
				DEBUG fprintf(stderr, "REOPENING FD\n");
				fd = openSerial(serialName, BAUD, 0, CS8, 1);
				vbuslogin(fd, password);
			}
			else
				fd = reopenSerial(fd, serialName, BAUD, 0, CS8, 1);
			
			return ready;
		}
		if (data.buf[0] == STARTSYNC) {
			DEBUG3 fprintf(stderr, "Sync skipped %d bytes .. ", numread);
			data.count = 1;
			return 0;	// success
		}
		numread++;
	}
}

/*************/
/* GET1STVAL */
/*************/
short int get1stval(unsigned char a[]) {
// byte a[0]: Bits 0 to 6. 
// byte a[1]: Bits 8 to 14
// byte 4: bits 7 and 15
	short int val = (a[0] & 0x7f) + ((a[1] & 0x7F) << 8);
	DEBUG3 fprintf(stderr, "1st %02x %02x %02x", a[0], a[1], a[4]);
	if (a[4] & 1) val += 128;
	if (a[4] & 2) val = val - 32768;
	DEBUG3 fprintf(stderr, "=%d ", val);
return val;
}

/*************/
/* GET2NDVAL */
/*************/
short int get2ndval(unsigned char a[]) {
// byte a[2]: Bits 0 to 6. 
// byte a[3]: Bits 8 to 14
// byte a[4]: bits 7 and 15
// if c is nonzero, treat it as bit 7.  As the bit 7 can be stored on any bit in byte c
	short int val = (a[2] & 0x7F) + ((a[3] & 0x7F) << 8);
	DEBUG3 fprintf(stderr, "2nd %02x %02x %02x", a[2], a[3], a[4]);
	if (a[4] & 4) val += 128;
	if (a[4] & 8) val = val - 32768;
	DEBUG3 fprintf(stderr, "=%d ", val);
return val;
}

/**********/
/* GETVAL */
/**********/
short int getval(int n) {	// Get the n'th value.  There are 2 per block
	DEBUG3 fprintf(stderr, "Val(%d) ", n);
	if (n % 2)
		return get2ndval(&data.buf[(n >> 1) * 6 + 10]);
	else
		return get1stval(&data.buf[(n >> 1) * 6 + 10]);
}

unsigned short int getuval(int n) {	// Get the n'th value.  There are 2 per block
	DEBUG3 fprintf(stderr, "Val(%d) ", n);
	if (n % 2)
		return get2ndval(&data.buf[(n >> 1) * 6 + 10]);
	else
		return get1stval(&data.buf[(n >> 1) * 6 + 10]);
}

int blockcheck(unsigned char a[], int n) {
	return ((a[n*6+10] + a[n*6+11] + a[n*6+12] + a[n*6+13] + a[n*6+14] + a[n*6+15]) & 0x7F) != 0x7F;
}

int blockcheckN(unsigned char * a, int n) {
	// Sum N bytes and ensure the result of either 0x7F or 0xFF
	int i;
	int check = 0;
	DEBUG3 fprintf(stderr," BlockcheckN: ");
	for (i = 0; i < n; i++) {
		check += a[i];
		DEBUG3 fprintf(stderr,"%02x ", a[i]);
	}
	DEBUG2 fprintf(stderr, "BlockcheckN of %d bytes = %x\n", n, check);
	return (check & 0x7f) != 0x7f;
}

/******************/
/* INITIALISEDATA */
/******************/
enum ModelType initialiseData(int model) {
	// Looks for a file resol.XXXX where XXX is hex value of model and loads it
	DEBUG fprintf(stderr, "InitialiseData with %04x ", model);
	FILE * f;
	int num, m;
	char c;
	char filename[24];
	char unitname[10];
	numparams = 0;
	sprintf(filename, "/root/resol.%04x", model);
	f = fopen(filename, "r");
	if (!f) {
		sprintf(buffer, "WARN " PROGNAME " failed to open %s: %s", filename, strerror(errno));
		logmsg(WARN, buffer);
		return undefModel;
	}
	
	// BODGE for Resol E as the WMZ data packet seems to come from a differnet source
	if (model == 0x7722) model = 0x7721;
	
	num = fscanf(f, "%x", &m);
	if (num !=1 ) fprintf(stderr, "Got %d - expected 1 on first line\n", num);
	if (m != model) {
		sprintf(buffer, "ERROR " PROGNAME " file %s is for %x, attached Resol is %x\n", filename, m, model);
		logmsg(ERROR, buffer);
		fclose(f);
		return undefModel;
	}
	
	// First line - model number in hex. Rest is comment and ignored.
	fgets(buffer, 100, f);	// eat to end of first line. Just validate model number.
	DEBUG2 fprintf(stderr, "Comments: '%s'\n", buffer);
	
	while (1) {
		// Get a line.
		// Ignore blank lines
		// Ignore lines starting with #
		// Process: name	src	dest	location	scale	units	comments
		fgets(buffer, 100, f);
		if (iscomment(buffer))
			continue;
		num = sscanf(buffer, "%10s %x %x %d %f %10s", &datadef[numdefs].name[0], &datadef[numdefs].src,
					 &datadef[numdefs].dest, &datadef[numdefs].location, &datadef[numdefs].scale,
					 &unitname[0]);
		// End of section
		if (strncasecmp(datadef[numdefs].name, "param", 5) == 0)
			break;
		
		if (num != 6) {
			fprintf(stderr, "Failed to parse 6 values from %s", buffer);
			continue;
		}
		if ((datadef[numdefs].units = unitsFromStr(unitname)) == UNITERROR) {
			fprintf(stderr, "Unrecognised units in '%s'", unitname);
			continue;
		}
		
		DEBUG fprintf(stderr, "Line %d: '%s' SRC %x DEST %x Loc %d Scale %f Units %d\n", numdefs,
					  datadef[numdefs].name, datadef[numdefs].src, datadef[numdefs].dest, 
					  datadef[numdefs].location, datadef[numdefs].scale, datadef[numdefs].units);
		numdefs++;
		if (numdefs > MAXDEFS) {
			fprintf(stderr, "ERROR Exceeded MAXDEFS - increase it");
			break;
		}
		
	}
	DEBUG fprintf(stderr, "Finished with %d datadefs\n", numdefs);
						
	for (numparams = 0; num; numparams++) {
		if (numparams > MAXPARAMS) {
			sprintf(buffer, "ERROR " PROGNAME " increase MAXPARAMS");
			logmsg(ERROR, buffer);
			fclose(f);
			return model;
		}
		
		// get a line
		// Change this 10 to value of SHORTNAMELEN ***
		num = fscanf(f, "%x %f %f %d %f %10s", &params[numparams].index, &params[numparams].min, &params[numparams].max,
					 &params[numparams].units, &params[numparams].def, &params[numparams].shortname[0]);
		if (num == -1) break;
		if (num != 6) {
			fprintf(stderr, "Expected 6 got %d on line %d\n", num, numparams+2);
			break;;
		}
		if (strlen(params[numparams].shortname) >= SHORTNAMELEN) {
			sprintf(buffer, "WARN " PROGNAME " increase SHORTNAMELEN");
			logmsg(WARN, buffer);
			break;;
		}
		// Check to make sure shortname hasn't already been used
		if (name2row(params[numparams].shortname) != -1) {
			sprintf(buffer, "WARN " PROGNAME " Parameter %s has already been used on line %d of %s",
				params[numparams].shortname, name2row(params[numparams].shortname), filename);
			logmsg(WARN, buffer);
		}
		// eat white space
		while (((c = fgetc(f)) == ' ') || c == '\t');
		ungetc(c, f);
		if (!fgets(params[numparams].fullname, FULLNAMELEN, f)) {
			fprintf(stderr, "Error reading full name: %s", strerror(errno));
		}
		// remove last char as it's a NL
		num = strlen(params[numparams].fullname);
		// DEBUG fprintf(stderr, "Fullname length = %d ", num);
		if (num + 1 == FULLNAMELEN) {
			sprintf(buffer, "WARN " PROGNAME " Increase FULLNAMELEN");
			logmsg(WARN, buffer);
			break;
		}
		if (params[numparams].fullname[num - 1] == '\n')
			params[numparams].fullname[num - 1] = 0;
		DEBUG3 fprintf(stderr, "Line %d: index 0x%4x min %.2f max %.2f units %d default %.2f short '%s' full '%s'\n",
					  numparams, params[numparams].index, params[numparams].min, params[numparams].max,
					  params[numparams].units, params[numparams].def, params[numparams].shortname,
					  params[numparams].fullname);
	}
	
	fclose(f);
	sprintf(buffer, "INFO " PROGNAME " Signature %04x OK Loaded %d Definitions and %d Parameters", model, numdefs, numparams);
	logmsg(INFO, buffer);
	return model;
}


/* ISCOMMENT */
int iscomment(char *cp) {
	// Return 1 if first character after whitespace is # or line is all whitespace
	int hash = strspn(cp, " \t");
	if (cp[hash] == '#') return 1;
	if (strlen(cp) -1 == hash) return 1;	// not a comment, but an empty line
	return 0;
}

/**************/
/* SENDSERIAL */
/**************/
int sendSerial(int fd, unsigned char data) {
	// Send a single byte.  Return 1 for a logged failure
	int retries = SERIALNUMRETRIES;
	int written;
#ifdef DEBUGCOMMS
	printf("Comm 0x%02x(%d) ", data, data);
	return 0;
#endif
	
	DEBUG2 fprintf(DEBUGFP, "[%02x]>> ", data);
	while ((written = write(fd, &data, 1)) < 1) {
        fprintf(DEBUGFP, "Serial wrote %d bytes errno = %d", written, errno);
        perror("");
		if (--retries == 0) {
			logmsg(WARN, "WARN " PROGNAME " timed out writing to serial port");
			return 1;
		}
		DEBUG fprintf(DEBUGFP, "Pausing %d ... ", SERIALRETRYDELAY);
		usleep(SERIALRETRYDELAY);
	}
	return 0;       // ok
}

// NOTE 7-bit version!
#define lobyte(x) (x & 0x7f)
#define hibyte(x) ((x >> 8) & 0x7F)

/***************/
/* SENDCOMMAND */
/***************/
int sendCommand(int fd, int dest, int src, unsigned int cmd, unsigned int d1, unsigned int d2) {
	// As before, return 1 for a logged failure, otherwise 0
	// The protocol seems to allow 3 pairs of bytes, thus using 
	unsigned char checksum, c;
	unsigned char extra;
	
	extra = 0;
	
	DEBUG fprintf(DEBUGFP, "\nSendCommand: %x %x %x; ", cmd, d1, d2);
#ifndef DEBUGCOMMS
	if (sendSerial(fd, 0xAA)) return 1;
	if (sendSerial(fd, c = lobyte(dest))) return 1;	checksum = c;
	if (sendSerial(fd, c = hibyte(dest))) return 1;	checksum += c;
	if (sendSerial(fd, c = lobyte(src))) return 1;	checksum += c;
	if (sendSerial(fd, c = hibyte(src))) return 1;	checksum += c;
	if (sendSerial(fd, c = 0x20)) return 1;			checksum += c;
	if (sendSerial(fd, c = lobyte(cmd))) return 1;	checksum += c;
	if (sendSerial(fd, c = hibyte(cmd))) return 1;	checksum += c;
	if (d1 & 0x80)  extra |= 1;
	if (d1 & 0x8000) extra  |= 2;
	if (d2 & 0x80)  extra |= 4;
	if (d2 & 0x8000) extra |= 8;
	if (sendSerial(fd, c = lobyte(d1))) return 1;	checksum += c;
	if (sendSerial(fd, c = hibyte(d1))) return 1;	checksum += c;
	if (sendSerial(fd, c = lobyte(d2))) return 1;	checksum += c;
	if (sendSerial(fd, c = hibyte(d2))) return 1;	checksum += c;
	if (sendSerial(fd, 0)) return 1;	// These are for d3 wich isn't used.
	if (sendSerial(fd, 0)) return 1;
	if (sendSerial(fd, c = extra)) return 1;		checksum += c;
	if (sendSerial(fd, lobyte(0xFF - checksum))) return 1;
#endif
	return 0;
}

/**************/
/* PARAMVALUE */
/**************/
void paramValue(unsigned char * data, int len) {
// Deal with a parameter value response
// If param2read is non-negative, write out a file containing the parameter value.
// param2read = 0: open file and write value
// param2read = numparams: write value and close file
	int v1, v2, v3, row;
	int hh, mm;
	static FILE *f = 0;
	if (len != 32) return;
	// ignore the first 16 bytes.
	data += 16;
	if (blockcheckN(data, 16)) fprintf(stderr, "Blockcheck failed\n");
	if (data[6] + data[7] << 8 != 0x0100) fprintf(stderr, "Not a Parameter Value\n");
	v1 = data[8] + (data[9] << 8);
	if (data[14] & 1) v1 += 128;
	if (data[14] & 2) v1 = v1 - 32768;
	v2 = data[10] + (data[11] << 8);
	if (data[14] & 4) v2 += 128;
	if (data[14] & 8) v2 = v2 - 32768;
	v3 = data[12] + (data[13] << 8);
	if (data[14] & 0x10) v3 += 128;
	if (data[14] & 0x20) v3 = v3 - 32768;
	DEBUG fprintf(stderr, "V1 %d 0x%x V2 %d 0x%x V3 %d 0x%x ", v1, v1, v2, v2, v3, v3);
	if (param2read == 1) {	// open file
		f = fopen("/tmp/resol.txt", "w");
		if (!f) {
			sprintf(buffer, "WARN " PROGNAME " Couldn't write /tmp/resol.txt %s", strerror(errno));
			logmsg(WARN, buffer);
			return;
		}
	}

	row = index2row(v1);
	if (row == -1) {
		sprintf(buffer, "WARN " PROGNAME " Parameter response to unknown parameter index: 0x%x", v1);
		logmsg(WARN, buffer);
		return;
	}
	switch (params[row].units) {		// Its a time in hh:mm format
	case -1:
		hh = v2 / 60;
		mm = v2 % 60;
		DEBUG fprintf(stderr, "Converting %d into %02d:%02d ", v2, hh, mm);
		sprintf(buffer, "INFO " PROGNAME " Param %x %s = %02d:%02d", v1, params[row].shortname, hh, mm);
		if (f) fprintf(f, "%6s %02d:%02d %s\n", params[row].shortname, hh, mm, params[row].fullname);
		break;
	case 1:
		sprintf(buffer, "INFO " PROGNAME " Param %x %s = %d", v1, params[row].shortname, v2);
		if (f) fprintf(f, "%6s %5d %s\n", params[row].shortname, v2, params[row].fullname);
		break;
	case 10:
		sprintf(buffer, "INFO " PROGNAME " Param %x %s = %.1f", v1, params[row].shortname, v2 / 10.0);
		if (f) fprintf(f, "%6s %5.1f %s\n", params[row].shortname, v2 / 10.0, params[row].fullname);
		break;
	default:
		sprintf(buffer, "WARN " PROGNAME " Scale not known for parameter %s", params[row].shortname);
	}
	logmsg(INFO,buffer);
	DEBUG2 fprintf(stderr, "row = %d numparams = %d\n", row, numparams);
}

void extractSeptet(int offset, int count) {
	// create overflow byte; remove 8th bit
	// count is number of data bytes not including overflow byte
	int i;
	unsigned char septet;
	septet = 0;
	for (i = 0; i < count; i++)
		if (data.buf[offset + i] & 0x80) {
			data.buf[offset + i] &= 0x7f;
			septet |= (1 << i);
		};
	data.buf[offset + count] = septet;
}

void injectSeptet(int offset, int count) {
	// reconstruct 8-bit values using overflow byte
	int i;
	unsigned char septet;
	septet = data.buf[offset + count];
	for (i = 0; i < count; i++)
		if (septet & (1 << i)) {
			data.buf[offset + i] |= 0x80;
		}
}

inline int getShort(int offset) {
	return data.buf[offset] | (data.buf[offset+1] << 8);
}

inline int getLong(int offset) {
	return data.buf[offset] | (data.buf[offset+1] << 8) | 
		(data.buf[offset+2] << 16) | (data.buf[offset+3] << 24);
}

int clamp(int value, int low, int high) {
	// Ensure value is within specified range
	if (value < low) {
		fprintf(stderr, "Warning: clamping %d to %d (low)", value, low);
		value = low;
	}
	if (value > high) {
		fprintf(stderr, "Warning: clamping %d to %d (high)", value, high);
		value = high;
	}
}

/*************/
/* PROTOCOL2 */
/*************/
void protocol2() {
	// Handle a Protocol 2 block always 16 bytes
	int cmd, index, value, row;
	int hh, mm;
	static FILE *f = 0;

	char *OffOn[] = {"Off", "On"};
	char *OffAutoOn[] = {"Off", "Auto", "On"};
	if (data.count != 16) {
		sprintf(buffer, "WARN " PROGNAME " Procotol 2 with %d bytes (not 16)", data.count);
		logmsg(WARN, buffer);
	}
	if (param2read == 1) {	// open file as it's on the first line!
		f = fopen("/tmp/resol.txt", "w");
		if (!f) {
			sprintf(buffer, "WARN " PROGNAME " Couldn't write /tmp/resol.txt %s", strerror(errno));
			logmsg(WARN, buffer);
			return;
		}
	}	
	cmd = data.buf[6] | (data.buf[7] << 8);
	injectSeptet(8, 6);	// Starting at byte 8 there's a short then a long
	index = getShort(8);
	value = getLong(10);
	if (blockcheckN(data.buf + 1, 15)) {
		sprintf(buffer, "WARN " PROGNAME " Protocol 2 blockcheck failed");
		logmsg(INFO, buffer);
	}
	row = index2row(index);
	switch (cmd) {
		case 0x100:	// Response with value
			DEBUG fprintf(stderr, "Prot2 0100 Response %x with value %d\n", index, value);
			if (row == -1) {
				sprintf(buffer, "WARN " PROGNAME " Can't find parameter 0x%x in lookup table", index);
				logmsg(WARN, buffer);
				break;
			}
			DEBUG fprintf(stderr, "Param 0x%x row %d min %f max %f units %d name '%s'\n",
						  index, row, params[row].min, params[row].max, params[row].units, params[row].shortname);
			
			if (row == -1) {
				sprintf(buffer, "WARN " PROGNAME " Parameter response to unknown parameter index: 0x%x", index);
				logmsg(WARN, buffer);
				return;
			}
			switch (params[row].units) {		// Its a time in hh:mm format
				case -1:
					hh = value / 60;
					mm = value % 60;
					DEBUG fprintf(stderr, "Converting %d into %02d:%02d ", value, hh, mm);
					sprintf(buffer, "INFO " PROGNAME " Param %s[%x] = %02d:%02d", params[row].shortname, index, hh, mm);
					if (f) fprintf(f, "%6s %02d:%02d %s\n", params[row].shortname, hh, mm, params[row].fullname);
					break;
				case 1:
					sprintf(buffer, "INFO " PROGNAME " Param %s[%x] = %d", params[row].shortname, index, value);
					if (f) fprintf(f, "%6s %5d %s\n", params[row].shortname, value, params[row].fullname);
					break;
				case 2:	// 0 = off 1 = on
					value = clamp(value, 0, 1);
					sprintf(buffer, "INFO " PROGNAME " Param %s[%x] = %s", params[row].shortname, index, OffOn[value], value);
					if (f) fprintf(f, "%6s %5s %s\n", params[row].shortname, OffOn[value], params[row].fullname);
					break;
				case 3:	// 0 = off 1 = auto 2 = on
					value = clamp(value, 0, 2);
					sprintf(buffer, "INFO " PROGNAME " Param %s[%x] = %s", params[row].shortname, index, OffAutoOn[value], value);
					if (f) fprintf(f, "%6s %5s %s\n", params[row].shortname, OffAutoOn[value], params[row].fullname);
					break;
				case 10:
					sprintf(buffer, "INFO " PROGNAME " Param %s[%x] = %.1f", params[row].shortname, index, value / 10.0);
					if (f) fprintf(f, "%6s %5.1f %s\n", params[row].shortname, value / 10.0, params[row].fullname);
					break;
				default:
					sprintf(buffer, "WARN " PROGNAME " Scale not known for parameter %s", params[row].shortname);
			}
			logmsg(INFO,buffer);
			DEBUG2 fprintf(stderr, "row = %d numparams = %d\n", row, numparams);
			if (f && (row + 1  == numparams)) {
				logmsg(INFO, "INFO " PROGNAME " Parameter file /tmp/resol.txt created");
				fclose(f);
				f = 0;
			}

			break;
		case 0x200:	// Write value - please respond
			DEBUG fprintf(stderr, "Prot2 0200 Write %x with value %d\n", index, value);
			break;
		case 0x300:	// Read value - please respond
			DEBUG fprintf(stderr, "Prot2 0300 Read %x with value %d\n", index, value);
			break;
		case 0x400:	// Write value - please respond
			DEBUG fprintf(stderr, "Prot2 0400 Write %x with value %d\n", index, value);
			break;
		case 0x500:	// Vbus clearance by Master
			DEBUG2 fprintf(stderr, "Prot2 0500 Vbus Master clearance\n");
			break;
		case 0x600:	// Vbus clearance by Slave
			break;
		default:	// Unknown
			sprintf(buffer, "WARN " PROGNAME " Invalid command %04x in Protocol 2 packet", cmd);
	}
}

int isPort(char * name) {
	// Return TRUE if name is of form hostname:port
	// else FALSE
	if (strchr(name, ':'))
		return 1;
	else
		return 0;
}

/*************/
/* VBUSLOGIN */
/*************/
void vbuslogin(int sock, char * password) {
	// Perform Resol VBUS login dialog
	char buf[30];
	int num;
	// Wait for "+HELLO"
	num = read(sock, buf, sizeof(buf));
	if (num > 0) {
		buf[num]=0;
		DEBUG fprintf(stderr, "Login read %d: '%s'\n", num, buf);
	} else {		// Problem writing. Valid socket?
		sprintf(buffer, "ERROR " PROGNAME " Failed to login to VBUS: %s", strerror(errno));
		logmsg(ERROR, buffer);
		return;
	}
	
	// Send password
	sprintf(buf, "PASS %s\r\n", password);
	write(sock, buf, strlen(buf));
	// Wait for +OK
	num = read(sock, buf, sizeof(buf));
	if (num > 0) {
		buf[num]=0;
		DEBUG fprintf(stderr, "Login read %d: '%s'\n", num, buf);
	}

	// Request DATA mode 
	strcpy(buf, "DATA\r\n");
	write(sock, buf, strlen(buf));
	num = read(sock, buf, sizeof(buf));
	if (num > 0) {
		buf[num]=0;
		DEBUG fprintf(stderr, "Login read %d: '%s'\n", num, buf);
	}
	
	// Expect data
	strcpy(buf, "\r\n");
	write(sock, buf, strlen(buf));
}

/***********/
/* ANALYSE */
/***********/
void analyse() {
	// output a buffer. When called with -a flag to analyse output
	fprintf(stderr, "Analyse packet %d long ", data.count);
	int i, value;
	int dest = getShort(1);
	int source = getShort(3);
	int protocol = data.buf[5];
	fprintf(stderr, "SRC: %04x DEST: %04x PROT: %02x ", source, dest, protocol);
	if (protocol == 0x10) {
		int command = getShort(6);
		int frames = data.buf[8];
		int chk = 0;
		for (i = 1; i < 9; i++) chk += data.buf[i];
		fprintf(stderr, "CMD: %04x Frames: %d CHK: %02x(%02x) %s\n", command, frames, data.buf[9], chk,
			(data.buf[9] + chk) & 0x7f == 0x7f ? "OK" : "FAIL");
		value = 0;
		for (i = 0; i < frames; i++) {
			fprintf(stderr, "Value %2d: %8d (0x%04x)\t%2d: %8d (0x%04x)\n", value++, get1stval(&data.buf[i * 6 + 10]), 
				get1stval(&data.buf[i * 6 + 10]), value++, get2ndval(&data.buf[i * 6 + 10]), get2ndval(&data.buf[i * 6 + 10]));
		}
	}
	
}

enum UnitsType unitsFromStr(char * name) {
	// Converta unit name to an int
	// enum UnitsType { UNITERROR = -1, INT16, INT32, WATTS48 };
	if (strcasecmp(name, "int") == 0) return INT16;
	if (strcasecmp(name, "int8") == 0) return INT8;
	if (strcasecmp(name, "int32") == 0) return INT32;
	if (strcasecmp(name, "watts48") == 0) return WATTS48;
	return UNITERROR;
}

/*****************/
/* PROCESSPACKET */
/*****************/
int processPacket() {
	// Deal with a packet
	// Return 1 = ok
	int src, dest, protocol, frames, command, check, line;
	int ivalue, loc;
	static int badblock = 0;
	double fvalue;
	char output[100], item[15];
	strcpy(output, "thermal ");
	
	DEBUG fprintf(stderr, "\nProcessPacket: ");
	if (data.count < 9) {
		sprintf(buffer, "WARN " PROGNAME " Packet too short %d - discarding", data.count);
		logmsg(WARN, buffer);
		return 0;
	}
	// More validation (already been done but can remove it from outside this function)
	if (data.buf[0] != STARTSYNC) {		// Can't happen !
		sprintf(buffer, "WARN " PROGNAME " Initial byte is not AA - discarding packet of %d bytes\n", data.count);
		DEBUG dumpbuf();
		return 0;
	}
	src = data.buf[3] | (data.buf[4] << 8);
	dest = data.buf[1] | (data.buf[2] << 8);
	protocol = data.buf[5];
	command = data.buf[7] | (data.buf[6] << 8);
	frames = data.buf[8];
	check = blockcheckN(data.buf + 1, 8);
	DEBUG fprintf(stderr, "SRC %04x DEST %04x Prot %.1f Frames %d chk %d\n", src, dest, (protocol / 16.0), frames, check);

	if (frames * 6 + 10 > data.count) {
		sprintf(buffer, "WARN " PROGNAME " Frames is %d but packet is only %d", frames, data.count);
		logmsg(WARN, buffer);
		return 0;	// Try to avoid core dumps!
	}

	// Only deal with protocol 1
	if (protocol != 0x10)
		return 0;
	
	// Given a source and destination, run through datadef processing each line.
	for (line = 0; line < numdefs; line++) {
		if (datadef[line].src == src && datadef[line].dest == dest) {
			// Validate location is within packet. Should take into account data length of value in case it falls over the end
			loc = datadef[line].location;
			
			// Do a checksum. If block fails, bomb out but increment blockcheckfile.
			if (blockcheck(data.buf, loc / 2)) {
				DEBUG fprintf(stderr, "Checksum loc %d failed ", loc);
				if (badblock++ % 100 == 0) {
					sprintf(buffer, "WARN " PROGNAME " %d bock checksum fails", badblock);
					logmsg(WARN, buffer);
				}
				return 0;
			}
			if (loc / 2 > frames) {
				sprintf(buffer, "WARN " PROGNAME " Packet has %d frames but requested location is %d", frames, loc);
				logmsg(WARN, buffer);
			}
			ivalue = 0;
			switch(datadef[line].units) {
				case INT8:  ivalue = getuval(loc) & 0xFF;	break;
				case INT16:	ivalue = getval(loc);	break;
				case INT32:	ivalue = getuval(loc) + (getval(loc + 1) << 16);	
					DEBUG fprintf(stderr, "A: %u B: %d T: %d ", getuval(loc), getval(loc+1) << 16, ivalue); break;
				case WATTS48:	ivalue = getval(loc) + getval(loc + 1) * 1000 + getval(loc + 2) * 1000000;	break;
			}
			fvalue = ivalue / datadef[line].scale;
			DEBUG fprintf(stderr, "\nLine %d Int %d Float %f Units %d ", line, ivalue, fvalue, datadef[line].units);
			if (datadef[line].scale == 1000)
				sprintf(item, "%s:%.3f ", datadef[line].name, fvalue);
			else
				sprintf(item, "%s:%.1f ", datadef[line].name, fvalue);
			strncat(output, item, sizeof(item));
		}
		else
			DEBUG2 fprintf(stderr, "Ignoring line %d as Src: %x Dest: %x\n", line, datadef[line].src, datadef[line].dest);
		
	}

	if (strlen(output) > 8) {
		DEBUG fprintf(stderr, "Writedata sending '%s' ", output);
		sockSend(sockfd[0], output);
	}
	
	
	return 1;
}


	
