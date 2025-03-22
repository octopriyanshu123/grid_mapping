/*******************************************************************************
 *					COPYRIGHT (C) 2022-205. Mybox Technologies PVT Ltd.
 *
 * File Name		: SERIALREADERMAIN.C
 *
 * File Fullname		: SERIAL READER MAIN.C
 *
 * Description
 * -----------
 *	Source file for Serial data parsing emitted from M1-OEB board with RS485 protocal.GUI based on GTK 2.0
 *
 * Revision History
 * ----------------
 * Date             : 15Nov2022
 * Author           : Magesh
 * Details          : Initial version
 *
 *******************************************************************************/
// compile comment -> gcc -g  serialreadermain.c -o p1 `pkg-config --cflags --libs gtk+-2.0` -lpthread

/*-------------------------------------------------------------------------
                                HEADER FILE
-------------------------------------------------------------------------*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int64.h"
#include "serialtoros/thick_arr.h"
#include "serialtoros/graph_arr.h"
#include "serialtoros/VDE_arr.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <execinfo.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>

/*-------------------------------------------------------------------------
                                TYPEDEF
-------------------------------------------------------------------------*/
typedef unsigned char Uint8;
typedef unsigned char BYTE;
typedef short int Uint16;
typedef unsigned short int Unit16;
typedef char Int8;
typedef unsigned int Uint32;
typedef unsigned short int WORD;
typedef void VOID;
/*-------------------------------------------------------------------------
                                    DEFINES
-------------------------------------------------------------------------*/
#define DISPLAY_BY_BYTE
#define DEBUG_PRINT_ENABLE
#define ENABlE_WITH_GUI
#define SERIAL_FILE_PATH "/dev/ut"
#define UNIT_MM "mm"
#define UNIT_INCHES "inches"
/*-------------------------------------------------------------------------
                                MACROS
-------------------------------------------------------------------------*/
#define GETBIT(var, bit) (((var) >> (bit)) & 1)
#define NULL_CHECK(val)               \
    if (val == NULL)                  \
    {                                 \
        printf("%d >NULL", __LINE__); \
        return;                       \
    }

#ifdef DEBUG_PRINT_ENABLE
#define DEBUG_PRINT(__X__) printf __X__
#else
#define DEBUG_PRINT(__X__)
#endif

/*-------------------------------------------------------------------------
                                FUNCTION PROTOTYPES
-------------------------------------------------------------------------*/
WORD CRC16(const BYTE *nData, WORD wLength);
int serial_readermain();
VOID signalHandler(int signal);
VOID Velocity_button_click(Uint16 ui16Velocity);
VOID Deepcoat_button_click(Uint16 deepcoat);
VOID SetAS_button_click(Uint16 StatusXrange);
VOID Parse_STATUS(Uint8 *pReceivedData, Uint16 ui16ReceivedLength);
VOID *serialWrite(void *vargp);
VOID Parse_STATUS_AS(Uint8 *pReceivedData, Uint16 ui16ReceivedLength);
VOID SetAS_Request();

/*-------------------------------------------------------------------------
                                STRUCTURE
-------------------------------------------------------------------------*/
typedef struct status
{
    Uint8 STX;
    Uint16 Preamble;
    Uint8 Ui8Reserved;

    Uint16 Command;
    Uint16 Size;

    Uint16 BITFIELD;
    Uint16 Ui16Reserved2;

    Uint32 SERIAL_NUMBER;

    Uint8 ID;
    Uint8 BATTERY;
    Uint16 VELOCITY;

    Uint8 ECHOES;
    Uint8 PROBE_INFO;
    Uint8 HVSETUP;
    Uint8 Ui8Reserved3;

    Uint16 RECORDS;
    Unit16 CMD_NUM;

    Uint16 LAST_CMD;
    Int8 THICKNESS[6];

    Uint16 CRC16;
    Uint16 Ui16Reserved4;
} STRUCT_STATUS;

typedef struct status_as
{
    Uint16 STX;
    Uint16 Preamble;
    Uint16 Command;
    Uint16 Size;
    Uint16 BITFIELD;
    Uint32 SERIAL_NUMBER;
    Uint8 ID;
    Uint8 BATTERY;
    Uint16 VELOCITY;
    Uint8 ECHOES;
    Uint8 DEEPCOAT;
    Uint8 PROBE_INFO;
    Uint8 HVSETUP;
    Uint16 RECORDS;
    Unit16 CMD_NUM;
    Uint16 LAST_CMD;
    Int8 THICKNESS[6];
    Uint16 XSIZE;
    Uint16 HOLDOFF;
    Uint8 XRANGE;
    Uint8 TE_MATCH_POINTS[9];
    Uint8 ASCAN[320];
    Uint16 CRC16;
    Uint16 StatusXrange;

} STRUCT_STATUS_AS;

/*-------------------------------------------------------------------------
                                GLOBAL VARIABLES
-------------------------------------------------------------------------*/
// User for testing purpose
Uint8 samplearray[] = {
    0x2, 0x55, 0x55, 0x5, 0x0, 0x22,
    0x0, 0x10, 0x93, 0xdc, 0x30, 0x0,
    0x0, 0x1, 0xb4, 0x20, 0x17, 0x0,
    0x2, 0xf, 0xde, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x20, 0x20, 0x35, 0x2e, 0x30,
    0x0, 0x5a, 0x7d};

Uint8 samplearrayAS[] = {
    0x2, 0x55, 0x55, 0x6, 0x0, 0x70, 0x1, 0x16, 0x93, 0xdc, 0x30, 0x0, 0x0, 0x1, 0x7b, 0x58, 0x1b, 0x3, 0x2, 0xf, 0x8b, 0x1, 0x1, 0x0, 0x1, 0x80,
    0x20, 0x36, 0x30, 0x2e, 0x39, 0x0, 0x40, 0x1, 0x4b, 0x0, 0x83, 0x10, 0x2, 0x1, 0x17, 0x4, 0x1, 0x20, 0x6, 0x1, 0xff, 0xff, 0xff, 0xdf, 0xe9, 0xae,
    0x2a, 0x10, 0xe, 0xa, 0xa, 0x2, 0x8, 0x6, 0x4, 0x4, 0x4, 0x6, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2,
    0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0xc, 0x51, 0x88, 0xae, 0x84, 0x45, 0x12, 0x12, 0x2, 0x6, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2,
    0x2, 0x2, 0x2, 0x2, 0x2, 0x4, 0x6, 0x6, 0x4, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x4, 0x4, 0x4, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2,
    0x2, 0x2, 0x16, 0x4b, 0xa2, 0xa4, 0x80, 0x1c, 0x12, 0xe, 0x2, 0x4, 0x4, 0x4, 0x2, 0x4, 0x2, 0x4, 0x2, 0x4, 0x4, 0x2, 0x2, 0x2, 0x6, 0x8, 0x8, 0x4,
    0x4, 0x4, 0x2, 0x2, 0x2, 0x4, 0x6, 0x4, 0x4, 0x6, 0x2, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x8, 0x6, 0x6, 0x6, 0x8, 0x24, 0x9e, 0xd5, 0xd1, 0x63,
    0x1e, 0x1a, 0x12, 0x4, 0x2, 0x4, 0xa, 0xa, 0xe, 0xe, 0xa, 0x4, 0x4, 0x4, 0x8, 0x6, 0xa, 0x10, 0xc, 0x4, 0x8, 0x8, 0x8, 0xa, 0x8, 0x6, 0x16, 0x16,
    0x12, 0xc, 0xa, 0x4, 0x6, 0x8, 0xa, 0xc, 0x8, 0x8, 0x8, 0x4, 0xa, 0xc, 0x4, 0xe, 0x6b, 0xd1, 0xff, 0xd1, 0x82, 0x43, 0x1c, 0x2c, 0x22, 0x10, 0x1a,
    0x12, 0x10, 0x18, 0x20, 0x1a, 0x10, 0x8, 0x8, 0xa, 0x12, 0x1a, 0x1c, 0x12, 0xe, 0x1c, 0x22, 0x14, 0x6, 0x14, 0x2c, 0x28, 0x14, 0x10, 0x1a, 0x10,
    0xe, 0x10, 0x14, 0xc, 0x18, 0x16, 0xa, 0x14, 0x26, 0x22, 0x1a, 0xa, 0x4d, 0xf3, 0xff, 0xff, 0xf1, 0x6d, 0xc7, 0x67, 0x38, 0x57, 0x55, 0x34, 0x28,
    0x36, 0x36, 0x2c, 0x24, 0x1a, 0x14, 0x1a, 0x41, 0x59, 0x4d, 0x34, 0x5b, 0x57, 0x98, 0x98, 0x84, 0x94, 0x45, 0x59, 0x49, 0x38, 0x82, 0x75, 0x67,
    0x6f, 0x6f, 0x4b, 0x43, 0x32, 0x6d, 0x86, 0x7d, 0x20, 0x30, 0x2c, 0x86, 0xcf, 0xff, 0xff, 0xff, 0xcf, 0x30, 0xc5, 0xff, 0xff, 0xff, 0x90, 0xa4, 0x88, 0x7b,
    0x8a, 0x5b, 0x3e, 0x5d, 0x84, 0x92, 0x84, 0x7d, 0x6d, 0xb0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x70, 0x17

};

static int counter = 0;
// GtkWidget *table, *label, *Velocity_edit, *Deep_coat_edit, *Velocity_button, *Deep_coat_button, *set_As_button, *multiedit,
// 	*Thickness = NULL, *unitvalue = NULL, *velocityvalue = NULL, *Deepcoatvalue = NULL, *Modevalue = NULL,
// 	*DeepcoatLable, *SendAsRequestLabel, *SendAsRequestEdit, *SendAsRequestButton;

// GtkTextBuffer *textBufferName;
// GtkTooltips *tooltipsV, *tooltipsD, *tooltipsSA;

int fd;

// SlopeScale * scale  = NULL;
// SlopeItem *  series = NULL;
double *pointx = NULL, *pointy = NULL;

const long numberofpoints = 320;
// const double dx = 4.0 * G_PI / 200;
// GtkWidget *  chart;
/*-------------------------------------------------------------------------
                                FUNCTION DEFINITION
-------------------------------------------------------------------------*/
WORD CRC16(const BYTE *Data, WORD wLength)
{
    static const WORD wCRCTable[] = {
        0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
        0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
        0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
        0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
        0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
        0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
        0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
        0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
        0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
        0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
        0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
        0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
        0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
        0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
        0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
        0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
        0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
        0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
        0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
        0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
        0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
        0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
        0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
        0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
        0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
        0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
        0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
        0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
        0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
        0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
        0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
        0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040};

    BYTE nTemp;
    WORD wCRCWord = 0xFFFF;

    while (wLength--)
    {
        nTemp = *Data++ ^ wCRCWord;
        wCRCWord >>= 8;
        wCRCWord ^= wCRCTable[nTemp];
    }
    return wCRCWord;
}

// VOID destroy(GtkWidget *widget, gpointer data)
// {
// #ifdef NEVER
// 	gtk_tooltips_destroy(tooltipsD);
// 	gtk_tooltips_destroy(tooltipsSA);
// #endif /* NEVER */
// 	gtk_main_quit();

// 	exit(0);
// }

VOID *serialread(void *vargp)
{
    while (ros::ok())
    {
        serial_readermain();
        sleep(1);

        ros::spinOnce();
    }

    return NULL;
}

VOID printbuffer(int rdlen, unsigned char *buf)
{
    unsigned char *p;

    if (rdlen > 0)
    {
        printf("Read %d:", rdlen);
        for (p = buf; rdlen-- > 0; p++)
        {
            printf(" %d ,", *p);
        }
        printf("\n");
    }
}

VOID signalHandler(int signal)
{
    VOID *array[500];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 500);
    // print out all the frames to stderr

    fprintf(stderr, "Error: signal %d:\n", signal);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

void registerSignalHandler()
{
    signal(SIGSEGV, signalHandler);
}

/*

0	STX			Uint8	1	0x02
1	Preamble		Uint16	2	0x5555
3	Command		Uint16	2	0x8002
5	Size			Uint16	2	11 Bytes
7	VELOCITY		Uint16	2	New Velocity value. Examples; 5920 in metric (5920 m/s) 2332 in imperial (0.2332 in/us)
9	CRC16			Uint16	2	As calculated

*/

// Subscriber 1

void VelocityCallback(const std_msgs::Int64::ConstPtr &msg)
{
    // DEBUG_PRINT(("Converted Velocity: %ld\n", msg->data));

    Velocity_button_click(msg->data);
}

VOID Velocity_button_click(Uint16 Velocity)
{
    Uint8 velocity[32] = {0};
    Uint16 Convertedvelocity = 0, ui16CalcuatedCRCvalue = 0, VelocitySize = 11;
    Uint8 *PforCharSeparted = NULL;
    Uint16 wlen, xlen;

    // const gchar *Velocity = gtk_entry_get_text(GTK_ENTRY(Velocity_edit));
    // DEBUG_PRINT(("Entered Velocity: %s\n", Velocity));

    Convertedvelocity = Velocity;

    // DEBUG_PRINT(("Converted Velocity: %X\n", Convertedvelocity));

    velocity[0] = 0x02;
    velocity[1] = 0x55;
    velocity[2] = 0x55;
    velocity[3] = 0x02;
    velocity[4] = 0x80;

    PforCharSeparted = (Uint8 *)&VelocitySize;

    // DEBUG_PRINT(("Velocity Size: %X %X\n", *PforCharSeparted, *(PforCharSeparted + 1)));

    velocity[5] = *PforCharSeparted;
    velocity[6] = *(PforCharSeparted + 1);

    PforCharSeparted = (Uint8 *)&Convertedvelocity;

    // DEBUG_PRINT(("Converted Velocity: %X %X\n", *PforCharSeparted, *(PforCharSeparted + 1)));

    velocity[7] = *PforCharSeparted;
    velocity[8] = *(PforCharSeparted + 1);

    ui16CalcuatedCRCvalue = CRC16(velocity, 9);

    // DEBUG_PRINT(("ui16CalcuatedCRCvalue: %X\n", ui16CalcuatedCRCvalue));

    PforCharSeparted = (Uint8 *)&ui16CalcuatedCRCvalue;

    // DEBUG_PRINT(("Converted CRCvalue: %X %X\n", *PforCharSeparted, *(PforCharSeparted + 1)));

    velocity[9] = *PforCharSeparted;
    velocity[10] = *(PforCharSeparted + 1);

    xlen = 11;

    // printbuffer(11, velocity);

    wlen = write(fd, velocity, xlen);
    if (wlen != xlen)
    {
        // DEBUG_PRINT(("Error from velocity write: %d, %d\n", wlen, errno));
    }
    tcdrain(fd); /* delay for output */

    // gtk_text_buffer_set_text(textBufferName, long_text, -1);
}

/*
0	STX			Uint8	1	0x02
1	Preamble		Uint16	2	0x5555
3	Command		Uint16	2	0x8003
5	Size			Uint16	2	10 Bytes
7	DEEPCOAT		Uint8	1	Set Deepcoat: 0=Off ,1=On
8	CRC16			Uint16	2	As calculated
*/

// SUBSCRIBER 2

void DeepcoatCallback(const std_msgs::Int8::ConstPtr &msg)
{
    // Uint16 deepcoat=msg->data;
    // DEBUG_PRINT(("Deepcoat value: %d\n", msg->data));

    Deepcoat_button_click(msg->data);
}

VOID Deepcoat_button_click(Uint16 DeepCoat)
{
    Uint8 Deepcoatdata[32] = {0};
    Uint16 ui16CalcuatedCRCvalue = 0, DeepcoatSize = 10;
    Uint8 *PforCharSeparted = NULL, ConvertedDeepcoat = 0;
    Uint16 wlen, xlen;

    // const gchar *Deep_coat = gtk_entry_get_text(GTK_ENTRY(Deep_coat_edit));
    // DEBUG_PRINT(("Entered Deep_coat: %s\n", Deep_coat));

    ConvertedDeepcoat = DeepCoat;

    // DEBUG_PRINT(("Converted Deep coat: %X\n", ConvertedDeepcoat));

    PforCharSeparted = (Uint8 *)&ConvertedDeepcoat;

    // DEBUG_PRINT(("Converted Deep coat: %X %X\n", *PforCharSeparted, *(PforCharSeparted + 1)));

    Deepcoatdata[0] = 0x02;
    Deepcoatdata[1] = 0x55;
    Deepcoatdata[2] = 0x55;
    Deepcoatdata[3] = 0x03;
    Deepcoatdata[4] = 0x80;

    PforCharSeparted = (Uint8 *)&DeepcoatSize;

    // DEBUG_PRINT(("Converted Size: %X %X\n", *PforCharSeparted, *(PforCharSeparted + 1)));

    Deepcoatdata[5] = *PforCharSeparted;
    Deepcoatdata[6] = *(PforCharSeparted + 1);

    Deepcoatdata[7] = ConvertedDeepcoat;

    ui16CalcuatedCRCvalue = CRC16(Deepcoatdata, 8);

    // DEBUG_PRINT(("ui16CalcuatedCRCvalue: %X\n", ui16CalcuatedCRCvalue));

    PforCharSeparted = (Uint8 *)&ui16CalcuatedCRCvalue;

    // DEBUG_PRINT(("Converted CRCvalue:: %X %X\n", *PforCharSeparted, *(PforCharSeparted + 1)));

    Deepcoatdata[8] = *PforCharSeparted;
    Deepcoatdata[9] = *(PforCharSeparted + 1);

    xlen = 10;

    // printbuffer(10, Deepcoatdata);

    wlen = write(fd, Deepcoatdata, xlen);
    if (wlen != xlen)
    {
        DEBUG_PRINT(("Error from velocity write: %d, %d\n", wlen, errno));
    }
    tcdrain(fd); /* delay for output */
}

/*
0	STX			Uint8	1	0x02
1	Preamble		Uint16	2	0x5555
3	Command		Uint16	2	0x8005
5	Size			Uint16	2	10 Bytes
7	ASCAN_XAR	Uint8	1	Sets the A-scan X range.
8	CRC16			Uint16	2	As calculated

*/
// SUBSCRIBER 3

void XrangeCallback(const std_msgs::Int16::ConstPtr &msg)
{
    DEBUG_PRINT(("Xrange value: %d\n", msg->data));

    SetAS_button_click(msg->data);
}

VOID SetAS_button_click(Uint16 StatusXrange)
{
    Uint8 StatusAS[32] = {0};
    Uint16 ui16CalcuatedCRCvalue = 0, StatusASSize = 10;
    Uint8 *PforCharSeparted = NULL, ConvertedStatusAS = 0;
    Uint16 wlen, xlen;

    // const gchar *Set_AS = gtk_entry_get_text(GTK_ENTRY(SendAsRequestEdit));
    // DEBUG_PRINT(("Entered Set_AS: %s\n", Set_AS));

    SetAS_Request();

    sleep(2);

    ConvertedStatusAS = StatusXrange;

    // DEBUG_PRINT(("Converted Deep coat: %X\n", ConvertedStatusAS));

    PforCharSeparted = (Uint8 *)&ConvertedStatusAS;

    // DEBUG_PRINT(("Converted Deep coat: %X %X\n", *PforCharSeparted, *(PforCharSeparted + 1)));

    StatusAS[0] = 0x02;
    StatusAS[1] = 0x55;
    StatusAS[2] = 0x55;
    StatusAS[3] = 0x05;
    StatusAS[4] = 0x80;

    PforCharSeparted = (Uint8 *)&StatusASSize;

    // DEBUG_PRINT(("Converted Size: %X %X\n", *PforCharSeparted, *(PforCharSeparted + 1)));

    StatusAS[5] = *PforCharSeparted;
    StatusAS[6] = *(PforCharSeparted + 1);

    StatusAS[7] = ConvertedStatusAS;

    ui16CalcuatedCRCvalue = CRC16(StatusAS, 8);

    // DEBUG_PRINT(("ui16CalcuatedCRCvalue: %X\n", ui16CalcuatedCRCvalue));

    PforCharSeparted = (Uint8 *)&ui16CalcuatedCRCvalue;
    // const gchar *Set_AS = gtk_entry_get_text(GTK_ENTRY(SendAsRequestEdit));
    // DEBUG_PRINT(("Converted CRCvalue:: %X %X\n", *PforCharSeparted, *(PforCharSeparted + 1)));

    StatusAS[8] = *PforCharSeparted;
    StatusAS[9] = *(PforCharSeparted + 1);

    xlen = 10;

    // printbuffer(xlen, StatusAS);

    wlen = write(fd, StatusAS, xlen);
    if (wlen != xlen)
    {
        // DEBUG_PRINT(("Error from velocity write: %d, %d\n", wlen, errno));
    }
    tcdrain(fd); /* delay for output */
}

/*
0	STX		Uint8	1	0x02
1	Preamble	Uint16	2	0x5555
3	Command	Uint16	2	0x8001
5	Size		Uint16	2	12 Bytes
7	XSIZE		Uint16	2	Number of X points to Send (default = 640, min=640, max=2048)
9	REPEAT	Uint8	1	Number of A-Scans to send. 0 = none, i.e.
                            Stop sending 1..254 = Send N then stop 255 = send continuously Note.
                            It may not be possible to send large A-scan data at full rate due to baud rate limitations.
10	CRC16		Uint16	2	As calculated


*/
VOID SetAS_Request(VOID)
{
    Uint8 StatusAS[32] = {0};
    Uint16 ui16CalcuatedCRCvalue = 0, StatusASSize = 12, NumberofXpointstoSend = 640;
    Uint8 *PforCharSeparted = NULL, ConvertedStatusAS = 0;
    Uint16 wlen, xlen;

    StatusAS[0] = 0x02;
    StatusAS[1] = 0x55;
    StatusAS[2] = 0x55;
    StatusAS[3] = 0x01;
    StatusAS[4] = 0x80;

    PforCharSeparted = (Uint8 *)&StatusASSize;

    // DEBUG_PRINT(("Converted StatusASSize: %X %X\n", *PforCharSeparted, *(PforCharSeparted + 1)));

    StatusAS[5] = *PforCharSeparted;
    StatusAS[6] = *(PforCharSeparted + 1);

    PforCharSeparted = (Uint8 *)&NumberofXpointstoSend;

    // DEBUG_PRINT(("Converted NumberofXpointstoSend: %X %X\n", *PforCharSeparted, *(PforCharSeparted + 1)));

    StatusAS[7] = *PforCharSeparted;
    StatusAS[8] = *(PforCharSeparted + 1);

    StatusAS[9] = 255;

    ui16CalcuatedCRCvalue = CRC16(StatusAS, 10);

    // DEBUG_PRINT(("ui16CalcuatedCRCvalue: %X\n", ui16CalcuatedCRCvalue));

    PforCharSeparted = (Uint8 *)&ui16CalcuatedCRCvalue;

    // DEBUG_PRINT(("Converted CRCvalue:: %X %X\n", *PforCharSeparted, *(PforCharSeparted + 1)));

    StatusAS[10] = *PforCharSeparted;
    StatusAS[11] = *(PforCharSeparted + 1);

    xlen = 12;

    // printbuffer(xlen, StatusAS);

    wlen = write(fd, StatusAS, xlen);
    if (wlen != xlen)
    {
        // DEBUG_PRINT(("Error from velocity write: %d, %d\n", wlen, errno));
    }
    tcdrain(fd); /* delay for output */
}

// global

//  publisher topic name
std::string thickness_pub_topic;
std::string graph_pub_topic;
std::string VDE_pub_topic;

// publisher
ros::Publisher thickness_pub;
ros::Publisher graph_pub;
ros::Publisher VDE_pub;

// Subscriber
ros::Subscriber velocity_sub;
ros::Subscriber Deepcoat_sub;
ros::Subscriber Xrange_sub;

serialtoros::thick_arr thick_val;
serialtoros::graph_arr graph_val;
serialtoros::VDE_arr VDE_val;

int main(int argc, char *argv[])
{
    // DEBUG_PRINT(("Main started: \n"));
    ros::init(argc, argv, "Ut_node");

    ros::NodeHandle n;

    velocity_sub = n.subscribe("/send_ut_velocity", 1, VelocityCallback);
    Deepcoat_sub = n.subscribe("/send_ut_deepcoat", 1, DeepcoatCallback);
    Xrange_sub = n.subscribe("/send_ut_xrange", 1, XrangeCallback);

    thickness_pub_topic = "/ut_thickness";
    graph_pub_topic = "/ut_graph";
    VDE_pub_topic = "/ut_VDE_values";

    thickness_pub = n.advertise<serialtoros::thick_arr>(thickness_pub_topic, 100);
    graph_pub = n.advertise<serialtoros::graph_arr>(graph_pub_topic, 1);
    VDE_pub = n.advertise<serialtoros::VDE_arr>(VDE_pub_topic, 100);

    registerSignalHandler();
    pthread_t thread_id_serial_read; // thread_id_serial_write;

    // pthread_create(&thread_id_serial_write, NULL, serialWrite, NULL);
    pthread_create(&thread_id_serial_read, NULL, serialread, NULL);

    // pthread_join(thread_id_serial_write, NULL);
    pthread_join(thread_id_serial_read, NULL);

    // while (1)
    // {
    //     printf("\n Main running");
    //     sleep(100000);
    // }
    return 0;
}

// -1: expired. 1: Not expired
char dueDay[] = "20221125";
int IsExpired()
{
    time_t currentTime;
    struct tm *timeInfo;
    char buffer[10];
    int value;

    time(&currentTime);
    timeInfo = localtime(&currentTime);
    strftime(buffer, sizeof(buffer), "%Y%m%d", timeInfo);

    // printf("IsExpired checking: Due:%s cur:%s \n",dueDay,buffer);

    value = strcmp(dueDay, buffer);

    if (value >= 0)
        return 0; // Not expired.
    else
        return 1; // Expired
}

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0)
    {
        // printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      /* 8-bit characters */
    tty.c_cflag &= ~PARENB;  /* no parity bit */
    tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0)
    {
        // printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5; /* half second timer */

    // if (tcsetattr(fd, TCSANOW, &tty) < 0)
    //  printf("Error tcsetattr: %s\n", strerror(errno));
}

// Used for testing purpose
#if 0
int serial_readermain()
{
	Parse_STATUS(samplearray,sizeof(samplearray));
	return 0;
}

#else
int serial_readermain()
{
    const char *portname = SERIAL_FILE_PATH;
    int wlen;

    // printf("Welcome to Serial Reader : version 1.0\n ");

    /*if (IsExpired())
    {
        printf("\n Pleae contact developer for more information");
        return 1;
    }*/

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        // printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);
    // set_mincount(fd, 0);                /* set to pure timed read */

    /* simple output
    wlen = write(fd, xstr, xlen);
    if (wlen != xlen) {
    printf("Error from write: %d, %d\n", wlen, errno);
    }
    tcdrain(fd);    /* delay for output */

    /* simple noncanonical input */
    do
    {
        Uint8 buf[400];
        Uint32 rdlen, Backuplen;

        rdlen = read(fd, buf, sizeof(buf) - 1);
        Backuplen = rdlen;
        if (rdlen > 0)
        {
            // ros

            // printbuffer(rdlen, buf);

            if (Backuplen == 34)
            {
                Parse_STATUS(buf, Backuplen);
            }
            else if (Backuplen == 368)
            {
                Parse_STATUS_AS(buf, Backuplen);
            }
        }
        else if (rdlen < 0)
        {
            // printf("Error from read: %d: %s\n", rdlen, strerror(errno));
        }
        else
        { /* rdlen == 0 */
            // printf("Timeout from read\n");
        }
        /* repeat read to get full message */
        ros::spinOnce();

    } while (ros::ok());
    return 0;
}
#endif

/*
Byte	Name		Type		Size		Details

0	STX					Uint8	1	0x02
1	Preamble			Uint16	2	0x5555
3	Command			Uint16	2	0x0005
5	Size				Uint16	2	34 Bytes
7	BITFIELD			Uint16	2	Bit[0] 1=Imperial, 0=Metric
9									Bit[1] 1=Metal Contact, 0=Idle
9									Bit[2] 1=Valid Thickness Measurement, 0=Not
9									Bit[3] 1=Deep Coat, 0=Normal
9									Bit[4] 1=SRATE=30MSPS, 0=SRATE=60MSPS
9									Bit[5] 1=TESTMODE, 0=NORMAL
9									Bit[6:7] Resolution enum (0=Low, 1=High, 2=Vhigh)
9									Bit[8:10] Measure Mode enum (1=SE, 2=EE, 3=EE)
9									Bit[11] Toggles when a measurment is Logged
9									Bit[12:15] M1-OEM Mk Number = 9
9	SERIAL_NUMBER	Uint32	4	M1-OEM Serial Number
13	ID					Uint8	1	N/A
14	BATTERY			Uint8	1	N/A
15	VELOCITY			Uint16	2	Current velocity of sound (i.e. 9520 or 2332)
17	ECHOES			Uint8	1	Number of Echoes Detected, 0..3
18	PROBE_INFO		Uint8	1	"Bit[0:5] Probe Type EnumBit[6] 1=Waiting Probe ZeroBit[7] 1=Probe Error"
19	HVSETUP			Uint8	1	N/A
20	RECORDS			Uint16	2	N/A
22	CMD NUM			Unit16	2	Number of commands successfully processed. Reset to zero when gauge is turned off.
24	LAST CMD			Uint16	2	Last command that was successfully received and processed. Initially set to 0 until the first command is received.
26	THICKNESS			Int8[6]	6	Thickness Measurement Value in ASCII, Examples; \93200.00\94, \93 0.254\94, \93      \93
32	CRC16				Uint16	2	As calculated
*/

VOID Parse_STATUS(Uint8 *pReceivedData, Uint16 ui16ReceivedLength)
{
    STRUCT_STATUS stStatusStruct = {0};
    Unit16 CalculatedCrcValue, ExpectedCrcValue, Rceciedvalue;
    const Int8 *unit_value = UNIT_MM;
    char acbuffer[32];

    NULL_CHECK(pReceivedData);

    ExpectedCrcValue = pReceivedData[ui16ReceivedLength - 2] | pReceivedData[ui16ReceivedLength - 1] << 8;
    CalculatedCrcValue = CRC16(pReceivedData, ui16ReceivedLength - 2);

    memset(stStatusStruct.THICKNESS, 0, sizeof(stStatusStruct.THICKNESS));

    // DEBUG_PRINT(("\n CalculatedCrcValue {%X} ExpectedCrcValue{%X} ", CalculatedCrcValue, ExpectedCrcValue));

    if (CalculatedCrcValue == ExpectedCrcValue)
    {
        unsigned short int bitfield = 0;

        // DEBUG_PRINT(("\n >>>>>>>>>>> CRC PASSED <<<<<<<<<<<<<<<<<"));

        stStatusStruct.STX = pReceivedData[0];
        stStatusStruct.Preamble = pReceivedData[1] | pReceivedData[2] << 8;
        stStatusStruct.Command = pReceivedData[3] | pReceivedData[4] << 8;
        stStatusStruct.Size = pReceivedData[5] | pReceivedData[6] << 8;
        stStatusStruct.BITFIELD = pReceivedData[7] | pReceivedData[8] << 8;
        stStatusStruct.SERIAL_NUMBER = (pReceivedData[9] << 24 | pReceivedData[10] << 16 |
                                        pReceivedData[11] << 8 | pReceivedData[12]);
        stStatusStruct.ID = pReceivedData[13];
        stStatusStruct.BATTERY = pReceivedData[14];
        stStatusStruct.VELOCITY = pReceivedData[15] | pReceivedData[16] << 8;
        stStatusStruct.ECHOES = pReceivedData[17];
        stStatusStruct.PROBE_INFO = pReceivedData[18];
        stStatusStruct.HVSETUP = pReceivedData[19];
        stStatusStruct.RECORDS = pReceivedData[20] | pReceivedData[21] << 8;
        stStatusStruct.CMD_NUM = pReceivedData[22] | pReceivedData[23] << 8;
        stStatusStruct.LAST_CMD = pReceivedData[24] | pReceivedData[25] << 8;

        memcpy(stStatusStruct.THICKNESS, pReceivedData + 26, 6);
        bitfield = pReceivedData[7] | pReceivedData[8] << 8;

        if (GETBIT(bitfield, 2) == 1)
        {
            if (GETBIT(bitfield, 0) == 0)
            {
                unit_value = "mm";
                thick_val.data[1] = 1;
            }
            else
            {
                unit_value = "inches";
                thick_val.data[1] = 0;
            }

            // printf("\n THICKNESS Value: %s %s \n\n", stStatusStruct.THICKNESS, unit_value);

            thick_val.data[0] = std::stof(stStatusStruct.THICKNESS);

            thickness_pub.publish(thick_val);
        }
        else
        {
            thick_val.data[0] = 0;
            thick_val.data[1] = -1;

            thickness_pub.publish(thick_val);
        }
        VDE_val.data[0] = stStatusStruct.VELOCITY;
        VDE_val.data[1] = stStatusStruct.ECHOES;
        VDE_val.data[2] = GETBIT(stStatusStruct.BITFIELD, 3);
        VDE_pub.publish(VDE_val);

        // DEBUG_PRINT(("\n Velocity [%d] Waiting_Probe Zero [6] %d \n Probe Error [7] %d",
        //   stStatusStruct.VELOCITY,
        //   GETBIT(stStatusStruct.PROBE_INFO, 6),
        //   GETBIT(stStatusStruct.PROBE_INFO, 7)));

        // DEBUG_PRINT(("\n Echoes %d \n Unit [0] %d \n Valid Thickness Measurement [2] %d \n Deep Coat [3] %d ",
        //   stStatusStruct.ECHOES,
        //   GETBIT(stStatusStruct.BITFIELD, 0),
        //   GETBIT(stStatusStruct.BITFIELD, 2),
        //   GETBIT(stStatusStruct.BITFIELD, 3)));

        // emit Numberchanged_STATUS(stStatusStruct);

        //*Thickness,*unitvalue,*velocityvalue,*Deepcoatvalue,*Modevalue
    }
}

/*
0	STX					Uint8	1	0x02
1	Preamble				Uint16	2	0x5555
3	Command				Uint16	2	0x0006
5	Size					Uint16	2	368 Bytes
7	BITFIELD				Uint16	2	Bit[0] 1=Imperial, 0=Metric
9										Bit[1] 1=Metal Contact, 0=Idle
9										Bit[2] 1=Valid Thickness Measurement, 0=Not
9										Bit[3] 1=Deep Coat, 0=Normal
9										Bit[4] 1=SRATE=30MSPS, 0=SRATE=60MSPS
9										Bit[5] 1=TESTMODE, 0=NORMAL
9										Bit[6:7] Resolution enum (0=Low, 1=High, 2=Vhigh)
9										Bit[8:10] Measure Mode enum (1=SE, 2=EE, 3=EE)
9										Bit[11] Toggles when a measurment is Logged
9											Bit[12:15] M1-OEM Mk Number = 9
9	SERIAL_NUMBER		Uint32	4	M1-OEM Serial Number
13	ID						Uint8	1	N/A
14	BATTERY				Uint8	1	N/A
15	VELOCITY				Uint16	2	Current velocity of sound (i.e. 9520 or 2332)
17	ECHOES				Uint8	1	Number of Echoes Detected, 0..3
18	PROBE_INFO			Uint8	1	Bit[0:5] Probe Type Enum Bit[6] 1=Waiting Probe Zero Bit[7] 1=Probe Error
19	HVSETUP				Uint8	1	N/A
20	RECORDS				Uint16	2	N/A
22	CMD NUM				Unit16	2	Number of commands successfully processed. Reset to zero when gauge is turned off.
24	LAST CMD				Uint16	2	Last command that was successfully received and processed. Initially set to 0 until the first command is received.
26	THICKNESS			Int8[6]	6	Thickness Measurement Value in ASCII, Examples; \93200.00\94, \93 0.254\94, \93      \93
32	XSIZE					Uint16	2	Number of A-Scan X points sent
34	HOLDOFF*				Uint16	2	Holdoff Gate end point. Graph plot is \93grey\94 before, \93green\94 after.
36	XRANGE				Uint8	1	A-scan range from one of the following; [Bit 0-6] : 0=15mm/0.6\94, 1=30mm/1.2\94 2=60mm/2.4\94, 3=100mm/4\94, 4=200mm/8\94 [Bit 7]     : AUTO Mode when set. NB. The values represent the thickness of material you are attempting to measure not the a-scan X axis range.
37	TE_MATCH POINTS*	{Uint16, Uint8}x3	9	Three XY values indicating which peaks in the A-scan data were selected as a triple echo match. {Uint16 X, Uint8 Y} x 3
46	ASCAN					Uint8 [320]	320	A-Scan Y Point values (0=0%, 254=100%)
366 CRC16	Uint16	2		As calculated
*/

int printRandoms(int lower, int upper)
{
    return (rand() % (upper - lower + 1)) + lower;
}

VOID Parse_STATUS_AS(Uint8 *pReceivedData, Uint16 ui16ReceivedLength)
{
    STRUCT_STATUS_AS stASStatusStruct = {0};
    Unit16 CalculatedCrcValue, ExpectedCrcValue, Rceciedvalue;
    const Int8 *unit_value = UNIT_MM;
    char acbuffer[400];

    NULL_CHECK(pReceivedData);

    ExpectedCrcValue = pReceivedData[ui16ReceivedLength - 2] | pReceivedData[ui16ReceivedLength - 1] << 8;
    CalculatedCrcValue = CRC16(pReceivedData, ui16ReceivedLength - 2);

    memset(stASStatusStruct.THICKNESS, 0, sizeof(stASStatusStruct.THICKNESS));

    // DEBUG_PRINT(("\n CalculatedCrcValue {%X} ExpectedCrcValue{%X} ", CalculatedCrcValue, ExpectedCrcValue));

    if (CalculatedCrcValue == ExpectedCrcValue)
    {
        unsigned short int bitfield = 0;

        // DEBUG_PRINT(("\n >>>>>>>>>>> CRC PASSED <<<<<<<<<<<<<<<<<"));

        stASStatusStruct.STX = pReceivedData[0];
        stASStatusStruct.Preamble = pReceivedData[1] | pReceivedData[2] << 8;
        stASStatusStruct.Command = pReceivedData[3] | pReceivedData[4] << 8;
        stASStatusStruct.Size = pReceivedData[5] | pReceivedData[6] << 8;
        stASStatusStruct.BITFIELD = pReceivedData[7] | pReceivedData[8] << 8;
        stASStatusStruct.SERIAL_NUMBER = (pReceivedData[9] << 24 | pReceivedData[10] << 16 |
                                          pReceivedData[11] << 8 | pReceivedData[12]);
        stASStatusStruct.ID = pReceivedData[13];
        stASStatusStruct.BATTERY = pReceivedData[14];
        stASStatusStruct.VELOCITY = pReceivedData[15] | pReceivedData[16] << 8;
        stASStatusStruct.ECHOES = pReceivedData[17];
        stASStatusStruct.PROBE_INFO = pReceivedData[18];
        stASStatusStruct.HVSETUP = pReceivedData[19];
        stASStatusStruct.RECORDS = pReceivedData[20] | pReceivedData[21] << 8;
        stASStatusStruct.CMD_NUM = pReceivedData[22] | pReceivedData[23] << 8;
        stASStatusStruct.LAST_CMD = pReceivedData[24] | pReceivedData[25] << 8;

        memcpy(stASStatusStruct.THICKNESS, pReceivedData + 26, 6);
        bitfield = pReceivedData[7] | pReceivedData[8] << 8;

        if (GETBIT(bitfield, 2) == 1)
        {
            if (GETBIT(bitfield, 0) == 0)
            {
                unit_value = "mm";
                thick_val.data[1] = 1;
            }
            else
            {
                unit_value = "inches";
                thick_val.data[1] = 0;
            }

            // printf("\n THICKNESS Value: %s %s \n\n", stASStatusStruct.THICKNESS, unit_value);

            thick_val.data[0] = std::stof(stASStatusStruct.THICKNESS);

            thickness_pub.publish(thick_val);

             if(std::stof(stASStatusStruct.THICKNESS) <= 15){
                graph_val.x_range = 60;
                graph_pub.publish(graph_val);

             }
             else if((std::stof(stASStatusStruct.THICKNESS) > 15) && (std::stof(stASStatusStruct.THICKNESS) <= 30)){
                graph_val.x_range = 120;
                graph_pub.publish(graph_val);
             }
             else if((std::stof(stASStatusStruct.THICKNESS) > 30) && (std::stof(stASStatusStruct.THICKNESS) <= 60)){
                graph_val.x_range = 240;
                graph_pub.publish(graph_val);
             }
             else if ((std::stof(stASStatusStruct.THICKNESS) > 60) && (std::stof(stASStatusStruct.THICKNESS) <= 120)){
                graph_val.x_range = 240;
                graph_pub.publish(graph_val);

             }
             else
             {
                graph_val.x_range = 240;
                graph_pub.publish(graph_val);
             }

        }
        else
        {
            thick_val.data[0] = 0;
            thick_val.data[1] = -1;

            thickness_pub.publish(thick_val);
        }

        stASStatusStruct.XSIZE = pReceivedData[32] | pReceivedData[33] << 8;
        stASStatusStruct.HOLDOFF = pReceivedData[34] | pReceivedData[35] << 8;
        stASStatusStruct.XRANGE = pReceivedData[36];

        memcpy(stASStatusStruct.TE_MATCH_POINTS, pReceivedData + 37, 9);

        memcpy(stASStatusStruct.ASCAN, pReceivedData + 46, 320);

        // DEBUG_PRINT(("Graph values: \n"));
        for (int i = 0; i < 320; i++)
        {
            // DEBUG_PRINT(("%d, ", stASStatusStruct.ASCAN[i]));
            graph_val.data[i] = stASStatusStruct.ASCAN[i];
        }
        // DEBUG_PRINT(("\n"));
        VDE_val.data[0] = stASStatusStruct.VELOCITY;
        VDE_val.data[1] = stASStatusStruct.ECHOES;
        VDE_val.data[2] = GETBIT(stASStatusStruct.BITFIELD, 3);
        VDE_pub.publish(VDE_val);
        // sleep(1);
        //  DEBUG_PRINT(("\n X THICKNESS [%s] ", stASStatusStruct.THICKNESS));

        // DEBUG_PRINT(("\n Echo Max X [%d] Y [%d] \n Echo Max X [%d] Y [%d ] \n Echo Max X [%d] Y [%d ",
        //  (stASStatusStruct.TE_MATCH_POINTS[0] | stASStatusStruct.TE_MATCH_POINTS[1] << 8),
        //  stASStatusStruct.TE_MATCH_POINTS[2],

        //  (stASStatusStruct.TE_MATCH_POINTS[3] | stASStatusStruct.TE_MATCH_POINTS[4] << 8),
        //  stASStatusStruct.TE_MATCH_POINTS[5],

        //  (stASStatusStruct.TE_MATCH_POINTS[6] | stASStatusStruct.TE_MATCH_POINTS[7] << 8),
        //  stASStatusStruct.TE_MATCH_POINTS[8]));

        // DEBUG_PRINT(("-------------------------------------------------------------------------------------"));

        graph_val.echo_arr[0] = (stASStatusStruct.TE_MATCH_POINTS[0] | stASStatusStruct.TE_MATCH_POINTS[1] << 8);
        graph_val.echo_arr[1] = (stASStatusStruct.TE_MATCH_POINTS[3] | stASStatusStruct.TE_MATCH_POINTS[4] << 8);
        graph_val.echo_arr[2] = (stASStatusStruct.TE_MATCH_POINTS[6] | stASStatusStruct.TE_MATCH_POINTS[7] << 8);

        graph_pub.publish(graph_val);

        // DEBUG_PRINT(("\n XSIZE [%d] HOLDOFF [%d] \n XRANGE [%d] ",
        // stASStatusStruct.XSIZE,
        // stASStatusStruct.HOLDOFF,
        // stASStatusStruct.XRANGE));

        // printbuffer(9, &stASStatusStruct.TE_MATCH_POINTS);

        // DEBUG_PRINT(("\n Velocity [%d] Waiting_Probe Zero [6] %d \n Probe Error [7] %d",
        //  stASStatusStruct.VELOCITY,
        //  GETBIT(stASStatusStruct.PROBE_INFO, 6),
        //  GETBIT(stASStatusStruct.PROBE_INFO, 7)));

        // DEBUG_PRINT(("\n Echoes %d \n Unit [0] %d \n Valid Thickness Measurement [2] %d \n Deep Coat [3] %d ",
        //  stASStatusStruct.ECHOES,
        //  GETBIT(stASStatusStruct.BITFIELD, 0),
        //  GETBIT(stASStatusStruct.BITFIELD, 2),
        //  GETBIT(stASStatusStruct.BITFIELD, 3)));
    }
}
