/*
 * File:   fzoomDriver.h
 *
 * Author: Ronaldo MErcado
 */

#ifndef FZOOMDRIVER_H
#define FZOOMDRIVER_H

/* function codes */
#define FZOOM_INITIALISE               0x01
#define FZOOM_WRITE_ZOOM_POSITION      0x02
#define FZOOM_WRITE_POSITION_THRESHOLD 0x03
#define FZOOM_WRITE_TIMEOUT_DELAY      0x03
#define FZOOM_READ_STATUS              0x03

/* Pack all structures defined here on 1-byte boundaries */
#pragma pack(1)

/*---------------------------------------------*/
/* general purpose instruction */
/*---------------------------------------------*/

typedef struct transmitInstruction
{
    unsigned char preamble[5];
    unsigned char data[2];
    unsigned char checkSum;
} transmitInstruction;

typedef struct statusRequest
{
    unsigned char message[10];
} statusRequest;

typedef struct statusMessage
{
    unsigned char preamble[10];
    unsigned short int data;
    unsigned char checkSum;
} statusMessage;

#define MAX_FZOOM_FRAME_SIZE 80 /* should be 13 characters for status message */

/* Revert to packing that was in effect when compilation started */
#pragma pack()


#define  FZOOM_STATUS_COMMAND_STRING            "FZOOM_STATUS"
#define  FZOOM_INITIALISE_COMMAND_STRING        "FZOOM_INITIALISE"
#define  FZOOM_POSITION_COMMAND_STRING          "FZOOM_POSITION"
#define  FZOOM_THRESHOLD_COMMAND_STRING         "FZOOM_THRESHOLD"
#define  FZOOM_TIMEOUT_DELAY_COMMAND_STRING     "FZOOM_TIMEOUT_DELAY"
#define  FZOOM_IO_ERRORS_COMMAND_STRING         "FZOOM_IO_ERRORS"
#define  FZOOM_CHECKSUM_ERRORS_COMMAND_STRING   "FZOOM_CHECKSUM_ERRORS"
#define  FZOOM_STATUS_MSG_ERRORS_COMMAND_STRING "FZOOM_MSG_ERRORS"
#define  FZOOM_READ_OK_COMMAND_STRING           "FZOOM_READ_OK"


#endif 
