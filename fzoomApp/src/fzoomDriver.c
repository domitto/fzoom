#include <assert.h>
#include <string.h>
#include <stdio.h>

#include <cantProceed.h>
#include <epicsExport.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <errlog.h>
#include <iocsh.h>
#include <dbDefs.h>

#include <asynCommonSyncIO.h>
#include <asynDriver.h>
#include <asynOctet.h>
#include <asynOctetSyncIO.h>
#include <asynStandardInterfaces.h>

#include "fzoomDriver.h"

/* Local variable declarations */
static char *driver = "drvFzoomAsyn";   /* String for asynPrint */

#define FZOOM_READ_TIMEOUT  4.0         /* Timeout for asynOctetSyncIO->writeRead */

#define MAX_EXTRA_READS 10              /* maximum number of retries reading extra */

typedef enum {
    fzoomStatusCmd, 
    fzoomInitialiseCmd,
    fzoomPositionCmd, 
    fzoomThresholdCmd,
    fzoomTimeoutDelayCmd,
    fzoomIOErrors,           /* driver interrogation commands */
    fzoomChecksumErrors,
    fzoomStatusMsgErrors,
    fzoomReadOK
} fzoomCommand;

/* MAX_FZOOM_COMMANDS has to match the number of entries in fzoomCommand enum */
#define MAX_FZOOM_COMMANDS 9

/**
*  \brief Mapping for record initialisation
*
*  structure to map the asyn 'reasons' using the INP/OUT string
*  in the drvUserCreate call from the asynDrvUser methods
*/
typedef struct {
    fzoomCommand command;
    char *commandString;
} fzoomCommandStruct;

static fzoomCommandStruct fzoomCommands[] =
{
    { fzoomStatusCmd,       FZOOM_STATUS_COMMAND_STRING},
    { fzoomInitialiseCmd,   FZOOM_INITIALISE_COMMAND_STRING},
    { fzoomPositionCmd,     FZOOM_POSITION_COMMAND_STRING},
    { fzoomThresholdCmd,    FZOOM_THRESHOLD_COMMAND_STRING},
    { fzoomTimeoutDelayCmd, FZOOM_TIMEOUT_DELAY_COMMAND_STRING},
    { fzoomIOErrors,        FZOOM_IO_ERRORS_COMMAND_STRING},
    { fzoomChecksumErrors,  FZOOM_CHECKSUM_ERRORS_COMMAND_STRING},
    { fzoomStatusMsgErrors, FZOOM_STATUS_MSG_ERRORS_COMMAND_STRING},
    { fzoomReadOK,          FZOOM_READ_OK_COMMAND_STRING},
};


/**
*  \brief Hardware status model
*  
*/
typedef struct fzoomDeviceInfo {
    /* STATUS MESSAGE DECODING */
    char readyToAcquire; /**< Status readback, from nibble 1 of status message */
    char errorCondition; /**< Status readback, From nibble 2 of status message */
    char errorThreshold; /**< From Nibble 3 of status message */
    char timeoutDelayRb; /**< From Nibble 4 of status message (readback) */
    
    /* BUFFERS for requests */
    int zoomPosition; /**< commanded position buffer. 
                           Valid values are from 0x0001 to 0x03E8 (1 to 1000) */
    int posThreshold; /**< Commanded position buffer.
                           Valid values are from 0x03F2 to 0x0400 (1010 to 1024) */
    int timeoutDelay; /**< Commanded position buffer.
                           Valid values are from 0x0401 to 0x0410 (1025 to 1040) */
    
} fzoomDeviceInfo;

/** \brief asyn private driver information */
typedef struct fzoomPvt {
    const char *portName;
    const char *octetPortName;
    asynUser *pasynUserOctet;   /*asynUser for asynOctet interface to asyn octet port*/ 
    int isConnected;            /* Connection status */
    int ioStatus;               /* I/O error status */
    asynUser  *pasynUserTrace;  /* asynUser for asynTrace on this port */
    asynStandardInterfaces asynStdInterfaces;
    /* RAW STATUS */
    unsigned short data;        /* 32 bit storage required */
    fzoomDeviceInfo devInfo;    /* unpacked status + requests */
    epicsMutexId mutexId;       /* Mutex for interlocking access to doFzoomIO */
    /* message */
    char fzoomRequest[MAX_FZOOM_FRAME_SIZE];
    char fzoomReply[MAX_FZOOM_FRAME_SIZE];
    /** count of IOErrors */
    int IOErrors;
    /** count of checkSum errors */
    int checkSumErrors;
    /** count of statusMsg errors */
    int statusMsgErrors;
    /** count of successful reads */
    int readOK;         
    int initialise;
} fzoomPvt;

typedef struct fzoomPvt *FZOOM_ID;

/* asynCommon methods */
static void asynReport(void *drvPvt,FILE *fp,int details);
static asynStatus asynConnect(void *drvPvt,asynUser *pasynUser);
static asynStatus asynDisconnect(void *drvPvt,asynUser *pasynUser);

static asynStatus drvUserCreate     (void *drvPvt, asynUser *pasynUser,
                                     const char *drvInfo,
                                     const char **pptypeName, size_t *psize);
static asynStatus drvUserGetType    (void *drvPvt, asynUser *pasynUser,
                                     const char **pptypeName, size_t *psize);
static asynStatus drvUserDestroy    (void *drvPvt, asynUser *pasynUser);

/* These functions are in the asynInt32 interface */
static asynStatus writeInt32        (void *drvPvt, asynUser *pasynUser,
                                     epicsInt32 value);
static asynStatus readInt32         (void *drvPvt, asynUser *pasynUser,
                                     epicsInt32 *value);
static asynStatus getBounds         (void *drvPvt, asynUser *pasynUser,
                                     epicsInt32 *low, epicsInt32 *high);
/* asynCommon methods */
static const struct asynCommon drvCommon = {
    asynReport,
    asynConnect,
    asynDisconnect
};
/* asynDrvUser methods */
static asynDrvUser drvUser = {
    drvUserCreate,
    drvUserGetType,
    drvUserDestroy
};

/* asynInt32 methods */
static asynInt32 drvInt32 = {
    writeInt32,
    readInt32,
    getBounds,
    NULL,
    NULL
};

/* prototype for zoom IO function */
static int doFzoomIO(FZOOM_ID pfzoom, asynUser *pasynUser, fzoomCommand function);

/* init routine */
int drvFzoomAsynConfigure(const char *portName, const char *octetPortName)
{
    FZOOM_ID pfzoomPvt;
    asynStatus status;

    assert(sizeof(unsigned short) == 2); /* 32 bits required */
    pfzoomPvt  = callocMustSucceed(1, sizeof(*pfzoomPvt), "drvFzoomAsynConfigure");
    pfzoomPvt->portName = epicsStrDup(portName);
    pfzoomPvt->octetPortName = epicsStrDup(octetPortName);
    pfzoomPvt->mutexId = epicsMutexMustCreate();

    /* initialise position, error threshold */
    pfzoomPvt->devInfo.zoomPosition = 1;
    pfzoomPvt->devInfo.posThreshold = 1010;
    pfzoomPvt->devInfo.timeoutDelay = 1025;

    /* connect to asyn octet port with asynOctetSyncIO */ 
    status = pasynOctetSyncIO->connect(octetPortName, 0, &pfzoomPvt->pasynUserOctet,
                                       0);
    if (status != asynSuccess) {
        errlogPrintf("%s::drvFzoomAsynConfigure port %s"
                     " can't connect to Octet port %s.\n",
                     driver, portName, octetPortName);
        return asynError;
    }

    /* Create asynUser for asynTrace */
    pfzoomPvt->pasynUserTrace = pasynManager->createAsynUser(0, 0);
    pfzoomPvt->pasynUserTrace->userPvt = pfzoomPvt;

    status = pasynManager->registerPort(pfzoomPvt->portName,
                                        ASYN_CANBLOCK,
                                        1, /* autoconnect */
                                        0, /* medium priority */
                                        0); /* default stack size */
    if (status != asynSuccess) {
        errlogPrintf("%s::drvFzoomAsynConfigure port %s"
                     "%s:: can't register port\n",
                     driver, pfzoomPvt->portName, pfzoomPvt->octetPortName);
        return(asynError);
    }

    /* Create asyn interfaces and register with asynManager */
    pfzoomPvt->asynStdInterfaces.common.pinterface          = (void *)&drvCommon;
    pfzoomPvt->asynStdInterfaces.drvUser.pinterface         = (void *)&drvUser;
    pfzoomPvt->asynStdInterfaces.int32.pinterface           = (void *)&drvInt32;

    status = pasynStandardInterfacesBase->initialize(pfzoomPvt->portName, &pfzoomPvt->asynStdInterfaces,
                                                     pfzoomPvt->pasynUserTrace, pfzoomPvt);
    if (status != asynSuccess) {
        errlogPrintf("%s::drvFzoomAsynConfigure port %s"
                     " can't register standard interfaces: %s\n",
                     driver, pfzoomPvt->portName, pfzoomPvt->pasynUserTrace->errorMessage);
        return(asynError);
    }
    return (asynSuccess);
}


/* asynDrvUser routines */
static asynStatus drvUserCreate(void *drvPvt, asynUser *pasynUser,
                                const char *drvInfo,
                                const char **pptypeName, size_t *psize)
{
    FZOOM_ID pfzoom = (FZOOM_ID)drvPvt;
    int i;
    char *pstring;
    
    for (i=0; i<MAX_FZOOM_COMMANDS; i++) {
        pstring = fzoomCommands[i].commandString;
        if (epicsStrCaseCmp(drvInfo, pstring) == 0) {
            pasynUser->reason = fzoomCommands[i].command;
            if (pptypeName) *pptypeName = epicsStrDup(pstring);
            if (psize) *psize = sizeof(fzoomCommands[i].command);
            asynPrint(pasynUser, ASYN_TRACE_FLOW,
                      "%s::drvUserCreate, port %s command=%s\n", 
                      driver, pfzoom->portName, pstring);
            return(asynSuccess);
        }
    }
    
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s::drvUserCreate, port %s unknown command=%s\n", 
              driver, pfzoom->portName, drvInfo);
    return(asynError);
}

static asynStatus drvUserGetType(void *drvPvt, asynUser *pasynUser,
                                 const char **pptypeName, size_t *psize)
{
    int command = pasynUser->reason;

    if (pptypeName)
        *pptypeName = epicsStrDup(fzoomCommands[command].commandString);
    if (psize) *psize = sizeof(command);
    return(asynSuccess);
}

static asynStatus drvUserDestroy(void *drvPvt, asynUser *pasynUser)
{
    return(asynSuccess);
}

/***********************/
/* asynCommon routines */
/***********************/

/* Connect */
static asynStatus asynConnect(void *drvPvt, asynUser *pasynUser)
{
    FZOOM_ID pfzoomPvt = (FZOOM_ID)drvPvt;
    asynStatus status;
    int signal;

    status = pasynManager->getAddr(pasynUser, &signal);
    if (status != asynSuccess) {
        errlogPrintf("%s::asynConnect getAddr failed port %s", driver, pfzoomPvt->portName);
        return (asynError);
    }
    status = pasynManager->exceptionConnect(pasynUser);
    if (status != asynSuccess) {
        errlogPrintf("%s::asynConnect exceptionConnect failed port %s",
            driver, pfzoomPvt->portName);
        return (asynError);
    }
    return(asynSuccess);
}

/* Disconnect */
static asynStatus asynDisconnect(void *drvPvt, asynUser *pasynUser)
{
    /* Does nothing for now.  
     * May be used if connection management is implemented */
    pasynManager->exceptionDisconnect(pasynUser);
    return(asynSuccess);
}


/* Report  parameters */
static void asynReport(void *drvPvt, FILE *fp, int details)
{
    FZOOM_ID pfzoom = (FZOOM_ID)drvPvt;

    fprintf(fp, "fzoom port: %s\n", pfzoom->portName);
    if (details) {
        fprintf(fp, "    asynOctet server:   %s\n", pfzoom->octetPortName);
        fprintf(fp, "    ready to acquire:   %d\n", pfzoom->devInfo.readyToAcquire);
        fprintf(fp, "    error condition:    %d\n", pfzoom->devInfo.errorCondition);
        fprintf(fp, "    error threshold:    %d\n", pfzoom->devInfo.errorThreshold);
        fprintf(fp, "    timeout delay rb:   %d (readback)\n", 
                                                    pfzoom->devInfo.timeoutDelayRb);
        fprintf(fp, "    zoom position:      %d\n", pfzoom->devInfo.zoomPosition);
        fprintf(fp, "    position threshold: %d\n", pfzoom->devInfo.posThreshold);
        fprintf(fp, "    timeout delay:      %d\n", pfzoom->devInfo.timeoutDelay);
    }
}

/* 
**  asynInt32 support
*/

static asynStatus readInt32 (void *drvPvt, asynUser *pasynUser, epicsInt32 *value)
{
    FZOOM_ID pfzoomPvt = (FZOOM_ID)drvPvt;
    asynStatus status;
    int offset;
    
    pasynManager->getAddr(pasynUser, &offset);
    *value = 0;
    
    switch(pasynUser->reason) {
        case fzoomStatusCmd:
            status = doFzoomIO(pfzoomPvt, pasynUser, fzoomStatusCmd);
            if (status != asynSuccess) {
                asynPrint(pfzoomPvt->pasynUserTrace, ASYN_TRACE_ERROR,
                          "%s::readInt32 port %s fzoomStatusCmd failed\n",
                          driver, pfzoomPvt->portName);
                return(asynError);
            }
            *value = pfzoomPvt->data;
            break;
        case fzoomInitialiseCmd:
            *value = pfzoomPvt->initialise;
            break;
        case fzoomPositionCmd:
            *value = pfzoomPvt->devInfo.zoomPosition;
            break;
        case fzoomThresholdCmd:
            *value = pfzoomPvt->devInfo.posThreshold;
            break;
        case fzoomTimeoutDelayCmd:
            *value = pfzoomPvt->devInfo.timeoutDelay;
            break;
        case fzoomIOErrors:
            *value = pfzoomPvt->IOErrors;
            break;
        case fzoomChecksumErrors:
            *value = pfzoomPvt->checkSumErrors;
            break;
        case fzoomStatusMsgErrors:
            *value = pfzoomPvt->statusMsgErrors;
            break;
        case fzoomReadOK:
            *value = pfzoomPvt->readOK;
            break;
        default:
            asynPrint(pfzoomPvt->pasynUserTrace, ASYN_TRACE_ERROR,
                      "%s::readInt32 port %s invalid pasynUser->reason %d\n",
                      driver, pfzoomPvt->portName, pasynUser->reason);
            return(asynError);
    }
    return(asynSuccess);
}

static asynStatus writeInt32(void *drvPvt, asynUser *pasynUser, epicsInt32 value)
{
    FZOOM_ID pfzoom = (FZOOM_ID)drvPvt;
    asynStatus status;

    switch(pasynUser->reason) {
        case fzoomInitialiseCmd:
            if (value) {
                status = doFzoomIO(pfzoom, pasynUser, fzoomInitialiseCmd);
                if (status != asynSuccess) return(status);
                asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                          "%s::writeInt32 port %s fzoomInitialiseCmd (performed)"
                          " value = %d\n",
                          driver, pfzoom->portName, value);
                pfzoom->initialise = 1;
            }
            else
            {
                asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                      "%s::writeInt32 port %s fzoomInitialiseCmd zero (ignored)\n",
                      driver, pfzoom->portName);
                pfzoom->initialise = 0;
            }
            break;
        case fzoomPositionCmd:
            if ((value < 1) || (value > 1000))
            {
                asynPrint(pasynUser, ASYN_TRACE_ERROR,
                      "%s::writeInt32 port %s invalid value for zoom position %d\n",
                      driver, pfzoom->portName, value);
                return(asynError);
            }
            pfzoom->devInfo.zoomPosition = value;
            status = doFzoomIO(pfzoom, pasynUser, fzoomPositionCmd);
            if (status != asynSuccess) return(status);
            asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                      "%s::writeInt32 port %s fzoomPositionCmd\n",
                      driver, pfzoom->portName);
            break;
        case fzoomThresholdCmd:
            if ((value < 1010) || (value > 1024))
            {
                asynPrint(pasynUser, ASYN_TRACE_ERROR,
                      "%s::writeInt32 port %s invalid value for threshold %d\n",
                      driver, pfzoom->portName, value);
                return(asynError);
            }
            pfzoom->devInfo.posThreshold = value;
            status = doFzoomIO(pfzoom, pasynUser, fzoomThresholdCmd);
            if (status != asynSuccess) return(status);
            asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                      "%s::writeInt32 port %s fzoomThresholdCmd",
                      driver, pfzoom->portName);
            break;
        case fzoomTimeoutDelayCmd:
            if ((value < 1025) || (value > 1040))
            {
                asynPrint(pasynUser, ASYN_TRACE_ERROR,
                      "%s::writeInt32 port %s invalid value for timeout %d\n",
                      driver, pfzoom->portName, value);
                return(asynError);
            }
            pfzoom->devInfo.timeoutDelay = value;
            status = doFzoomIO(pfzoom, pasynUser, fzoomTimeoutDelayCmd);
            if (status != asynSuccess) return(status);
            asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                      "%s::writeInt32 port %s fzoomTimeoutDelayCmd",
                      driver, pfzoom->portName);
            break;
        default:
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                      "%s::writeInt32 port %s invalid pasynUser->reason %d\n",
                      driver, pfzoom->portName, pasynUser->reason);
            return(asynError);
    }
    return(asynSuccess);
}

/* This function should return the valid range for asynInt32 data.  
 * However, for Modbus devices there is no way for the driver to know what the 
 * valid range is, since the device could be an 8-bit unipolar ADC, 12-bit bipolar DAC,
 * etc.  Just return 0.
 * It is up to device support to correctly interpret the numbers that are returned. */
 
static asynStatus getBounds (void *drvPvt, asynUser *pasynUser, 
                             epicsInt32 *low, epicsInt32 *high)
{
    *high = 0;
    *low = 0;
    return(asynSuccess);
}

/*
 *  Calculates a 16-bit checksum for the transmit instruction
 *  Ignores the last byte, assumed to be where the checksum will be
 *  written
 */
static unsigned char calculateChecksum(const char *transmit, size_t count)
{
    long sum = 0;
    size_t index;
    for (index = 0; index < count-1; index++)
    {
        sum += (unsigned int) transmit[index];
    }
    return (unsigned char)(sum);
}

/* fzoom module message templates */
const statusRequest statusRequestTemplate = { 
  {
     '\x08', '\x0F', '\xF0', '\xB0', '\x04', 
     '\x0F', '\xF1', '\x03', '\xC4', '\x82' 
  }
};

const statusMessage statusMessageTemplate =
   {
     { '\x4F', '\x0A', '\x0F', '\xF1', '\xB4', '\x04', 
       '\x0F', '\xF0', '\x03', '\xC4', },
     0x0000,
     'S' 
   };

const transmitInstruction transmitInstructionTemplate =
{
    {'\x06', '\x0F', '\xF0', '\x21', '\xC2'},  /* preamble*/
    { 'X', 'X' },                              /* data */
    'S'                                        /* checksum */
};

const transmitInstruction initializeInstruction =
{
    {'\x06', '\x0F', '\xF0', '\x21', '\xC2'},  /* preamble*/
    {'\x0', '\x0' },                           /* data */
    '\xE8'                                     /* checksum */
};

static struct messageSize {
    fzoomCommand function;
    size_t       requestSize;
    size_t       replySize;
} msgSizes[] = {
    {fzoomStatusCmd,       sizeof(statusRequest), sizeof(statusMessage) },
    {fzoomPositionCmd,     sizeof(transmitInstruction), 1},
    {fzoomThresholdCmd,    sizeof(transmitInstruction), 1},
    {fzoomTimeoutDelayCmd, sizeof(transmitInstruction), 1},
    {fzoomInitialiseCmd,   sizeof(transmitInstruction), 1}
};
static const int MAX_MSG_SIZES = sizeof(msgSizes)/sizeof(msgSizes[0]);

static int doFzoomIO(FZOOM_ID pfzoom, asynUser *pasynUser, fzoomCommand function)
{
    asynStatus status=asynSuccess;
    int autoConnect;
    size_t byteCount;
    transmitInstruction *transmit;
    statusMessage *statusMsg;
    size_t requestSize = 0;
    size_t replySize = 0;
    size_t nwrite;
    size_t nread;
    size_t nreadExtra;
    int eomReason;
    unsigned char checkSum;
    int extraReadCount;
    int index;
    int found = FALSE;
 
    /* We protect the code in this function with a Mutex, 
     * because the "data" element of pfzoom may be used by several threads */ 
    epicsMutexMustLock(pfzoom->mutexId);

    /* If the Octet driver is not set for autoConnect then do connection 
       management ourselves */
    status = pasynManager->isAutoConnect(pfzoom->pasynUserOctet, &autoConnect);
    if (!autoConnect) {
        /* See if we are connected */
        status = pasynManager->isConnected(pfzoom->pasynUserOctet, 
                                           &pfzoom->isConnected);
         /* If we have an I/O error or are disconnected then disconnect device 
            and reconnect */
        if ((pfzoom->ioStatus != asynSuccess) || !pfzoom->isConnected) {
            if (pfzoom->ioStatus != asynSuccess) 
                asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                          "%s::doFzoomIO port %s has I/O error\n",
                          driver, pfzoom->portName);
            if (!pfzoom->isConnected) 
                asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                          "%s::doFzoomIO port %s is disconnected\n",
                          driver, pfzoom->portName);
            status = pasynCommonSyncIO->disconnectDevice(pfzoom->pasynUserOctet);
            if (status == asynSuccess) {
                asynPrint(pasynUser, ASYN_TRACE_FLOW, 
                          "%s::doFzoomIO port %s disconnect device OK\n",
                          driver, pfzoom->portName);
            } else {
                asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                          "%s::doFzoomIO port %s disconnect error=%s\n",
                          driver, pfzoom->portName, 
                          pfzoom->pasynUserOctet->errorMessage);
            }
            status = pasynCommonSyncIO->connectDevice(pfzoom->pasynUserOctet);
            if (status == asynSuccess) {
                asynPrint(pasynUser, ASYN_TRACE_FLOW, 
                          "%s::doFzoomIO port %s connect device OK\n",
                          driver, pfzoom->portName);
            } else {
                asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                          "%s::doFzoomIO port %s connect device error=%s\n",
                          driver, pfzoom->portName, 
                          pfzoom->pasynUserOctet->errorMessage);
                goto done;
            }
        }
    }
        
    switch (function) {
        case fzoomStatusCmd:
            byteCount = sizeof(statusRequestTemplate);
            memcpy(pfzoom->fzoomRequest, 
                  (const void *) &statusRequestTemplate, byteCount );
            asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, 
                      pfzoom->fzoomRequest, byteCount,
                      "%s::doFzoomIO, port %s fzoomStatusCmd\n", 
                      driver, pfzoom->portName);
            break;
        case fzoomInitialiseCmd:
            byteCount = sizeof(initializeInstruction);
            memcpy(pfzoom->fzoomRequest, 
                   (const void *) &initializeInstruction, byteCount );
            asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, 
                      pfzoom->fzoomRequest, byteCount,
                      "%s::doFzoomIO, port %s fzoomInitialiseCmd\n", 
                      driver, pfzoom->portName);
            break;
        case fzoomPositionCmd:
            byteCount = sizeof(transmitInstruction);
            memcpy(pfzoom->fzoomRequest, 
                   (const void *) &transmitInstructionTemplate, byteCount);
            transmit = (transmitInstruction *) pfzoom->fzoomRequest;
            transmit->data[0] = (pfzoom->devInfo.zoomPosition & 0xFF00) >> 8;
            transmit->data[1] = pfzoom->devInfo.zoomPosition & 0x00FF;
            transmit->checkSum = calculateChecksum((char *)transmit, 
                                                    sizeof(transmitInstruction) );
            asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, 
                      pfzoom->fzoomRequest, byteCount,
                      "%s::doFzoomIO, port %s fzoomPositionCmd\n", 
                      driver, pfzoom->portName);
            break;
        case fzoomThresholdCmd:
            byteCount = sizeof(transmitInstruction);
            memcpy(pfzoom->fzoomRequest, 
                   (const void *) &transmitInstructionTemplate, byteCount);
            transmit = (transmitInstruction *) pfzoom->fzoomRequest;
            transmit->data[0] = (pfzoom->devInfo.posThreshold & 0xFF00) >> 8 ;
            transmit->data[1] = pfzoom->devInfo.posThreshold & 0x00FF;
            transmit->checkSum = calculateChecksum((char *)transmit, 
                                                    sizeof(transmitInstruction) );
            asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, 
                      pfzoom->fzoomRequest, byteCount,
                      "%s::doFzoomIO, port %s fzoomThresholdCmd\n", 
                      driver, pfzoom->portName);
            break;
        case fzoomTimeoutDelayCmd:
            byteCount = sizeof(transmitInstruction);
            memcpy(pfzoom->fzoomRequest, 
                   (const void *) &transmitInstructionTemplate, byteCount);
            transmit = (transmitInstruction *) pfzoom->fzoomRequest;
            transmit->data[0] = (pfzoom->devInfo.timeoutDelay & 0xFF00) >> 8 ;
            transmit->data[1] = pfzoom->devInfo.timeoutDelay & 0x00FF;
            transmit->checkSum = calculateChecksum((char *)transmit, 
                                                    sizeof(transmitInstruction) );
            asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, 
                      pfzoom->fzoomRequest, byteCount,
                      "%s::doFzoomIO, port %s fzoomTimeoutDelayCmd,\n", 
                      driver, pfzoom->portName);
            break;
        default:
            asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                      "%s::doFzoomIO, port %s unsupported function code %d\n", 
                      driver, pfzoom->portName, function);
            status = asynError;
            goto done;
    }

    /* find requestSize and replySize */
    for (index = 0; !found && index < MAX_MSG_SIZES; index++) 
    {
        if (msgSizes[index].function == function)
        {
            requestSize = msgSizes[index].requestSize;
            replySize   = msgSizes[index].replySize;
            found = TRUE;
        }
    }

    assert(found);
    assert(replySize > 0 ); /* otherwise it does not make sense to writeRead */

    /* Do the I/O as a write/read cycle */
    status = pasynOctetSyncIO->writeRead(pfzoom->pasynUserOctet, 
                                     pfzoom->fzoomRequest, requestSize,
                                     pfzoom->fzoomReply, replySize,
                                     FZOOM_READ_TIMEOUT,
                                     &nwrite, &nread, &eomReason);
    if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
             "%s::doFzoomIO port %s error calling writeRead,"
             " error=%s, nwrite=%d/%d, nread=%d\n", 
             driver, pfzoom->portName, 
             pfzoom->pasynUserOctet->errorMessage, nwrite, requestSize, nread);
        pfzoom->IOErrors++;
        goto done;
    }
    extraReadCount = 0; 
    while ( (eomReason != ASYN_EOM_CNT) && (extraReadCount < MAX_EXTRA_READS))
    {
        assert(replySize > nread);
        status = pasynOctetSyncIO->read(pfzoom->pasynUserOctet,
                               pfzoom->fzoomReply + nread, replySize - nread, 
                               FZOOM_READ_TIMEOUT, &nreadExtra, &eomReason);
        if (status != asynSuccess) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                 "%s::doFzoomIO port %s error calling read (extra),"
                 " error=%s, nread=%d\n", 
                 driver, pfzoom->portName, 
                 pfzoom->pasynUserOctet->errorMessage, nread);
            pfzoom->IOErrors++;
            goto done;
        }
        nread += nreadExtra;
        extraReadCount ++;
        if (extraReadCount >= MAX_EXTRA_READS) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                 "%s::doFzoomIO port %s error calling read (extra),"
                 " error=%s, nread=%d, extraReadCount=%d\n", 
                 driver, pfzoom->portName, 
                 pfzoom->pasynUserOctet->errorMessage, nread, extraReadCount);
            pfzoom->IOErrors++;
            goto done;

        }
    }
                                         
    if (function == fzoomStatusCmd) {
        
        /* Make sure the status message has the correct checksum */
        pfzoom->readOK++;
        statusMsg = (statusMessage *) pfzoom->fzoomReply;
        checkSum = calculateChecksum(pfzoom->fzoomReply, sizeof(statusMessage));
        if ( checkSum != statusMsg->checkSum) 
        {
            asynPrint(pasynUser, ASYN_TRACE_FLOW,
               "%s::doFzoomIO port %s checksum error in status message: "
                "calculated %x, received %x\n",
                driver, pfzoom->portName, checkSum, statusMsg->checkSum);
            pfzoom->checkSumErrors++;
        }
        
        if ( strncmp((const char *) &statusMessageTemplate, 
                     pfzoom->fzoomReply, 
                     sizeof(statusMessage)-3) != 0 )
        {
            asynPrint(pasynUser, ASYN_TRACE_FLOW,
               "%s::doFzoomIO port %s status msg preamble does not match "
               "expected format\n", driver, pfzoom->portName);
            pfzoom->statusMsgErrors++;
        }
        asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, 
                            (char *) &statusMsg->data, 2, 
                            "%s::doFzoomIO port %s status message\n",
                            driver, pfzoom->portName);

        pfzoom->data = statusMsg->data;
    }
    done:
    epicsMutexUnlock(pfzoom->mutexId);
    return(status);
}


/* iocsh functions */
static const iocshArg ConfigureArg0 = {"Port name",            iocshArgString};
static const iocshArg ConfigureArg1 = {"Octet port name",      iocshArgString};

static const iocshArg * const drvFzoomAsynConfigureArgs[2] = {
    &ConfigureArg0,
    &ConfigureArg1,
};

static const iocshFuncDef drvFzoomAsynConfigureFuncDef=
                                                    {"drvFzoomAsynConfigure", 2,
                                                     drvFzoomAsynConfigureArgs};
static void drvFzoomAsynConfigureCallFunc(const iocshArgBuf *args)
{
  drvFzoomAsynConfigure(args[0].sval, args[1].sval);
}

static void drvFzoomAsynRegister(void)
{
  iocshRegister(&drvFzoomAsynConfigureFuncDef, drvFzoomAsynConfigureCallFunc);
}

epicsExportRegistrar(drvFzoomAsynRegister);

