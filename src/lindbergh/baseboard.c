#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include "baseboard.h"
#include "config.h"
#include "jvs.h"
#include "jvsserial.h"


#define SERIAL_STRING "FE11-X018012022X"

#define BASEBOARD_INIT 0x300
#define BASEBOARD_GET_VERSION 0x8004BC02
#define BASEBOARD_SEEK_SHM 0x400
#define BASEBOARD_READ_SRAM 0x601
#define BASEBOARD_WRITE_SRAM 0x600
#define BASEBOARD_REQUEST 0xC020BC06
#define BASEBOARD_RECEIVE 0xC020BC07
#define BASEBOARD_GET_SERIAL 0x120
#define BASEBOARD_WRITE_FLASH 0x180
#define BASEBOARD_GET_SENSE_LINE 0x210
#define BASEBOARD_PROCESS_JVS 0x220
#define BASEBOARD_READY 0x201

typedef struct {
    uint32_t srcAddress;
    uint32_t srcSize;
    uint32_t destAddress;
    uint32_t destSize;
} BaseboardCommand;

BaseboardCommand jvsCommand = {0};
BaseboardCommand serialCommand = {0};

typedef struct {
    uint32_t *data;
    uint32_t offset;
    uint32_t size;
} readData_t;

typedef struct {
    uint32_t offset;
    uint32_t *data;
    uint32_t size;
} writeData_t;

FILE *sram = NULL;

unsigned int sharedMemoryIndex = 0;
uint8_t sharedMemory[1024 * 32] = {0};

int selectReply = -1;
int jvsFileDescriptor = -1;

/**
 * Set up the fake Lindy baseboard
 * @return
 */
int initBaseboard() {
    // TODO: move SRAM part to a dedicated function
    char *sramPath = getConfig()->sramPath;
    sram = fopen(sramPath, "a");
    // Create file if it doesn't exist
    if (sram == NULL) {
        printf("Error: Cannot open %s\n", sramPath);
        return 1;
    }
    fclose(sram);
    sram = fopen(sramPath, "rb+");
    fseek(sram, 0, SEEK_SET);



    // Config disable JVS emulation and jvsPath is set, we open serial port for JVS interface
    if (getConfig()->emulateJVS == 0 && strcmp(getConfig()->jvsPath, "none") != 0) {
        jvsFileDescriptor = openJVSSerial(getConfig()->jvsPath);
        if (jvsFileDescriptor < 0) {
            printf("Error: Failed to open '%s' for JVS\n", getConfig()->jvsPath);
            exit(1);
        }

        // Set up the serial port settings, so it acts as a JVS interface
        initJVSSerial(jvsFileDescriptor);
    }

    return 0;
}

ssize_t baseboardRead(int fd, void *buf, size_t count) {
    memcpy(buf, &sharedMemory[sharedMemoryIndex], count);
    return count;
}

ssize_t baseboardWrite(int fd, const void *buf, size_t count) {
    memcpy(&sharedMemory[sharedMemoryIndex], buf, count);
    return count;
}

int baseboardSelect(int nfds, fd_set *restrict readfds, fd_set *restrict writefds, fd_set *restrict exceptfds,
                    struct timeval *restrict timeout) {
    return selectReply;
}

void baseboardIoctlRequest(uint32_t *_data) {

    switch (_data[0]) {

        case BASEBOARD_GET_SERIAL: // bcCmdSysInfoGetReq
        {
            serialCommand.destAddress = _data[1];
            serialCommand.destSize = _data[2];
        }
            break;

        case BASEBOARD_WRITE_FLASH: // bcCmdSysFlashWrite
        {
            printf("Warning: The game attempted to write to the baseboard flash\n");
        }
            break;

        case BASEBOARD_PROCESS_JVS: {
            jvsCommand.srcAddress = _data[1];
            jvsCommand.srcSize = _data[2];
            jvsCommand.destAddress = _data[3];
            jvsCommand.destSize = _data[4];
            memcpy(inputBuffer, &sharedMemory[jvsCommand.srcAddress], jvsCommand.srcSize);

            printf("JVS DEBUG: Writing ~~~~~~~~~~~~~~~~~~~~~~~~\n");
            printf("JVS DEBUG: Data as Hex: ");
            for (int i = 0; i < jvsCommand.srcSize; i++) {
                printf("%02X ", (unsigned char) inputBuffer[i]);
            }
            printf("\n");

            if (getConfig()->emulateJVS) {
                processPacket();
            } else if (jvsFileDescriptor >= 0) {
                // F0 D9 Command: Reset (RESET)
                for (int i = 0; i < jvsCommand.srcSize; i++) {
                    write(jvsFileDescriptor, &inputBuffer[i], 1);
                    if (inputBuffer[i] == 0xF0) {
                        printf("JVS DEBUG: got 0xF0 (Reset), soft senseLine is set to 3.\n");
                        setSenseLine(3);
                    }

                    if (inputBuffer[i] == 0xF1) {
                        printf("JVS DEBUG: got 0xF1 (Set Address), soft senseLine is set to 1.\n");
                        setSenseLine(1);
                    }
                }

                printf("JVS DEBUG: Hardware Control lines: CTS %02X - DSR %02X - DCD %02X \n", getCTS(jvsFileDescriptor), getDSR(jvsFileDescriptor), getDCD(jvsFileDescriptor));
            }
        }
            break;

        case BASEBOARD_GET_SENSE_LINE:
            printf("JVS DEBUG: SEND SEND SEND GetSenseLine\n");
            break;

        default:
            printf("Error: Unknown baseboard command %X\n", _data[0]);
    }

    // Acknowledge the command
    _data[0] |= 0xF0000000;
}


void baseboardIoctlReceive(uint32_t *_data) {
    switch (_data[0] & 0xFFF) {

        case BASEBOARD_GET_SERIAL: {
            memcpy(&sharedMemory[serialCommand.destAddress + 96], SERIAL_STRING, strlen(SERIAL_STRING));
            _data[1] = 1; // Set the status to success
        }
            break;

        case BASEBOARD_GET_SENSE_LINE: {
            /*
             * Values are:
             *  3 = no device, after a RESET
             *  1 = address assigned
             */
            if (getConfig()->emulateJVS) {
                _data[2] = getSenseLine();
                _data[1] = 1; // Set the status to success
            } else {
                _data[2] = (getDSR(jvsFileDescriptor) == 1) ? 1 : 3;
                _data[1] = getCTS(jvsFileDescriptor);
            }
            printf("JVS DEBUG: GetSenseLine: %02X - %02X \n", _data[2], _data[1]);
        }
            break;

        case BASEBOARD_PROCESS_JVS: {
            if (getConfig()->emulateJVS) {
                memcpy(&sharedMemory[jvsCommand.destAddress], outputBuffer, outputPacket.length + 3);
                _data[2] = jvsCommand.destAddress;
                _data[3] = outputPacket.length + 3;
                _data[1] = 1; // Set the status to success
            } else if (jvsFileDescriptor >= 0) {
                /*
                // int count = readBytes(jvsFileDescriptor, &sharedMemory[jvsCommand.destAddress], 1024);
                // int count = readJVSFrameNonBlocking(jvsFileDescriptor, &sharedMemory[jvsCommand.destAddress]);
                // int count = readJVSFrame(jvsFileDescriptor, &sharedMemory[jvsCommand.destAddress]);

                if (count == -1)
                    count = 0;

                _data[2] = jvsCommand.destAddress;
                _data[3] = count;
                _data[1] = getCTS(jvsFileDescriptor);
                */

                struct jvsFrame frame = readJVSFrameFromThread();
                memcpy(&sharedMemory[jvsCommand.destAddress], frame.buffer, frame.size);

                _data[2] = jvsCommand.destAddress;
                _data[3] = frame.size;
                _data[1] = frame.ready;
            }

            if (_data[3] > 0) {
                printf("JVS DEBUG: Reading ########################\n");
                printf("JVS DEBUG: Data extraction: Ready: %d - Address: %d - Length: %d \n", _data[1], _data[2], _data[3]);
                printf("JVS DEBUG: Hardware Control lines: CTS %02X - DSR %02X - DCD %02X \n", getCTS(jvsFileDescriptor), getDSR(jvsFileDescriptor), getDCD(jvsFileDescriptor));
                /*
                printf("JVS DEBUG: Data as String: "); // Does not use native printf because data contains null bytes
                for (int i = 0; i < _data[3]; i++) {
                    // printf("%02X ", outputBuffer[i]);
                    printf("%c", (&sharedMemory[jvsCommand.destAddress])[i]);
                }
                printf("\n");
                */
                printf("JVS DEBUG: Data as Hex: ");
                for (int i = 0; i < _data[3]; i++) {
                    // printf("%02X ", outputBuffer[i]);
                    printf("%02X ", (&sharedMemory[jvsCommand.destAddress])[i]);
                }
                printf("\n");
            }
        }
            break;

        default:
            printf("Error: Unknown baseboard receive command %X\n", _data[0] & 0xFFF);
    }

    // Acknowledge the command
    _data[0] |= 0xF0000000;
}

int baseboardIoctl(int fd, unsigned int request, void *data) {
    switch (request) {

        case BASEBOARD_GET_VERSION: {
            uint8_t versionData[4] = {0x00, 0x19, 0x20, 0x07};
            memcpy(data, versionData, 4);
            return 0;
        }
            break;

        case BASEBOARD_INIT: {
            // selectReply = -1; Considering adding this in
            return 0;
        }
            break;

        case BASEBOARD_READY: // Not sure this is what it should be called
        {
            selectReply = 0;
            return 0;
        }
            break;

        case BASEBOARD_SEEK_SHM: {
            sharedMemoryIndex = (unsigned int) data;
            return 0;
        }
            break;

        case BASEBOARD_READ_SRAM: {
            readData_t *_data = data;
            fseek(sram, _data->offset, SEEK_SET);
            fread(_data->data, 1, _data->size, sram);
            return 0;
        }
            break;

        case BASEBOARD_WRITE_SRAM: {
            writeData_t *_data = data;
            fseek(sram, _data->offset, SEEK_SET);
            fwrite(_data->data, 1, _data->size, sram);
            return 0;
        }
            break;

        case BASEBOARD_REQUEST: {
            uint32_t *_data = data;
            baseboardIoctlRequest(_data);

            return 0;
        }
            break;

        case BASEBOARD_RECEIVE: {
            uint32_t *_data = data;
            baseboardIoctlReceive(_data);

            return 0;
        }
            break;

        default:
            printf("Error: Unknown baseboard ioctl %X\n", request);
    }

    return 0;
}
