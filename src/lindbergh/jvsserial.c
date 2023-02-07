#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>

#include "jvsserial.h"

#define TIMEOUT_SELECT 200
#define CTS_ON_RETRY 20

// Used to read JVS frame in a non-blocking way
struct jvsFrame jvsFrameBuffer;
pthread_mutex_t jvsBuffer_lock = PTHREAD_MUTEX_INITIALIZER;


/**
 * Open the serial interface and return a file descriptor
 * @param jvsPath The serial port path. Ex: "/dev/ttyS3"
 * @return A file descriptor
 */
int openJVSSerial(char *jvsPath) {
    int jvsFileDescriptor = -1;

    // TODO: check O_NOCTTY declaration
    jvsFileDescriptor = open(jvsPath, O_RDWR | O_NOCTTY);
    if (jvsFileDescriptor < 0) {
        printf("Error: Failed to open '%s' for JVS.\n", jvsPath);
    }

    return jvsFileDescriptor;
}


/**
 * Init a serial port (using file descriptor) so it behaves correctly for JVS usage
 * @param fd
 * @return 0|1
 */
int initJVSSerial(int fd) {
    struct termios options;
    int status;

    //  Get the current options
    if (tcgetattr(fd, &options) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    // Set rates
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    /* From doc:
     * If the CLOCAL flag for a line is off, the hardware carrier detect (DCD) signal is significant,
     * an open(2) of the corresponding terminal will block until DCD is asserted, unless the O_NONBLOCK
     * flag is given. If CLOCAL is set, the line behaves as if DCD is always asserted. The software
     * carrier flag is usually turned on for local devices, and is off for lines with modems.
     */
    // options.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines
    options.c_cflag |= CREAD; // Turn on READ, let ctrl lines work

    options.c_cflag &= ~PARENB; // Clear parity bit & disable parity
    options.c_cflag &= ~CSTOPB; // Clear stop field, 1 stop bit
    options.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    options.c_cflag |= CS8; // 8 bits
    options.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control

    options.c_lflag &= ~ICANON; // Disable canonical mode, so no input processing is performed
    options.c_lflag &= ~ECHO; // Disable echo
    options.c_lflag &= ~ECHOE; // Disable erasure
    options.c_lflag &= ~ECHONL; // Disable new-line echo
    options.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                         ICRNL); // Disable any special handling of received bytes

    options.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    options.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Set VMIN and VTIME to 0, so it returns immediately when no data are present
    // options.c_cc[VMIN] = 0;
    // options.c_cc[VTIME] = 0;

    // With threaded serial read we should rely on a blocking read() function so the loop doesn't run crazy => read() could block indefinitely.
    // options.c_cc[VMIN] = 1;
    // options.c_cc[VTIME] = 0;

    // Block until either VMIN characters have been received or VTIME **after first character** has been received
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;


    tcsetattr(fd, TCSANOW, &options);

    /* No use ? Save it for later
    // Set the serial port to non-blocking mode
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    */

    return 0;
}


/**
 * The DCD (Data Carrier Detect) status of a serial port indicates whether a carrier is present on the line, meaning that a connection has been established with another device.
 * @param fd File descriptor of JVS (serial) port
 * @return 0|1 According to control line status
 */
int getDCD(int fd) {
    int status;
    ioctl(fd, TIOCMGET, &status);
    return (status & TIOCM_CAR) != 0;
}

/**
 * The DSR (Data Set Ready) status of a serial port indicates whether the device at the other end of the connection is ready to receive data.
 * @param fd File descriptor of JVS (serial) port
 * @return 0|1 According to control line status
 */
int getDSR(int fd) {
    int status;
    ioctl(fd, TIOCMGET, &status);
    return (status & TIOCM_DSR) != 0;
}

/**
 * The CTS (Clear To Send) status of a serial port indicates whether the device at the other end of the connection is ready to accept data.
 * @param fd File descriptor of JVS (serial) port
 * @return 0|1 According to control line status
 */
int getCTS(int fd) {
    int status;
    ioctl(fd, TIOCMGET, &status);
    return (status & TIOCM_CTS) != 0;
}

/**
 * In charge of reading serial port and bufferize JVS frame
 * @param arg
 * @return
 */
void *readJVSFrameThread(void * arg)
{
    int fd = *((int *) arg);
    int byteCount, bytesRead, ackSize, waitForEnd;
    int ctsRetry = CTS_ON_RETRY;
    char localBuffer[JVSBUFFER_SIZE];

    while (1)
    {
        // Reset local variable
        byteCount = 0;
        ackSize = 0;
        waitForEnd = 0;

        do {
            // printf("SERIAL thread debug: trying to read byte.\n");
            // Try to read a byte from serial, this call will be blocking if VMIN > 0 and VTIME = 0
            bytesRead = read(fd, &localBuffer[byteCount], 1);

            // If nothing on serial and CTS is ON, we try to read "CTS_ON_RETRY" times
            ctsRetry = CTS_ON_RETRY;
            while (bytesRead < 1 && --ctsRetry > 0 && getCTS(fd) > 0 && jvsFrameBuffer.ready == 0) {
                bytesRead = read(fd, &localBuffer[byteCount], 1);
                printf("SERIAL debug: RETRY %d.\n", ctsRetry);
            }

            if (bytesRead > 0) {
                // Sync byte, we will stick in the loop
                if (byteCount == 0 && localBuffer[byteCount] == (char) 0xE0) {
                    // printf("SERIAL thread debug: found SYNC byte.\n");
                    waitForEnd = 1;
                }

                // Size byte
                if (byteCount == 2) {
                    // printf("SERIAL thread debug: found size of %d byte.\n", localBuffer[byteCount]);
                    ackSize = localBuffer[byteCount] + 3;
                }

                // Start counting bytes only if SYNC has been found
                if (waitForEnd) {
                    byteCount++;
                }

                // Reached the end of the message
                if (byteCount == ackSize) {
                    waitForEnd = 0;
                    // printf("SERIAL thread debug: message complete.\n");
                }
            }
        } while (waitForEnd);


        // Lock the buffer while we write to it
        pthread_mutex_lock(&jvsBuffer_lock);

        // Fake a response if CTS is ON but no message from JVS board, not sure about this...
        if (ctsRetry == 0) {
            char hexData[] = {0xE0, 0x00, 0x02, 0x01, 0x03};
            byteCount = sizeof(hexData);
            memcpy(localBuffer, hexData, byteCount);
        }

        // We copy local buffer and length to mutexed buffer
        if (/* ctsRetry == 0 || */ byteCount > 0) {
            memcpy(jvsFrameBuffer.buffer, localBuffer, byteCount);
            jvsFrameBuffer.size = byteCount;
            jvsFrameBuffer.ready = 1;

            // Reset local variable
            byteCount = 0;
            ctsRetry = CTS_ON_RETRY;
        }

        pthread_mutex_unlock(&jvsBuffer_lock);
    }

    return NULL;
}

/**
 * Init the thread in charge of reading serial port
 * @param fd
 * @return 0|1
 */
int startJVSFrameThread(int * fd) {
    int fdlocal = *((int *) fd);
    printf("SERIAL thread debug: starting thread.\n");
    printf("Thread has file descriptor: %d\n", fdlocal);

    // Clean shared struct buffer
    jvsFrameBuffer.ready = 0;
    jvsFrameBuffer.size = 0;
    memset(jvsFrameBuffer.buffer, 0, JVSBUFFER_SIZE);

    pthread_t jvsFrameThread;
    int ret = pthread_create(&jvsFrameThread, NULL, readJVSFrameThread, fd);
    if (ret != 0)
    {
        printf("ERROR Failed to create reader thread");
        exit(1);
        return 1;
    }

    return 0;
}

/**
 * Return a jvsFrame structure with empty or full data, no in between
 * @return
 */
struct jvsFrame readJVSFrameFromThread() {

    struct jvsFrame frame;
    // Lock while reading/writing to shared frame
    pthread_mutex_lock(&jvsBuffer_lock);

    // Check if we have a valid frame
    if (jvsFrameBuffer.ready == 1) {
        frame = jvsFrameBuffer;
        // It has been red, we disable this frame
        jvsFrameBuffer.ready = 0;
    } else {
        frame.ready = 0;
        frame.size = 0;
        memset(frame.buffer, 0, JVSBUFFER_SIZE);
    }
    pthread_mutex_unlock(&jvsBuffer_lock);

    return frame;
}



char readJVSFrameNonBlockingBuffer[1024];
int readJVSFrameNonBlockingByteCount = 0;
int readJVSFrameNonBlockingFrameStarted = 0;
int readJVSFrameNonBlockingFrameSize = 255;
int readJVSFrameNonBlockingInterleavedCalls = 0;

/**
 * Read a full JVS frame without blocking main thread. This is very very ugly code...
 * @param fd File descriptor of JVS (serial) port
 * @param buffer This will contains the JVS data
 * @return int Length of data returned
 * @deprecated
 */
int readJVSFrameNonBlocking(int fd, unsigned char *buffer) {
    int bytes_read = 0;
    if (readJVSFrameNonBlockingFrameStarted == 1) {
        readJVSFrameNonBlockingInterleavedCalls++;
    }

    // try to read a byte from serial
    do {
        bytes_read = read(fd, &readJVSFrameNonBlockingBuffer[readJVSFrameNonBlockingByteCount], 1);

        // If nothing on serial but CTS is ON, we wait and retry
        int retry = 10;
        while (bytes_read != 1 && getCTS(fd) > 0 && retry-- > 0) {
            usleep(100);
            bytes_read = read(fd, &readJVSFrameNonBlockingBuffer[readJVSFrameNonBlockingByteCount], 1);
            // printf("SERIAL debug: RETRY %d.\n", retry);
        }

        if (bytes_read > 0) {
            // Sync byte, we will stick in the loop
            if (readJVSFrameNonBlockingByteCount == 0 &&
                readJVSFrameNonBlockingBuffer[readJVSFrameNonBlockingByteCount] == (char) 0xE0) {
                printf("SERIAL debug: found SYNC byte.\n");
                readJVSFrameNonBlockingFrameStarted = 1;
                readJVSFrameNonBlockingFrameSize = 256;
            }

            // Size byte
            if (readJVSFrameNonBlockingByteCount == 2) {
                printf("SERIAL debug: found size of %d byte.\n",
                       readJVSFrameNonBlockingBuffer[readJVSFrameNonBlockingByteCount]);
                readJVSFrameNonBlockingFrameSize = readJVSFrameNonBlockingBuffer[readJVSFrameNonBlockingByteCount] + 3;
            }

            // Start counting bytes only if SYNC has been found
            if (readJVSFrameNonBlockingFrameStarted) {
                readJVSFrameNonBlockingByteCount++;
            }

            // Reached the end of the message
            if (readJVSFrameNonBlockingByteCount == readJVSFrameNonBlockingFrameSize) {
                printf("SERIAL debug: message complete.\n");
            }
        }
    } while (bytes_read > 0 && readJVSFrameNonBlockingByteCount != readJVSFrameNonBlockingFrameSize);

    // We got something to copy
    if (readJVSFrameNonBlockingByteCount > 0 && readJVSFrameNonBlockingByteCount == readJVSFrameNonBlockingFrameSize) {
        int returnValue = readJVSFrameNonBlockingFrameSize;
        // Copy data
        memcpy(buffer, readJVSFrameNonBlockingBuffer, readJVSFrameNonBlockingByteCount);
        printf("SERIAL debug: message returned with %d interleaved calls.\n", readJVSFrameNonBlockingInterleavedCalls);

        // Reset for next frame and return length
        readJVSFrameNonBlockingFrameStarted = 0;
        readJVSFrameNonBlockingByteCount = 0;
        readJVSFrameNonBlockingInterleavedCalls = 0;

        return readJVSFrameNonBlockingFrameSize;
    }

    return 0;
}


/**
 * Read a full JVS frame. Should work correctly and makes use of ctrol lines.
 * @param fd File descriptor of JVS (serial) port
 * @param buffer This will contains the JVS data
 * @return int Length of data returned
 * @deprecated
 */
int readJVSFrame(int fd, unsigned char *buffer) {
    int byteCount = 0, bytes_read = 0, ackSize = 0;
    char tmpBuffer[1024];
    int waitForEnd = 0;

    do {
        // try to read a byte from serial
        bytes_read = read(fd, &tmpBuffer[byteCount], 1);

        // If nothing on serial but CTS is ON, we wait and retry
        if (bytes_read != 1 && getCTS(fd) > 0) {
            usleep(1000);
            bytes_read = read(fd, &tmpBuffer[byteCount], 1);
            // printf("SERIAL debug: RETRY.\n");
        }

        if (bytes_read > 0) {
            // Sync byte, we will stick in the loop
            if (byteCount == 0 && tmpBuffer[byteCount] == (char) 0xE0) {
                printf("SERIAL debug: found SYNC byte.\n");
                waitForEnd = 1;
            }

            // Size byte
            if (byteCount == 2) {
                printf("SERIAL debug: found size of %d byte.\n", tmpBuffer[byteCount]);
                ackSize = tmpBuffer[byteCount] + 3;
            }

            // Start counting bytes only if SYNC has been found
            if (waitForEnd) {
                byteCount++;
            }

            // Reached the end of the message
            if (byteCount == ackSize) {
                waitForEnd = 0;
                printf("SERIAL debug: message complete.\n");
            }
        }
    } while (waitForEnd);

    // We got something to copy
    if (byteCount > 0) {
        memcpy(buffer, tmpBuffer, byteCount);
    }

    return byteCount;
}

/**
 * Read a full JVS frame. First attempt, code is quite ugly
 * @param fd File descriptor of JVS (serial) port
 * @param buffer This will contains the JVS data
 * @return int Length of data returned
 * @deprecated
 */
int readJVSFrameFirstTryOfCode(int fd, unsigned char *buffer) {

    // printf("SERIAL debug: Trying to read %X bytes from serial.\n", amount);
    // printf("SERIAL DEBUG: Hardware Control lines: CTS %02X - DSR %02X - DCD %02X \n", getCTS(fd), getDSR(fd), getDCD(fd));
    fd_set fd_serial;
    struct timeval tv;

    // Initializes the file descriptor set read_fds to an empty set and adds the file descriptor fd to the set
    FD_ZERO(&fd_serial);
    FD_SET(fd, &fd_serial);

    tv.tv_sec = 0;
    tv.tv_usec = TIMEOUT_SELECT * 1000;

    int byteCount = 0;

    // Wait for data to be available to be read on the file descriptor
    int filesReadyToRead = select(fd + 1, &fd_serial, NULL, NULL, &tv);
    if (filesReadyToRead > 0 && FD_ISSET(fd, &fd_serial)) {
        printf("SERIAL debug: something to read.\n");
        // There is at least one byte to read
        int bytes_read = 0, ackSize = 0;
        char tmpBuffer[1024];
        int waitForEnd = 0;
        do {
            bytes_read = read(fd, &tmpBuffer[byteCount], 1);
            if (bytes_read > 0) {
                // Sync byte, we will stick in the loop
                if (byteCount == 0 && tmpBuffer[byteCount] == (char) 0xE0) {
                    printf("SERIAL debug: found SYNC byte.\n");
                    waitForEnd = 1;
                }

                // Size byte
                if (byteCount == 2) {
                    printf("SERIAL debug: found size of %d byte.\n", tmpBuffer[byteCount]);
                    ackSize = tmpBuffer[byteCount] + 3;
                }

                // Start counting bytes only if SYNC has been found
                if (waitForEnd) {
                    byteCount++;
                }

                // Reached the end of the message
                if (byteCount == ackSize) {
                    waitForEnd = 0;
                    printf("SERIAL debug: message complete.\n");
                }
            }
        } while (waitForEnd);

        memcpy(buffer, tmpBuffer, 1024);

        return byteCount;
    }

    return 0;
}

int readBytes(int fd, unsigned char *buffer, int amount) {

    // printf("SERIAL debug: Trying to read %X bytes from serial.\n", amount);
    // printf("SERIAL DEBUG: Hardware Control lines: CTS %02X - DSR %02X - DCD %02X \n", getCTS(fd), getDSR(fd), getDCD(fd));
    fd_set fd_serial;
    struct timeval tv;

    // Initializes the file descriptor set read_fds to an empty set and adds the file descriptor fd to the set
    FD_ZERO(&fd_serial);
    FD_SET(fd, &fd_serial);

    tv.tv_sec = 0;
    tv.tv_usec = TIMEOUT_SELECT * 1000;

    // Wait for data to be available to be read on the file descriptor
    int filesReadyToRead = select(fd + 1, &fd_serial, NULL, NULL, &tv);
    if (filesReadyToRead > 0 && FD_ISSET(fd, &fd_serial) && getCTS(fd) != 0) {
        int bytes_read = read(fd, buffer, amount);
        if (bytes_read > 0) {
            printf("SERIAL debug: %d bytes from serial.\n", bytes_read);
            return bytes_read;
        }
    }

    // printf("SERIAL debug: nothing to read.\n");
    return 0;
}
