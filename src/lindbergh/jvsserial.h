#define JVSBUFFER_SIZE 1024

struct jvsFrame {
    int ctsCounter;
    int ready;
    int size;
    char buffer[JVSBUFFER_SIZE];
};

int getDCD(int fd);
int getDSR(int fd);
int getCTS(int fd);

struct jvsFrame readJVSFrameFromThread();
int startJVSFrameThread(int fd);
void * readJVSFrameThread(void * arg);

int openJVSSerial(char *jvsPath);
int initJVSSerial(int fd);
int readJVSFrame(int fd, unsigned char *buffer);


int readJVSFrameNonBlocking(int fd, unsigned char *buffer);
int readJVSFrameFirstTryOfCode(int fd, unsigned char *buffer);
int readBytes(int fd, unsigned char *buffer, int amount);
