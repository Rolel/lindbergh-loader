#include <stdint.h>
#include <stdio.h>

int initDriveboard();
void processDrivePacket(uint8_t *buf, int player);
ssize_t driveboardRead(int fd, void *buf, size_t count);
ssize_t driveboardWrite(int fd, const void *buf, size_t count);
int driveBoardioctl(int fd, unsigned int request, void *data);