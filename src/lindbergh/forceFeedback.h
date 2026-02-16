#ifndef FFB_H
#define FFB_H

#include <stddef.h>

void sdlFfbInit(void);
void sdlFfbDhutdown(void);
void sdlFfbRumble(float left, float right, int duration_ms);
void sdlFfbDriveboard(const unsigned char *buffer, size_t count);
void sdlFfbOutput(const unsigned char *buffer, size_t count);

#endif